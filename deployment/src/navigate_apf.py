from __future__ import annotations

import argparse
import os
import time
from collections import deque
from pathlib import Path
from typing import Deque, List

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from PIL import Image as PILImage
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, Float32MultiArray
import torch
import yaml
from diffusers.schedulers.scheduling_ddpm import DDPMScheduler

from utils import msg_to_pil, to_numpy, transform_images, load_model
from vint_train.training.train_utils import get_action
from topic_names import IMAGE_TOPIC, WAYPOINT_TOPIC, SAMPLED_ACTIONS_TOPIC

from UniDepth.unidepth.models import UniDepthV2
from UniDepth.unidepth.utils.camera import Pinhole

# ---------------------------------------------------------------------------
# CONFIG & CONSTANTS
# ---------------------------------------------------------------------------
THIS_DIR = Path(__file__).resolve().parent
ROBOT_CONFIG_PATH = THIS_DIR / "../config/robot.yaml"
MODEL_CONFIG_PATH = THIS_DIR / "../config/models.yaml"
TOPOMAP_IMAGES_DIR = THIS_DIR / "../topomaps/images"

with open(ROBOT_CONFIG_PATH, "r") as f:
    ROBOT_CONF = yaml.safe_load(f)
MAX_V = ROBOT_CONF["max_v"]
MAX_W = ROBOT_CONF["max_w"]
RATE = ROBOT_CONF["frame_rate"]  # Hz

# Visualisation -------------------------------------------------------------
PIXELS_PER_M = 40.0  # px for 1 m (feel free to tune)
ORIGIN_Y_RATIO = 0.95  # where to anchor trajectories vertically

# ---------------------------------------------------------------------------


def _load_model(model_name: str, device: torch.device):
    with open(MODEL_CONFIG_PATH, "r") as f:
        model_paths = yaml.safe_load(f)

    mconf_path = model_paths[model_name]["config_path"]
    ckpt_path = model_paths[model_name]["ckpt_path"]
    with open(mconf_path, "r") as f:
        model_params = yaml.safe_load(f)

    if not os.path.exists(ckpt_path):
        raise FileNotFoundError(f"Model weights not found at {ckpt_path}")

    print(f"[INFO] Loading model from {ckpt_path}")
    model = load_model(ckpt_path, model_params, device).to(device).eval()
    return model, model_params


class NavigationNode(Node):
    """Sub‑goal navigation with topomap + trajectory visualisation."""

    def __init__(self, args: argparse.Namespace):
        super().__init__("navigation")
        self.args = args

        # Torch / model ------------------------------------------------------
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        self.get_logger().info(f"Using device: {self.device}")

        self.model, self.model_params = _load_model(args.model, self.device)
        if self.model_params["model_type"] != "nomad":
            raise NotImplementedError("Only model_type 'nomad' supported in ROS 2 port")

        self.context_size: int = self.model_params["context_size"]
        self.noise_scheduler = DDPMScheduler(
            num_train_timesteps=self.model_params["num_diffusion_iters"],
            beta_schedule="squaredcos_cap_v2",
            clip_sample=True,
            prediction_type="epsilon",
        )

        self.bridge = CvBridge()
        self.context_queue: Deque[np.ndarray] = deque(maxlen=self.context_size + 1)
        self.last_ctx_time = self.get_clock().now()
        self.ctx_dt = 0.25

        self.current_waypoint = np.zeros(2)
        self.obstacle_points = None

        self.top_view_size = (400, 400)
        self.proximity_threshold = 0.8
        self.top_view_resolution = self.top_view_size[0] / self.proximity_threshold
        self.top_view_sampling_step = 5

        if args.robot == "locobot":
            self.safety_margin = 0.0
        elif args.robot == "robomaster":
            self.safety_margin = -0.1
        elif args.robot == "turtlebot4":
            self.safety_margin = 0.17

        # self.safety_margin = 0.17

        # 로봇 타입에 따른 이미지 크기 설정
        if args.robot == "locobot":
            self.DIM = (320, 240)
        elif args.robot == "robomaster":
            self.DIM = (640, 480)
        elif args.robot == "turtlebot4":
            self.DIM = (320, 200)

        self._init_depth_model()

        # Topological map ----------------------------------------------------
        self.topomap: List[PILImage] = self._load_topomap(args.dir)
        self.goal_node = (
            (len(self.topomap) - 1) if args.goal_node == -1 else args.goal_node
        )
        self.closest_node = 0

        # ROS interfaces -----------------------------------------------------
        if args.robot == "locobot":
            image_topic = "/camera/image"  # 상수에서 가져옴
        elif args.robot == "robomaster":
            image_topic = "/camera/image_color"
        elif args.robot == "turtlebot4":
            image_topic = "/robot2/oakd/rgb/preview/image_raw"  # 적절한 토픽으로 변경
        else:
            raise ValueError(f"Unknown robot type: {args.robot}")

        self.create_subscription(Image, image_topic, self._image_cb, 1)
        self.waypoint_pub = self.create_publisher(Float32MultiArray, WAYPOINT_TOPIC, 1)
        self.sampled_actions_pub = self.create_publisher(
            Float32MultiArray, SAMPLED_ACTIONS_TOPIC, 1
        )
        self.goal_pub = self.create_publisher(Bool, "/topoplan/reached_goal", 1)
        self.viz_pub = self.create_publisher(Image, "navigation_viz", 1)
        self.subgoal_pub = self.create_publisher(Image, "navigation_subgoal", 1)
        self.goal_pub_img = self.create_publisher(Image, "navigation_goal", 1)
        self.create_timer(1.0 / RATE, self._timer_cb)
        self.get_logger().info("Navigation node initialised. Waiting for images…")

        # ------------------------------------------------------------------

    # Helper: topomap
    # ------------------------------------------------------------------

    def _load_topomap(self, subdir: str) -> List[PILImage.Image]:
        dpath = TOPOMAP_IMAGES_DIR / subdir
        if not dpath.exists():
            raise FileNotFoundError(f"Topomap directory {dpath} does not exist")
        img_files = sorted(os.listdir(dpath), key=lambda x: int(os.path.splitext(x)[0]))
        return [PILImage.open(dpath / f) for f in img_files]

    def _init_depth_model(self):
        if self.args.robot == "locobot":
            self.K = np.load("./UniDepth/assets/fisheye/fisheye_intrinsics.npy")
            self.D = np.load("./UniDepth/assets/fisheye/fisheye_distortion.npy")
            self.map1, self.map2 = cv2.fisheye.initUndistortRectifyMap(
                self.K, self.D, np.eye(3), self.K, self.DIM, cv2.CV_16SC2
            )
        elif self.args.robot == "robomaster":
            self.K = np.load("./UniDepth/assets/robomaster/intrinsics.npy")
        elif self.args.robot == "turtlebot4":
            self.K = np.load("./UniDepth/assets/turtlebot4/intrinsics.npy")
        else:
            raise ValueError(f"Unsupported robot type: {self.args.robot}")

        # self.intrinsics_torch = torch.from_numpy(self.K).unsqueeze(0).to(self.device)
        # self.camera = Pinhole(K=self.intrinsics_torch)
        self.depth_model = (
            UniDepthV2.from_pretrained("lpiccinelli/unidepth-v2-vits14")
            .to(self.device)
            .eval()
        )

    # ------------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------------

    def _image_cb(self, msg: Image):
        now = self.get_clock().now()
        if (now - self.last_ctx_time).nanoseconds < self.ctx_dt * 1e9:
            return
        self.context_queue.append(msg_to_pil(msg))
        self.last_ctx_time = now

        cv2_img = self.bridge.imgmsg_to_cv2(msg)

        if self.args.robot == "locobot":
            frame = cv2.resize(cv2_img, self.DIM)
            undistorted = cv2.remap(
                frame, self.map1, self.map2, interpolation=cv2.INTER_LINEAR
            )
            rgb = cv2.cvtColor(undistorted, cv2.COLOR_BGR2RGB)
        elif self.args.robot == "robomaster":
            frame = cv2_img.copy()
            rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        elif self.args.robot == "turtlebot4":
            frame = cv2_img.copy()
            rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            # pass

        rgb_torch = (
            torch.from_numpy(rgb).permute(2, 0, 1).unsqueeze(0).float().to(self.device)
        )

        self.intrinsics_torch = torch.from_numpy(self.K).unsqueeze(0).to(self.device)
        self.camera = Pinhole(K=self.intrinsics_torch)

        with torch.no_grad():
            outputs = self.depth_model.infer(rgb_torch, self.camera)
            points = outputs["points"].squeeze().cpu().numpy()

        X, Y, Z = points[0].flatten(), points[1].flatten(), points[2].flatten()
        mask = (Z > 0) & (Z <= self.proximity_threshold) & (Y >= -0.05)
        self._update_top_view_and_obstacles(X[mask], Y[mask], Z[mask])

    def _update_top_view_and_obstacles(self, X, Y, Z_0):
        Z = np.maximum(Z_0 - self.safety_margin, 1e-3)
        img_x = np.int32(self.top_view_size[0] // 2 + X * self.top_view_resolution)
        img_y = np.int32(self.top_view_size[1] - Z * self.top_view_resolution)

        valid = (
            (img_x >= 0)
            & (img_x < self.top_view_size[0])
            & (img_y >= 0)
            & (img_y < self.top_view_size[1])
        )
        # depth_vals, real_x, real_z = Z[valid], X[valid], Z[valid]
        img_x = img_x[valid]
        img_y = img_y[valid]
        depth_vals = Z[valid]
        real_x = X[valid]
        real_z = Z[valid]

        sampled_obstacles = []
        for x in range(0, self.top_view_size[0], self.top_view_sampling_step):
            col_idxs = np.where(img_x == x)[0]
            if len(col_idxs) > 0:
                closest_idx = col_idxs[np.argmin(depth_vals[col_idxs])]
                sampled_obstacles.append([real_x[closest_idx], real_z[closest_idx]])

        if sampled_obstacles:
            obs_array = np.array(sampled_obstacles)
            x_local = obs_array[:, 1]
            y_local = -obs_array[:, 0]
            self.obstacle_points = np.stack([x_local, y_local], axis=1)
        else:
            self.obstacle_points = None

    def compute_repulsive_force(
        self, point: np.ndarray, obstacles: np.ndarray, influence_range=1.0
    ) -> np.ndarray:
        rep_force = np.zeros(2)
        if obstacles is None:
            return rep_force
        for obs in obstacles:
            vec = point - obs
            dist = np.linalg.norm(vec)
            if dist < 1e-6 or dist > influence_range:
                continue
            rep_force += (1.0 / dist**3) * (vec / dist)
        return rep_force

    def apply_repulsive_forces_to_trajectories(
        self, trajectories: np.ndarray
    ) -> np.ndarray:
        if self.obstacle_points is None or len(self.obstacle_points) == 0:
            return trajectories * (MAX_V / RATE)

        updated_trajs = trajectories.copy()
        for i in range(updated_trajs.shape[0]):
            max_force = np.zeros(2)
            max_magnitude = 0.0

            for j in range(updated_trajs.shape[1]):
                pt = updated_trajs[i, j] * (MAX_V / RATE)
                rep_force = self.compute_repulsive_force(pt, self.obstacle_points)
                mag = np.linalg.norm(rep_force)
                if mag > max_magnitude:
                    max_magnitude = mag
                    max_force = rep_force

            angle = np.arctan2(max_force[1], max_force[0])
            angle = np.clip(angle, -np.pi / 4, np.pi / 4)
            rotation_matrix = np.array(
                [
                    [np.cos(angle), -np.sin(angle)],
                    [np.sin(angle), np.cos(angle)],
                ]
            )
            updated_trajs[i] = (rotation_matrix @ updated_trajs[i].T).T

        return updated_trajs * (MAX_V / RATE)

    def _angle_between(self, v1: np.ndarray, v2: np.ndarray) -> float:
        n1, n2 = np.linalg.norm(v1), np.linalg.norm(v2)
        if n1 < 1e-3 or n2 < 1e-3:
            return np.pi
        return np.arccos(np.clip(np.dot(v1, v2) / (n1 * n2), -1.0, 1.0))

    def _select_closest_traj_angle(
        self, trajs: np.ndarray, default_idx: int = 0
    ) -> int:
        if self.obstacle_points is None or len(self.obstacle_points) == 0:
            return default_idx

        prev_wp = (
            self.current_waypoint
        )  # if hasattr(self, "current_waypoint") else np.zeros(2)

        if np.linalg.norm(prev_wp) < 1e-3:
            return default_idx

        cand_wps = trajs[:, self.args.waypoint]
        angles = np.array([self._angle_between(prev_wp, wp) for wp in cand_wps])
        return int(np.argmin(angles))

    def _timer_cb(self):
        if len(self.context_queue) <= self.context_size:
            self.get_logger().info(
                f"Waiting for context images… {len(self.context_queue)}/{self.context_size}"
            )
            return

        # -----------------------------------------------------------------
        # 1. Compute closest node via distance prediction
        # -----------------------------------------------------------------
        start = max(self.closest_node - self.args.radius, 0)
        end = min(self.closest_node + self.args.radius + 1, self.goal_node)

        # Build batch of (obs, goal) tensors
        obs_images = transform_images(
            list(self.context_queue),
            self.model_params["image_size"],
            center_crop=False,
        ).to(self.device)
        obs_images = torch.split(obs_images, 3, dim=1)
        obs_images = torch.cat(obs_images, dim=1)  # merge context

        batch_goal_imgs = []
        for g_idx in range(start, end + 1):
            g_img = transform_images(
                self.topomap[g_idx], self.model_params["image_size"], center_crop=False
            )
            batch_goal_imgs.append(g_img)
        goal_tensor = torch.cat(batch_goal_imgs, dim=0).to(self.device)

        mask = torch.zeros(1, device=self.device, dtype=torch.long)
        with torch.no_grad():
            obsgoal_cond = self.model(
                "vision_encoder",
                obs_img=obs_images.repeat(len(goal_tensor), 1, 1, 1),
                goal_img=goal_tensor,
                input_goal_mask=mask.repeat(len(goal_tensor)),
            )
            dists = self.model("dist_pred_net", obsgoal_cond=obsgoal_cond)
            dists_np = to_numpy(dists.flatten())

        min_idx = int(np.argmin(dists_np))
        self.closest_node = start + min_idx
        sg_idx = min(
            min_idx + int(dists_np[min_idx] < self.args.close_threshold),
            len(goal_tensor) - 1,
        )
        obs_cond = obsgoal_cond[sg_idx].unsqueeze(0)
        sg_global_idx = start + sg_idx  # ← 새로 추가
        sg_pil = self.topomap[sg_global_idx]  # ← 새로 추가
        goal_pil = self.topomap[self.goal_node]  # ← 새로 추가

        # -----------------------------------------------------------------
        # 2. Sample trajectories towards sub‑goal (diffusion)
        # -----------------------------------------------------------------
        with torch.no_grad():
            if obs_cond.ndim == 2:
                obs_cond = obs_cond.repeat(self.args.num_samples, 1)
            else:
                obs_cond = obs_cond.repeat(self.args.num_samples, 1, 1)

            len_traj = self.model_params["len_traj_pred"]
            naction = torch.randn(
                (self.args.num_samples, len_traj, 2), device=self.device
            )
            self.noise_scheduler.set_timesteps(self.model_params["num_diffusion_iters"])
            for k in self.noise_scheduler.timesteps:
                noise_pred = self.model(
                    "noise_pred_net", sample=naction, timestep=k, global_cond=obs_cond
                )
                naction = self.noise_scheduler.step(noise_pred, k, naction).prev_sample

        traj_batch = to_numpy(get_action(naction))

        is_apf_applied = (
            self.obstacle_points is not None and len(self.obstacle_points) > 0
        )

        # Repulsive force 적용
        self.original_trajectories = traj_batch.copy()
        traj_batch = self.apply_repulsive_forces_to_trajectories(traj_batch)

        # 최종 trajectory 선택
        chosen_idx = self._select_closest_traj_angle(traj_batch, default_idx=0)
        chosen_waypoint = traj_batch[chosen_idx][self.args.waypoint]
        self.current_waypoint = chosen_waypoint

        # -----------------------------------------------------------------
        # 3. Publish ROS messages
        # -----------------------------------------------------------------
        self._publish_msgs(traj_batch, chosen_waypoint)
        self._publish_viz(traj_batch, is_apf_applied)
        self._publish_goal_images(sg_pil, goal_pil)  # ← 호출

    # ------------------------------------------------------------------
    # Publish helpers (추가)
    # ------------------------------------------------------------------
    def _publish_goal_images(self, sg_img: PILImage.Image, goal_img: PILImage.Image):
        """Publish current sub‑goal and final goal images as ROS sensor_msgs/Image."""
        for img, pub in [(sg_img, self.subgoal_pub), (goal_img, self.goal_pub_img)]:
            cv_img = cv2.cvtColor(np.array(img.convert("RGB")), cv2.COLOR_RGB2BGR)
            msg = self.bridge.cv2_to_imgmsg(cv_img, encoding="bgr8")
            msg.header.stamp = self.get_clock().now().to_msg()
            pub.publish(msg)

    # ------------------------------------------------------------------
    # Publish helpers
    # ------------------------------------------------------------------

    def _publish_msgs(self, traj_batch: np.ndarray, chosen: np.ndarray):
        # sampled actions
        actions_msg = Float32MultiArray()
        actions_msg.data = [0.0] + [float(x) for x in traj_batch.flatten()]
        self.sampled_actions_pub.publish(actions_msg)

        # chosen waypoint
        # chosen = traj_batch[0][self.args.waypoint]

        # if self.model_params.get("normalize", False):
        #     chosen *= MAX_V / RATE

        wp_msg = Float32MultiArray()
        wp_msg.data = [float(chosen[0]), float(chosen[1]), 0.0, 0.0]  # 4‑D compat
        self.waypoint_pub.publish(wp_msg)

        # goal status
        reached = bool(self.closest_node == self.goal_node)
        self.goal_pub.publish(Bool(data=reached))

    def _publish_viz(self, traj_batch: np.ndarray, is_apf_applied: bool = False):
        frame = np.array(self.context_queue[-1])
        h, w = frame.shape[:2]
        viz = frame.copy()

        cx = w // 2
        cy = int(h * ORIGIN_Y_RATIO)

        for i, traj in enumerate(traj_batch):
            pts = []
            acc_x = acc_y = 0.0
            for dx, dy in traj:
                acc_x += dx
                acc_y += dy
                px = int(cx - dy * PIXELS_PER_M)
                py = int(cy - acc_x * PIXELS_PER_M)
                pts.append((px, py))
            if len(pts) >= 2:
                # *** APF 적용 여부에 따라 색상 변경 ***
                if is_apf_applied:
                    color = (
                        (0, 0, 255) if i == 0 else (180, 0, 255)
                    )  # Blue for main, purple for others
                else:
                    color = (
                        (0, 255, 0) if i == 0 else (255, 200, 0)
                    )  # Original green and yellow

                cv2.polylines(viz, [np.array(pts, dtype=np.int32)], False, color, 1)

        img_msg = self.bridge.cv2_to_imgmsg(viz, encoding="rgb8")
        img_msg.header.stamp = self.get_clock().now().to_msg()
        self.viz_pub.publish(img_msg)


def cv2_to_pil(cv2_img: np.ndarray) -> PILImage.Image:
    rgb_img = cv2.cvtColor(cv2_img, cv2.COLOR_BGR2RGB)
    pil_img = PILImage.fromarray(rgb_img)
    pil_img.load()  # 이 줄을 추가해 lazy wrapper 제거
    return pil_img


# ---------------------------------------------------------------------------
# ENTRY POINT
# ---------------------------------------------------------------------------


def main():
    parser = argparse.ArgumentParser("Topological navigation (ROS 2)")
    parser.add_argument("--model", "-m", default="nomad")
    parser.add_argument(
        "--dir", "-d", default="topomap", help="sub‑directory under ../topomaps/images/"
    )
    parser.add_argument(
        "--goal-node", "-g", type=int, default=-1, help="Goal node index (-1 = last)"
    )
    parser.add_argument("--waypoint", "-w", type=int, default=2)
    parser.add_argument("--close-threshold", "-t", type=float, default=3.0)
    parser.add_argument("--radius", "-r", type=int, default=4)
    parser.add_argument("--num-samples", "-n", type=int, default=8)
    parser.add_argument(
        "--robot",
        type=str,
        default="locobot",
        choices=["locobot", "robomaster", "turtlebot4"],
        help="Robot type (locobot, robomaster, turtlebot4)",
    )
    args = parser.parse_args()

    rclpy.init()
    node = NavigationNode(args)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
