from __future__ import annotations

import argparse
import os
from collections import deque
from pathlib import Path
from typing import Deque

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray
import torch
import yaml
from diffusers.schedulers.scheduling_ddpm import DDPMScheduler

from utils import msg_to_pil, to_numpy, transform_images, load_model
from vint_train.training.train_utils import get_action
from topic_names import IMAGE_TOPIC, WAYPOINT_TOPIC, SAMPLED_ACTIONS_TOPIC

from unidepth.utils.camera import Pinhole
from unidepth.models import UniDepthV2

THIS_DIR = Path.cwd()
ROBOT_CONFIG_PATH = THIS_DIR / "deployment/config/robot.yaml"
MODEL_CONFIG_PATH = THIS_DIR / "deployment/config/models.yaml"

with open(ROBOT_CONFIG_PATH, "r") as f:
    ROBOT_CONF = yaml.safe_load(f)
MAX_V = ROBOT_CONF["max_v"]
MAX_W = ROBOT_CONF["max_w"]
RATE = ROBOT_CONF["frame_rate"]  # Hz


def _load_model(model_name: str, device: torch.device):
    with open(MODEL_CONFIG_PATH, "r") as f:
        model_paths = yaml.safe_load(f)

    model_config_path = model_paths[model_name]["config_path"]
    with open(model_config_path, "r") as f:
        model_params = yaml.safe_load(f)

    ckpt_path = model_paths[model_name]["ckpt_path"]
    if not os.path.exists(ckpt_path):
        raise FileNotFoundError(f"Model weights not found at {ckpt_path}")

    print(f"Loading model from {ckpt_path}")
    model = load_model(ckpt_path, model_params, device)
    return model.to(device).eval(), model_params


class ExplorationNode(Node):

    def __init__(self, args: argparse.Namespace):
        super().__init__("exploration")
        self.args = args

        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        self.get_logger().info(f"Using device: {self.device}")

        self.model, self.model_params = _load_model(args.model, self.device)
        self.context_size: int = self.model_params["context_size"]
        self.last_ctx_time = self.get_clock().now()
        self.ctx_dt = 0.25

        self.noise_scheduler = DDPMScheduler(
            num_train_timesteps=self.model_params["num_diffusion_iters"],
            beta_schedule="squaredcos_cap_v2",
            clip_sample=True,
            prediction_type="epsilon",
        )

        self.context_queue: Deque[np.ndarray] = deque(maxlen=self.context_size + 1)
        self.bridge = CvBridge()

        if args.robot == "locobot":
            image_topic = "/robot1/camera/image"
            waypoint_topic = "/robot1/waypoint"
            sampled_actions_topic = "/robot1/sampled_actions"
            trajectory_viz_topic = "/robot1/trajectory_viz"
        elif args.robot == "locobot2":
            image_topic = "/robot3/camera/image"
            waypoint_topic = "/robot3/waypoint"
            sampled_actions_topic = "/robot3/sampled_actions"
            trajectory_viz_topic = "/robot3/trajectory_viz"
        elif args.robot == "robomaster":
            image_topic = "/camera/image_color"
            waypoint_topic = "/robot3/waypoint"
            sampled_actions_topic = "/robot3/sampled_actions"
            trajectory_viz_topic = "/robot3/trajectory_viz"
        elif args.robot == "turtlebot4":
            image_topic = "/robot2/oakd/rgb/preview/image_raw"
            waypoint_topic = "/robot2/waypoint"
            sampled_actions_topic = "/robot2/sampled_actions"
            trajectory_viz_topic = "/robot2/trajectory_viz"
        else:
            raise ValueError(f"Unknown robot type: {args.robot}")

        self.create_subscription(Image, image_topic, self._image_cb, 1)
        self.waypoint_pub = self.create_publisher(Float32MultiArray, waypoint_topic, 1)
        self.sampled_actions_pub = self.create_publisher(
            Float32MultiArray, sampled_actions_topic, 1
        )
        self.viz_pub = self.create_publisher(Image, trajectory_viz_topic, 1)

        self.create_timer(1.0 / RATE, self._timer_cb)
        self.get_logger().info(
            f"Exploration node initialised for {args.robot}. Waiting for images…"
        )

        self.current_waypoint = np.zeros(2)
        self.obstacle_points = None
        self.top_view_size = (400, 400)

        if args.robot == "locobot" or args.robot == "locobot2":
            self.safety_margin = 0.05
            self.proximity_threshold = 1.0
        elif args.robot == "robomaster":
            self.safety_margin = -0.1
            self.proximity_threshold = 1.3
        elif args.robot == "turtlebot4":
            self.safety_margin = 0.2
            self.proximity_threshold = 1.4
        else:
            raise ValueError(f"Unsupported robot type: {self.args.robot}")

        self.top_view_resolution = self.top_view_size[0] / self.proximity_threshold
        self.top_view_sampling_step = 5

        if args.robot == "locobot" or args.robot == "locobot2":
            self.DIM = (320, 240)
        elif args.robot == "robomaster":
            self.DIM = (640, 360)
        elif args.robot == "turtlebot4":
            self.DIM = (320, 200)

        intrinsics_path = self._get_intrinsics_path_from_config()
        self._init_depth_model(intrinsics_path)

        # 시작하기 전에 중요한 파라미터들 출력
        self.get_logger().info("=" * 60)
        self.get_logger().info("EXPLORATION NODE PARAMETERS (with APF)")
        self.get_logger().info("=" * 60)
        self.get_logger().info(f"Robot type: {self.args.robot}")
        self.get_logger().info(f"Image topic: {image_topic}")
        self.get_logger().info("-" * 60)
        self.get_logger().info("ROBOT CONFIGURATION:")
        self.get_logger().info(f"  - Max linear velocity: {MAX_V} m/s")
        self.get_logger().info(f"  - Max angular velocity: {MAX_W} rad/s")
        self.get_logger().info(f"  - Frame rate: {RATE} Hz")
        self.get_logger().info(f"  - Safety margin: {self.safety_margin} m")
        self.get_logger().info(f"  - Proximity threshold: {self.proximity_threshold} m")
        self.get_logger().info("-" * 60)
        self.get_logger().info("CAMERA CONFIGURATION:")
        self.get_logger().info(f"  - Image dimensions: {self.DIM}")
        self.get_logger().info(f"  - Intrinsics path: {intrinsics_path}")
        self.get_logger().info(f"  - Camera matrix (K):\n{self.K}")
        self.get_logger().info("-" * 60)
        self.get_logger().info("MODEL CONFIGURATION:")
        self.get_logger().info(f"  - Model name: {self.args.model}")
        self.get_logger().info(f"  - Device: {self.device}")
        self.get_logger().info(f"  - Context size: {self.context_size}")
        self.get_logger().info(f"  - Context update interval: {self.ctx_dt} seconds")
        self.get_logger().info(f"  - Trajectory length: {self.model_params['len_traj_pred']}")
        self.get_logger().info(f"  - Diffusion iterations: {self.model_params['num_diffusion_iters']}")
        self.get_logger().info(f"  - Image size: {self.model_params['image_size']}")
        self.get_logger().info("-" * 60)
        self.get_logger().info("DEPTH MODEL CONFIGURATION:")
        self.get_logger().info(f"  - UniDepth model: UniDepthV2")
        self.get_logger().info(f"  - Pretrained weights: lpiccinelli/unidepth-v2-vits14")
        self.get_logger().info("-" * 60)
        self.get_logger().info("OBSTACLE AVOIDANCE CONFIGURATION:")
        self.get_logger().info(f"  - Top view size: {self.top_view_size}")
        self.get_logger().info(f"  - Top view resolution: {self.top_view_resolution:.2f} pixels/m")
        self.get_logger().info(f"  - Top view sampling step: {self.top_view_sampling_step} pixels")
        self.get_logger().info("-" * 60)
        self.get_logger().info("ROS TOPICS:")
        self.get_logger().info(f"  - Subscribing to: {image_topic}")
        self.get_logger().info(f"  - Publishing waypoints to: {WAYPOINT_TOPIC}")
        self.get_logger().info(f"  - Publishing sampled actions to: {SAMPLED_ACTIONS_TOPIC}")
        self.get_logger().info(f"  - Publishing visualization to: /trajectory_viz")
        self.get_logger().info("-" * 60)
        self.get_logger().info("EXECUTION PARAMETERS:")
        self.get_logger().info(f"  - Waypoint index: {self.args.waypoint}")
        self.get_logger().info(f"  - Number of samples: {self.args.num_samples}")
        self.get_logger().info("-" * 60)
        self.get_logger().info("VISUALIZATION PARAMETERS:")
        self.get_logger().info(f"  - Pixels per meter: 3.0")
        self.get_logger().info(f"  - Lateral scale: 1.0")
        self.get_logger().info(f"  - Robot symbol length: 10 pixels")
        self.get_logger().info("=" * 60)

    def _get_intrinsics_path_from_config(self) -> str:
        if "intrinsics_path" in ROBOT_CONF:
            intrinsics_path = ROBOT_CONF["intrinsics_path"]
        else:
            if self.args.robot == "locobot":
                intrinsics_path = ROBOT_CONF.get("locobot_intrinsics_path",
                                                 "intrinsic/locobot/intrinsics.npy")
            elif self.args.robot == "robomaster":
                intrinsics_path = ROBOT_CONF.get("robomaster_intrinsics_path",
                                                 "intrinsic/robomaster/intrinsics.npy")
            elif self.args.robot == "turtlebot4":
                intrinsics_path = ROBOT_CONF.get("turtlebot4_intrinsics_path",
                                                 "intrinsic/turtlebot4/intrinsics.npy")
            else:
                raise ValueError(f"No intrinsics path configured for robot type: {self.args.robot}")

        if not os.path.isabs(intrinsics_path):
            intrinsics_path = str(THIS_DIR / intrinsics_path)

        return intrinsics_path

    def _init_depth_model(self, intrinsics_path: str):
        if not intrinsics_path:
            raise ValueError("Intrinsics path is not provided.")
        if not os.path.exists(intrinsics_path):
            raise FileNotFoundError(f"Intrinsics file not found: {intrinsics_path}")

        self.K = np.load(intrinsics_path)
        self.get_logger().info(f"Loaded camera intrinsics from: {intrinsics_path}")

        self.depth_model = UniDepthV2.from_pretrained("lpiccinelli/unidepth-v2-vits14").to(self.device)
        self.depth_model.eval()

    def _image_cb(self, msg: Image):
        now = self.get_clock().now()
        if (now - self.last_ctx_time).nanoseconds < self.ctx_dt * 1e9:
            return
        self.context_queue.append(msg_to_pil(msg))
        self.last_ctx_time = now
        self.get_logger().info(
            f"Image added to context queue ({len(self.context_queue)})"
        )

        cv2_img = self.bridge.imgmsg_to_cv2(msg)

        if self.args.robot == "locobot" or self.args.robot == "locobot2":
            frame = cv2.resize(cv2_img, self.DIM)
            # undistorted = cv2.remap(
            #     frame, self.map1, self.map2, interpolation=cv2.INTER_LINEAR
            # )
            rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        elif self.args.robot == "robomaster":
            frame = cv2_img.copy()
            rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        elif self.args.robot == "turtlebot4":
            frame = cv2_img.copy()
            # frame = cv2.resize(cv2_img, self.DIM)
            rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

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
            return trajectories

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
        return updated_trajs

    def _timer_cb(self):
        if len(self.context_queue) <= self.context_size:
            return

        obs_imgs = transform_images(
            list(self.context_queue), self.model_params["image_size"], center_crop=False
        ).to(self.device)
        fake_goal = torch.randn(
            (1, 3, *self.model_params["image_size"]), device=self.device
        )
        mask = torch.ones(1, device=self.device, dtype=torch.long)

        with torch.no_grad():
            obs_cond = self.model(
                "vision_encoder",
                obs_img=obs_imgs,
                goal_img=fake_goal,
                input_goal_mask=mask,
            )
            rep_fn = (
                (lambda x: x.repeat(self.args.num_samples, 1))
                if obs_cond.ndim == 2
                else (lambda x: x.repeat(self.args.num_samples, 1, 1))
            )
            obs_cond = rep_fn(obs_cond)

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

        traj_batch = self.apply_repulsive_forces_to_trajectories(traj_batch)
        self._publish_action_msgs(traj_batch)
        self._publish_viz_image(traj_batch, is_apf_applied)


    def _publish_action_msgs(self, traj_batch: np.ndarray):
        sampled_actions_msg = Float32MultiArray()
        sampled_actions_msg.data = [0.0] + [float(x) for x in traj_batch.flatten()]
        self.sampled_actions_pub.publish(sampled_actions_msg)

        chosen = traj_batch[0][self.args.waypoint]
        waypoint_msg = Float32MultiArray()
        waypoint_msg.data = [float(chosen[0]), float(chosen[1])]
        self.waypoint_pub.publish(waypoint_msg)

    def _publish_viz_image(self, traj_batch: np.ndarray, is_apf_applied: bool = False):
        frame = np.array(self.context_queue[-1])
        img_h, img_w = frame.shape[:2]
        viz = frame.copy()

        cx = img_w // 2
        cy = int(img_h * 0.95)

        pixels_per_m = 3.0
        lateral_scale = 1.0
        robot_symbol_length = 10

        cv2.line(
            viz,
            (cx - robot_symbol_length, cy),
            (cx + robot_symbol_length, cy),
            (255, 0, 0),
            2,
        )
        cv2.line(
            viz,
            (cx, cy - robot_symbol_length),
            (cx, cy + robot_symbol_length),
            (255, 0, 0),
            2,
        )

        # Draw each trajectory
        for i, traj in enumerate(traj_batch):
            pts = []
            pts.append((cx, cy))

            acc_x, acc_y = 0.0, 0.0
            for dx, dy in traj:
                acc_x += dx
                acc_y += dy
                px = int(cx - acc_y * pixels_per_m * lateral_scale)
                py = int(cy - acc_x * pixels_per_m)
                pts.append((px, py))

            if len(pts) >= 2:
                if is_apf_applied:
                    color = (
                        (0, 0, 255) if i == 0 else (180, 0, 255)
                    )
                else:
                    color = (
                        (0, 255, 0) if i == 0 else (255, 200, 0)
                    )
                cv2.polylines(viz, [np.array(pts, dtype=np.int32)], False, color, 2)

        img_msg = self.bridge.cv2_to_imgmsg(viz, encoding="rgb8")
        img_msg.header.stamp = self.get_clock().now().to_msg()
        self.viz_pub.publish(img_msg)


# ---------------------------------------------------------------------------
# ENTRY POINT
# ---------------------------------------------------------------------------


def main():
    parser = argparse.ArgumentParser("GNM‑Diffusion exploration (ROS 2)")
    parser.add_argument("--model", "-m", default="nomad")
    parser.add_argument("--waypoint", "-w", type=int, default=2)
    parser.add_argument("--num-samples", "-n", type=int, default=8)
    parser.add_argument(
        "--robot",
        type=str,
        default="locobot",
        choices=["locobot", "locobot2", "robomaster", "turtlebot4"],
        help="Robot type (locobot, robomaster, turtlebot4)",
    )
    args = parser.parse_args()

    rclpy.init()
    node = ExplorationNode(args)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()