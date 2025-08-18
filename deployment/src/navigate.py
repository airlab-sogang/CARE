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

from unidepth.models import UniDepthV2
from unidepth.utils.camera import Pinhole

THIS_DIR = Path(__file__).resolve().parent
ROBOT_CONFIG_PATH = THIS_DIR / "../config/robot.yaml"
MODEL_CONFIG_PATH = THIS_DIR / "../config/models.yaml"
TOPOMAP_IMAGES_DIR = THIS_DIR / "../topomaps/images"

with open(ROBOT_CONFIG_PATH, "r") as f:
    ROBOT_CONF = yaml.safe_load(f)
MAX_V = ROBOT_CONF["max_v"]
MAX_W = ROBOT_CONF["max_w"]
RATE = ROBOT_CONF["frame_rate"]  # Hz

def _load_model(model_name: str, device: torch.device):
    with open(MODEL_CONFIG_PATH, "r") as f:
        model_paths = yaml.safe_load(f)

    mconf_path = model_paths[model_name]["config_path"]
    ckpt_path = model_paths[model_name]["ckpt_path"]
    with open(mconf_path, "r") as f:
        model_params = yaml.safe_load(f)

    if not os.path.exists(ckpt_path):
        raise FileNotFoundError(f"Model weights not found at {ckpt_path}")

    print(f"Loading model from {ckpt_path}")
    model = load_model(ckpt_path, model_params, device).to(device).eval()
    return model, model_params


class NavigationNode(Node):
    """Sub‑goal navigation with topomap + trajectory visualisation."""

    def __init__(self, args: argparse.Namespace):
        super().__init__("navigation")
        self.args = args

        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        self.get_logger().info(f"Using device: {self.device}")

        self.model, self.model_params = _load_model(args.model, self.device)

        self.get_logger().info(f"Using model type: {self.model_params['model_type']}")

        self.context_size: int = self.model_params["context_size"]

        if self.model_params["model_type"] == "nomad":
            self.noise_scheduler = DDPMScheduler(
                num_train_timesteps=self.model_params["num_diffusion_iters"],
                beta_schedule="squaredcos_cap_v2",
                clip_sample=True,
                prediction_type="epsilon",
            )

        self.bridge = CvBridge()
        self.context_queue = deque(maxlen=self.context_size + 1)
        self.last_ctx_time = self.get_clock().now()
        self.ctx_dt = 0.25

        self.current_waypoint = np.zeros(2)
        self.obstacle_points = None

        self.top_view_size = (400, 400)
        self.proximity_threshold = 0.8
        self.top_view_resolution = self.top_view_size[0] / self.proximity_threshold
        self.top_view_sampling_step = 5
        self.safety_margin = 0.17
        self.DIM = (640, 480)

        # Topological map ----------------------------------------------------
        self.topomap: List[PILImage] = self._load_topomap(args.dir)
        self.goal_node = (
            (len(self.topomap) - 1) if args.goal_node == -1 else args.goal_node
        )
        self.closest_node = 0

        if args.robot == "locobot":
            image_topic = "/robot1/camera/image"
            waypoint_topic = "/robot1/waypoint"
            sampled_actions_topic = "/robot1/sampled_actions"
        elif args.robot == "robomaster":
            image_topic = "/camera/image_color"
            waypoint_topic = "/robot3/waypoint"
            sampled_actions_topic = "/robot3/sampled_actions"
        elif args.robot == "turtlebot4":
            image_topic = "/robot2/oakd/rgb/preview/image_raw"
            waypoint_topic = "/robot2/waypoint"
            sampled_actions_topic = "/robot2/sampled_actions"
        else:
            raise ValueError(f"Unknown robot type: {args.robot}")

        self.create_subscription(Image, image_topic, self._image_cb, 1)
        self.waypoint_pub = self.create_publisher(Float32MultiArray, waypoint_topic, 1)
        self.sampled_actions_pub = self.create_publisher(
            Float32MultiArray, sampled_actions_topic, 1
        )
        self.goal_pub = self.create_publisher(Bool, "/topoplan/reached_goal", 1)
        self.viz_pub = self.create_publisher(Image, "navigation_viz", 1)
        self.subgoal_pub = self.create_publisher(Image, "navigation_subgoal", 1)
        self.goal_pub_img = self.create_publisher(Image, "navigation_goal", 1)
        self.create_timer(1.0 / RATE, self._timer_cb)
        self.get_logger().info("Navigation node initialised. Waiting for images…")

        self.get_logger().info("=" * 60)
        self.get_logger().info("NAVIGATION NODE PARAMETERS")
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
        self.get_logger().info(f"  - Camera matrix (K):\n{self.K}")
        self.get_logger().info(f"  - Distortion coefficients (D):\n{self.D}")
        self.get_logger().info("-" * 60)
        self.get_logger().info("MODEL CONFIGURATION:")
        self.get_logger().info(f"  - Model name: {self.args.model}")
        self.get_logger().info(f"  - Model type: {self.model_params['model_type']}")
        self.get_logger().info(f"  - Device: {self.device}")
        self.get_logger().info(f"  - Context size: {self.context_size}")
        self.get_logger().info(f"  - Context update interval: {self.ctx_dt} seconds")
        if self.model_params["model_type"] == "nomad":
            self.get_logger().info(
                f"  - Trajectory length: {self.model_params['len_traj_pred']}"
            )
            self.get_logger().info(
                f"  - Diffusion iterations: {self.model_params['num_diffusion_iters']}"
            )
        self.get_logger().info(f"  - Image size: {self.model_params['image_size']}")
        self.get_logger().info(
            f"  - Normalize: {self.model_params.get('normalize', False)}"
        )
        self.get_logger().info("-" * 60)
        self.get_logger().info("DEPTH MODEL CONFIGURATION:")
        self.get_logger().info(f"  - UniDepth model: UniDepthV2")
        self.get_logger().info(
            f"  - Pretrained weights: lpiccinelli/unidepth-v2-vits14"
        )
        self.get_logger().info("-" * 60)
        self.get_logger().info("TOPOLOGICAL MAP CONFIGURATION:")
        self.get_logger().info(f"  - Topomap directory: {self.args.dir}")
        self.get_logger().info(f"  - Number of nodes: {len(self.topomap)}")
        self.get_logger().info(f"  - Goal node: {self.goal_node}")
        self.get_logger().info(f"  - Search radius: {self.args.radius}")
        self.get_logger().info(f"  - Close threshold: {self.args.close_threshold}")
        self.get_logger().info("-" * 60)
        self.get_logger().info("OBSTACLE AVOIDANCE CONFIGURATION:")
        self.get_logger().info(f"  - Top view size: {self.top_view_size}")
        self.get_logger().info(
            f"  - Top view resolution: {self.top_view_resolution:.2f} pixels/m"
        )
        self.get_logger().info(
            f"  - Top view sampling step: {self.top_view_sampling_step} pixels"
        )
        self.get_logger().info("-" * 60)
        self.get_logger().info("ROS TOPICS:")
        self.get_logger().info(f"  - Subscribing to: {image_topic}")
        self.get_logger().info(f"  - Publishing waypoints to: {waypoint_topic}")
        self.get_logger().info(
            f"  - Publishing sampled actions to: {sampled_actions_topic}"
        )
        self.get_logger().info(
            f"  - Publishing navigation visualization to: /navigation_viz"
        )
        self.get_logger().info(f"  - Publishing subgoal image to: /navigation_subgoal")
        self.get_logger().info(f"  - Publishing goal image to: /navigation_goal")
        self.get_logger().info(
            f"  - Publishing goal reached status to: /topoplan/reached_goal"
        )
        self.get_logger().info("-" * 60)
        self.get_logger().info("EXECUTION PARAMETERS:")
        self.get_logger().info(f"  - Waypoint index: {self.args.waypoint}")
        self.get_logger().info(f"  - Number of samples: {self.args.num_samples}")
        self.get_logger().info("-" * 60)
        self.get_logger().info("VISUALIZATION PARAMETERS:")
        self.get_logger().info(f"  - Pixels per meter: 3.0")
        self.get_logger().info(f"  - Lateral scale: 1.0")
        self.get_logger().info(f"  - Horizontal scale: 4.0")
        self.get_logger().info(f"  - Robot symbol length: 10 pixels")
        self.get_logger().info("=" * 60)

    # Helper: topomap
    # ------------------------------------------------------------------

    def _load_topomap(self, subdir: str) -> List[PILImage.Image]:
        dpath = TOPOMAP_IMAGES_DIR / subdir
        if not dpath.exists():
            raise FileNotFoundError(f"Topomap directory {dpath} does not exist")
        img_files = sorted(os.listdir(dpath), key=lambda x: int(os.path.splitext(x)[0]))
        return [PILImage.open(dpath / f) for f in img_files]

    def _image_cb(self, msg: Image):
        now = self.get_clock().now()
        if (now - self.last_ctx_time).nanoseconds < self.ctx_dt * 1e9:
            return  # 아직 0.25 s 안 지났으면 무시
        self.context_queue.append(msg_to_pil(msg))
        self.last_ctx_time = now
        self.get_logger().info(
            f"Image added to context queue ({len(self.context_queue)})"
        )

    def _timer_cb(self):
        if len(self.context_queue) <= self.context_size:
            return

        if self.model_params["model_type"] == "nomad":
            self._timer_cb_nomad()
        else:
            self._timer_cb_other()

        if self.closest_node == self.goal_node:
            self.get_logger().info("Reached goal! Stopping...")

    def _timer_cb_nomad(self):
        """NOMAD 모델을 위한 타이머 콜백 처리"""
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
        sg_global_idx = start + sg_idx
        sg_pil = self.topomap[sg_global_idx]
        goal_pil = self.topomap[self.goal_node]

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

        self._publish_msgs(traj_batch)
        self._publish_viz_image(traj_batch)
        self._publish_goal_images(sg_pil, goal_pil)

    def _timer_cb_other(self):
        start = max(self.closest_node - self.args.radius, 0)
        end = min(self.closest_node + self.args.radius + 1, self.goal_node)

        batch_obs_imgs = []
        batch_goal_data = []

        for i, sg_img in enumerate(self.topomap[start : end + 1]):
            transf_obs_img = transform_images(
                list(self.context_queue), self.model_params["image_size"]
            )
            goal_data = transform_images(sg_img, self.model_params["image_size"])
            batch_obs_imgs.append(transf_obs_img)
            batch_goal_data.append(goal_data)

        batch_obs_imgs = torch.cat(batch_obs_imgs, dim=0).to(self.device)
        batch_goal_data = torch.cat(batch_goal_data, dim=0).to(self.device)

        with torch.no_grad():
            distances, waypoints = self.model(batch_obs_imgs, batch_goal_data)
            distances_np = to_numpy(distances)
            waypoints_np = to_numpy(waypoints)

        min_dist_idx = np.argmin(distances_np)

        chosen_waypoint = np.zeros(4)

        if distances_np[min_dist_idx] > self.args.close_threshold:
            chosen_waypoint[:2] = waypoints_np[min_dist_idx][self.args.waypoint][:2]
            selected_waypoints = waypoints_np[
                min_dist_idx
            ]
            self.closest_node = start + min_dist_idx
        else:
            next_idx = min(min_dist_idx + 1, len(waypoints_np) - 1)
            chosen_waypoint[:2] = waypoints_np[next_idx][self.args.waypoint][:2]
            selected_waypoints = waypoints_np[next_idx]
            self.closest_node = min(start + min_dist_idx + 1, self.goal_node)

        if self.model_params.get("normalize", False):
            chosen_waypoint[:2] *= MAX_V / RATE

        waypoint_msg = Float32MultiArray()
        waypoint_msg.data = chosen_waypoint.tolist()
        self.waypoint_pub.publish(waypoint_msg)

        reached_goal = bool(self.closest_node == self.goal_node)
        self.goal_pub.publish(Bool(data=reached_goal))

        sg_global_idx = min(
            start
            + min_dist_idx
            + int(distances_np[min_dist_idx] <= self.args.close_threshold),
            self.goal_node,
        )
        sg_pil = self.topomap[sg_global_idx]
        goal_pil = self.topomap[self.goal_node]

        if selected_waypoints is not None:
            traj_vis = np.zeros((1, len(selected_waypoints), 2))
            for i in range(len(selected_waypoints)):
                traj_vis[0, i] = selected_waypoints[i][:2]

            # if self.model_params.get("normalize", False):
            #     traj_vis *= MAX_V / RATE

            self._publish_viz_image(traj_vis)

        self._publish_goal_images(sg_pil, goal_pil)

    def _publish_goal_images(self, sg_img: PILImage.Image, goal_img: PILImage.Image):
        """Publish current sub‑goal and final goal images as ROS sensor_msgs/Image."""
        for img, pub in [(sg_img, self.subgoal_pub), (goal_img, self.goal_pub_img)]:
            cv_img = cv2.cvtColor(np.array(img.convert("RGB")), cv2.COLOR_RGB2BGR)
            msg = self.bridge.cv2_to_imgmsg(cv_img, encoding="bgr8")
            msg.header.stamp = self.get_clock().now().to_msg()
            pub.publish(msg)


    def _publish_msgs(self, traj_batch: np.ndarray):
        # sampled actions
        actions_msg = Float32MultiArray()
        actions_msg.data = [0.0] + [float(x) for x in traj_batch.flatten()]
        self.sampled_actions_pub.publish(actions_msg)

        # chosen waypoint
        chosen = traj_batch[0][self.args.waypoint]
        if self.model_params.get("normalize", False):
            chosen *= MAX_V / RATE
        wp_msg = Float32MultiArray()
        wp_msg.data = [float(chosen[0]), float(chosen[1]), 0.0, 0.0]  # 4‑D compat
        self.waypoint_pub.publish(wp_msg)

        # goal status
        reached = bool(self.closest_node == self.goal_node)
        self.goal_pub.publish(Bool(data=reached))

    def _publish_viz_image(self, traj_batch: np.ndarray):
        frame = np.array(self.context_queue[-1])  # latest RGB frame
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
                color = (
                    (0, 255, 0) if i == 0 else (255, 200, 0)
                )
                cv2.polylines(viz, [np.array(pts, dtype=np.int32)], False, color, 2)

        img_msg = self.bridge.cv2_to_imgmsg(viz, encoding="rgb8")
        img_msg.header.stamp = self.get_clock().now().to_msg()
        self.viz_pub.publish(img_msg)


def main():
    parser = argparse.ArgumentParser("Topological navigation (ROS 2)")
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
