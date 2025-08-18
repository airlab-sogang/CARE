from __future__ import annotations

import argparse
import os
from collections import deque
from pathlib import Path
from typing import Deque

import cv2
import numpy as np
import rclpy
import torch
import yaml
from cv_bridge import CvBridge
from diffusers.schedulers.scheduling_ddpm import DDPMScheduler
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray

from utils import msg_to_pil, to_numpy, transform_images, load_model
from vint_train.training.train_utils import get_action

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
    """ROS 2 node: image‑conditioned waypoint sampling + visualisation."""

    def __init__(self, args: argparse.Namespace):
        super().__init__("exploration")
        self.args = args

        # Torch / model ------------------------------------------------------
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

        # State & ROS‑interfaces --------------------------------------------
        self.context_queue: Deque[np.ndarray] = deque(maxlen=self.context_size + 1)
        self.bridge = CvBridge()

        if args.robot == "locobot":
            image_topic = "/robot1/camera/image"
            waypoint_topic = "/robot1/waypoint"
            sampled_actions_topic = "/robot1/sampled_actions"
            trajectory_viz_topic = "/robot1/trajectory_viz"
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

        self.get_logger().info("=" * 60)
        self.get_logger().info("EXPLORATION NODE PARAMETERS")
        self.get_logger().info("=" * 60)
        self.get_logger().info(f"Robot type: {args.robot}")
        self.get_logger().info(f"Image topic: {image_topic}")
        self.get_logger().info("-" * 60)
        self.get_logger().info("ROBOT CONFIGURATION:")
        self.get_logger().info(f"  - Max linear velocity: {MAX_V} m/s")
        self.get_logger().info(f"  - Max angular velocity: {MAX_W} rad/s")
        self.get_logger().info(f"  - Frame rate: {RATE} Hz")
        self.get_logger().info("-" * 60)
        self.get_logger().info("MODEL CONFIGURATION:")
        self.get_logger().info(f"  - Model name: {args.model}")
        self.get_logger().info(f"  - Device: {self.device}")
        self.get_logger().info(f"  - Context size: {self.context_size}")
        self.get_logger().info(f"  - Context update interval: {self.ctx_dt} seconds")
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
        self.get_logger().info("ROS TOPICS:")
        self.get_logger().info(f"  - Subscribing to: {image_topic}")
        self.get_logger().info(f"  - Publishing waypoints to: {waypoint_topic}")
        self.get_logger().info(
            f"  - Publishing sampled actions to: {sampled_actions_topic}"
        )
        self.get_logger().info(f"  - Publishing visualization to: /trajectory_viz")
        self.get_logger().info("-" * 60)
        self.get_logger().info("EXECUTION PARAMETERS:")
        self.get_logger().info(f"  - Waypoint index: {args.waypoint}")
        self.get_logger().info(f"  - Number of samples: {args.num_samples}")
        self.get_logger().info("-" * 60)
        self.get_logger().info("VISUALIZATION PARAMETERS:")
        self.get_logger().info(f"  - Pixels per meter: 3.0")
        self.get_logger().info(f"  - Lateral scale: 1.0")
        self.get_logger().info(f"  - Robot symbol length: 10 pixels")
        self.get_logger().info("=" * 60)
        self.get_logger().info(
            f"Exploration node initialized for {args.robot}. Waiting for {image_topic}"
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
        self.get_logger().info(f"Image added to context queue ({len(self.context_queue)})")

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

        naction_np = to_numpy(get_action(naction))
        self._publish_action_msgs(naction_np)
        self._publish_viz_image(naction_np)

    def _publish_action_msgs(self, traj_batch: np.ndarray):
        sampled_actions_msg = Float32MultiArray()
        sampled_actions_msg.data = [0.0] + [float(x) for x in traj_batch.flatten()]
        self.sampled_actions_pub.publish(sampled_actions_msg)

        chosen = traj_batch[0][self.args.waypoint]
        if self.model_params.get("normalize", False):
            chosen *= MAX_V / RATE
        waypoint_msg = Float32MultiArray()
        waypoint_msg.data = [float(chosen[0]), float(chosen[1])]
        self.waypoint_pub.publish(waypoint_msg)

    def _publish_viz_image(self, traj_batch: np.ndarray):
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
    parser = argparse.ArgumentParser("GNM‑Diffusion exploration (ROS 2)")
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
