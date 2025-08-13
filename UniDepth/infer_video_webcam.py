import argparse
import cv2
import numpy as np
import torch
import os
import time
from PIL import Image
import matplotlib.pyplot as plt

from unidepth.models import UniDepthV2, UniDepthV1, UniDepthV2old
from unidepth.utils import colorize
from unidepth.utils.camera import Pinhole


def infer_frame(model, frame, intrinsics, device):
    # Convert frame to tensor and move to device
    rgb_torch = torch.from_numpy(frame).permute(2, 0, 1).unsqueeze(0).float().to(device)
    intrinsics_torch = torch.from_numpy(intrinsics).unsqueeze(0).to(device)
    camera = Pinhole(K=intrinsics_torch)

    # For UniDepthV1 or UniDepthV2old, use the K matrix directly (as in demo)
    if isinstance(model, (UniDepthV2old, UniDepthV1)):
        camera = camera.K.squeeze(0)

    # Perform inference
    with torch.no_grad():
        predictions = model.infer(rgb_torch, camera)
    depth = predictions["depth"].squeeze().cpu().numpy()
    return depth


def live_depth_estimation(
    model,
    device,
    camera_id=0,
    intrinsics_path=None,
    side_by_side=True,
    depth_range=(0.01, 10.0),
    record_video=False,
    output_dir="output",
    model_info=None,
):
    """
    Perform live depth estimation using a USB camera.

    Args:
        model: UniDepth model instance
        device: torch device
        camera_id: USB camera ID
        intrinsics_path: Path to camera intrinsics file
        side_by_side: Whether to display RGB and depth side-by-side
        depth_range: Min and max depth for visualization
        record_video: Whether to record the output as a video
        output_dir: Directory to save the output video
        model_info: String describing the model configuration
    """
    # Create output directory if recording video
    if record_video:
        os.makedirs(output_dir, exist_ok=True)

    # Load intrinsics
    if intrinsics_path and os.path.exists(intrinsics_path):
        intrinsics = np.load(intrinsics_path)
        print(f"Loaded intrinsics from {intrinsics_path}")
    else:
        # Default intrinsics for a webcam with 640x480 resolution
        # These are approximate values - for better results, calibrate your camera
        fx = 600.0  # Approximate focal length in x
        fy = 600.0  # Approximate focal length in y
        cx = 320.0  # Principal point x (usually width/2)
        cy = 240.0  # Principal point y (usually height/2)

        intrinsics = np.array(
            [[fx, 0.0, cx], [0.0, fy, cy], [0.0, 0.0, 1.0]], dtype=np.float32
        )
        print("Using default camera intrinsics")

    # Initialize webcam
    cap = cv2.VideoCapture(camera_id)

    # Check if the camera opened successfully
    if not cap.isOpened():
        print("Error: Could not open webcam")
        return

    # Set camera properties (optional) - try to get 640x480 resolution
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

    # Get actual webcam resolution
    width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    fps = cap.get(cv2.CAP_PROP_FPS)

    print(f"Camera resolution: {width}x{height}, FPS: {fps}")

    # Initialize video writer if recording
    writer = None
    if record_video:
        timestamp = time.strftime("%Y%m%d-%H%M%S")
        video_out_path = f"{output_dir}/live_depth_{model_info}_{timestamp}.mp4"
        fourcc = cv2.VideoWriter_fourcc(*"mp4v")
        # Will set output dimensions after first frame

    # FPS calculation variables
    prev_frame_time = 0
    new_frame_time = 0

    print("Press 'q' to quit, 'r' to toggle recording")
    recording = record_video

    try:
        while True:
            # Capture frame from webcam
            ret, frame = cap.read()
            if not ret:
                print("Error: Failed to capture frame")
                break

            # Convert BGR to RGB (webcam captures in BGR)
            frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

            # Calculate FPS
            new_frame_time = time.time()
            fps_measured = (
                1 / (new_frame_time - prev_frame_time) if prev_frame_time > 0 else 0
            )
            prev_frame_time = new_frame_time

            # Infer depth
            depth = infer_frame(model, frame_rgb, intrinsics, device)

            # Colorize depth map
            depth_color = colorize(
                depth, vmin=depth_range[0], vmax=depth_range[1], cmap="magma_r"
            )

            # Convert back to BGR for display with OpenCV
            depth_color_bgr = cv2.cvtColor(depth_color, cv2.COLOR_RGB2BGR)

            # Create output frame
            if side_by_side:
                # Ensure both frames have the same height for side-by-side display
                h, w = depth_color_bgr.shape[:2]
                frame_resized = cv2.resize(
                    frame, (int(w * frame.shape[0] / frame.shape[1]), h)
                )
                output_frame = np.hstack((frame_resized, depth_color_bgr))
            else:
                output_frame = depth_color_bgr

            # Add FPS text
            cv2.putText(
                output_frame,
                f"FPS: {fps_measured:.1f}",
                (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                1,
                (0, 255, 0),
                2,
            )

            # Add recording indicator
            if recording:
                cv2.putText(
                    output_frame,
                    "REC",
                    (output_frame.shape[1] - 70, 30),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    1,
                    (0, 0, 255),
                    2,
                )

            # Initialize video writer with output dimensions (after first frame is processed)
            if recording and writer is None:
                output_height, output_width = output_frame.shape[:2]
                writer = cv2.VideoWriter(
                    video_out_path, fourcc, fps, (output_width, output_height)
                )
                print(f"Recording to {video_out_path}")

            # Write frame if recording
            if recording and writer is not None:
                writer.write(output_frame)

            # Display the output frame
            cv2.imshow("UniDepth Live", output_frame)

            # Check for key presses
            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                print("Quitting")
                break
            elif key == ord("r"):
                # Toggle recording
                recording = not recording
                if recording:
                    # Start a new recording
                    if writer is not None:
                        writer.release()
                    timestamp = time.strftime("%Y%m%d-%H%M%S")
                    video_out_path = (
                        f"{output_dir}/live_depth_{model_info}_{timestamp}.mp4"
                    )
                    output_height, output_width = output_frame.shape[:2]
                    writer = cv2.VideoWriter(
                        video_out_path, fourcc, fps, (output_width, output_height)
                    )
                    print(f"Started recording to {video_out_path}")
                else:
                    # Stop recording
                    if writer is not None:
                        writer.release()
                        writer = None
                        print("Stopped recording")

    finally:
        # Clean up
        cap.release()
        if writer is not None:
            writer.release()
        cv2.destroyAllWindows()
        print("Cleaned up resources")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Live depth estimation with UniDepth")
    parser.add_argument("--camera", type=int, default=0, help="Camera device ID")
    parser.add_argument(
        "--intrinsics",
        type=str,
        default="assets/demo/intrinsics.npy",
        help="Path to intrinsics.npy (optional)",
    )
    parser.add_argument(
        "--output", type=str, default="output", help="Output directory for recordings"
    )
    parser.add_argument(
        "--model_type",
        type=str,
        default="s",  # Using small model for better performance with webcam
        choices=["s", "b", "l"],
        help="Model size: s(mall), b(ase), l(arge)",
    )
    parser.add_argument("--v1", action="store_true", help="Use UniDepthV1 instead of V2")
    parser.add_argument(
        "--side_by_side",
        action="store_true",
        default=True,
        help="Show RGB and depth side-by-side",
    )
    parser.add_argument("--record", action="store_true", help="Record video on startup")
    parser.add_argument(
        "--min_depth", type=float, default=0.1, help="Min depth for colorization"
    )
    parser.add_argument(
        "--max_depth", type=float, default=2.0, help="Max depth for colorization"
    )
    parser.add_argument(
        "--resolution_level", type=int, default=None, help="Resolution level (optional)"
    )

    args = parser.parse_args()

    print("Torch version:", torch.__version__)
    print("OpenCV version:", cv2.__version__)

    # Select the model version (V1 or V2)
    if args.v1:
        ModelClass = UniDepthV1
        model_name = f"unidepth-v1-vit{args.model_type}14"
        model_version = "V1"
    else:
        ModelClass = UniDepthV2
        model_name = f"unidepth-v2-vit{args.model_type}14"
        model_version = "V2"

    # Create model info string for the filename
    model_size_map = {"s": "small", "b": "base", "l": "large"}
    model_size = model_size_map[args.model_type]
    model_info = f"UniDepth{model_version}_{model_size}"

    # Add resolution level if specified
    if args.resolution_level is not None:
        model_info += f"_res{args.resolution_level}"

    print(f"Loading {model_name}...")
    model = ModelClass.from_pretrained(f"lpiccinelli/{model_name}")

    # Set model parameters (for V2 only)
    if isinstance(model, UniDepthV2):
        model.interpolation_mode = "bilinear"
        if args.resolution_level is not None:
            model.resolution_level = args.resolution_level

    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    print(f"Device: {device}")
    model = model.to(device).eval()

    # Run live depth estimation
    live_depth_estimation(
        model,
        device,
        camera_id=args.camera,
        intrinsics_path=args.intrinsics,
        side_by_side=args.side_by_side,
        depth_range=(args.min_depth, args.max_depth),
        record_video=args.record,
        output_dir=args.output,
        model_info=model_info,
    )
