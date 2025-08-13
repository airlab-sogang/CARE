import argparse
import cv2
import numpy as np
import torch
import os
from PIL import Image
from tqdm import tqdm
import re

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


def main(
    video_path,
    intrinsics_path,
    output_dir,
    model,
    device,
    side_by_side=False,
    depth_range=(0.01, 10.0),
    model_info=None,
):
    # Create output directory if it doesn't exist
    os.makedirs(output_dir, exist_ok=True)

    # Load ground truth intrinsics
    intrinsics = np.load(intrinsics_path)

    # Open the video file and get its properties
    cap = cv2.VideoCapture(video_path)
    fps = cap.get(cv2.CAP_PROP_FPS)
    total_frames = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))

    # Read the first frame to determine the output frame dimensions
    ret, frame = cap.read()
    if not ret:
        print("Error reading the first frame.")
        return

    # Infer depth and colorize using demo settings
    depth = infer_frame(model, frame, intrinsics, device)
    depth_color = colorize(
        depth, vmin=depth_range[0], vmax=depth_range[1], cmap="magma_r"
    )
    # Convert RGB to BGR for OpenCV compatibility
    depth_color_bgr = cv2.cvtColor(depth_color, cv2.COLOR_RGB2BGR)

    # Determine output dimensions
    if side_by_side:
        # Resize frame to match depth map height if needed
        h, w = depth_color_bgr.shape[:2]
        frame_resized = cv2.resize(frame, (int(w * frame.shape[0] / frame.shape[1]), h))
        output_frame = np.hstack((frame_resized, depth_color_bgr))
        output_width, output_height = output_frame.shape[1], output_frame.shape[0]
    else:
        output_frame = depth_color_bgr
        output_height, output_width = depth_color_bgr.shape[:2]

    # Generate output filename with video name and model info
    video_basename = os.path.basename(video_path)
    video_name = os.path.splitext(video_basename)[0]
    display_mode = "side_by_side" if side_by_side else "depth_only"

    # Create descriptive output filename
    video_out_path = f"{output_dir}/{video_name}_{model_info}_{display_mode}.mp4"

    # Initialize VideoWriter (using mp4 codec)
    fourcc = cv2.VideoWriter_fourcc(*"mp4v")
    writer = cv2.VideoWriter(video_out_path, fourcc, fps, (output_width, output_height))

    # Write the first processed frame
    writer.write(output_frame)

    # Process remaining frames with progress bar
    pbar = tqdm(total=total_frames - 1, desc="Processing frames")
    frame_idx = 1

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        depth = infer_frame(model, frame, intrinsics, device)
        depth_color = colorize(
            depth, vmin=depth_range[0], vmax=depth_range[1], cmap="magma_r"
        )
        depth_color_bgr = cv2.cvtColor(depth_color, cv2.COLOR_RGB2BGR)

        if side_by_side:
            h, w = depth_color_bgr.shape[:2]
            frame_resized = cv2.resize(
                frame, (int(w * frame.shape[0] / frame.shape[1]), h)
            )
            output_frame = np.hstack((frame_resized, depth_color_bgr))
        else:
            output_frame = depth_color_bgr

        writer.write(output_frame)
        frame_idx += 1
        pbar.update(1)

    cap.release()
    writer.release()
    pbar.close()
    print(f"Depth video saved to {video_out_path}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Process a video with UniDepthV2")
    parser.add_argument(
        "--video",
        type=str,
        default="assets/demo/video/driving_sample_1.mp4",
        help="Path to input video",
    )
    parser.add_argument(
        "--intrinsics",
        type=str,
        default="assets/demo/intrinsics.npy",
        help="Path to intrinsics.npy",
    )
    parser.add_argument(
        "--output", type=str, default="assets/demo", help="Output directory"
    )
    parser.add_argument(
        "--model_type",
        type=str,
        default="l",
        choices=["s", "b", "l"],
        help="Model size: s(mall), b(ase), l(arge)",
    )
    parser.add_argument("--v1", action="store_true", help="Use UniDepthV1 instead of V2")
    parser.add_argument(
        "--side_by_side", action="store_true", help="Show RGB and depth side-by-side"
    )
    parser.add_argument(
        "--min_depth", type=float, default=0.01, help="Min depth for colorization"
    )
    parser.add_argument(
        "--max_depth", type=float, default=100.0, help="Max depth for colorization"
    )
    parser.add_argument(
        "--resolution_level", type=int, default=None, help="Resolution level (optional)"
    )

    args = parser.parse_args()

    print("Torch version:", torch.__version__)

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

    # Add depth range info
    model_info += f"_d{args.min_depth}-{args.max_depth}"

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

    main(
        args.video,
        args.intrinsics,
        args.output,
        model,
        device,
        side_by_side=args.side_by_side,
        depth_range=(args.min_depth, args.max_depth),
        model_info=model_info,
    )
