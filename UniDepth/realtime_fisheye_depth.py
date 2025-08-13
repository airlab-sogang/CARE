import cv2
import time
import torch
import numpy as np
import matplotlib.pyplot as plt

from unidepth.models import UniDepthV2
from unidepth.utils.camera import Pinhole
from unidepth.utils import colorize


def run_fisheye_depth(model, intrinsics_path, distortion_path):
    cap = cv2.VideoCapture(0)

    # Load fisheye calibration
    K = np.load(intrinsics_path)
    D = np.load(distortion_path)
    DIM = (640, 480)  # 이미지 크기 맞추기

    map1, map2 = cv2.fisheye.initUndistortRectifyMap(
        K, D, np.eye(3), K, DIM, cv2.CV_16SC2
    )
    intrinsics_torch = torch.from_numpy(K).unsqueeze(0).to(device)
    camera = Pinhole(K=intrinsics_torch)

    print("🐟 Fish-eye 보정 영상 + 중심 depth 측정 시작 (q 키 종료)")

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        frame = cv2.resize(frame, DIM)
        undistorted = cv2.remap(frame, map1, map2, interpolation=cv2.INTER_LINEAR)
        frame_rgb = cv2.cvtColor(undistorted, cv2.COLOR_BGR2RGB)

        # 추론
        rgb_torch = (
            torch.from_numpy(frame_rgb).permute(2, 0, 1).unsqueeze(0).float().to(device)
        )
        with torch.no_grad():
            depth = model.infer(rgb_torch, camera)["depth"].squeeze().cpu().numpy()

        h, w = depth.shape
        center_depth = np.mean(
            depth[h // 2 - 10 : h // 2 + 10, w // 2 - 10 : w // 2 + 10]
        )

        # 시각화
        depth_color = colorize(depth, vmin=0.1, vmax=2.0, cmap="magma_r")
        depth_bgr = cv2.cvtColor(depth_color, cv2.COLOR_RGB2BGR)

        cv2.circle(undistorted, (w // 2, h // 2), 5, (0, 0, 255), -1)
        cv2.putText(
            undistorted,
            f"Center Depth: {center_depth:.2f} m",
            (30, 40),
            cv2.FONT_HERSHEY_SIMPLEX,
            1.0,
            (0, 255, 0),
            2,
        )

        output = np.hstack((undistorted, depth_bgr))
        cv2.imshow("Fish-eye Undistorted + Depth Map", output)

        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    intrinsics_path = "assets/fisheye/fisheye_intrinsics.npy"
    distortion_path = "assets/fisheye/fisheye_distortion.npy"

    model = UniDepthV2.from_pretrained("lpiccinelli/unidepth-v2-vitb14")
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    model = model.to(device).eval()

    run_fisheye_depth(model, intrinsics_path, distortion_path)
