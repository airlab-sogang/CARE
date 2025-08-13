import os
import time
import cv2
import torch
import numpy as np
import matplotlib.pyplot as plt

from unidepth.models import UniDepthV2
from unidepth.utils.camera import Pinhole


def generate_error_heatmap(pred_depth, actual_distance):
    # 계산: 각 픽셀에서의 절대 오차
    error_map = np.abs(pred_depth - actual_distance)

    # 시각화용 colormap 적용
    plt.figure(figsize=(10, 5))
    plt.imshow(error_map, cmap="inferno")
    plt.title(f"Error Heatmap (Actual: {actual_distance:.2f} m)")
    plt.colorbar(label="Absolute Error (m)")
    plt.axis("off")
    plt.tight_layout()
    plt.show()


def evaluate_with_error_heatmap(model, intrinsics, device):
    cap = cv2.VideoCapture(0)
    intrinsics_torch = torch.from_numpy(intrinsics).unsqueeze(0).to(device)
    camera = Pinhole(K=intrinsics_torch)

    while True:
        ret, frame = cap.read()
        if not ret:
            print("❌ 웹캠 프레임 오류")
            break

        frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

        rgb_torch = (
            torch.from_numpy(frame_rgb).permute(2, 0, 1).unsqueeze(0).float().to(device)
        )

        with torch.no_grad():
            depth = model.infer(rgb_torch, camera)["depth"].squeeze().cpu().numpy()

        h, w = depth.shape
        center_depth = np.mean(
            depth[h // 2 - 10 : h // 2 + 10, w // 2 - 10 : w // 2 + 10]
        )
        print(f"\n📷 Predicted center depth: {center_depth:.2f} m")

        # 사용자 입력 거리
        user_input = input(
            "📏 화면 전체에 해당하는 물체의 실제 거리 (m)를 입력하세요 (종료하려면 'q'): "
        )
        if user_input.lower() == "q":
            break

        try:
            actual_distance = float(user_input)
        except ValueError:
            print("⚠️ 숫자를 입력해주세요.")
            continue

        # 히트맵 생성
        generate_error_heatmap(depth, actual_distance)

    cap.release()
    print("📁 종료됨")


if __name__ == "__main__":
    intrinsics = np.load("assets/demo/intrinsics.npy")
    model = UniDepthV2.from_pretrained("lpiccinelli/unidepth-v2-vitb14")
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    model = model.to(device).eval()

    evaluate_with_error_heatmap(model, intrinsics, device)
