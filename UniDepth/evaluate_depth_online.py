import os
import time
import csv
import cv2
import torch
import numpy as np
import matplotlib.pyplot as plt

from unidepth.models import UniDepthV2
from unidepth.utils.camera import Pinhole


def evaluate_depth_online(model, intrinsics, device, output_csv="output/depth_eval.csv"):
    cap = cv2.VideoCapture(0)
    intrinsics_torch = torch.from_numpy(intrinsics).unsqueeze(0).to(device)
    camera = Pinhole(K=intrinsics_torch)

    os.makedirs(os.path.dirname(output_csv), exist_ok=True)

    timestamps = []
    actuals = []
    preds = []
    errors = []

    with open(output_csv, mode="w", newline="") as file:
        writer = csv.writer(file)
        writer.writerow(
            [
                "timestamp",
                "actual_distance_m",
                "predicted_distance_m",
                "abs_error_m",
                "percent_error",
            ]
        )

        while True:
            ret, frame = cap.read()
            if not ret:
                print("❌ 웹캠을 읽을 수 없습니다.")
                break

            # RGB 변환
            frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

            # 추론
            rgb_torch = (
                torch.from_numpy(frame_rgb)
                .permute(2, 0, 1)
                .unsqueeze(0)
                .float()
                .to(device)
            )
            with torch.no_grad():
                depth = model.infer(rgb_torch, camera)["depth"].squeeze().cpu().numpy()

            # 중심 depth 계산
            h, w = depth.shape
            center_depth = np.mean(
                depth[h // 2 - 10 : h // 2 + 10, w // 2 - 10 : w // 2 + 10]
            )
            print(f"\n📷 Predicted center depth: {center_depth:.2f} m")

            # 사용자 입력
            user_input = input("📏 실제 거리 (미터 단위) 입력 (종료하려면 'q'): ")
            if user_input.lower() == "q":
                break

            try:
                actual_distance = float(user_input)
            except ValueError:
                print("⚠️ 숫자를 입력해주세요.")
                continue

            abs_error = abs(center_depth - actual_distance)
            percent_error = 100 * abs_error / actual_distance
            timestamp = time.strftime("%Y-%m-%d %H:%M:%S")

            # 기록
            writer.writerow(
                [timestamp, actual_distance, center_depth, abs_error, percent_error]
            )
            print(f"✅ 기록됨: Error = {abs_error:.2f} m ({percent_error:.1f}%)")

            # 그래프용 데이터 저장
            timestamps.append(timestamp)
            actuals.append(actual_distance)
            preds.append(center_depth)
            errors.append(abs_error)

    cap.release()
    print(f"\n📁 결과 저장됨: {output_csv}")

    # 📊 시각화
    if actuals:
        plt.figure(figsize=(10, 5))
        plt.plot(actuals, label="Actual Distance (m)", marker="o")
        plt.plot(preds, label="Predicted Distance (m)", marker="x")
        plt.title("Depth Prediction Accuracy")
        plt.xlabel("Measurement Index")
        plt.ylabel("Distance (m)")
        plt.grid(True)
        plt.legend()
        plt.tight_layout()
        plt.show()

        plt.figure(figsize=(8, 4))
        plt.bar(range(len(errors)), errors)
        plt.title("Absolute Errors per Measurement")
        plt.xlabel("Measurement Index")
        plt.ylabel("Error (m)")
        plt.grid(True)
        plt.tight_layout()
        plt.show()


if __name__ == "__main__":
    # Load intrinsics (calibrated)
    intrinsics = np.load("assets/demo/intrinsics.npy")

    # Load UniDepthV2 model
    model_name = "unidepth-v2-vitb14"
    model = UniDepthV2.from_pretrained(f"lpiccinelli/{model_name}")

    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    print("🖥 Using device:", device)
    model = model.to(device).eval()

    # Run the evaluation
    evaluate_depth_online(model, intrinsics, device)
