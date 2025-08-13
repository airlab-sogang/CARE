import cv2
import torch
import numpy as np
import time

from unidepth.models import UniDepthV2
from unidepth.utils.camera import Pinhole


def get_center_depth(model, intrinsics, device):
    cap = cv2.VideoCapture(0)
    intrinsics_torch = torch.from_numpy(intrinsics).unsqueeze(0).to(device)
    camera = Pinhole(K=intrinsics_torch)

    print("📸 실시간 중심 거리 측정 시작 (q 키로 종료)")
    time.sleep(1.0)

    while True:
        ret, frame = cap.read()
        if not ret:
            print("❌ 웹캠 오류")
            break

        frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

        # 모델 추론
        rgb_torch = (
            torch.from_numpy(frame_rgb).permute(2, 0, 1).unsqueeze(0).float().to(device)
        )
        with torch.no_grad():
            depth = model.infer(rgb_torch, camera)["depth"].squeeze().cpu().numpy()

        # 중심 픽셀 영역의 평균
        h, w = depth.shape
        center_depth = np.mean(
            depth[h // 2 - 10 : h // 2 + 10, w // 2 - 10 : w // 2 + 10]
        )

        # 시각화용 표시
        cv2.circle(frame, (w // 2, h // 2), 5, (0, 0, 255), -1)
        cv2.putText(
            frame,
            f"Center Depth: {center_depth:.2f} m",
            (30, 40),
            cv2.FONT_HERSHEY_SIMPLEX,
            1.0,
            (0, 255, 0),
            2,
        )

        cv2.imshow("Real-Time Center Depth", frame)
        key = cv2.waitKey(1) & 0xFF
        if key == ord("q"):
            break

    cap.release()
    cv2.destroyAllWindows()
    print("🛑 종료됨")


if __name__ == "__main__":
    # Load camera intrinsics
    intrinsics = np.load("assets/demo/intrinsics.npy")

    # Load UniDepthV2 model
    model = UniDepthV2.from_pretrained("lpiccinelli/unidepth-v2-vits14")
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    model = model.to(device).eval()

    # Run the live center-depth estimator
    get_center_depth(model, intrinsics, device)
