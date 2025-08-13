import cv2
import numpy as np
import os

# 체커보드 내부 코너 개수
CHECKERBOARD = (7, 6)  # 내부 코너 기준 (사각형은 8x7 개 필요)
SQUARE_SIZE = 0.025  # 한 칸 크기 (미터 단위)

objp = np.zeros((1, CHECKERBOARD[0] * CHECKERBOARD[1], 3), np.float32)
objp[0, :, :2] = np.mgrid[0 : CHECKERBOARD[0], 0 : CHECKERBOARD[1]].T.reshape(-1, 2)
objp *= SQUARE_SIZE

objpoints = []  # 3D 점
imgpoints = []  # 2D 점

cap = cv2.VideoCapture(0)
print("📷 체커보드를 다양한 각도로 보여주세요")
print("📸 'c'로 캡처 / 'q'로 종료 및 캘리브레이션 수행")

frame_size = None

while True:
    ret, frame = cap.read()
    if not ret:
        break

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    frame_size = gray.shape[::-1]

    found, corners = cv2.findChessboardCorners(
        gray,
        CHECKERBOARD,
        cv2.CALIB_CB_ADAPTIVE_THRESH
        + cv2.CALIB_CB_FAST_CHECK
        + cv2.CALIB_CB_NORMALIZE_IMAGE,
    )

    if found:
        cv2.drawChessboardCorners(frame, CHECKERBOARD, corners, found)

    cv2.putText(
        frame,
        f"Captures: {len(objpoints)}",
        (10, 30),
        cv2.FONT_HERSHEY_SIMPLEX,
        1,
        (0, 255, 0),
        2,
    )
    cv2.imshow("Fish-eye Calibration", frame)

    key = cv2.waitKey(1) & 0xFF
    if key == ord("c") and found:
        print(f"✔ 캡처됨: {len(objpoints) + 1}")
        corners = cv2.cornerSubPix(
            gray,
            corners,
            (3, 3),
            (-1, -1),
            (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 1e-6),
        )
        imgpoints.append(corners)
        objpoints.append(objp)
    elif key == ord("q"):
        break

cap.release()
cv2.destroyAllWindows()

if len(objpoints) < 10:
    print("❌ 최소 10장 이상 캡처해야 합니다.")
    exit()

print("📐 캘리브레이션 수행 중...")

K = np.zeros((3, 3))
D = np.zeros((4, 1))
rvecs = []
tvecs = []

flags = (
    cv2.fisheye.CALIB_RECOMPUTE_EXTRINSIC
    + cv2.fisheye.CALIB_CHECK_COND
    + cv2.fisheye.CALIB_FIX_SKEW
)

rms, _, _, _, _ = cv2.fisheye.calibrate(
    objpoints,
    imgpoints,
    frame_size,
    K,
    D,
    rvecs,
    tvecs,
    flags,
    (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 100, 1e-6),
)

print(f"✅ RMS error: {rms:.4f}")
print("📏 Intrinsics (K):\n", K)
print("🔁 Distortion (D):\n", D.T)

# 저장
os.makedirs("assets/fisheye", exist_ok=True)
np.save("assets/fisheye/fisheye_intrinsics.npy", K)
np.save("assets/fisheye/fisheye_distortion.npy", D)
print("📁 저장 완료: fisheye_intrinsics.npy, fisheye_distortion.npy")
