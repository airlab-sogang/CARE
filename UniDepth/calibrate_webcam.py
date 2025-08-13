import cv2
import numpy as np
import os

# 체커보드 패턴 설정 (사용할 체커보드의 내부 코너 수)
CHECKERBOARD = (7, 6)  # 7x6 내부 코너
SQUARE_SIZE = 0.025  # 각 사각형의 실제 크기 (미터 단위)

# 3D 점의 실제 좌표를 설정 (z=0 평면에 위치)
objp = np.zeros((CHECKERBOARD[0] * CHECKERBOARD[1], 3), np.float32)
objp[:, :2] = np.mgrid[0 : CHECKERBOARD[0], 0 : CHECKERBOARD[1]].T.reshape(-1, 2)
objp *= SQUARE_SIZE  # 실제 크기 반영

objpoints = []  # 실제 3D 좌표
imgpoints = []  # 이미지 상의 2D 좌표

cap = cv2.VideoCapture(0)
print("캘리브레이션을 위해 체커보드를 카메라에 여러 각도로 보여주세요.")
print("키보드에서 'c'를 눌러 캡처, 'q'를 눌러 종료 및 캘리브레이션 시작.")

while True:
    ret, frame = cap.read()
    if not ret:
        break
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # 체커보드 코너 찾기
    ret_corners, corners = cv2.findChessboardCorners(gray, CHECKERBOARD, None)

    # 코너 찾았을 경우 그리기
    if ret_corners:
        cv2.drawChessboardCorners(frame, CHECKERBOARD, corners, ret_corners)

    cv2.imshow("Calibration", frame)
    key = cv2.waitKey(1) & 0xFF

    if key == ord("c") and ret_corners:
        print("체커보드 캡처됨")
        objpoints.append(objp)
        imgpoints.append(corners)
    elif key == ord("q"):
        print("캘리브레이션 시작")
        break

cap.release()
cv2.destroyAllWindows()

# 최소 10장 이상 수집됐는지 확인
if len(objpoints) < 10:
    print("캘리브레이션을 위해 최소 10장의 캡처가 필요합니다.")
    exit()

# 카메라 캘리브레이션 수행
ret, K, dist, rvecs, tvecs = cv2.calibrateCamera(
    objpoints, imgpoints, gray.shape[::-1], None, None
)

print("카메라 내참행렬 (intrinsic matrix):\n", K)
print("왜곡 계수 (distortion coefficients):\n", dist)

# intrinsic matrix만 저장
os.makedirs("assets/demo", exist_ok=True)
np.save("assets/demo/intrinsics.npy", K)
print("intrinsics.npy 로 저장 완료 ✅")
