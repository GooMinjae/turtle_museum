import cv2
import os

# 변환할 비디오 파일 경로
video_path = "/home/rokey/turtlebot4_ws/src/output.mp4"

# 출력 폴더 (없으면 생성)
output_dir = "frames_jpg"
os.makedirs(output_dir, exist_ok=True)

# 비디오 캡처 객체 생성
cap = cv2.VideoCapture(video_path)

frame_num = 0

if not cap.isOpened():
    print("Error: 비디오 파일을 열 수 없습니다.")
else:
    while True:
        ret, frame = cap.read()
        if not ret:
            break  # 더 이상 프레임이 없으면 종료

        # PNG 파일로 저장
        if frame_num % 3 == 0:
            frame_filename = os.path.join(output_dir, f"frame_{frame_num:05d}.jpg")
            cv2.imwrite(frame_filename, frame)

        frame_num += 1

    cap.release()
    print(f"총 {frame_num}개의 프레임을 PNG로 저장했습니다. 저장 경로: {output_dir}")
