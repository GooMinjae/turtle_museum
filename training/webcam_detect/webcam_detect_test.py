from ultralytics import YOLO
import cv2
import time
import torch

# 모델 경로를 선택하여 주석을 해제하세요.
# train_8n_patient20_avg80
# model_path = "/home/rokey/train_ws/src/detect_webcam/runs/detect/train_8n_patient20_avg80/weights/best.pt"

# train_8n_patient30_avg80
# model_path = "/home/rokey/train_ws/src/detect_webcam/runs/detect/train_8n_patient30_avg80/weights/best.pt"

# train_8s_patient20_avg83
# model_path = "/home/rokey/train_ws/src/detect_webcam/runs/detect/train_8s_patient20_avg83/weights/best.pt"

# train_8s_patient30_avg83
# model_path = "/home/rokey/train_ws/src/detect_webcam/runs/detect/train_8s_patient30_avg83/weights/best.pt"

# train_11s_patient20_avg77
# model_path = "/home/rokey/train_ws/src/detect_webcam/runs/detect/train_11s_patient20_avg77/weights/best.pt"

# train_8s_mosaic08_avg84
# model_path = "/home/rokey/train_ws/src/detect_webcam/runs/detect/train_8s_mosaic08_avg84/weights/best.pt"

# train_8s_mosaic10_avg83
# model_path = "/home/rokey/train_ws/src/detect_webcam/runs/detect/train_8s_mosaic10_avg83/weights/best.pt"
model_path = "/home/rokey/turtlebot4_ws/src/mini_project/resource/best.pt"
# 1. 모델 로드 및 GPU 설정
model = YOLO(model_path)
device = "cuda" if torch.cuda.is_available() else "cpu"
model.to(device)
print(f"모델을 {device}로 로드했습니다.")

# 2. 웹캠 연결
cap = cv2.VideoCapture(2)
if not cap.isOpened():
    print("오류: 웹캠을 열 수 없습니다.")
    exit()

# 3. 10초 동안 실시간 추론 및 시간 측정
print("실시간 객체 탐지를 시작합니다. 10초 후에 자동으로 종료됩니다.")
inference_times = []
frame_count = 0
start_time = time.time()
end_time_limit = start_time + 10

while time.time() < end_time_limit:
    success, frame = cap.read()
    if success:
        inference_start = time.time()
        results = model(frame, verbose=False, conf=0.5)
        inference_end = time.time()
        inference_time_ms = (inference_end - inference_start) * 1000
        inference_times.append(inference_time_ms)

        im_with_boxes = results[0].plot()
        remaining_time = int(end_time_limit - time.time())
        cv2.putText(im_with_boxes, f"Time Left: {remaining_time}s", (10, 60), 
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
        cv2.imshow("YOLO Real-Time Detection", im_with_boxes)
        
        frame_count += 1
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break
    else:
        break

# 4. 자원 해제 및 실시간 성능 출력
cap.release()
cv2.destroyAllWindows()
final_end_time = time.time()

if frame_count > 0:
    average_fps = frame_count / (final_end_time - start_time)
    average_inference_time = sum(inference_times) / len(inference_times)
    
    print("\n" + "="*50)
    print("실시간 추론 성능 평가 (10초)")
    print("="*50)
    print(f"총 프레임 수: {frame_count}")
    print(f"총 실행 시간: {final_end_time - start_time:.2f} 초")
    print(f"평균 FPS: {average_fps:.2f}")
    print(f"평균 추론 시간: {average_inference_time:.2f} ms")

# 5. 모델의 정량적 정확도(mAP) 평가
# data.yaml 파일의 경로를 정확하게 지정해야 합니다.
print("\n" + "="*50)
print("모델 정확도 평가 (mAP)")
print("="*50)
metrics = model.val(data="/home/rokey/datasets/rccar.v2-webcam_detect_org.yolov8/data.yaml", split="val")
print(f"mAP@0.5-0.95: {metrics.box.map}")
print(f"mAP@0.5: {metrics.box.map50}")
print(f"mAP@0.75: {metrics.box.map75}")