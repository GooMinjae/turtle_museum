import cv2
from ultralytics import YOLO

# YOLOv8 모델 로드
model = YOLO("/home/rokey/train_ws/src/runs/detect/yolov8-custom-org_dataset5/weights/best.pt")

# 웹캠 열기 (카메라 인덱스 확인 필요)
cap = cv2.VideoCapture(2)

while True:
    ret, frame = cap.read()
    if not ret:
        break

    # 모델 추론
    results = model.predict(frame, imgsz=640, conf=0.5, verbose=False)

    # 결과에서 바운딩 박스와 신뢰도 표시
    for r in results:
        boxes = r.boxes  # 검출된 박스 정보
        for box in boxes:
            x1, y1, x2, y2 = map(int, box.xyxy[0])  # 좌표
            conf = float(box.conf[0])               # 신뢰도
            cls_id = int(box.cls[0])                # 클래스 ID
            label = f"{model.names[cls_id]} {conf:.2f}"  # 라벨 + confidence

            # 사각형 그리기
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.putText(frame, label, (x1, y1 - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

    # 화면 출력
    cv2.imshow("YOLOv8 Detection", frame)

    # ESC로 종료
    if cv2.waitKey(1) & 0xFF == 27:
        break

cap.release()
cv2.destroyAllWindows()
