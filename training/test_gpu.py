from ultralytics import YOLO
import cv2
import torch
import time

try:
    # 모델 로드
    model = YOLO('/home/rokey/train_ws/src/runs/detect/yolov8-custom-org_dataset2/weights/best.pt')

    # 웹캠 연결
    # 0은 보통 기본 웹캠을 의미합니다. 다른 웹캠을 사용하려면 숫자를 변경하세요.
    cap = cv2.VideoCapture(2)  
    if not cap.isOpened():
        print("오류: 웹캠을 열 수 없습니다.")
        exit()

    print("\n***실시간 웹캠 추적 시작 (GPU 가속)")
    
    # FPS(Frames Per Second) 계산을 위한 변수
    start_time = time.time()
    frame_count = 0
    
    while True:
        # 프레임 읽기
        success, frame = cap.read()
        if not success:
            break

        # 프레임 뒤집기 (선택 사항: 필요에 따라 주석 처리)
        # frame = cv2.flip(frame, 1)

        # PyTorch GPU 추론 및 추적
        # source=frame을 사용해 실시간 프레임을 입력으로 사용합니다.
        results = model.track(source=frame, show=False, tracker='bytetrack.yaml', device='cuda', conf=0.5, iou=0.5, persist=True)
        
        # 결과 시각화
        im_with_boxes = results[0].plot()

        # FPS 계산 및 표시
        frame_count += 1
        elapsed_time = time.time() - start_time
        fps = frame_count / elapsed_time if elapsed_time > 0 else 0
        cv2.putText(im_with_boxes, f"FPS: {fps:.2f}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        
        # 화면에 표시
        cv2.imshow('YOLOv8 Real-Time Tracking', im_with_boxes)

        # 'q' 키를 누르면 종료
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

except KeyboardInterrupt:
    print("\nCtrl+C 감지. 종료합니다...")

finally:
    # 자원 해제
    if 'cap' in locals() and cap.isOpened():
        cap.release()
    cv2.destroyAllWindows()