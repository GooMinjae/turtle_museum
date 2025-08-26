from ultralytics import YOLO
model = YOLO("/home/rokey/turtlebot4_ws/src/training/runs/detect/yolov8-custom-org_dataset5/weights/best.pt")                 # 학습된 가중치
metrics = model.val(
    data="/home/rokey/datasets/rccar.v2-webcam_detect_org.yolov8/data.yaml",                   # 클래스/경로가 들어있는 yaml
    iou=0.50,                           # mAP/F1 계산용 IoU 임계값 (보통 0.5)
    conf=0.001,                         # 낮은 conf부터 평가하도록 충분히 낮게
    plots=True                          # PR/F1 곡선 이미지 저장
)