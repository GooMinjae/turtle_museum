from ultralytics import YOLO

data_path="/home/rokey/datasets/turtlebot.v1-turtlebot4_detect.yolov8/data.yaml"
# YOLOv8 모델 불러오기 (v8n, v8s, v8m, v8l, v8x 가능)

#################################################################################
model = YOLO('yolov8s.pt')  # 또는 yolov8n.pt 등

# # 학습 시작
model.train(
    data=data_path,
    epochs=300,
    imgsz=640,
    batch=16,
    patience=30,
    name='yolov8-turtlebot4-custom',
    device=0,  # GPU 사용, CPU는 device='cpu'
    # mosaic=0.8,
    optimizer='adamw',
    lr0=0.003,
    cos_lr=True
)

#################################################################################

model = YOLO('yolov8n.pt')  # 또는 yolov8n.pt 등

# # 학습 시작
model.train(
    data=data_path,
    epochs=300,
    imgsz=640,
    batch=16,
    patience=30,
    name='yolov8-turtlebot4-custom',
    device=0,  # GPU 사용, CPU는 device='cpu'
    # mosaic=0.8,
    optimizer='adamw',
    lr0=0.003,
    cos_lr=True
)

#################################################################################