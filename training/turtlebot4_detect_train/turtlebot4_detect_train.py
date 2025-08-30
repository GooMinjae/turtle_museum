from ultralytics import YOLO

data_path="/home/rokey/turtlebot4_ws/src/training/turtlebot4_detect_train/persondetect.v1-123433454434asd.yolov8/data.yaml"
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
    name='web_8s_pt',
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
    name='web_8n',
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
    # patience=30,
    name='web_8n_no_earlystop',
    device=0,  # GPU 사용, CPU는 device='cpu'
    # mosaic=0.8,
    optimizer='adamw',
    lr0=0.003,
    cos_lr=True
)

#################################################################################