from ultralytics import YOLO

# data_path="/home/rokey/datasets/rccar.v1-webcam_detect.yolov8/data.yaml"
# YOLOv8 모델 불러오기 (v8n, v8s, v8m, v8l, v8x 가능)
data_path = "/home/rokey/datasets/rccar.v2-webcam_detect_org.yolov8/data.yaml"

#################################################################################
# model = YOLO('yolov8s.pt')  # 또는 yolov8n.pt 등

# # # 학습 시작
# model.train(
#     data=data_path,
#     epochs=300,
#     imgsz=640,
#     batch=16,
#     patience=30,
#     name='yolov8-custom-org_dataset',
#     device=0,  # GPU 사용, CPU는 device='cpu'
#     optimizer='adamw',
#     lr0=0.003,
#     cos_lr=True
# )

#################################################################################
model = YOLO('yolov8s.pt')  # 또는 yolov8s.pt 등

# # 학습 시작
model.train(
    data=data_path,
    epochs=200,
    patience=20, 
    imgsz=640,
    batch=32,
    name='yolov8-custom-org_dataset',
    device=0,  # GPU 사용, CPU는 device='cpu'
    optimizer='adamw',
    lr0=0.001,
    cos_lr=True,
    augment=True,
    label_smoothing=0.1
)

# model = YOLO('yolov8n.pt')

# model.train(
#     data=data_path,
#     epochs=300,
#     imgsz=640,
#     batch=16,
#     patience=30,
#     name='yolov8-custom-org_dataset',
#     device=0,
#     optimizer='adamw',
#     lr0=0.003,
#     lrf=0.01, # 추가
#     cos_lr=True,
#     warmup_epochs=3.0, # 추가
#     # 데이터 증강 파라미터 추가
#     degrees=15.0,
#     translate=0.1,
#     scale=0.5,
#     hsv_h=0.015,
#     hsv_s=0.7,
#     hsv_v=0.4,
#     flipud=0.0,
#     fliplr=0.5,
#     mosaic=1.0,
#     mixup=0.0,
#     # 정규화 파라미터 추가
#     dropout=0.1,
# )

#################################################################################
# model = YOLO('yolov8n.pt')  # 또는 yolov8n.pt 등

# # # 학습 시작
# model.train(
#      data=data_path,
#      epochs=300,
#      imgsz=640,
#      batch=32,
#      patience=30,
#      name='yolov8-custom',
#      device=0,  # GPU 사용, CPU는 device='cpu'
#      mosaic=0.8,
#      optimizer='adamw',
#      lr0=0.003,
#      cos_lr=True
# )

# #################################################################################
# model = YOLO('yolov8n.pt')  # 또는 yolov8n.pt 등

# # # 학습 시작
# model.train(
#      data=data_path,
#      epochs=300,
#      imgsz=640,
#      batch=32,
#      patience=20,
#      name='yolov8-custom',
#      device=0,  # GPU 사용, CPU는 device='cpu'
#      mosaic=0.8,
#      optimizer='adamw',
#      lr0=0.003,
#      cos_lr=True
# )

#################################################################################
# from ultralytics import YOLO

# # 1. 모델 로드
# model = YOLO('yolov8s.pt')  # 또는 'yolov8m.pt' 등

# # 2. 학습 시작
# model.train(
#     task='detect',
#     data= data_path,  # 데이터셋 yaml
#     epochs=300,
#     imgsz=640,
#     batch=32,
#     name='yolov8-custom2-improved',

#     # --- 모델 및 학습 설정 ---
#     pretrained=True,
#     optimizer='AdamW',
#     device=0,
#     seed=0,
#     deterministic=True,
#     amp=True,                  # mixed precision
#     dropout=0.2,               # regularization
#     label_smoothing=0.05,      # 확신 줄이기

#     # --- LR 스케줄링 ---
#     lr0=0.003,
#     lrf=0.01,
#     cos_lr=True,
#     warmup_epochs=3.0,

#     # --- Loss 구성 ---
#     box=7.5,
#     cls=0.5,
#     dfl=1.5,

#     # --- Augmentation ---
#     mosaic=0.8,
#     hsv_h=0.015,
#     hsv_s=0.7,
#     hsv_v=0.4,
#     translate=0.1,
#     scale=0.5,
#     fliplr=0.5,
#     auto_augment='randaugment',
#     erasing=0.4,

#     # --- 기타 설정 ---
#     save=True,
#     save_period=50,            # 50 에폭마다 저장
#     resume=False,
#     verbose=True,
#     plots=True,
#     val=True,
#     patience=30
# )
