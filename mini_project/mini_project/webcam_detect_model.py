# from ultralytics import YOLO
# import cv2
# import os

# # project_folder = "cvs"
# # model_filename = "best.pt" # train 디렉토리에 있는 best.pt를 project_folder로 복사

# # 모델 로드
# # 자동으로 현재 폴더 기준 경로 설정
# # BASE_DIR = os.getcwd()  # 현재 폴더 기준
# # OUTPUT_DIR = os.path.join(BASE_DIR, project_folder)  # 원하는 상위 폴더 이름

# # model_path = os.path.join(OUTPUT_DIR, model_filename)
# # model_path = "/home/rokey/train_ws/src/detect_webcam/runs/detect/train/weights/best.pt"
# # model_path = "/home/rokey/train_ws/src/runs/detect/yolov8-custom/weights/best.pt" # 70 ~ 80 
# # model_path = "/home/rokey/train_ws/src/runs/detect/yolov8-custom2/weights/best.pt" # 높은 신뢰도를 가지고 있지만 오인식이 심하다
# # model_path = "/home/rokey/train_ws/src/runs/detect/yolov8-custom3/weights/best.pt" # 낮은 신뢰도(65 ~ 80)를 가지고 있지만 오인식이 없다
# # model_path = "/home/rokey/train_ws/src/runs/detect/yolov8-custom4/weights/best.pt" # 낮은 신뢰도(65 ~ 75)를 가지고 있지만 오인식이 없다. 
# # model_path = "/home/rokey/train_ws/src/runs/detect/yolov8-custom5/weights/best.pt" # 낮은 신뢰도(60 ~ 70)를 가지고 인식률이 너무 낮다. 
# # model_path = "/home/rokey/train_ws/src/runs/detect/yolov8-custom2-improved2/weights/best.pt" # 그나마 현재 최선?
# model_path = "/home/rokey/train_ws/src/runs/detect/yolov8-custom-org_dataset4/weights/best.pt"

# model = YOLO(model_path)

# # 카메라 열기 (0: 기본 내장 카메라)
# cap = cv2.VideoCapture(2)
# if not cap.isOpened():
#     print("카메라를 열 수 없습니다.")
#     exit()

# print("실시간 예측 시작 (종료: Q 키 누르기)")

# while True:
#     ret, frame = cap.read()
#     if not ret:
#         break

#     # 프레임 예측 (stream=False: 프레임 1개씩 예측)
#     results = model.predict(source=frame, conf=0.10, verbose=False)

#     # 예측 결과 가져오기
#     result = results[0]
#     boxes = result.boxes  # bounding box 정보
#     classes = result.names  # 클래스 이름들

#     for box in boxes:
#         cls_id = int(box.cls[0])              # 클래스 ID
#         conf = float(box.conf[0]) * 100       # 신뢰도 (0~1 → 0~100%)
#         label = f"{classes[cls_id]} {conf:.1f}%"

#         # Bounding box 좌표
#         x1, y1, x2, y2 = map(int, box.xyxy[0])  # 정수 변환

#         # 박스 그리기
#         cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
#         # 클래스 및 확률 표시
#         cv2.putText(frame, label, (x1, y1 - 10),
#                     cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

#     # 화면에 출력
#     cv2.imshow("YOLO", frame)

#     # Q 키 누르면 종료
#     if cv2.waitKey(1) & 0xFF == ord('q'):
#         break

# # 종료 처리
# cap.release()
# cv2.destroyAllWindows()
# print("예측 종료")
#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# ROS2 dual‑camera YOLO integration

## yolo\_dual\_cam\_node.py

#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
두 대의 웹캠으로 YOLO 추론을 수행하고, 주기적으로 주석(박스) 입힌 이미지를 퍼블리시합니다.
또한 main_node.py가 서비스 요청을 보내면 마지막 프레임을 handoff 토픽으로 1회 퍼블리시하고 노드를 종료합니다.

[핵심 포인트]
1) 카메라 2대: CAMERA_INDEXES로 설정 (예: [0, 2])
2) rqt용 스트림: /cam0/yolo_image, /cam1/yolo_image 로 계속 퍼블리시
3) main_node.py 요청(서비스) 시: /handoff/cam0, /handoff/cam1 에 "마지막 프레임" 1회 퍼블리시 (Transient Local: 구독이 늦어도 전달)
4) 퍼블리시 후 깔끔하게 종료

작성: ChatGPT, ROS2 Humble + Python3, Ultralytics YOLOv8
"""

import os
import cv2
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from std_srvs.srv import Trigger
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from threading import Lock

# ===== (필수) Ultralytics YOLO =====
from ultralytics import YOLO


class DualCamYoloNode(Node):
    def __init__(self):
        super().__init__('webcam_yolo_node')

        # ===============
        # [설정값 한번에]
        # ===============
        # [MODEL]
        # ▶ 모델 경로 (필요 시 주석 해제/수정)
        # self.model_path = "/home/rokey/train_ws/src/runs/detect/yolov8-custom2-improved2/weights/best.pt"  # 예시
        self.model_path = "/home/rokey/turtlebot4_ws/src/mini_project/resource/best.pt"  # 현재 최종
        self.conf_thres = 0.75  # YOLO confidence

        # [CAMERA]
        # ▶ 사용 장치 번호 (예: 노트북 내장=0, 외장=2)
        self.CAMERA_INDEXES = [5, 2]  # 필요 시 [1, 2] 등으로 변경
        self.frame_width = None  # None이면 원본
        self.frame_height = None
        self.target_fps = 20.0  # 퍼블리시 주기(FPS)

        # [TOPIC]
        # ▶ rqt용 실시간 스트림 토픽 (변경 가능)
        self.OUT_TOPICS = [
            '/cam0/yolo_image',
            '/cam1/yolo_image',
        ]
        # ▶ main_node.py로 1회 전달할 handoff 토픽 (변경 가능)
        self.HANDOFF_TOPICS = [
            '/handoff/cam0',
            '/handoff/cam1',
        ]

        # ===============
        # [내부 상태]
        # ===============
        self.bridge = CvBridge()
        self.model = None
        self.caps = []  # 각 카메라용 cv2.VideoCapture
        self.last_frames = [None for _ in self.CAMERA_INDEXES]
        self.last_lock = Lock()
        self.exit_after_handoff = False

        # ===============
        # [퍼블리셔 준비]
        # ===============
        # rqt 실시간 스트림용 (일반 QoS)
        qos_stream = QoSProfile(depth=1)
        qos_stream.reliability = ReliabilityPolicy.RELIABLE
        qos_stream.history = HistoryPolicy.KEEP_LAST

        # handoff 1회 전달용 (지속성: 구독이 늦어도 1회 받은 값 유지)
        qos_handoff = QoSProfile(depth=1)
        qos_handoff.reliability = ReliabilityPolicy.RELIABLE
        qos_handoff.history = HistoryPolicy.KEEP_LAST
        qos_handoff.durability = DurabilityPolicy.TRANSIENT_LOCAL

        self.pub_stream = []
        self.pub_handoff = []
        for i, _ in enumerate(self.CAMERA_INDEXES):
            self.pub_stream.append(
                self.create_publisher(Image, self.OUT_TOPICS[i], qos_stream)
            )
            self.pub_handoff.append(
                self.create_publisher(Image, self.HANDOFF_TOPICS[i], qos_handoff)
            )

        # ===============
        # [서비스 준비]
        # ===============
        # main_node.py가 호출: 마지막 프레임을 handoff 토픽에 1회 퍼블리시 후 종료
        self.srv = self.create_service(Trigger, '/yolo_dual_cam/push_once_and_exit', self.on_push_once_and_exit)

        # ===============
        # [모델/카메라 초기화]
        # ===============
        self._init_model()
        self._init_cameras()

        # ===============
        # [주기 타이머]
        # ===============
        self.timer = self.create_timer(1.0 / self.target_fps, self.on_timer)
        self.get_logger().info('DualCamYoloNode started. Press Ctrl+C to stop.')
        self.get_logger().info(f"Streaming topics: {self.OUT_TOPICS}")
        self.get_logger().info(f"Handoff topics  : {self.HANDOFF_TOPICS}")

    # ------------------------------------------------------------------
    # 초기화: YOLO 모델
    # ------------------------------------------------------------------
    def _init_model(self):
        if not os.path.exists(self.model_path):
            self.get_logger().error(f"YOLO model not found: {self.model_path}")
            raise FileNotFoundError(self.model_path)
        self.model = YOLO(self.model_path)
        self.get_logger().info(f"Loaded YOLO model: {self.model_path}")

    # ------------------------------------------------------------------
    # 초기화: 카메라 2대
    # ------------------------------------------------------------------
    def _init_cameras(self):
        for idx in self.CAMERA_INDEXES:
            cap = cv2.VideoCapture(idx)
            if not cap.isOpened():
                self.get_logger().error(f"Failed to open camera index: {idx}")
                raise RuntimeError(f"Camera open failed: {idx}")
            # 옵션: 크기 지정 (None이면 원본 유지)
            if self.frame_width and self.frame_height:
                cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.frame_width)
                cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.frame_height)
            cap.set(cv2.CAP_PROP_FPS, self.target_fps)
            self.caps.append(cap)
        self.get_logger().info(f"Opened cameras: {self.CAMERA_INDEXES}")

    # ------------------------------------------------------------------
    # 타이머 콜백: 각 카메라에서 프레임 → YOLO → 박스 그려서 퍼블리시
    # ------------------------------------------------------------------
    def on_timer(self):
        # 종료 플래그가 이미 서면 타이머 종료
        if self.exit_after_handoff:
            self.get_logger().info('Exit flag set; stopping timer and shutting down...')
            self.timer.cancel()
            # 약간의 여유를 준 뒤 shutdown
            rclpy.shutdown()
            return

        for cam_i, cap in enumerate(self.caps):
            ok, frame = cap.read()
            if not ok:
                self.get_logger().warning(f"Camera {self.CAMERA_INDEXES[cam_i]} read failed")
                continue

            # YOLO 추론 (ultralytics: numpy 배열 입력 가능)
            results = self.model.predict(source=frame, conf=self.conf_thres, verbose=False)
            result = results[0]
            boxes = result.boxes
            names = result.names

            # 박스 그리기 (OpenCV)
            annotated = frame.copy()
            if boxes is not None:
                for box in boxes:
                    cls_id = int(box.cls[0])
                    conf = float(box.conf[0]) * 100.0
                    x1, y1, x2, y2 = map(int, box.xyxy[0])
                    label = f"{names[cls_id]} {conf:.1f}%"
                    cv2.rectangle(annotated, (x1, y1), (x2, y2), (0, 255, 0), 2)
                    cv2.putText(annotated, label, (x1, y1 - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

            # 최근 프레임 보관 (handoff 용)
            with self.last_lock:
                self.last_frames[cam_i] = annotated.copy()

            # rqt용 스트림 퍼블리시
            msg = self.bridge.cv2_to_imgmsg(annotated, encoding='bgr8')
            self.pub_stream[cam_i].publish(msg)

    # ------------------------------------------------------------------
    # 서비스: 마지막 프레임 1회 handoff 퍼블리시 후 종료
    # ------------------------------------------------------------------
    def on_push_once_and_exit(self, request, response):  # Trigger.srv
        self.get_logger().info('Service called: push_once_and_exit')
        # 마지막 프레임을 handoff 토픽으로 퍼블리시
        with self.last_lock:
            for cam_i, frame in enumerate(self.last_frames):
                if frame is None:
                    continue
                msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
                self.pub_handoff[cam_i].publish(msg)
        # 퍼블리시가 네트워크로 흘러가도록 잠깐 대기
        time.sleep(0.2)
        # 종료 플래그 ON
        self.exit_after_handoff = True
        response.success = True
        response.message = 'Published last frames to handoff topics; exiting node.'
        return response

    # ------------------------------------------------------------------
    # 종료 처리
    # ------------------------------------------------------------------
    def destroy_node(self):
        for cap in self.caps:
            try:
                cap.release()
            except Exception:
                pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = DualCamYoloNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()