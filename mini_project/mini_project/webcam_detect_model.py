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
#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
두 대의 웹캠으로 YOLO 추론을 수행하고, 주기적으로 주석(박스) 입힌 이미지를 퍼블리시합니다.
또한 main_node.py가 서비스 요청을 보내면 마지막 프레임을 handoff 토픽으로 1회 퍼블리시하고,
가장 최근 유효 탐지가 있었던 카메라를 'living_room'/'kitchen' 중 하나로 응답한 뒤 종료합니다.
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

from ultralytics import YOLO


class DualCamYoloNode(Node):
    def __init__(self):
        super().__init__('dual_cam_yolo_node')

        # ===============
        # [설정값 한번에]
        # ===============
        # [MODEL]
        self.model_path = "/home/rokey/turtlebot4_ws/src/mini_project/resource/best.pt"
        self.conf_thres = 0.70
        self.imgsz = 640
        self.device = None  # 'cuda:0' 등

        # [CAMERA]
        self.CAMERA_INDEXES = [5, 2]  # cam0=5(living_room), cam1=2(kitchen)
        self.frame_width = None
        self.frame_height = None
        self.target_fps = 20.0

        # [TOPIC]
        self.OUT_TOPICS = ['/cam0/yolo_image', '/cam1/yolo_image']
        self.HANDOFF_TOPICS = ['/handoff/cam0', '/handoff/cam1']

        # [오검출 완화(옵션)]
        self.allowed_classes = None   # 예: ['person'], None이면 제한 없음
        self.min_area_ratio = 0.01
        self.n_consec = 2

        # ===============
        # [내부 상태]
        # ===============
        self.bridge = CvBridge()
        self.model = None
        self.caps = []
        self.last_frames = [None for _ in self.CAMERA_INDEXES]
        self.last_lock = Lock()
        self.exit_after_handoff = False

        self.pos_counter = [0, 0]
        self.last_valid = [False, False]
        self.last_best_conf = [0.0, 0.0]
        self.last_detect_time = [0.0, 0.0]

        # [시작/탐색 로그 제어]
        self._search_logged = False
        self.get_logger().info('[시작] DualCamYoloNode를 시작합니다.')

        # ===============
        # [퍼블리셔 준비]
        # ===============
        qos_stream = QoSProfile(depth=1)
        qos_stream.reliability = ReliabilityPolicy.BEST_EFFORT
        qos_stream.history = HistoryPolicy.KEEP_LAST

        qos_handoff = QoSProfile(depth=1)
        qos_handoff.reliability = ReliabilityPolicy.RELIABLE
        qos_handoff.history = HistoryPolicy.KEEP_LAST
        qos_handoff.durability = DurabilityPolicy.TRANSIENT_LOCAL

        self.pub_stream = []
        self.pub_handoff = []
        for i, cam_dev in enumerate(self.CAMERA_INDEXES):
            self.pub_stream.append(self.create_publisher(Image, self.OUT_TOPICS[i], qos_stream))
            self.pub_handoff.append(self.create_publisher(Image, self.HANDOFF_TOPICS[i], qos_handoff))

        # ===============
        # [서비스 준비]
        # ===============
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
        self.get_logger().info(f"Streaming topics: {self.OUT_TOPICS}")
        self.get_logger().info(f"Handoff topics  : {self.HANDOFF_TOPICS}")

    def _init_model(self):
        if not os.path.exists(self.model_path):
            self.get_logger().error(f"YOLO model not found: {self.model_path}")
            raise FileNotFoundError(self.model_path)
        self.model = YOLO(self.model_path)
        if self.device:
            try:
                self.model.to(self.device)
                self.get_logger().info(f"YOLO device set to: {self.device}")
            except Exception as e:
                self.get_logger().warn(f"Failed to move model to {self.device}: {e}")
        self.get_logger().info(f"Loaded YOLO model: {self.model_path}")

    def _init_cameras(self):
        for i, cam_dev in enumerate(self.CAMERA_INDEXES):
            cap = cv2.VideoCapture(cam_dev)
            if not cap.isOpened():
                self.get_logger().error(f"Failed to open camera index: {cam_dev}")
                raise RuntimeError(f"Camera open failed: {cam_dev}")
            if self.frame_width and self.frame_height:
                cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.frame_width)
                cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.frame_height)
            cap.set(cv2.CAP_PROP_FPS, self.target_fps)
            try:
                cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
            except Exception:
                pass
            try:
                cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
            except Exception:
                pass
            self.caps.append(cap)

    def _evaluate_detection(self, result, frame_shape):
        boxes = result.boxes
        names = result.names
        if boxes is None or len(boxes) == 0:
            return False, 0.0
        h, w = frame_shape[:2]
        min_area = self.min_area_ratio * (w * h)
        best_conf = 0.0
        any_valid = False
        for box in boxes:
            cls_id = int(box.cls[0])
            conf = float(box.conf[0])
            x1, y1, x2, y2 = map(int, box.xyxy[0])
            area = max(0, x2 - x1) * max(0, y2 - y1)
            if conf < self.conf_thres:
                continue
            if area < min_area:
                continue
            if self.allowed_classes is not None:
                cls_name = names[cls_id]
                if cls_name not in self.allowed_classes:
                    continue
            any_valid = True
            if conf > best_conf:
                best_conf = conf
        return any_valid, best_conf

    def on_timer(self):
        if self.exit_after_handoff:
            self.timer.cancel()
            rclpy.shutdown()
            return

        if not self._search_logged:
            self.get_logger().info('[탐색] 카메라에서 사람을 탐색 중...')
            self._search_logged = True

        now = time.time()
        for cam_i, cap in enumerate(self.caps):
            ok, frame = cap.read()
            if not ok:
                self.get_logger().warning(f"Camera {self.CAMERA_INDEXES[cam_i]} read failed (cam{cam_i})")
                continue

            results = self.model.predict(
                source=frame, conf=self.conf_thres, imgsz=self.imgsz,
                device=self.device if self.device else None, verbose=False
            )
            result = results[0]

            has_valid, best_conf = self._evaluate_detection(result, frame.shape)
            if has_valid:
                self.pos_counter[cam_i] = min(self.pos_counter[cam_i] + 1, self.n_consec)
            else:
                self.pos_counter[cam_i] = max(self.pos_counter[cam_i] - 1, 0)

            self.last_valid[cam_i] = (self.pos_counter[cam_i] >= self.n_consec)
            self.last_best_conf[cam_i] = best_conf if self.last_valid[cam_i] else 0.0
            if self.last_valid[cam_i]:
                self.last_detect_time[cam_i] = now

            annotated = frame
            boxes = result.boxes
            names = result.names
            if boxes is not None:
                for box in boxes:
                    cls_id = int(box.cls[0])
                    conf = float(box.conf[0]) * 100.0
                    x1, y1, x2, y2 = map(int, box.xyxy[0])
                    if self.allowed_classes is not None and names[cls_id] not in self.allowed_classes:
                        continue
                    label = f"{names[cls_id]} {conf:.1f}%"
                    cv2.rectangle(annotated, (x1, y1), (x2, y2), (0, 255, 0), 2)
                    cv2.putText(annotated, label, (x1, y1 - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

            with self.last_lock:
                self.last_frames[cam_i] = annotated.copy()

            msg = self.bridge.cv2_to_imgmsg(annotated, encoding='bgr8')
            self.pub_stream[cam_i].publish(msg)

    def on_push_once_and_exit(self, request, response):
        # 마지막 프레임 1회 handoff 퍼블리시
        with self.last_lock:
            for cam_i, frame in enumerate(self.last_frames):
                if frame is None:
                    continue
                msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
                self.pub_handoff[cam_i].publish(msg)

        # living_room / kitchen 결정 (기존 로직 유지)
        cam_name = ["living_room", "kitchen"]
        def score(i):
            cur_valid = 1 if self.last_valid[i] else 0
            return (cur_valid, self.last_detect_time[i], self.last_best_conf[i])
        choice = 0
        if score(1) > score(0):
            choice = 1

        response.success = True
        response.message = cam_name[choice]

        # == 요청하신 한국어 로그 ==
        self.get_logger().info(f'[결과] 사람이 {response.message}에 있습니다')
        self.get_logger().info('[끝] 노드를 종료합니다.')

        time.sleep(0.2)
        self.exit_after_handoff = True
        return response

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
