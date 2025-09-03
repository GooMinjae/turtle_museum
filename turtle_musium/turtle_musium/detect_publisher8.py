#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

from sensor_msgs.msg import Image, CameraInfo, CompressedImage
from geometry_msgs.msg import PointStamped
from std_msgs.msg import String, Bool, Int32

from cv_bridge import CvBridge
from ultralytics import YOLO
from std_srvs.srv import Trigger

import numpy as np
import cv2
import time
import threading

class YoloPerson(Node):
    def __init__(self):
        super().__init__('yolo_person_detector')

        # QoS
        self.qos_sensor = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.camera_frame_id = None
        # ---- Topics (기존 유지) ----
        self.topic_rgb = '/robot9/oakd/rgb/image_raw/compressed'
        self.topic_depth = '/robot9/oakd/stereo/image_raw'
        self.topic_caminfo = '/robot9/oakd/rgb/camera_info'
        self.pub_point_topic = '/robot9/point_camera'
        self.pub_frame_topic = '/robot9/frame'
        # 신규 추가(탐지된 사람 수)
        self.pub_people_cnt_topic = '/robot9/audience_count_from_detector'

        # ---- Publishers ----
        self.pub_label = self.create_publisher(String,'/robot9/painting',10)
        
        self.person_point_cam_pub = self.create_publisher(PointStamped, self.pub_point_topic, 10)
        self.person_pub = self.create_publisher(Image, self.pub_frame_topic, 10)
        # self.pub_people_count = self.create_publisher(Int32, self.pub_people_cnt_topic, 10)

        # ---- Subscribers ----
        self.bridge = CvBridge()
        self.last_rgb = None
        self.last_rgb_stamp = None
        self.last_depth = None
        self.last_depth_stamp = None
        self.K = None
        self.sub_gift_start = self.create_subscription(Bool,'/robot9/gift_start',self.callback_giftshop,10)
        self.sub_person = self.create_subscription(Bool,'/robot9/person',self.callback_person,10)
        self.sub_painting = self.create_subscription(Bool,'/robot9/paint_check',self.callback_painting,10)

        self.sub_rgb = self.create_subscription(
            CompressedImage, self.topic_rgb, self.cb_rgb, self.qos_sensor
        )
        self.sub_depth = self.create_subscription(
            Image, self.topic_depth, self.cb_depth, self.qos_sensor
        )
        self.sub_caminfo = self.create_subscription(
            CameraInfo, self.topic_caminfo, self.cb_caminfo, 10
        )

        # ---- YOLO (모델 단일 로드) ----
        # 기존 경로 보존: only_people_8n_batch32.pt (사람 전용)
        self.model = YOLO('/home/rokey/turtlebot4_ws/src/turtle_musium/resource/only_people_8n_batch32.pt')
        self.conf = 0.7

        # 처리 스레드      
        self.gift_data = []
        self.gift_name = ["moo", "pinga", "haowl", "pingu"]
        self.gift_client = self.create_client(Trigger, '/robot9/gift_data')
        self.lock = threading.Lock()
        self.display_frame = None
        self.last_processed_stamp = None
        self.processing = True
        self.th = threading.Thread(target=self.process_loop, daemon=True)
        self.th.start()

    # === Callbacks ===
    def callback_painting(self,msg: Bool):
        self.should_infer = msg.data

    def callback_giftshop(self, msg: Bool):
        self.gift_data = []

        if not self.gift_client.wait_for_service(timeout_sec=3.0):
            self.get_logger().error('gift_data 서비스 없음')
            return

        req = Trigger.Request()
        future = self.gift_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is None:
            self.get_logger().error('서비스 응답 없음')
            return

        result = future.result()
        if not result.success:
            self.get_logger().error(f"서비스 실패: {result.message}")
            return

        data = result.message.strip()  # "1010"

        # gift_name 길이에 맞춰 보정
        if len(data) < len(self.gift_name):
            data += "0" * (len(self.gift_name) - len(data))
        data = data[:len(self.gift_name)]

        # '1'인 항목만 추가
        for idx, num in enumerate(data):
            if num == "1":
                self.gift_data.append(self.gift_name[idx])

        self.should_infer = msg.data
        self.get_logger().info(f"gift_data={self.gift_data}, should_infer={self.should_infer}")




    def callback_person(self,msg: Bool):
        self.should_infer = msg.data
        if msg.data:
            self.person_detect = True
        else:
            self.person_detect = False


    def cb_caminfo(self, msg: CameraInfo):
        self.camera_frame_id = msg.header.frame_id
        if self.K is None:
            self.K = np.array(msg.k, dtype=np.float32).reshape(3, 3)
            self.get_logger().info(f"Camera K set: {self.K.tolist()}")

    def cb_rgb(self, msg: CompressedImage):
        try:
            frame = self.bridge.compressed_imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().warn(f"RGB decode failed: {e}")
            return
        with self.lock:
            self.last_rgb = frame
            self.last_rgb_stamp = msg.header.stamp

    def cb_depth(self, msg: Image):
        try:
            depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            if depth.dtype != np.uint16 and depth.dtype != np.float32:
                depth = depth.astype(np.uint16)
        except Exception as e:
            self.get_logger().warn(f"Depth decode failed: {e}")
            return
        with self.lock:
            self.last_depth = depth
            self.last_depth_stamp = msg.header.stamp

    def process_loop(self):
        rate = self.create_rate(30)
        while rclpy.ok() and self.processing:
            if self.should_infer:
                try:
                    self.process_frame()
                except Exception as e:
                    self.get_logger().warn(f"process_frame error: {e}")
                rate.sleep()

    def process_frame(self):
        with self.lock:
            if self.last_rgb is None or self.last_depth is None:
                return
            # 타임스탬프 최신 조합 사용
            rgb = self.last_rgb.copy()
            depth = self.last_depth.copy()
            stamp = self.last_rgb_stamp if self.last_rgb_stamp is not None else self.get_clock().now().to_msg()

        # YOLO 추론
        res = self.model.predict(rgb, conf=self.conf, verbose=False)
        if not res:
            return
        results = res[0]

        H, W = rgb.shape[:2]
        frame = rgb.copy()
        person_cnt = 0
        best_main_center = None
        best_main_depth = None

        # 단일 가장 큰 person 박스를 대표로 선택 (기존 동작 가정)
        max_area = 0
        for det in results.boxes:
            cls = int(det.cls[0])
            label = self.model.names[cls].lower()
            conf = float(det.conf[0])
            x1, y1, x2, y2 = map(int, det.xyxy[0].tolist())

            if label == 'person' and self.person_detect:
                # person_cnt += 1
                area = (x2 - x1) * (y2 - y1)
                if area > max_area:
                    max_area = area
                    cx = int((x1 + x2) / 2)
                    cy = int((y1 + y2) / 2)
                    best_main_center = (cx, cy)
            elif label in self.gift_data:
                msg = String()
                msg.data = label
                self.pub_label.publish(msg)
                area = (x2 - x1) * (y2 - y1)
                if area > max_area:
                    max_area = area
                    cx = int((x1 + x2) / 2)
                    cy = int((y1 + y2) / 2)
                    best_main_center = (cx, cy)
                self.should_infer = False
            elif label in ('pice 1', 'pice2', 'pice3'):
                msg = String()
                msg.data = label
                self.pub_label.publish(msg)

            

            # 시각화
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.putText(frame, f"{label} {conf:.2f}", (x1, max(0, y1-5)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

        # 메인 타겟의 3D 포인트(카메라 좌표)
        if best_main_center is not None:
            cx, cy = best_main_center
            # 깊이(mm 또는 m). 일반적으로 depth가 uint16(mm)일 수 있음
            d = float(depth[cy, cx])
            if depth.dtype == np.uint16:
                Z = d / 1000.0
            else:
                Z = d  # 이미 m 단위라고 가정

            if self.K is not None and Z > 0:
                fx, fy = self.K[0, 0], self.K[1, 1]
                cx0, cy0 = self.K[0, 2], self.K[1, 2]
                X = (cx - cx0) * Z / fx
                Y = (cy - cy0) * Z / fy

                pt = PointStamped()
                pt.header.frame_id = self.camera_frame_id or 'oakd_rgb_camera_frame'  # 안전장치
                pt.header.stamp = stamp
                # pt = PointStamped()
                # pt.header.frame_id = 'camera_link'  # 기존 좌표계 유지(카메라 프레임명은 기존 사용 중인 값으로)
                # pt.header.stamp = stamp
                
                pt.point.x = X
                pt.point.y = Y
                pt.point.z = Z
                self.person_point_cam_pub.publish(pt)

        # 시각화 프레임 퍼블리시
        self.person_pub.publish(self.bridge.cv2_to_imgmsg(frame, encoding='bgr8'))

        # 사람 수 퍼블리시(신규)
        # self.pub_people_count.publish(Int32(data=person_cnt))

def main():
    rclpy.init()
    node = YoloPerson()
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

# import rclpy
# from rclpy.node import Node
# from rclpy.executors import MultiThreadedExecutor
# from rclpy.qos import QoSProfile, QoSReliabilityPolicy

# from sensor_msgs.msg import Image, CameraInfo, CompressedImage
# from geometry_msgs.msg import PointStamped
# from std_msgs.msg import String

# from cv_bridge import CvBridge
# from ultralytics import YOLO

# import numpy as np
# import cv2
# import time
# import threading
# from collections import namedtuple
# from message_filters import Subscriber, ApproximateTimeSynchronizer

# Pair = namedtuple("Pair", "rgb depth stamp frame_id")


# class YoloPerson(Node):
#     def __init__(self):
#         super().__init__('detect_to_thing')

#         # === Internal state ===
#         self.bridge = CvBridge()
#         self.K = None
#         self.rgb_image = None
#         self.depth_image = None
#         self.camera_frame = None
#         self.shutdown_requested = False
#         self.logged_intrinsics = False

#         self._pair_lock = threading.Lock()
#         self._latest_pair = None
#         self.infer_lock = threading.Lock()
#         self.last_processed_stamp = None

#         # === Load YOLO model ===
#         self.model = YOLO(
#             "/home/rokey/turtlebot4_ws/src/turtle_musium/resource/only_people_8n_batch32.pt"
#         )
#         self.model.to('cuda')
#         self.get_logger().info("YOLOv8 model loaded.")

#         # === Publishers ===
#         self.main_pub = self.create_publisher(String, '/robot9/which_hand', 10)
#         self.person_point_cam_pub = self.create_publisher(PointStamped, '/robot9/point_camera', 10)

#         # === Subscriptions ===
#         qos_profile = QoSProfile(depth=2)
#         qos_profile.reliability = QoSReliabilityPolicy.BEST_EFFORT

#         self.create_subscription(CameraInfo, '/robot9/oakd/rgb/camera_info', self.camera_info_callback, 10)

#         self.rgb_sub = Subscriber(self, CompressedImage, '/robot9/oakd/rgb/image_raw/compressed', qos_profile=qos_profile)
#         self.depth_sub = Subscriber(self, Image, '/robot9/oakd/stereo/image_raw', qos_profile=qos_profile)

#         self.ts = ApproximateTimeSynchronizer([self.rgb_sub, self.depth_sub], queue_size=10, slop=0.4)
#         self.ts.registerCallback(self.synced_rgb_depth_cb)

#         # === Threads ===
#         self.infer_thread = threading.Thread(target=self.run_inference_thread, daemon=True)
#         self.infer_thread.start()

#         # self.display_frame = None
#         # self.display_thread = threading.Thread(target=self.display_loop, daemon=True)
#         # self.display_thread.start()

#     # ---------------------------
#     # Callbacks & processing
#     # ---------------------------
#     def camera_info_callback(self, msg):
#         self.K = np.array(msg.k).reshape(3, 3)
#         if not self.logged_intrinsics:
#             self.get_logger().info("Camera intrinsics received")
#             self.logged_intrinsics = True

#     def synced_rgb_depth_cb(self, rgb_msg: CompressedImage, depth_msg: Image):
#         try:
#             # Decode RGB
#             np_arr = np.frombuffer(rgb_msg.data, np.uint8)
#             rgb = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

#             # Decode depth
#             depth_raw = self.bridge.imgmsg_to_cv2(depth_msg, "passthrough")
#             if depth_msg.encoding == "16UC1":
#                 depth_raw = depth_raw.astype(np.float32) / 1000.0  # mm -> m
#             elif depth_msg.encoding == "32FC1":
#                 pass
#             else:
#                 raise ValueError(f"Unexpected encoding: {depth_msg.encoding}")

#             # Crop depth to align with RGB FOV
#             h, w = depth_raw.shape
#             crop_x = int(0.26 * w / 2) * 2
#             crop_y = int(0.18 * h / 2) * 2
#             depth_crop = depth_raw[crop_y:h - crop_y, crop_x:w - crop_x]
#             depth_aligned = cv2.resize(depth_crop, (w, h), cv2.INTER_NEAREST)

#             with self._pair_lock:
#                 self._latest_pair = Pair(
#                     rgb=rgb,
#                     depth=depth_aligned,
#                     stamp=rgb_msg.header.stamp,
#                     frame_id=rgb_msg.header.frame_id or "oakd_rgb_frame"
#                 )

#         except Exception as e:
#             self.get_logger().error(f"Sync decode failed: {e}")

#     # ---------------------------
#     # Inference loop
#     # ---------------------------
#     def run_inference_thread(self):
#         while not self.shutdown_requested:
#             self.process_frame()
#             time.sleep(0.2)

#     def process_frame(self):
#         if self.K is None:
#             return
#         if not self.infer_lock.acquire(blocking=False):
#             return

#         try:
#             with self._pair_lock:
#                 if self._latest_pair is None:
#                     return
#                 pair = self._latest_pair
#                 # self._latest_pair = None

#             rgb = pair.rgb.copy()
#             depth = pair.depth.copy()

#             results = self.model.predict(rgb, conf=0.7, verbose=False)[0]
#             frame = rgb.copy()

#             any_main = False
#             H, W = depth.shape[:2]

#             for det in results.boxes:
#                 cls = int(det.cls[0])
#                 label = self.model.names[cls].lower()
#                 conf = float(det.conf[0])
#                 x1, y1, x2, y2 = map(int, det.xyxy[0].tolist())

#                 cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
#                 cv2.putText(frame, f"{label} {conf:.2f}", (x1, max(0, y1-5)),
#                             cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

#                 # Publish gesture
#                 if label in ("one", "five"):
#                     self.main_pub.publish(String(data=label))
#                     self.get_logger().info(f"{label}")

#                     any_main = True

#                 # Publish 3D point
#                 if label in ("car", "bottle"):
#                     u = (x1 + x2) // 2
#                     v = (y1 + y2) // 2
#                     if not (0 <= u < W and 0 <= v < H):
#                         continue
#                     z = float(depth[v, u])
#                     if z <= 0.0 or np.isnan(z):
#                         continue
#                     fx, fy = self.K[0, 0], self.K[1, 1]
#                     cx, cy = self.K[0, 2], self.K[1, 2]
#                     X = (u - cx) * z / fx
#                     Y = (v - cy) * z / fy

#                     pt = PointStamped()
#                     pt.header.frame_id = pair.frame_id
#                     pt.header.stamp = pair.stamp
#                     pt.point.x, pt.point.y, pt.point.z = X, Y, z
#                     self.person_point_cam_pub.publish(pt)
#                     self.get_logger().info(f"{label}")


#             if not any_main:
#                 self.main_pub.publish(String(data="none"))

#             self.display_frame = frame
#             self.last_processed_stamp = pair.stamp
#             # self.get_logger().info("Camera intrinsics received")


#         finally:
#             self.infer_lock.release()

#     # ---------------------------
#     # Display loop
#     # ---------------------------
#     def display_loop(self):
#         while not self.shutdown_requested:
#             if self.display_frame is not None:
#                 cv2.imshow('yolo', self.display_frame)
#                 key = cv2.waitKey(1) & 0xFF
#                 if key == 27:  # ESC
#                     self.shutdown_requested = True
#                     break
#             else:
#                 time.sleep(0.01)
#         cv2.destroyAllWindows()


# def main():
#     rclpy.init()
#     node = YoloPerson()
#     executor = MultiThreadedExecutor(num_threads=3)
#     executor.add_node(node)
#     try:
#         executor.spin()
#     finally:
#         executor.shutdown()
#         node.destroy_node()
#         rclpy.shutdown()


# if __name__ == '__main__':
#     main()
