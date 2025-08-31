import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, QoSReliabilityPolicy

from sensor_msgs.msg import Image, CameraInfo, CompressedImage
from geometry_msgs.msg import PointStamped
from std_msgs.msg import String ,Bool
from std_srvs.srv import Trigger

from cv_bridge import CvBridge
from ultralytics import YOLO

import numpy as np
import cv2
import time
import threading
from collections import namedtuple
from message_filters import Subscriber, ApproximateTimeSynchronizer

Pair = namedtuple("Pair", "rgb depth stamp frame_id")


class YoloPerson(Node):
    def __init__(self):
        super().__init__('detect_to_thing')

        # === Internal state ===
        self.bridge = CvBridge()
        self.K = None
        self.rgb_image = None
        self.depth_image = None
        self.camera_frame = None
        self.shutdown_requested = False
        self.logged_intrinsics = False
        self.should_infer = False
        self._pair_lock = threading.Lock()
        self._latest_pair = None
        self.infer_lock = threading.Lock()
        self.last_processed_stamp = None
        self.person_detect = False
        self.label = None
        self.processing_done_event = threading.Event()  # Event 객체
        self.response_event = threading.Event()  # Response 이벤트
        self.response = None
        # === Load YOLO model ===
        self.model = YOLO(
            "/home/rokey/Downloads/detect/web_8n_batch32/weights/best.pt"
        )
        self.model.to('cuda')
        self.get_logger().info("YOLOv8 model loaded.")

        # === Publishers ===

        self.person_point_cam_pub = self.create_publisher(PointStamped, '/robot9/point_camera', 10)

        # === Subscriptions ===
        self.srv_painting = self.create_service(Trigger, '/robot9/painting', self.callback_painting)
        self.sub_gift_start = self.create_subscription(Bool,'/robot9/gift_start',self.callback_gift_start,10)
        self.sub_person = self.create_subscription(Bool,'/robot9/person',self.callback_person,10)
        self.put_giftshop = self.create_publisher(Bool,'/robot9/gift_shop',10)
        # self.put_painting = self.create_publisher(String,'/robot9/painting',10)
        # self.sub_painting = self.create_subscription(Bool,'/robot9/paint_check',self.callback_painting,10)

        qos_profile = QoSProfile(depth=2)
        qos_profile.reliability = QoSReliabilityPolicy.BEST_EFFORT

        self.create_subscription(CameraInfo, '/robot9/oakd/rgb/camera_info', self.camera_info_callback, 10)

        self.rgb_sub = Subscriber(self, CompressedImage, '/robot9/oakd/rgb/image_raw/compressed', qos_profile=qos_profile)
        self.depth_sub = Subscriber(self, Image, '/robot9/oakd/stereo/image_raw', qos_profile=qos_profile)
        

        self.ts = ApproximateTimeSynchronizer([self.rgb_sub, self.depth_sub], queue_size=10, slop=0.4)
        self.ts.registerCallback(self.synced_rgb_depth_cb)

        # === Threads ===
        self.infer_thread = threading.Thread(target=self.run_inference_thread, daemon=True)
        self.infer_thread.start()

        self.display_frame = None
        self.display_thread = threading.Thread(target=self.display_loop, daemon=True)
        self.display_thread.start()

    # ---------------------------
    # Callbacks & processing
    # ---------------------------
    def camera_info_callback(self, msg):
        self.K = np.array(msg.k).reshape(3, 3)
        if not self.logged_intrinsics:
            self.get_logger().info("Camera intrinsics received")
            self.logged_intrinsics = True

    def synced_rgb_depth_cb(self, rgb_msg: CompressedImage, depth_msg: Image):
        try:
            # Decode RGB
            np_arr = np.frombuffer(rgb_msg.data, np.uint8)
            rgb = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

            # Decode depth
            depth_raw = self.bridge.imgmsg_to_cv2(depth_msg, "passthrough")
            if depth_msg.encoding == "16UC1":
                depth_raw = depth_raw.astype(np.float32) / 1000.0  # mm -> m
            elif depth_msg.encoding == "32FC1":
                pass
            else:
                raise ValueError(f"Unexpected encoding: {depth_msg.encoding}")

            # Crop depth to align with RGB FOV
            h, w = depth_raw.shape
            crop_x = int(0.26 * w / 2) * 2
            crop_y = int(0.18 * h / 2) * 2
            depth_crop = depth_raw[crop_y:h - crop_y, crop_x:w - crop_x]
            depth_aligned = cv2.resize(depth_crop, (w, h), cv2.INTER_NEAREST)

            with self._pair_lock:
                self._latest_pair = Pair(
                    rgb=rgb,
                    depth=depth_aligned,
                    stamp=rgb_msg.header.stamp,
                    frame_id=rgb_msg.header.frame_id or "oakd_rgb_frame"
                )

        except Exception as e:
            self.get_logger().error(f"Sync decode failed: {e}")
    def callback_painting(self, request, response):
        self.get_logger().info("Start processing painting request")
        
        # 이미지 처리
        self.handle_painting_request(response)
        
        # 응답 반환
        return response

    def handle_painting_request(self, response):
        self.should_infer = True

        # 시간 제한을 두고 기다리도록 함 (예: 100초)
        timeout = 100  # timeout in seconds
        start_time = time.time()

        # self.label이 pice1, pice2, pice3 중 하나가 될 때까지 확인
        while self.label not in ("pice 1", "pice2", "pice3"):
            self.get_logger().info(f"{self.label}")
            if time.time() - start_time > timeout:
                self.get_logger().warn("Timeout reached, no valid label detected.")
                response.success = False
                break

            # 반복적으로 인식을 위해 잠시 대기
            time.sleep(0.1)
            if self.label in ("pice 1", "pice2", "pice3"):
                break

        # label이 pice1, pice2, pice3 중 하나가 되면
        response.success = True
        self.label = None  # label 초기화 (필요한 경우)
        self.should_infer = False  # inference을 중지
        self.get_logger().info("response")




    def callback_person(self,msg: Bool):
        self.should_infer = msg.data
        if msg.data:
            self.person_detect = True
        else:
            self.person_detect = False


    def callback_gift_start(self,msg: Bool):
        self.should_infer = msg.data
        self.put_giftshop.publish(msg)


        
    # ---------------------------
    # Inference loop
    # ---------------------------
    def run_inference_thread(self):
        while not self.shutdown_requested:
            if self.should_infer:
                self.process_frame()
            time.sleep(0.2)

    def process_frame(self):
        if self.K is None:
            return
        if not self.infer_lock.acquire(blocking=False):
            return

        try:
            with self._pair_lock:
                if self._latest_pair is None:
                    return
                pair = self._latest_pair
                # self._latest_pair = None

            rgb = pair.rgb.copy()
            depth = pair.depth.copy()

            results = self.model.predict(rgb, conf=0.7, verbose=False)[0]
            frame = rgb.copy()

            any_main = False
            H, W = depth.shape[:2]

            for det in results.boxes:
                cls = int(det.cls[0])
                self.label = self.model.names[cls].lower()
                conf = float(det.conf[0])
                x1, y1, x2, y2 = map(int, det.xyxy[0].tolist())

                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.putText(frame, f"{self.label} {conf:.2f}", (x1, max(0, y1-5)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
                self.get_logger().info(f"{self.label}")
                # msg = String()
                # msg.data = self.label
                # self.put_painting.publish(msg)
                
                if (self.label == 'person' and self.person_detect) or self.label == 'gift_data':
                    u = (x1 + x2) // 2
                    v = (y1 + y2) // 2
                    if not (0 <= u < W and 0 <= v < H):
                        continue
                    z = float(depth[v, u])
                    if z <= 0.0 or np.isnan(z):
                        continue
                    fx, fy = self.K[0, 0], self.K[1, 1]
                    cx, cy = self.K[0, 2], self.K[1, 2]
                    X = (u - cx) * z / fx
                    Y = (v - cy) * z / fy

                    pt = PointStamped()
                    pt.header.frame_id = pair.frame_id
                    pt.header.stamp = pair.stamp
                    pt.point.x, pt.point.y, pt.point.z = X, Y, z
                    self.person_point_cam_pub.publish(pt)
                    self.get_logger().info(f"{self.label}")







            self.display_frame = frame
            self.last_processed_stamp = pair.stamp
            # self.get_logger().info("Camera intrinsics received")


        finally:
            self.infer_lock.release()
            # self.processing_done_event.set()

    # ---------------------------
    # Display loop
    # ---------------------------
    def display_loop(self):
        while not self.shutdown_requested:
            if self.display_frame is not None:
                cv2.imshow('yolo', self.display_frame)
                key = cv2.waitKey(1) & 0xFF
                if key == 27:  # ESC
                    self.shutdown_requested = True
                    break
            else:
                time.sleep(0.01)
        cv2.destroyAllWindows()


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
