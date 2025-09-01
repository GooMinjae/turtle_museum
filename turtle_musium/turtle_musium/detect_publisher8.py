import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, QoSReliabilityPolicy

from sensor_msgs.msg import Image, CameraInfo, CompressedImage
from geometry_msgs.msg import PointStamped
from std_msgs.msg import String, Bool

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
        self.inference_active = False

        self._pair_lock = threading.Lock()
        self._latest_pair = None
        self.infer_lock = threading.Lock()
        self.last_processed_stamp = None

        # === Load YOLO model ===
        self.model = YOLO(
            "/home/rokey/turtlebot4_ws/src/turtle_musium/resource/only_people_8n_batch32.pt"
        )
        self.model.to('cuda')
        self.get_logger().info("YOLOv8 model loaded.")

        # === Publishers ===
        self.main_pub = self.create_publisher(String, '/robot8/which_hand', 10)
        self.person_point_cam_pub = self.create_publisher(PointStamped, '/robot8/point_camera', 10)

        # === Subscriptions ===
        qos_profile = QoSProfile(depth=2)
        qos_profile.reliability = QoSReliabilityPolicy.BEST_EFFORT

        self.create_subscription(Bool, '/robot8/people_check', self.people_check_cb, 10)
        self.create_subscription(CameraInfo, '/robot8/oakd/rgb/camera_info', self.camera_info_callback, 10)

        self.rgb_sub = Subscriber(self, CompressedImage, '/robot8/oakd/rgb/image_raw/compressed', qos_profile=qos_profile)
        self.depth_sub = Subscriber(self, Image, '/robot8/oakd/stereo/image_raw', qos_profile=qos_profile)

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
    def people_check_cb(self, msg: Bool):
        if msg.data:
            self.get_logger().info("People check True -> start inference")
            self.inference_active = True
        else:
            self.inference_active = False

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

    # ---------------------------
    # Inference loop
    # ---------------------------
    def run_inference_thread(self):
        while not self.shutdown_requested:
            self.process_frame()
            time.sleep(0.2)

    def process_frame(self):
        if not self.inference_active:
            return
        if self.K is None:
            return
        if not self.infer_lock.acquire(blocking=False):
            return

        try:
            with self._pair_lock:
                if self._latest_pair is None:
                    return
                pair = self._latest_pair

            rgb = pair.rgb.copy()
            depth = pair.depth.copy()

            # ★ 변경: predict -> track (ID 사용)
            results = self.model.track(
                rgb, conf=0.7, verbose=False, persist=True, tracker='bytetrack.yaml'
            )[0]

            frame = rgb.copy()
            any_main = False
            H, W = depth.shape[:2]

            persons = []  # (track_id, conf, (x1,y1,x2,y2), (cx,cy))

            for det in results.boxes:
                cls = int(det.cls[0])
                label = self.model.names[cls].lower()
                conf = float(det.conf[0])
                x1, y1, x2, y2 = map(int, det.xyxy[0].tolist())

                # 시각화 (원래 코드 유지)
                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.putText(frame, f"{label} {conf:.2f}", (x1, max(0, y1-5)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

                # 제스처 퍼블리시 (원래 로직 유지)
                if label in ("one", "five"):
                    self.main_pub.publish(String(data=label))
                    any_main = True

                # ★ 변경: person만 추적 candidate로 수집 (ID 포함)
                if label == "person":
                    # det.id가 None일 수 있어 안전 처리
                    tid = int(det.id[0]) if getattr(det, 'id', None) is not None else -1
                    cx = (x1 + x2) // 2
                    cy = (y1 + y2) // 2
                    persons.append((tid, conf, (x1, y1, x2, y2), (cx, cy)))

            # ★ 변경: 타겟 ID가 없으면(초기) conf가 가장 높은 사람으로 설정
            if self.target_id is None:
                if persons:
                    best = max(persons, key=lambda p: p[1])  # conf 최대
                    self.target_id = best[0] if best[0] != -1 else None
                    self.lost_count = 0

            # ★ 변경: 현재 타겟 ID와 매칭되는 detection 찾기
            target_det = None
            if self.target_id is not None:
                for tid, conf, bbox, center in persons:
                    if tid == self.target_id:
                        target_det = (tid, conf, bbox, center)
                        break

            # ★ 변경: 타겟이 보이면 그 사람만 퍼블리시, 아니면 분실 카운트 증가
            if target_det is not None:
                self.lost_count = 0
                _, _, (x1, y1, x2, y2), (u, v) = target_det

                # 표시용 점
                cv2.circle(frame, (u, v), 4, (0, 255, 0), -1)

                # 3D 계산 및 퍼블리시 (원래 코드 유지)
                if 0 <= u < W and 0 <= v < H:
                    z = float(depth[v, u])
                    if z > 0.0 and not np.isnan(z):
                        fx, fy = self.K[0, 0], self.K[1, 1]
                        cx0, cy0 = self.K[0, 2], self.K[1, 2]
                        X = (u - cx0) * z / fx
                        Y = (v - cy0) * z / fy

                        pt = PointStamped()
                        pt.header.frame_id = pair.frame_id
                        pt.header.stamp = pair.stamp
                        pt.point.x, pt.point.y, pt.point.z = X, Y, z
                        self.person_point_cam_pub.publish(pt)
            else:
                # 타겟 분실
                self.lost_count += 1
                if self.lost_count > self.MAX_LOST:
                    # 너무 오래 못 보면 다시 “conf 최고”로 재획득
                    if persons:
                        best = max(persons, key=lambda p: p[1])
                        self.target_id = best[0] if best[0] != -1 else None
                        self.lost_count = 0
                    else:
                        self.target_id = None  # 화면에 아무도 없음

            if not any_main:
                self.main_pub.publish(String(data="none"))

            self.display_frame = frame
            self.last_processed_stamp = pair.stamp

        finally:
            self.infer_lock.release()

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
