import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, QoSReliabilityPolicy

from sensor_msgs.msg import Image, CameraInfo, CompressedImage

from cv_bridge import CvBridge
from ultralytics import YOLO

import numpy as np
import cv2
import time
import threading
from collections import namedtuple
from message_filters import Subscriber, ApproximateTimeSynchronizer

from std_msgs.msg import String,Bool
from geometry_msgs.msg import PointStamped, PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from tf2_ros import Buffer, TransformListener
import tf2_geometry_msgs  
from turtlebot4_navigation.turtlebot4_navigator import TurtleBot4Directions, TurtleBot4Navigator

from rclpy.executors import SingleThreadedExecutor

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
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.close_enough_distance = 1.0

        self._pair_lock = threading.Lock()
        self._latest_pair = None
        self.infer_lock = threading.Lock()
        self.last_processed_stamp = None
        self.latest_map_point = None
        self.goal_handle = None
        self.block_goal_updates = True
        self.result_bottle = False

        self.close_distance_hit_count = 0
        self.last_feedback_log_time = 0
        self.pose = PoseStamped()

        # === Load YOLO model ===
        self.model = YOLO(
            "/home/rokey/turtlebot4_ws/src/training/runs/detect/yolov8-turtlebot4-custom2/weights/best.pt"
        )
        self.model.to('cuda')
        self.get_logger().info("YOLOv8 model loaded.")

        # === Publishers ===
        self.main_pub = self.create_publisher(String, '/robot8/which_hand', 10)
        self.person_point_cam_pub = self.create_publisher(PointStamped, '/robot8/point_camera', 10)
        self.action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # === Subscriptions ===
        qos_profile = QoSProfile(depth=2)
        qos_profile.reliability = QoSReliabilityPolicy.BEST_EFFORT

        self.create_subscription(CameraInfo, '/robot8/oakd/rgb/camera_info', self.camera_info_callback, 10)

        self.rgb_sub = Subscriber(self, CompressedImage, '/robot8/oakd/rgb/image_raw/compressed', qos_profile=qos_profile)
        self.depth_sub = Subscriber(self, Image, '/robot8/oakd/stereo/image_raw', qos_profile=qos_profile)

        self.ts = ApproximateTimeSynchronizer([self.rgb_sub, self.depth_sub], queue_size=10, slop=0.3)
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

    # ---------------------------
    # Inference loop
    # ---------------------------
    def run_inference_thread(self):
        while not self.shutdown_requested:
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
                label = self.model.names[cls].lower()
                conf = float(det.conf[0])
                x1, y1, x2, y2 = map(int, det.xyxy[0].tolist())

                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.putText(frame, f"{label} {conf:.2f}", (x1, max(0, y1-5)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

                # Publish gesture
                if label in ("one", "five"):
                    self.main_pub.publish(String(data=label))
                    self.get_logger().info(f"{label}")

                    any_main = True

                # Publish 3D point
                if label in ("car", "bottle"):
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
                    # self.person_point_cam_pub.publish(pt)
                    self.get_logger().info(f"{label}")

                    pt.point.z = pt.point.z - 0.3 # 거리 조절
                    self.get_logger().info(f"Detected person at map: ({pt.point.x:.2f}, {pt.point.y:.2f})")

                    self.pt = pt
                    # pt_map = self.tf_buffer.transform(pt, 'map', timeout=rclpy.duration.Duration(seconds=0.7))
                    # self.latest_map_point = pt_map

                    # Don't send more goals if we're already close
                    if self.block_goal_updates:
                        self.get_logger().info(f"Within ({self.close_enough_distance}) meter — skipping further goal updates.")
                        

                    # self.get_logger().info(f"Detected person at map: ({pt_map.point.x:.2f}, {pt_map.point.y:.2f})")

                    if self.goal_handle:
                        self.get_logger().info("Canceling previous goal...")
                        self.goal_handle.cancel_goal_async()

                    self.send_goal()

                    

                    # self.target_result = 'None'  # 연속적인 tracking





            if not any_main:
                self.main_pub.publish(String(data="none"))

            self.display_frame = frame
            self.last_processed_stamp = pair.stamp
            # self.get_logger().info("Camera intrinsics received")


        finally:
            self.infer_lock.release()

    def send_goal(self):
        try:
            if self.tf_buffer.can_transform('map', self.pt.header.frame_id, rclpy.time.Time()):
                self.latest_map_point = self.tf_buffer.transform(self.pt, 'map', timeout=rclpy.duration.Duration(seconds=0.5))
            else:
                self.get_logger().warn(f"Transform from {self.add_on_set_parameters_callbackpt.header.frame_id} to 'map' not available yet.")
        except Exception as e:
            self.get_logger().error(f"Transform error: {e}")

        # pt_map = self.tf_buffer.transform(self.pt, 'map', timeout=rclpy.duration.Duration(seconds=0.5))
        # self.latest_map_point = pt_map

        self.pose.header.frame_id = 'map'
        self.pose.header.stamp = self.get_clock().now().to_msg()
        self.pose.pose.position.x = self.latest_map_point.point.x
        self.pose.pose.position.y = self.latest_map_point.point.y
        self.pose.pose.orientation.w = 1.0

        # pose.pose.position.x = -2.0
        # pose.pose.position.y = 1.0

        goal = NavigateToPose.Goal()
        goal.pose = self.pose

        self.get_logger().info(f"Sending goal to: ({self.pose.pose.position.x:.2f}, {self.pose.pose.position.y:.2f})")
        self.action_client.wait_for_server()
        self._send_goal_future = self.action_client.send_goal_async(goal, feedback_callback=self.feedback_callback)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.current_distance = feedback.distance_remaining

        # # Require 3 close readings to trigger the lock
        # if self.current_distance is not None and self.current_distance < self.close_enough_distance:
        #     self.close_distance_hit_count += 1
        # else:
        #     self.close_distance_hit_count = 0

        # if self.close_distance_hit_count >= 3 and not self.block_goal_updates:
        #     self.block_goal_updates = True
        #     self.get_logger().info("Confirmed: within 1 meter — blocking further goal updates.")

        now = time.time()
        if now - self.last_feedback_log_time > 1.0:
            self.get_logger().info(f"Distance remaining: {self.current_distance:.2f} m")
            self.last_feedback_log_time = now

    def goal_response_callback(self, future):
        self.goal_handle = future.result()
        if not self.goal_handle.accepted:
            self.get_logger().warn("Goal was rejected.")
            return
        self.get_logger().info("Goal accepted.")
        self._get_result_future = self.goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.goal_result_callback)

    def goal_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f"Goal finished with result code: {future.result().status}")
        # if self.result_bottle:
        #     time.sleep(1)
        #     self.pub_done.publish(Bool(data=True))
        #     self.get_logger().info("pub_True")
        #     self.result_bottle = False
            # self.pose = PoseStamped()



        self.goal_handle = None


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
