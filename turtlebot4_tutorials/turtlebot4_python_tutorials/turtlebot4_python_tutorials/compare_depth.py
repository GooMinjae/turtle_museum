import os
import sys
import rclpy
import threading
import numpy as np
from queue import Queue
from rclpy.node import Node
from sensor_msgs.msg import Image , CompressedImage, CameraInfo
from cv_bridge import CvBridge
from ultralytics import YOLO
import cv2
import struct


from collections import deque
import cv2, math, numpy as np

class compare_depth_node(Node):
    def __init__(self):
        super().__init__('compare_depth_node')
        self.bridge = CvBridge()

        # timestamp와 이미지 같이 저장 (deque로 관리)
        self.image_buffer = deque(maxlen=10)  # [(timestamp, img), ...]
        self.depth_buffer = deque(maxlen=10)

        self.x = 0
        self.y = 0
        self.clicked = False

        self.rgb_info = None
        self.depth_info = None

        self.sub_rgb_info = self.create_subscription(
            CameraInfo, '/robot8/oakd/rgb/camera_info', self.rgb_info_cb, 10)
        self.sub_depth_info = self.create_subscription(
            CameraInfo, '/robot8/oakd/stereo/camera_info', self.depth_info_cb, 10)

        self.sub_image = self.create_subscription(
            Image,
            '/robot8/oakd/rgb/image_raw',
            self.image_callback,
            10)

        self.sub_depth = self.create_subscription(
            CompressedImage,
            '/robot8/oakd/stereo/image_raw/compressedDepth',
            # Image,
            # '/robot8/oakd/stereo/image_raw/',
            self.depth_callback,
            10
        )

    def image_callback(self, msg):
        try:
            img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            ts = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            self.image_buffer.append((ts, img))
        except Exception as e:
            self.get_logger().error(f"Image conversion failed: {e}")

    def depth_callback(self, msg):
        # try:
        #     # img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        #     self.get_logger().info(msg.format)
        #     np_arr = np.frombuffer(msg.data, np.uint8)
        #     img = cv2.imdecode(np_arr, cv2.IMREAD_UNCHANGED)

        #     if img is None:
        #         self.get_logger().warn("Depth image decode 실패: imdecode 결과 None")
        #         return

        #     if not self.depth_queue.full():
        #         self.depth_queue.put(img)
        # except Exception as e:
        #     self.get_logger().error(f"Image conversion failed: {e}")
        if msg.format != "16UC1; compressedDepth":
            self.get_logger().warn(f"Unsupported format: {msg.format}")
            return

        try:
            # Step 1: 압축 헤더 정보 읽기 (12바이트)
            depth_param_bytes = msg.data[0:12]
            depthQuantA, depthQuantB = struct.unpack('ff', depth_param_bytes[:8])

            # Step 2: 압축된 PNG 이미지 추출
            compressed_data = msg.data[12:]
            np_arr = np.frombuffer(compressed_data, np.uint8)

            # Step 3: PNG 이미지 디코딩
            depth_img = cv2.imdecode(np_arr, cv2.IMREAD_UNCHANGED)

            if depth_img is None:
                self.get_logger().error("Failed to decode depth image")
                return

            # self.get_logger().info(f"Decoded depth image: shape={depth_img.shape}, dtype={depth_img.dtype}")
            depth = depthQuantA / (depth_img.astype(np.float32) + depthQuantB)

            ts = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            self.depth_buffer.append((ts, depth,depth_img))
            # Step 4 (선택): 실 depth 계산
            # 실제 depth 값이 필요한 경우:
            # 또는 그냥 depth_img 그대로 사용 가능

        except Exception as e:
            self.get_logger().error(f"Exception during depth decode: {e}")

    def get_synced_frames(self, max_diff=0.033):
        if not self.image_buffer or not self.depth_buffer:
            return None, None, None

        best_pair = None
        min_diff = float('inf')
        for ts_img, img in self.image_buffer:
            for ts_depth, depth, raw_depth_img in self.depth_buffer:
                diff = abs(ts_img - ts_depth)
                if diff < min_diff and diff < max_diff:
                    min_diff = diff
                    best_pair = (ts_img, img, ts_depth, depth, raw_depth_img)

        if best_pair:
            ts_img, img, ts_depth, depth, raw_depth_img = best_pair
            self.image_buffer = deque([(t, i) for (t, i) in self.image_buffer if t != ts_img])
            self.depth_buffer = deque([(t, d, r) for (t, d, r) in self.depth_buffer if t != ts_depth])
            return img, depth, raw_depth_img
        else:
            return None, None, None
        



    def mouse_callback(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            print(f"[마우스 클릭] 좌표: x={x}, y={y}")
            self.x = x
            self.y = y
            self.clicked = True

def map_resized_to_original(x, y, input_shape, cropped_shape):
    """리사이즈된 이미지의 좌표 (x, y)를 원본 이미지 기준으로 역변환"""
    h_resized, w_resized = input_shape
    h_cropped, w_cropped = cropped_shape

    scale_x = w_cropped / w_resized
    scale_y = h_cropped / h_resized

    x_orig = int(x * scale_x)
    y_orig = int(y * scale_y)

    return x_orig, y_orig

def crop_depth_to_rgb_fov(depth_img, hfov_rgb_deg=66.0, hfov_depth_deg=80.0):
    """정렬된 depth를 RGB 가로화각에 맞춰 중앙 크롭"""
    h, w = depth_img.shape[:2]
    r = math.tan(math.radians(hfov_rgb_deg/2)) / math.tan(math.radians(hfov_depth_deg/2))
    new_w = int(w * r)              # 66° 기준이면 대략 0.774 * w
    x0 = (w - new_w) // 2
    cropped = depth_img[:, x0:x0+new_w]
    # 필요 시 RGB 해상도에 맞춰 리사이즈(깊이는 NEAREST!)
    return cv2.resize(cropped, (w, h), interpolation=cv2.INTER_NEAREST)

def main(args=None):
    rclpy.init(args=args)
    node = compare_depth_node()

    cv2.namedWindow("RGB + Depth")
    cv2.setMouseCallback("RGB + Depth", node.mouse_callback)
    
    def spin_thread():
        rclpy.spin(node)

    threading.Thread(target=spin_thread, daemon=True).start()

    while True:
        img, depth, raw_depth_img = node.get_synced_frames()

        if img is not None and raw_depth_img is not None:
            # 시각화는 raw (정수형) depth_img 기준
            # depth = crop_depth_to_rgb_fov(depth)
            depth_cropped = crop_depth_to_rgb_fov(raw_depth_img)
            depth_vis = cv2.convertScaleAbs(depth_cropped, alpha=0.03)
            depth_colored = cv2.applyColorMap(depth_vis, cv2.COLORMAP_JET)
            depth_colored = cv2.resize(depth_colored, (img.shape[1], img.shape[0]))

            combined = np.hstack((img, depth_colored))
            cv2.imshow("RGB + Depth", combined)

            # 거리 추출은 실수형 depth 기준
            if node.clicked:
                x, y = node.x, node.y
                # 클릭 좌표를 원본 depth 해상도로 매핑
                x_orig, y_orig = map_resized_to_original(
                    x, y,
                    input_shape=depth_colored.shape[:2],       # 화면 상 좌표
                    cropped_shape=depth.shape[:2],              # 원본 실수형 depth
                )
                if 0 <= x_orig < depth.shape[1] and 0 <= y_orig < depth.shape[0]:
                    depth_value = depth[y_orig, x_orig]
                    print(f"[Depth Info] (resized:{x},{y} → original:{x_orig},{y_orig}) 깊이: {depth_value:.2f}")
                else:
                    print("[경고] 변환된 좌표가 범위를 벗어났습니다.")

                node.clicked = False

        key = cv2.waitKey(1)
        if key == 27:
            break

    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()
