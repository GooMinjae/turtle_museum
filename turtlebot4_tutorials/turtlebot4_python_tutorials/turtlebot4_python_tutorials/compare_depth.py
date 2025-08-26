import os
import sys
import rclpy
import threading
import numpy as np
from queue import Queue
from rclpy.node import Node
from sensor_msgs.msg import Image , CompressedImage
from cv_bridge import CvBridge
from ultralytics import YOLO
import cv2
import struct
# from collections import deque

# def mouse_callback(event, x, y, flags, param):
#     if event == cv2.EVENT_LBUTTONDOWN:
#         print(f"[마우스 클릭] 좌표: x={x}, y={y}")


# class compare_depth_node(Node):
#     def __init__(self):
#         super().__init__('compare_depth_node')
#         self.bridge = CvBridge()
#         self.image_queue = Queue(maxsize=1)
#         self.depth_queue = Queue(maxsize=1)
#         self.x = 0
#         self.y = 0
        

#         # super().__init__('compare_depth_node')
#         self.sub_image = self.create_subscription(
#             Image,
#             '/robot8/oakd/rgb/preview/image_raw',
#             self.image_callback,
#             10)
#         # self.sub_depth = self.create_subscription(
#         #     Image,
#         #     '/robot8/oakd/stereo/image_raw',
#         #     self.depth_callback,
#         #     10
#         # )
#         self.sub_depth = self.create_subscription(
#             Image,
#             # '/robot8/oakd/stereo/image_raw/compressedDepth',
#             '/robot8/oakd/stereo/image_raw',
#             # '/robot8/oakd/rgb/preview/depth',
#             self.depth_callback,
#             10
#         )
    
#     def image_callback(self,msg):
#         try:
#             img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
#             if not self.image_queue.full():
#                 self.image_queue.put(img)
#         except Exception as e:
#             self.get_logger().error(f"Image conversion failed: {e}")
#     def depth_callback(self, msg):
#         try:
#             img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
#             if img is None:
#                 self.get_logger().warn("Depth image decode 실패: imdecode 결과 None")
#                 return

#             if not self.depth_queue.full():
#                 self.depth_queue.put(img)
#         except Exception as e:
#             self.get_logger().error(f"Depth image conversion failed: {e}")

    # def depth_callback(self, msg):
    #     try:
    #         # img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
    #         np_arr = np.frombuffer(msg.data, np.uint8)
    #         img = cv2.imdecode(np_arr, cv2.IMREAD_UNCHANGED)

    #         if img is None:
    #             self.get_logger().warn("Depth image decode 실패: imdecode 결과 None")
    #             return

    #         if not self.depth_queue.full():
    #             self.depth_queue.put(img)
    #     except Exception as e:
    #         self.get_logger().error(f"Image conversion failed: {e}")

#     def mouse_callback(self, event, x, y, flags, param):
#         if event == cv2.EVENT_LBUTTONDOWN:
#             print(f"[마우스 클릭] 좌표: x={x}, y={y}")
#             self.x = x
#             self.y = y


#     def get_synced_frames(self, max_diff=0.033):
#         """
#         두 버퍼에서 timestamp가 가장 가까운 RGB와 Depth 쌍을 찾아 반환
#         max_diff: 허용 최대 시간 차이 (초 단위), 예: 0.033초 ~ 30fps 기준
#         """
#         if not self.image_buffer or not self.depth_buffer:
#             return None, None

#         # 가장 가까운 timestamp 찾기
#         best_pair = None
#         min_diff = float('inf')
#         for ts_img, img in self.image_buffer:
#             for ts_depth, depth in self.depth_buffer:
#                 diff = abs(ts_img - ts_depth)
#                 if diff < min_diff and diff < max_diff:
#                     min_diff = diff
#                     best_pair = (ts_img, img, ts_depth, depth)

#         if best_pair:
#             ts_img, img, ts_depth, depth = best_pair
#             # 사용한 프레임은 버퍼에서 제거
#             self.image_buffer = deque([(t, i) for (t, i) in self.image_buffer if t != ts_img])
#             self.depth_buffer = deque([(t, d) for (t, d) in self.depth_buffer if t != ts_depth])
#             return img, depth
#         else:
#             return None, None



# def main(args=None):
#     rclpy.init(args=args)
#     node = compare_depth_node()

#     # OpenCV 창 생성 및 마우스 콜백 연결
#     cv2.namedWindow("RGB Image")
#     cv2.setMouseCallback("RGB Image", node.mouse_callback)
#     # cv2.namedWindow("Depth Image")

#     # ROS2 스핀을 백그라운드 스레드로 실행
#     def spin_thread():
#         rclpy.spin(node)

#     threading.Thread(target=spin_thread, daemon=True).start()



#     # while True:
#     #     if not node.image_queue.empty() and not node.depth_queue.empty():
#     #         image = node.image_queue.get()
#     #         depth_image = node.depth_queue.get()

#     #         # Step 1: Depth를 보기 좋게 컬러맵 적용
#     #         depth_vis = cv2.convertScaleAbs(depth_image, alpha=0.03)
#     #         depth_colored = cv2.applyColorMap(depth_vis, cv2.COLORMAP_JET)

#     #         # Step 2: Depth 이미지를 RGB 이미지 크기로 resize
#     #         depth_colored = cv2.resize(depth_colored, (image.shape[1], image.shape[0]))



#     #         # Step 4: 두 이미지 나란히 표시
#     #         combined = np.hstack((image, depth_colored))
#     #         cv2.imshow("RGB Image", combined)

#     #     # combined = np.hstack((image, depth_colored))  # 좌우로 붙이기
#     #     # cv2.imshow("RGB Image", combined)
#     #     key = cv2.waitKey(1)
#     #     if key == 27:  # ESC로 종료
#     #         break


#     while True:
#         img, depth = node.get_synced_frames()
#         if img is not None and depth is not None:
#             # 컬러맵 적용 + resize
#             depth_vis = cv2.convertScaleAbs(depth, alpha=0.03)
#             depth_colored = cv2.applyColorMap(depth_vis, cv2.COLORMAP_JET)
#             depth_colored = cv2.resize(depth_colored, (img.shape[1], img.shape[0]))

#             combined = np.hstack((img, depth_colored))
#             cv2.imshow("RGB + Depth", combined)

#         key = cv2.waitKey(1)
#         if key == 27:
#             break

#     node.destroy_node()
#     rclpy.shutdown()
#     cv2.destroyAllWindows()

# if __name__ == '__main__':
#     main()


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
