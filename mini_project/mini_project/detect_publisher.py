import rclpy                               # ROS 2 파이썬 클라이언트 라이브러리
from rclpy.node import Node                # ROS 2 노드 베이스 클래스
from sensor_msgs.msg import Image, CameraInfo, CompressedImage # RGB/Depth 이미지, 카메라 내부파라미터
from geometry_msgs.msg import PointStamped, PoseStamped # 좌표계 포함 점/포즈 메시지
from cv_bridge import CvBridge            # ROS Image <-> OpenCV 변환 브릿지
import numpy as np                        # 수치연산
from ultralytics import YOLO              # YOLOv8 모델 로더/추론
import threading                          # GUI 표시(이미지 창)용 별도 스레드
import time                               # 피드백 주기 제어용
import cv2                                # OpenCV
from std_msgs.msg import String
import threading
from collections import namedtuple
Pair = namedtuple("Pair", "rgb depth stamp frame_id")
from message_filters import Subscriber, ApproximateTimeSynchronizer, TimeSynchronizer
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy, QoSReliabilityPolicy
# import cv2, os
# cv2.setNumThreads(1)
# os.environ["OMP_NUM_THREADS"] = "1"

class YoloPerson(Node):
    def __init__(self):
        super().__init__('detect_to_thing')  # 노드 이름 설정
        # Internal state
        self.bridge = CvBridge()           # ROS <-> OpenCV 변환기
        self.K = None                      # 카메라 내참행렬(3x3) 저장
        self.depth_image = None            # 최신 Depth 이미지
        self.rgb_image = None              # 최신 RGB 이미지
        self.camera_frame = None           # RGB 프레임 이름(헤더에서 읽음)
        self.shutdown_requested = False    # GUI 스레드에서 ESC 눌러 종료할 때 사용
        self.logged_intrinsics = False     # 내참행렬 로그를 한 번만 찍기 위한 플래그
        self.current_distance = None       # Nav2 피드백(남은 거리)
        self.block_goal_updates = False    # 가까워지면 더 이상 goal을 갱신하지 않도록 막음
        qos_profile = QoSProfile(depth=2)
        qos_profile.reliability = QoSReliabilityPolicy.BEST_EFFORT
        self._pair_lock = threading.Lock()
        self._latest_pair = None

        # Load YOLOv8 model
        self.model = YOLO("/home/rokey/turtlebot4_ws/src/training/runs/detect/yolov8-turtlebot4-custom2/weights/best.pt")
        self.model.to('cuda')
        print("Model loaded.")
        #  YOLOv8n 가중치 로드(경로는 로컬 파일)

        self.main_pub = self.create_publisher(String, '/robot8/which_hand', 10)

        self.person_point_cam_pub = self.create_publisher(
            PointStamped, '/robot8/point_camera', 10
        )

        # Display
        self.display_frame = None                         # 화면에 띄울 프레임
        self.display_thread = threading.Thread(target=self.display_loop, daemon=True)
        self.display_thread.start()                       # OpenCV imshow 전용 스레드 시작
        # ROS 2 subscriptions
        self.create_subscription(CameraInfo, '/robot8/oakd/rgb/camera_info', self.camera_info_callback, 10)
        # 카메라 내부파라미터 구독(큐 크기 10)
        # self.create_subscription(CompressedImage, '/robot8/oakd/rgb/image_raw/compressed', self.rgb_compressed_callback, 10)
        # RGB 이미지 구독
        # self.create_subscription(CompressedImage, '/robot8/oakd/stereo/image_raw/compressedDepth', self.depth_compressed_callback, 10)
        # Depth 이미지 구독
        # RGB / Depth 동기화 구독 (Approximate: 슬롭 30ms)
        # qos_sensor = QoSProfile(
        #                 depth=1,
        #                 reliability=ReliabilityPolicy.BEST_EFFORT,
        #                 history=HistoryPolicy.KEEP_LAST,
        #                 durability=DurabilityPolicy.VOLATILE
        #             )
        self.rgb_sub   = Subscriber(self, CompressedImage, '/robot8/oakd/rgb/image_raw/compressed', qos_profile=qos_profile)
        self.depth_sub = Subscriber(self, Image, '/robot8/oakd/stereo/image_raw', qos_profile=qos_profile)
        # self.rgb_sub   = Subscriber(self, CompressedImage, '/robot8/oakd/rgb/image_raw/compressed')
        # self.depth_sub = Subscriber(self, Image, '/robot8/oakd/stereo/image_raw')

        self.ts  = ApproximateTimeSynchronizer([self.rgb_sub, self.depth_sub], queue_size=2, slop=0.03)
        self.ts.registerCallback(self.synced_rgb_depth_cb)

        self.last_pair_stamp = None

        self.infer_lock = threading.Lock()
        self.last_processed_stamp = None

        # Periodic detection and goal logic
        self.create_timer(0.2, self.process_frame)       # 0.5초마다 감지/목표 갱신 로직 실행
        self.last_feedback_log_time = 0                  # 피드백 로그 간격 조절용

    def synced_rgb_depth_cb(self, rgb_msg: CompressedImage, depth_msg: Image):
        try:
            # --- RGB 디코드 ---
            np_arr = np.frombuffer(rgb_msg.data, np.uint8)
            rgb = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            depth_raw = self.bridge.imgmsg_to_cv2(depth_msg, "passthrough")  # mono16 mm

            if rgb is None or depth_raw is None:
                return
            
            # --- Depth [m]로 변환 ---
            if depth_msg.encoding == "16UC1":       # 보통 mm
                depth_raw = depth_raw.astype(np.float32) / 1000.0
            elif depth_msg.encoding == "32FC1":     # 이미 m
                depth_raw = depth_raw
            else:
                raise ValueError(f"Unexpected encoding: {depth_msg.encoding}")
            
            # ─ crop stereo depth to approximate RGB FOV ─
            h, w = depth_raw.shape              # 480 × 640 expected
            crop_x = int(0.26 * w / 2) * 2      # 26 % total width ⇒ 13 % per side
            crop_y = int(0.18 * h / 2) * 2      # 18 % total height ⇒  9 % top/bot
            depth_crop = depth_raw[crop_y : h - crop_y, crop_x : w - crop_x]
            depth_aligned = cv2.resize(depth_crop, (w, h), cv2.INTER_NEAREST)
            
            with self._pair_lock:
                self._latest_pair = Pair(
                    rgb=rgb,
                    depth=depth_aligned,
                    stamp=rgb_msg.header.stamp,
                    frame_id=rgb_msg.header.frame_id or "oakd_rgb_frame"
                )

            # --- 같은 페어로 상태 갱신 ---
            # self.rgb_image = rgb
            # self.depth_image = depth_aligned

            # # 프레임/타임스탬프 저장
            # if rgb_msg.header.frame_id:
            #     self.camera_frame = rgb_msg.header.frame_id
            # self.last_pair_stamp = rgb_msg.header.stamp

            # (옵션) 실제 시간차 로그
            rgbs = rgb_msg.header.stamp.sec + rgb_msg.header.stamp.nanosec*1e-9
            deps = depth_msg.header.stamp.sec + depth_msg.header.stamp.nanosec*1e-9
            dt_ms = (deps - rgbs) * 1000.0
            if abs(dt_ms) > 30.0:
                self.get_logger().warn(f'RGB-Depth stamp diff = {dt_ms:.1f} ms')
        except Exception as e:
            self.get_logger().error(f"Sync decode failed: {e}")

    def camera_info_callback(self, msg):
        self.K = np.array(msg.k).reshape(3, 3)           # 9개 요소를 3x3 내참행렬로 변환
        if not self.logged_intrinsics:
            # self.get_logger().info(
            #     f"Camera intrinsics received: fx={self.K[0,0]:.2f}, fy={self.K[1,1]:.2f}, "
            #     f"cx={self.K[0,2]:.2f}, cy={self.K[1,2]:.2f}"
            # )
            self.get_logger().info("Camera intrinsics")
            self.logged_intrinsics = True                # 한 번만 로그

    def _clear_current_frame(self):
        # 추론이 끝난 뒤 현재 프레임을 비워서 이전 프레임 재사용을 차단
        self.rgb_image = None
        self.depth_image = None

    def display_loop(self):  # 추가: 간단한 imshow 루프
        while not self.shutdown_requested:
            if self.display_frame is not None:
                cv2.imshow('yolo', self.display_frame)
                key = cv2.waitKey(1) & 0xFF
                if key == 27:  # ESC
                    self.shutdown_requested = True
                    break
            else:
                # 약간 쉼
                time.sleep(0.01)
        cv2.destroyAllWindows()

    def process_frame(self):
        if self.K is None:
            return                                      # 준비 안 됐으면 스킵
        if not self.infer_lock.acquire(blocking=False):
            return  # 이전 추론이 아직 끝나지 않음
        try:
            # 최신 페어 스냅샷 로드
            with self._pair_lock:
                pair = self._latest_pair
            if pair is None:
                return
            
            #같은 프레임 재처리 방지
            if self.last_processed_stamp is not None:
                if (pair.stamp.sec == self.last_processed_stamp.sec and
                    pair.stamp.nanosec == self.last_processed_stamp.nanosec):
                    return

            #오래된 프레임 폐기
            now = self.get_clock().now().to_msg()
            age = (now.sec - pair.stamp.sec) + (now.nanosec - pair.stamp.nanosec)*1e-9
            if age > 0.5:
                return

            self.rgb_image = pair.rgb.copy()
            self.depth_image = pair.depth.copy()
            # results = self.model.track(self.rgb_image, conf=0.7, persist=True, tracker='bytetrack.yaml', verbose=False)[0]
            results = self.model.predict(self.rgb_image, conf=0.7, verbose=False)[0]
            frame = self.rgb_image.copy()                   # 표시용 복사본

            # if self.last_pair_stamp != stamp:
                # return

            any_main = False
            H, W = self.depth_image.shape[:2]

            for det in results.boxes:
                cls = int(det.cls[0])
                label = self.model.names[cls].lower()
                conf  = float(det.conf[0])
                x1, y1, x2, y2 = map(int, det.xyxy[0].tolist())

                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.putText(frame, f"{label} {conf:.2f}", (x1, max(0, y1-5)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

                # 제스처 탐지: 프레임당 여러 번 나올 수 있음(OK)
                if label in ("one", "five"):
                    self.main_pub.publish(String(data=label))
                    any_main = True

                # 3D 포인트 (필요시 person으로 교체)
                if label in ("car", "bottle"):
                    u = (x1 + x2) // 2
                    v = (y1 + y2) // 2
                    if not (0 <= u < W and 0 <= v < H):
                        continue
                    z = float(self.depth_image[v, u])  # 이미 meters
                    if z <= 0.0 or np.isnan(z):
                        continue

                    fx, fy = self.K[0, 0], self.K[1, 1]
                    cx, cy = self.K[0, 2], self.K[1, 2]
                    # ★ 단위 수정: 1000 나누지 않음
                    X = (u - cx) * z / fx
                    Y = (v - cy) * z / fy

                    pt = PointStamped()
                    # ★ 동기 페어의 frame_id / stamp 사용
                    pt.header.frame_id = pair.frame_id
                    pt.header.stamp    = pair.stamp
                    pt.point.x, pt.point.y, pt.point.z = X, Y, z
                    self.person_point_cam_pub.publish(pt)

            # ★ 프레임당 한 번만 'none' (검출된 제스처가 하나도 없을 때)
            if not any_main:
                self.main_pub.publish(String(data="none"))

            self.display_frame = frame
            self.last_processed_stamp = pair.stamp
            self._clear_current_frame()

            with self._pair_lock:
                self._latest_pair = None
        finally:
            # === 중요: 반드시 락 해제 ===
            self.infer_lock.release()

from rclpy.executors import MultiThreadedExecutor

def main():
    # rclpy.init()
    # node = YoloPerson()
    # try:
    #     while rclpy.ok() and not node.shutdown_requested:
    #         rclpy.spin_once(node, timeout_sec=0.1)
    # except KeyboardInterrupt:
    #     pass
    # finally:
    #     node.destroy_node()
    #     cv2.destroyAllWindows()
    #     if rclpy.ok():
    #         rclpy.shutdown()
    rclpy.init()
    node = YoloPerson()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()