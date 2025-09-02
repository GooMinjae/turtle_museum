# --------- ROS/메시지/액션/TF 관련 import ---------
from geometry_msgs.msg import PointStamped, PoseStamped, PoseWithCovarianceStamped, Twist  # 포인트/포즈 메시지
from nav2_msgs.action import NavigateToPose  # Nav2의 내비게이션 액션 정의
from rclpy.action import ActionClient  # ROS2 액션 클라이언트
from tf2_ros import Buffer, TransformListener  # TF 변환 버퍼/리스너
from rclpy.node import Node  # ROS2 노드 기본 클래스
from voice_processing import tts
from std_msgs.msg import String
from rclpy.time import Time
from transforms3d.euler import euler2quat

#------------------------------YOLO-----------------
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

from sensor_msgs.msg import Image, CameraInfo, CompressedImage
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

import tf2_geometry_msgs  # PointStamped 등 TF 변환 타입 지원을 위한 임포트(실사용은 없지만 등록 용도)
# --------- ROS2 유틸/시간/수학/외부프로세스 ---------
import rclpy  # ROS2 파이썬 클라이언트 라이브러리
from rclpy.duration import Duration  # TF 변환 타임아웃 등에 사용
import math  # 거리 계산(hypot) 등에 사용


class TrackerPersonNode(Node):
    """
    사람 포인트('/robot8/point_camera')만 추종하는 단순 트래커.
    - 카메라 좌표계의 포인트를 TF로 map 좌표계로 변환
    - NavigateToPose 액션으로 목표를 주기적 갱신(디바운스 적용)
    - 남은 거리 < close_enough_distance 조건이 연속 3회면 업데이트 일시 차단
    - /robot8/amcl_pose 로봇 위치가 특정 존에 진입하면: goal 취소 + TTS + wait_sec 후 자동 재개
    """
    def __init__(self):
        super().__init__('detect_tracker_node')  # 노드 이름 설정

        # ---------- 파라미터 선언(기본값 포함) ----------
        self.declare_parameter('close_enough_distance', 0.1)  # 근접 판정 임계값(m)
        self.declare_parameter('min_goal_interval_sec', 0.5)  # 목표 갱신 최소 시간 간격(s)
        self.declare_parameter('min_goal_translation', 0.05)  # 목표 갱신 최소 이동 거리(m)
        self.declare_parameter('approach_offset', 0.3)  # 사람에게 다가갈 때 z축으로 뺄 거리(m)
        self.declare_parameter('zone_check_hz', 10.0)  # 존 체크 주파수(Hz)
        self.declare_parameter('tts_command', 'espeak -s 160 -v ko+f3 "{text}"')  # 로컬 TTS 명령 템플릿
        
        # zones: ROS 2 파라미터가 아닌, 일반적인 클래스 변수로 직접 정의합니다.
        self.zones = [
            {'name': 'gallery_A', 'x': -1.02, 'y': -0.74, 'radius': 0.9,
             'tts': '이 작품은 핑구 킹입니다. 21세기 아동 문학의 주도문화였습니다. 그리고 대한민국에서도 많은 사랑을 받았습니다.'
             , 'wait_sec': 0},
            {'name': 'gallery_B', 'x': -1.68, 'y': 2.85, 'radius': 1.3,
             'tts': '이 작품은 요리하는 핑구입니다. 에피소드 4에서 핑구가 제빵을 해. 모든 사람에게 인정을 받는 이야기입니다.'
             , 'wait_sec': 0},
            {'name': 'gallery_C', 'x': -3.88, 'y': 0.363, 'radius': 1,
             'tts': '마지막 작품은 화난 핑구입니다. 이 작품은 2010년대 중반 "Noot Noot!"이라는 밈으로 더 유명한 작품입니다.'
             , 'wait_sec': 0},
        ]
        # ---------- 파라미터 실제 값 가져오기 ----------
        # === Internal state ===
        self.bridge = CvBridge()
        self.K = None
        self.rgb_image = None
        self.depth_image = None
        self.camera_frame = None
        self.shutdown_requested = False
        self.logged_intrinsics = False
        self.inference_active = False
        self.target_id = None
        self.lost_count = 0
        self.MAX_LOST = 10  # 타겟 분실 후 재획득까지 허용 프레임 수

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

        # zones는 파라미터가 아니므로 get_parameter()로 가져오지 않습니다.
        self._last_tf_warn_ns = 0
        self.rotating = False
        self.pub_cmd_vel = self.create_publisher(Twist, '/cmd_vel', 10)
        # self.map_frame = self.get_parameter('map_frame').value
        self.close_enough_distance = float(self.get_parameter('close_enough_distance').value)
        self.min_goal_interval = float(self.get_parameter('min_goal_interval_sec').value)
        self.min_goal_translation = float(self.get_parameter('min_goal_translation').value)
        self.approach_offset = float(self.get_parameter('approach_offset').value)
        self.zone_check_hz = float(self.get_parameter('zone_check_hz').value)
        self.tts_command = self.get_parameter('tts_command').value
        
        # ---------- TF 버퍼/리스너 준비 ----------
        self.tf_buffer = Buffer()  # TF 데이터를 저장하고 쿼리하는 버퍼
        self.tf_listener = TransformListener(self.tf_buffer, self)  # 버퍼를 실제 TF 토픽에 연결

        # ---------- NavigateToPose 액션 클라이언트 ----------
        self.action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')


        # === Publishers ===
        self.person_pub = self.create_publisher(Image, '/robot8/frame', 10)
        # === Subscriptions ===
        qos_profile = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,  # 최신 depth 개수만 유지
            depth=2,                            # 큐 크기 (10개 유지)
            reliability=QoSReliabilityPolicy.BEST_EFFORT
        )

        self.create_subscription(Bool, '/robot8/people', self.people_check_cb, 10)
        self.create_subscription(CameraInfo, '/robot8/oakd/rgb/camera_info', self.camera_info_callback, 10)

        self.rgb_sub = Subscriber(self, CompressedImage, '/robot8/oakd/rgb/image_raw/compressed', qos_profile=qos_profile)
        self.depth_sub = Subscriber(self, Image, '/robot8/oakd/stereo/image_raw', qos_profile=qos_profile)
        self.ts = ApproximateTimeSynchronizer([self.rgb_sub, self.depth_sub], queue_size=10, slop=0.4)
        self.ts.registerCallback(self.synced_rgb_depth_cb)

        # ---------- 구독자/퍼블리셔 ----------  #
        self.create_subscription(PoseWithCovarianceStamped, '/robot8/amcl_pose', self.cb_amcl, 10)  # 로봇 현재 위치(존 체크용)
        self.pub_done = self.create_publisher(Bool, '/robot8/is_done_track', 10)  # (옵션) 완료 신호, 필요 없으면 제거 가능
        self.pub_zone_event = self.create_publisher(String, '/robot8/now_loc', 10)
        self.pub_ready_to_track1 = self.create_publisher(Bool, '/robot8/end_track1', 10)
        self.pub_ready_to_track3 = self.create_publisher(Bool, '/robot8/end_track3', 10)

        # ---------- 런타임 상태 변수 ----------
        self.latest_map_point = None          # 최근 변환된 사람 위치(map 좌표)
        self.goal_handle = None               # 현재 진행 중인 액션 goal 핸들

        self.block_goal_updates = False       # 근접 래치/존 일시정지 중에는 True로 목표 갱신 차단
        self.close_distance_hit_count = 0     # 근접 판정 연속 횟수 카운터
        self.last_feedback_log_time = 0.0     # 남은거리 로그 간격 제어
        self.last_goal_send_time = 0.0        # 목표 갱신 시간 디바운스
        self.last_goal_xy = None              # 목표 갱신 거리 디바운스

        self.pose = PoseStamped()             # 전송에 사용할 PoseStamped 캐리어(필드 재사용)
        self.robot_xy = None                  # AMCL로 받은 로봇 현재 위치

        self.paused_for_zone = False          # 존 설명 중 일시정지 상태
        self.active_zone = None               # 현재 설명 중인 존 이름.
        self.zone_inside_flags = {z['name']: False for z in self.zones}  # 존 재진입 방지 히스테리시스

        # ---------- 존 체크 타이머 ----------
        # 주기적으로 현재 위치가 어느 존에 들어왔는지 감시
        # self.create_timer(1.0 / max(1.0, self.zone_check_hz), self.zone_check_tick)
        # self.get_logger().info('tracker_person_node started.')  # 시작 로그

        # === Threads ===
        self.infer_thread1 = threading.Thread(target=self.zone_check_tick, daemon=True)
        self.infer_thread2 = threading.Thread(target=self.run_inference_thread, daemon=True)
        self.infer_thread1.start()
        self.infer_thread2.start()

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

                        self.pt = PointStamped()
                        self.pt.header.frame_id = pair.frame_id
                        self.pt.header.stamp = pair.stamp
                        self.pt.point.x, self.pt.point.y, self.pt.point.z = X, Y, z
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
            self.person_pub.publish(Image(data=frame))
            self.last_processed_stamp = pair.stamp

        finally:
            self.infer_lock.release()

    def publish_multiple(self, pub, msg, repeat: int = 5, interval: float = 0.2):
        """
        스레드를 이용해 반복 퍼블리시
        pub: rclpy.publisher.Publisher 객체
        msg: 퍼블리시할 메시지 (ROS2 메시지 타입 인스턴스)
        repeat: 반복 횟수
        interval: 반복 간격 (초)
        """
        def worker():
            for i in range(repeat):
                pub.publish(msg)
                self.get_logger().info(f"[MultiPub] {type(msg).__name__} -> {msg} (#{i+1})")
                time.sleep(interval)
        threading.Thread(target=worker, daemon=True).start()

    # ===================== 콜백: AMCL 위치 수신 =====================
    def cb_amcl(self, msg: PoseWithCovarianceStamped):
        self.robot_xy = (msg.pose.pose.position.x, msg.pose.pose.position.y)  # 로봇의 현재 (x,y) 저장

    # ===================== 콜백: 카메라 포인트 수신 =====================
    def cb_point_from_camera(self):
        """카메라 좌표계에서 들어온 사람 포인트를 map으로 변환하고 goal을 전송(디바운스 적용)."""
        if self.paused_for_zone or self.block_goal_updates:
            return  # 존 설명 중이거나 근접 래치면 목표 갱신 중단
        
            # ----- 회전 중이면 멈춤 -----
        if self.rotating:  # <<< MODIFY
            self.rotating = False
            self.get_logger().info("Point received, stopping rotation.")  # <<< ADD
            twist = Twist()  # 회전 정지
            self.pub_cmd_vel.publish(twist)  # <<< ADD

        # ----- 마지막 포인트 저장 (회전 방향 결정용) ----- 
        self.last_camera_point = self.pt  # <<< ADD

        # 사람에게 너무 가까이 붙지 않도록 목표 z 조정
        if self.pt.point.z > self.approach_offset:
            self.pt.point.z = self.pt.point.z - self.approach_offset
        else:
            self.pt.point.z = self.approach_offset


        try:
            pt_map = self.tf_buffer.transform(self.pt, 'map', timeout=rclpy.duration.Duration(seconds=1.5))
            self.latest_map_point = pt_map
            xy = (pt_map.point.x, pt_map.point.y)
        except Exception as e:
            self.get_logger().warn(f'TF transform failed {e}')
            return

        # ---- 디바운스(시간/거리) 기존 로직 유지 ----
        now = time.time()
        if (now - self.last_goal_send_time) < self.min_goal_interval:
            return
        if self.last_goal_xy is not None:
            dx = xy[0] - self.last_goal_xy[0]
            dy = xy[1] - self.last_goal_xy[1]
            if math.hypot(dx, dy) < self.min_goal_translation:
                return

        if self.goal_handle:
            try:
                self.goal_handle.cancel_goal_async()
            except Exception:
                pass

        self._send_goal(xy)
        self.last_goal_send_time = now
        self.last_goal_xy = xy
    # ===================== 헬퍼: Goal 송신 =====================
    def _send_goal(self, xy):
        # PoseStamped 헤더/포지션 설정
        self.pose.header.frame_id = 'map'  # 목표 프레임은 map
        self.pose.header.stamp = self.get_clock().now().to_msg()  # 현재 시간 스탬프
        self.pose.pose.position.x = xy[0]  # 목표 x
        self.pose.pose.position.y = xy[1]  # 목표 y
        # 로봇 현재 위치와 목표 좌표 차이로 yaw 계산
        if self.robot_xy is not None:
            dx = xy[0] - self.robot_xy[0]
            dy = xy[1] - self.robot_xy[1]
            yaw = math.atan2(dy, dx)
            q = euler2quat(0, 0, yaw)  # w, x, y, z 순서
            self.pose.pose.orientation.x = q[1]
            self.pose.pose.orientation.y = q[2]
            self.pose.pose.orientation.z = q[3]
            self.pose.pose.orientation.w = q[0]

        # 액션 Goal 메시지에 포즈를 담기
        goal = NavigateToPose.Goal()
        goal.pose = self.pose

        self.get_logger().info(f"Sending goal: ({xy[0]:.2f}, {xy[1]:.2f})")  # 전송 로그
        self.action_client.wait_for_server()  # 액션 서버 준비될 때까지 대기
        # 비동기 전송 + 피드백 콜백 등록
        self._send_goal_future = self.action_client.send_goal_async(goal, feedback_callback=self._feedback_cb)
        # goal 수락/거절 응답 콜백 등록
        self._send_goal_future.add_done_callback(self._goal_resp_cb)

    # ===================== 콜백: 액션 피드백 =====================
    def _feedback_cb(self, feedback_msg):
        fb = feedback_msg.feedback  # 피드백 메시지 본문
        dist = getattr(fb, 'distance_remaining', None)  # 남은 거리(m), 구현에 따라 없을 수 있어 안전 접근

        # 근접 판정: 남은 거리가 임계치보다 작으면 카운트 증가, 아니면 리셋
        if dist is not None and dist < self.close_enough_distance:
            self.close_distance_hit_count += 1
        else:
            self.close_distance_hit_count = 0

        # 3회 연속 근접 → 목표 갱신 차단(래치)
        if self.close_distance_hit_count >= 3 and not self.block_goal_updates:
            self.block_goal_updates = True
            self.get_logger().info(f"Within {self.close_enough_distance:.2f} m — blocking goal updates.")

        # 남은 거리 주기적 로그(1초 간격)
        now = time.time()
        if dist is not None and now - self.last_feedback_log_time > 1.0:
            self.get_logger().info(f"Distance remaining: {dist:.2f} m")
            self.last_feedback_log_time = now

    # ===================== 콜백: 액션 goal 수락/거절 응답 =====================
    def _goal_resp_cb(self, future):
        self.goal_handle = future.result()  # goal 핸들 획득(수락/거절 여부 포함)
        if not self.goal_handle or not self.goal_handle.accepted:  # 거절된 경우
            self.get_logger().warn("Goal was rejected.")
            self.goal_handle = None
            return
        self.get_logger().info("Goal accepted.")  # 수락 로그
        # 완료 결과 콜백 등록(비동기)
        self._get_result_future = self.goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self._goal_result_cb)

    # ===================== 콜백: 액션 완료 결과 =====================
    def _goal_result_cb(self, future):
        status = future.result().status
        self.get_logger().info(f"Goal finished with result code: {status}")
        self.goal_handle = None

            # ----- 마지막 목표 도착 후 회전 모드 시작 -----
        if hasattr(self, 'last_camera_point') and self.last_camera_point is not None:  # <<< MODIFY
            self.rotating = True
            self._start_rotation(self.last_camera_point)  # <<< MODIFY

        # 최신 위치가 이전 Goal과 충분히 다를 때만 재전송
        if self.latest_map_point is not None:
            xy = (self.latest_map_point.point.x, self.latest_map_point.point.y)
            if self.last_goal_xy is None or math.hypot(xy[0]-self.last_goal_xy[0], xy[1]-self.last_goal_xy[1]) > self.min_goal_translation:
                self._send_goal(xy)
                self.last_goal_send_time = time.time()
                self.last_goal_xy = xy

    # ===================== 회전 루프 함수 수정 =====================
    def _start_rotation(self, pt_camera):  # <<< MODIFY
        """마지막 목표 후, point_camera 수신 전까지 회전"""
        # 회전 방향 결정
        angular_speed = 0.5  # rad/s 기본값
        if pt_camera.point.x > 0:  # 오른쪽이면 시계 방향
            angular_speed = -abs(angular_speed)
        else:  # 왼쪽이면 반시계 방향
            angular_speed = abs(angular_speed)

        def rotate_loop():
            rate = self.create_rate(10)  # 10Hz
            twist = Twist()
            twist.angular.z = angular_speed
            while rclpy.ok() and self.rotating:
                self.pub_cmd_vel.publish(twist)
                rate.sleep()
            # 회전 멈춤
            twist.angular.z = 0.0
            self.pub_cmd_vel.publish(twist)

        threading.Thread(target=rotate_loop, daemon=True).start()  # <<< ADD

# ===================== 존 로직: 타이머 콜백 =====================
    def zone_check_tick(self):
        """
        현재 로봇 위치가 존 내부로 진입하면:
        - 목표 갱신 차단(block_goal_updates=True)
        - 진행 중 goal 취소
        - TTS(멘트) 실행
        - 바로 추적 재개 (wait 없음)
        """
        if self.robot_xy is None:
            return  # AMCL 위치 없으면 스킵

        x, y = self.robot_xy
        triggered = None

        # 모든 존 확인
        for z in self.zones:
            name = z['name']
            dx = x - float(z['x'])
            dy = y - float(z['y'])
            inside = (dx*dx + dy*dy) <= (float(z['radius']) ** 2)

            if inside and not self.zone_inside_flags[name]:
                triggered = z
            self.zone_inside_flags[name] = inside

        if triggered is None:
            return  # 진입 없음

        # ---- 트리거 동작 ----
        self.block_goal_updates = True
        if self.goal_handle is not None:
            try:
                self.goal_handle.cancel_goal_async()
            except Exception:
                pass

        # 토픽 발행
        msg = String()
        msg.data = triggered['name']
        # self.pub_zone_event.publish(msg)
        self.publish_multiple(self.pub_zone_event, msg, repeat=40, interval=0.1)


        text = str(triggered.get('tts'))
        self.get_logger().info(f'[ZONE:{triggered["name"]}] TTS: "{text}" (resume immediately)')
        # self._run_tts_async(text, msg.data)
        tts.say(text)

        # ---- 즉시 추적 재개 ----
        self.block_goal_updates = False
        self.close_distance_hit_count = 0

        msg = String()
        msg.data = "None"
        # self.pub_zone_event.publish(msg)
        self.publish_multiple(self.pub_zone_event, msg, repeat=40, interval=0.1)


        # ---- Bool 퍼블리시 (A, C 존에서만) ----
        if triggered['name'] in ['gallery_A']:
            ready_msg = Bool()
            ready_msg.data = True
            self.publish_multiple(self.pub_ready_to_track1, ready_msg, repeat=40, interval=0.1)

        if triggered['name'] == 'gallery_C':
            done_msg = Bool()
            done_msg.data = True
            self.publish_multiple(self.pub_done, done_msg, repeat=10, interval=0.2)

            ready_msg = Bool()
            ready_msg.data = True
            self.publish_multiple(self.pub_ready_to_track3, ready_msg, repeat=40, interval=0.1)

    # ===================== 로컬 TTS 실행(비블로킹) =====================
    def _run_tts_async(self, text: str):
        try:
            # voice_processing/tts.py 안의 say_async 사용
            tts.say_async(text)
            # for _ in range(60):
            #     self.pub_zone_event.publish(msg)
            #     time.sleep(0.1)
        except Exception as e:
            self.get_logger().error(f'TTS failed: {e}')

# ===================== 엔트리 포인트 =====================
def main(args=None):
    rclpy.init(args=args)         # ROS2 초기화
    node = TrackerPersonNode()    # 노드 인스턴스 생성
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)
    try:
        rclpy.spin(node)          # 콜백/타이머를 돌리며 대기
        executor.spin()
    except KeyboardInterrupt:
        pass                      # Ctrl+C 등으로 종료
    finally:
        if rclpy.ok():
            node.destroy_node()   # 노드 정리
            rclpy.shutdown()      # ROS2 종료

if __name__ == '__main__':  # 스크립트 직접 실행 시
    main()                  # main() 호출
