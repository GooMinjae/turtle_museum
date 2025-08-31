#!/usr/bin/env python3
from ultralytics import YOLO
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Int32, Bool
from turtlebot4_navigation.turtlebot4_navigator import TurtleBot4Directions, TurtleBot4Navigator
from geometry_msgs.msg import PoseWithCovarianceStamped
import numpy as np
import cv2
import threading, time
from std_srvs.srv import Trigger


class Turtlebot4GuideCounter(Node):
    """
    상태머신:
      - WAIT_AUDIENCE: 관객 수 대기
      - NAVIGATING: undock + goal 이동 중
      - INFERENCING: YOLO로 사람 수 확인 중 (연속 N프레임 일치 시 True)
      - LATCHED: True 보낸 뒤 정지, 새 audience_count 오면 해제
    """
    def __init__(self):
        super().__init__('robot8_guide_counter')
        # ===== Parameters =====
        # Perception
        self.declare_parameter('input_topic', '/robot8/oakd/rgb/image_raw/compressed')
        self.declare_parameter('conf', 0.5)
        self.declare_parameter('match_threshold', 3)        # 연속 일치 프레임 수
        self.declare_parameter('bypass_inference', False)   # 테스트 모드: 추론 건너뛰기(= n_people = audience_count)
        # Navigation
        self.declare_parameter('initial_x', 0.0)
        self.declare_parameter('initial_y', 0.0)
        self.declare_parameter('initial_dir', 'NORTH')      # NORTH/EAST/SOUTH/WEST
        self.declare_parameter('goal_x', -13.0)
        self.declare_parameter('goal_y', 9.0)
        self.declare_parameter('goal_dir', 'EAST')
        self.declare_parameter('timeout_sec', 180.0)
        # Load params
        self.input_topic = self.get_parameter('input_topic').get_parameter_value().string_value
        self.conf = float(self.get_parameter('conf').value)
        self.match_threshold = int(self.get_parameter('match_threshold').value)
        self.bypass = bool(self.get_parameter('bypass_inference').value)
        self.initial_x = float(self.get_parameter('initial_x').value)
        self.initial_y = float(self.get_parameter('initial_y').value)
        self.initial_dir = str(self.get_parameter('initial_dir').value).upper()

                # ===== 추가 목표 좌표 (순차 이동용) =====
        self.targets = [
            (-0.283, 1.12, "WEST"),   # 첫 번째 목표 (지금 쓰는 것)
            (-0.0671, -0.367, "SOUTH"),     # 두 번째 목표
        ]
        self.current_target_idx = 0

        self.timeout_sec = float(self.get_parameter('timeout_sec').value)
        # ===== State =====
        self.STATE_WAIT = 'WAIT_AUDIENCE'
        self.STATE_NAV = 'NAVIGATING'
        self.STATE_INF = 'INFERENCING'
        self.STATE_LATCHED = 'LATCHED'
        self.state = self.STATE_WAIT
        self.rgb_image = None
        self.audience_count = 0
        self.consec_match = 0
        self.latched = False
        self._nav_busy = False
        self._lock = threading.Lock()
        # ===== YOLO =====
        self.model = None
        if not self.bypass:
            try:
                self.model = YOLO('yolov8n.pt')
            except Exception as e:
                self.get_logger().warn(f'YOLO load failed, switching to bypass: {e}')
                self.bypass = True
        # ===== IO =====
        self.create_subscription(
            CompressedImage, self.input_topic, self.image_cb, qos_profile_sensor_data
        )
        self.create_subscription(
            Int32, '/robot8/audience_count', self.audience_cb, 10
        )
        self.pub_people_check = self.create_publisher(Bool, '/robot8/people_check', 10)
        self.pub_arrived = self.create_publisher(Bool, '/robot8/arrived', 10)
        # ===== Timer for inference =====
        self.create_timer(0.1, self.infer_tick)  # 10Hz
        self.get_logger().info(
            f'Started. state={self.state}, match_threshold={self.match_threshold}, '
            f'conf={self.conf}, bypass_inference={self.bypass}'
        )
        # ===== Service =====
        self.create_service(Trigger, '/robot8/start_action', self.handle_start_action)
        self._action_requested = False

    # ---------- Service Callback ----------
    def handle_start_action(self, request, response):
        """
        로봇 제어 노드에서 서비스 요청 시 호출.
        True라면 현재 상태가 LATCHED이면 바로 실행 가능.
        """
        if self.latched:
            response.success = True
            response.message = "Action started."
            # 여기서 실행할 행동을 넣거나 상태 전이 가능
        else:
            response.success = False
            response.message = "People check not True yet."
        return response
    # ---------- Callbacks ----------
    def image_cb(self, msg: CompressedImage):
        try:
            np_arr = np.frombuffer(msg.data, np.uint8)
            frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            if frame is not None:
                self.rgb_image = frame
        except Exception as e:
            self.get_logger().warn(f'Image decode failed: {e}')
    def audience_cb(self, msg: Int32):
        val = int(msg.data)
        with self._lock:
            self.audience_count = val
            # 어떤 값이든 수신 시 래치 해제 & 일치 카운터 리셋
            self.consec_match = 0
            if self.latched:
                self.latched = False
                self.pub_people_check.publish(Bool(data=False))
            # 상태 전이: WAIT/LATCHED에서 관객수>0이면 NAV 시작
            if val > 0 and not self._nav_busy and self.state in (self.STATE_WAIT, self.STATE_LATCHED):
                self._transition_to(self.STATE_NAV)
                self._start_navigation_thread()
    # ---------- Navigation ----------
    def _dir_enum(self, name: str):
        return getattr(TurtleBot4Directions, name)
    def _start_navigation_thread(self):
        self._nav_busy = True
        threading.Thread(target=self._run_nav_flow, daemon=True).start()
    def _run_nav_flow(self):
        try:
            nav = TurtleBot4Navigator("/robot8")
            # Nav2 활성 대기
            self.get_logger().info("Waiting for Nav2 to become active...")
            nav.waitUntilNav2Active()
            # undock (필요 시)
            try:
                if nav.getDockedStatus():
                    nav.undock()
                    self.get_logger().info("Undocking...")
            except Exception as e:
                self.get_logger().warn(f"Undock skipped: {e}")

            # 목표 pose # 수정
            target = self.targets[self.current_target_idx]
            goal_pose = nav.getPoseStamped([target[0], target[1]], self._dir_enum(target[2]))
            self.get_logger().info(f"Navigating to ({target[0]:.2f}, {target[1]:.2f}) dir={target[2]}")
            nav.startToPose(goal_pose)

            # 완료 대기 (타임아웃 포함)
            t0 = time.time()
            while not nav.isTaskComplete():
                if time.time() - t0 > self.timeout_sec:
                    nav.cancelTask()
                    self.get_logger().warn("Navigation timeout; canceled.")
                    break
                time.sleep(0.2)

            arrived = nav.isTaskComplete()
            self.pub_arrived.publish(Bool(data=arrived))
            if arrived:
                self.get_logger().info("Arrived at goal, switching to INFERENCING")
                self._transition_to(self.STATE_INF)
            else:
                self._transition_to(self.STATE_WAIT)
        except Exception as e:
            self.get_logger().error(f"Navigation error: {e}")
            self._transition_to(self.STATE_WAIT)
        finally:
            self._nav_busy = False
    # ---------- Inference ----------
    def infer_tick(self):
        if self.state != self.STATE_INF:
            return
        if self.latched or self.rgb_image is None:
            return
        
        try:
            if self.bypass:
                n_people = self.audience_count
            else:
                frame = self.rgb_image.copy()
                results = self.model.predict(source=frame, classes=[0], conf=self.conf, verbose=False)
                r = results[0] if results else None
                n_people = int(len(r.boxes)) if (r is not None and getattr(r, 'boxes', None) is not None) else 0
            # 연속 일치 판정
            if (self.audience_count != 0) and (n_people == self.audience_count):
                self.consec_match += 1
            else:
                self.consec_match = 0
            if self.consec_match >= self.match_threshold:
                self.pub_people_check.publish(Bool(data=True))
                self.latched = True
                self._transition_to(self.STATE_LATCHED)
                self.get_logger().info(
                    f'people_check -> True (matched {self.consec_match} frames). Latched.'
                )
                self._action_requested = True

            # ===== 다음 좌표 이동 =====
            if self.current_target_idx + 1 < len(self.targets):          
                self.current_target_idx += 1                             
                self._transition_to(self.STATE_NAV)                      
                self._start_navigation_thread()           

        except Exception as e:
            self.get_logger().error(f'Inference failed: {e}')
    # ---------- Utils ----------
    def _transition_to(self, new_state: str):
        if new_state == self.state:
            return
        self.get_logger().info(f'State: {self.state} -> {new_state}')
        self.state = new_state
        if new_state in (self.STATE_WAIT, self.STATE_NAV):
            # 추론 카운터/래치 초기화
            self.consec_match = 0
            self.latched = False
def main():
    rclpy.init()
    node = Turtlebot4GuideCounter()
    try:
        # 콜백/타이머 동시성 여유
        executor = MultiThreadedExecutor(num_threads=2)
        executor.add_node(node)
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
if __name__ == '__main__':
    main()