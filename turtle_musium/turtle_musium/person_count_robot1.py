#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
import math


from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType
from std_msgs.msg import Int32, Bool
from sensor_msgs.msg import CompressedImage
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
      - INFERENCING: 관객 수 일치 여부 검사(연속 N프레임)
      - LATCHED: 일치 확인됨(people_check 발행 후 래치)
    """
    STATE_WAIT = 0
    STATE_INF = 1
    STATE_LATCHED = 2

    def __init__(self):
        super().__init__('robot8_guide_counter')

        # QoS
        self.qos_sensor = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )

        # ===== Parameters (기존 유지) =====
        self.declare_parameter('input_topic', '/robot8/oakd/rgb/image_raw/compressed')
        self.declare_parameter('conf', 0.7)
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
        self.topic_detected_people = '/robot8/audience_count_from_detector'

        self.input_topic = self.get_parameter('input_topic').get_parameter_value().string_value
        self.conf = float(self.get_parameter('conf').value)
        self.match_threshold = int(self.get_parameter('match_threshold').value)
        self.bypass = bool(self.get_parameter('bypass_inference').value)
        self.initial_x = float(self.get_parameter('initial_x').value)
        self.initial_y = float(self.get_parameter('initial_y').value)
        self.initial_dir = str(self.get_parameter('initial_dir').value).upper()
        self.navigator = TurtleBot4Navigator("/robot8")
        self.get_logger().info("Waiting for Nav2 to become active...")
        self.navigator.waitUntilNav2Active()
        try:
            if self.navigator.getDockedStatus():
                self.navigator.undock()
        except Exception:
            pass
                # ===== 추가 목표 좌표 (순차 이동용) =====
        self.targets = [
            (-0.283, 1.12, "WEST"),   # 첫 번째 목표 (지금 쓰는 것)
            (-0.0671, -0.367, "SOUTH"),     # 두 번째 목표
            (0, 0, "NORTH") # home
        ]
        self.current_target_idx = 0
        self.timeout_sec = float(self.get_parameter('timeout_sec').value)
        self.bypass = True  # 탐지 노드가 대신 추론하므로 항상 True로 운용

        # 상태
        self.detected_people_count = 0
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

        self.goal_x = float(self.get_parameter('goal_x').value)
        self.goal_y = float(self.get_parameter('goal_y').value)
        self.goal_dir = self.get_parameter('goal_dir').get_parameter_value().string_value

        self.nav_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')

        # ===== Topics (기존 유지) =====
        self.topic_req_people = '/robot8/people'           # (있다면 사용) 외부에서 요구 인원 알림
        self.topic_audience = '/robot8/audience_count'      # 목표 인원수(Int32)
        self.topic_people_check = '/robot8/people_check'    # 일치 시 퍼블리시(Int32)
        # 신규: 탐지 노드 제공 실 감지 인원수
        self.create_subscription(Int32, self.topic_audience, self.audience_cb, 10)
        self.create_subscription(Int32, self.topic_detected_people, self.detected_cb, 10)
        self.pub_people_check = self.create_publisher(Int32, '/robot8/people_check', 10)
        self.pub_people = self.create_publisher(Bool, '/robot8/people', 10)
        self.pub_arrived = self.create_publisher(Bool, '/robot8/arrived', 10)

        # ===== IO =====
        # 이미지 구독은 불필요. 기존 파라미터/토픽은 유지하지만 미사용
        # self.create_subscription(CompressedImage, self.input_topic, self.image_cb, self.qos_sensor)




        # 루프 스레드
        self.running = True
        self.th = threading.Thread(target=self.loop, daemon=True)
        self.th.start()


    def _enable_tracker(self, enable: bool = True):
        # tracker 노드 네임스페이스/이름에 맞춰 서비스 이름 구성
        srv_name = '/robot8/tracker_person_node/set_parameters'
        cli = self.create_client(SetParameters, srv_name)
        if not cli.wait_for_service(timeout_sec=3.0):
            self.get_logger().warn(f"tracker param service not available: {srv_name}")
            return
        # 요청 생성
        req = SetParameters.Request()
        p = Parameter()
        p.name = 'enable_follow'
        p.value = ParameterValue(type=ParameterType.PARAMETER_BOOL, bool_value=bool(enable))
        req.parameters = [p]
        fut = cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=3.0)
        if fut.result() is not None:
            self.get_logger().info(f"[people_count] tracker enable_follow set to {enable}")
        else:
            self.get_logger().warn("Failed to set tracker parameter")

    def dir_to_quat(self, dir_str: str):
        yaw_map = {
            'EAST': 0.0,
            'NORTH': math.pi/2,
            'WEST': math.pi,
            'SOUTH': -math.pi/2,
        }
        yaw = yaw_map.get(dir_str.upper(), 0.0)
        return math.cos(yaw/2.0), math.sin(yaw/2.0)

    def send_goal(self, x, y, dir_str):
        if not self.nav_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().warn("NavigateToPose server not available")
            return
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        qw, qz = self.dir_to_quat(dir_str)
        pose.pose.orientation.w = qw
        pose.pose.orientation.z = qz
        goal = NavigateToPose.Goal()
        goal.pose = pose
        self.nav_client.send_goal_async(goal)
        self.get_logger().info(f"[NAV2] Goal sent: ({x}, {y}, dir={dir_str})")


    # ===== Callbacks =====
    def audience_cb(self, msg: Int32):
        self.audience_count = int(msg.data)
        self.get_logger().info(f"[audience_count] -> {self.audience_count}")
        if self.state == self.STATE_WAIT and self.audience_count > 0:
            self.state = self.STATE_INF
            self.consec_match = 0
            threading.Thread(target=self._run_nav_flow, daemon=True).start()
            self.latched = False

    def detected_cb(self, msg: Int32):
        self.detected_people_count = int(msg.data)

    # ===== Main Loop =====
    def loop(self):
        rate = self.create_rate(15)
        start_t = time.time()
        while rclpy.ok() and self.running:
            try:
                if self.state == self.STATE_INF and not self.latched:
                    self.infer_tick()
                # timeout 처리(기존 유지)
                if (time.time() - start_t) > self.timeout_sec:
                    self.get_logger().warn("counting timeout")
                    start_t = time.time()
            except Exception as e:
                self.get_logger().warn(f"loop error: {e}")
            rate.sleep()

    def _dir_enum(self, name: str):
        return getattr(TurtleBot4Directions, name)
    def _start_navigation_thread(self):
        self._nav_busy = True
        threading.Thread(target=self._run_nav_flow, daemon=True).start()
    def _run_nav_flow(self):
        try:
            nav = self.navigator
            self.get_logger().info("Waiting for Nav2 to become active...")
            nav.waitUntilNav2Active()

            try:
                if nav.getDockedStatus():
                    nav.undock()
                    self.get_logger().info("Undocking...")
            except Exception as e:
                self.get_logger().warn(f"Undock skipped: {e}")

            target = self.targets[self.current_target_idx]
            goal_pose = nav.getPoseStamped([target[0], target[1]], self._dir_enum(target[2]))
            self.get_logger().info(f"Navigating to ({target[0]:.2f}, {target[1]:.2f}) dir={target[2]}")
            nav.startToPose(goal_pose)

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
                if self.current_target_idx == len(self.targets) - 2:
                    self.pub_people.publish(Bool(data=True))
                    self._transition_to(self.STATE_INF)

                elif self.current_target_idx == len(self.targets) - 1:
                    # ✅ 마지막 목표 도착: 루프 종료
                    self.get_logger().info("Final target reached. Shutting down.")

                    arrived_back = nav.isTaskComplete()
                    if arrived_back:
                        self.get_logger().info("Return OK; enabling tracker and finishing.")
                        self._enable_tracker(True)   # ✅ 여기서 tracker ON
                    else:
                        self.get_logger().warn("Return FAILED; tracker remains OFF.")

                    # 마무리(원하면 종료)
                    # self.running = False
                    return

                else:
                    self._transition_to(self.STATE_INF)
            else:
                self._transition_to(self.STATE_WAIT)

        except Exception as e:
            self.get_logger().error(f"Navigation error: {e}")
            self._transition_to(self.STATE_WAIT)
        finally:
            self._nav_busy = False

    def infer_tick(self):
        # 탐지 노드에서 전달된 값 사용
        n_people = int(self.detected_people_count)

        if (self.audience_count != 0) and (n_people == self.audience_count):
            self.consec_match += 1
        else:
            self.consec_match = 0

        self.get_logger().info(f"[people] detected={n_people}, target={self.audience_count}, match={self.consec_match}/{self.match_threshold}")

        if self.consec_match >= self.match_threshold:
            self.pub_people_check.publish(Int32(data=self.audience_count))
            self.latched = True
            self.state = self.STATE_LATCHED
            self.get_logger().info("[people_check] MATCH confirmed and latched")
            if hasattr(self, "targets") and hasattr(self, "current_target_idx"):
                # 다음 인덱스가 범위 내인지 확인
                if self.current_target_idx < len(self.targets) - 1:
                    self.current_target_idx += 1
                    self.get_logger().info(f"Next target index -> {self.current_target_idx}")
                else:
                    self.get_logger().info("No more targets. Staying latched.")
                    self.state = self.STATE_LATCHED
                    return
            else:
                # targets 없이 goal 파라미터만 쓰는 경우라면, 여기서 끝내거나
                # 별도 goal_x2/goal_y2 파라미터를 쓰는 기존 코드가 이미 있다면 그 로직 호출
                self.state = self.STATE_LATCHED
                return

            # 2) 상태를 WAIT로 전환하고 _run_nav_flow 재개
            self.state = self.STATE_WAIT
            if not getattr(self, "_nav_busy", False):
                self._nav_busy = True
                threading.Thread(target=self._run_nav_flow, daemon=True).start()


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
