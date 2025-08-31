# --------- ROS/메시지/액션/TF 관련 import ---------
from std_msgs.msg import Bool  # (옵션) 완료 신호 퍼블리시용
from geometry_msgs.msg import PointStamped, PoseStamped, PoseWithCovarianceStamped  # 포인트/포즈 메시지
from nav2_msgs.action import NavigateToPose  # Nav2의 내비게이션 액션 정의
from rclpy.action import ActionClient  # ROS2 액션 클라이언트
from tf2_ros import Buffer, TransformListener  # TF 변환 버퍼/리스너
from rclpy.node import Node  # ROS2 노드 기본 클래스
from voice_processing import tts
from std_msgs.msg import String

import tf2_geometry_msgs  # PointStamped 등 TF 변환 타입 지원을 위한 임포트(실사용은 없지만 등록 용도)
# --------- ROS2 유틸/시간/수학/외부프로세스 ---------
import rclpy  # ROS2 파이썬 클라이언트 라이브러리
from rclpy.duration import Duration  # TF 변환 타임아웃 등에 사용
import time  # 디바운스 및 로그 주기 제어
import math  # 거리 계산(hypot) 등에 사용
import shlex, subprocess  # TTS 명령 실행(비블로킹) 안전 처리

class TrackerPersonNode(Node):
    """
    사람 포인트('/robot8/point_camera')만 추종하는 단순 트래커.
    - 카메라 좌표계의 포인트를 TF로 map 좌표계로 변환
    - NavigateToPose 액션으로 목표를 주기적 갱신(디바운스 적용)
    - 남은 거리 < close_enough_distance 조건이 연속 3회면 업데이트 일시 차단
    - /robot8/amcl_pose 로봇 위치가 특정 존에 진입하면: goal 취소 + TTS + wait_sec 후 자동 재개
    """
    def __init__(self):
        super().__init__('tracker_person_node')  # 노드 이름 설정

        # ---------- 파라미터 선언(기본값 포함) ----------
        self.declare_parameter('map_frame', 'map')  # TF 변환 대상 프레임
        self.declare_parameter('close_enough_distance', 1.0)  # 근접 판정 임계값(m)
        self.declare_parameter('min_goal_interval_sec', 0.5)  # 목표 갱신 최소 시간 간격(s)
        self.declare_parameter('min_goal_translation', 0.15)  # 목표 갱신 최소 이동 거리(m)
        self.declare_parameter('approach_offset', 0.3)  # 사람에게 다가갈 때 z축으로 뺄 거리(m)
        self.declare_parameter('zone_check_hz', 10.0)  # 존 체크 주파수(Hz)
        self.declare_parameter('tts_command', 'espeak -s 160 -v ko+f3 "{text}"')  # 로컬 TTS 명령 템플릿
        
        # NOTE: 이 부분이 수정된 부분입니다.
        # zones: ROS 2 파라미터가 아닌, 일반적인 클래스 변수로 직접 정의합니다.
        self.zones = [
            {'name': 'gallery_A', 'x': -1.02, 'y': -0.74, 'radius': 1,
             'tts': '이 작품은 핑구 킹입니다.', 'wait_sec': 8},
            {'name': 'gallery_B', 'x': -1.68, 'y': 2.85, 'radius': 1,
             'tts': '이 작품은 요리하는 핑구입니다.', 'wait_sec': 8},
            {'name': 'gallery_C', 'x': -3.88, 'y': 0.363, 'radius': 1,
             'tts': '마지막 작품은 화난 핑구입니다.', 'wait_sec': 8},
        ]
        
        # ---------- 파라미터 실제 값 가져오기 ----------
        # zones는 파라미터가 아니므로 get_parameter()로 가져오지 않습니다.
        self.map_frame = self.get_parameter('map_frame').value
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

        # ---------- 구독자/퍼블리셔 ----------
        self.create_subscription(PointStamped, '/robot8/point_camera', self.cb_point_from_camera, 10)  # 사람 포인트 입력
        self.create_subscription(PoseWithCovarianceStamped, '/robot8/amcl_pose', self.cb_amcl, 10)  # 로봇 현재 위치(존 체크용)
        self.pub_done = self.create_publisher(Bool, '/robot8/is_done_track', 10)  # (옵션) 완료 신호, 필요 없으면 제거 가능
        self.pub_zone_event = self.create_publisher(String, '/robot8/now_loc', 10)
        self.pub_ready_to_track = self.create_publisher(Bool, '/robot8/go_to_track', 10)
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
        self.active_zone = None               # 현재 설명 중인 존 이름
        # zones를 파라미터로 가져오지 않으므로, 이 부분도 수정합니다.
        self.zone_inside_flags = {z['name']: False for z in self.zones}  # 존 재진입 방지 히스테리시스

        # ---------- 존 체크 타이머 ----------
        # 주기적으로 현재 위치가 어느 존에 들어왔는지 감시
        self.create_timer(1.0 / max(1.0, self.zone_check_hz), self.zone_check_tick)

        self.get_logger().info('tracker_person_node started.')  # 시작 로그

    # ===================== 콜백: AMCL 위치 수신 =====================
    def cb_amcl(self, msg: PoseWithCovarianceStamped):
        self.robot_xy = (msg.pose.pose.position.x, msg.pose.pose.position.y)  # 로봇의 현재 (x,y) 저장

    # ===================== 콜백: 카메라 포인트 수신 =====================
    def cb_point_from_camera(self, pt: PointStamped):
        """카메라 좌표계에서 들어온 사람 포인트를 map으로 변환하고 goal을 전송(디바운스 적용)."""
        if self.paused_for_zone or self.block_goal_updates:
            return  # 존 설명 중이거나 근접 래치면 목표 갱신 중단

        try:
            # 사람에게 너무 가까이 붙지 않도록 카메라 Z를 약간 줄여 목표를 살짝 앞에 두기
            if pt.point.z > self.approach_offset:
                pt.point.z = pt.point.z - self.approach_offset

            # TF 변환: 카메라 프레임 → map 프레임 (타임아웃 0.5초)
            pt_map = self.tf_buffer.transform(pt, self.map_frame, timeout=Duration(seconds=0.5))
            self.latest_map_point = pt_map  # 최근 변환 결과 저장
            xy = (pt_map.point.x, pt_map.point.y)  # 목표 좌표 (x,y) 추출
        except Exception as e:
            self.get_logger().warn(f'TF to "{self.map_frame}" failed: {e}')  # 변환 실패 시 경고 후 종료
            return

        # ---- 디바운스(시간) : 너무 자주 목표를 보내지 않도록 최소 간격 보장 ----
        now = time.time()
        if (now - self.last_goal_send_time) < self.min_goal_interval:
            return  # 최소 보낸 지점 이후 min_goal_interval이 지나지 않으면 건너뜀

        # ---- 디바운스(거리) : 목표 이동이 너무 작으면 무시 ----
        if self.last_goal_xy is not None:
            dx = xy[0] - self.last_goal_xy[0]
            dy = xy[1] - self.last_goal_xy[1]
            if math.hypot(dx, dy) < self.min_goal_translation:
                return  # 이동량이 임계값보다 작으면 갱신하지 않음

        # 진행 중 goal이 있으면 갱신을 위해 취소(새 goal 송신 전)
        if self.goal_handle:
            try:
                self.goal_handle.cancel_goal_async()  # 비동기 취소 요청
            except Exception:
                pass  # 취소 중 예외는 무시

        # 실제 goal 송신
        self._send_goal(xy)
        self.last_goal_send_time = now  # 마지막 송신 시각 갱신
        self.last_goal_xy = xy         # 마지막 목표 좌표 갱신

    # ===================== 헬퍼: Goal 송신 =====================
    def _send_goal(self, xy):
        # PoseStamped 헤더/포지션 설정
        self.pose.header.frame_id = self.map_frame  # 목표 프레임은 map
        self.pose.header.stamp = self.get_clock().now().to_msg()  # 현재 시간 스탬프
        self.pose.pose.position.x = xy[0]  # 목표 x
        self.pose.pose.position.y = xy[1]  # 목표 y
        self.pose.pose.orientation.w = 1.0  # 간단히 yaw=0 (필요하면 atan2로 yaw 계산 가능)

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
        status = future.result().status  # 종료 상태 코드
        self.get_logger().info(f"Goal finished with result code: {status}")  # 상태 출력
        self.goal_handle = None  # 현재 goal 핸들 비우기

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

        text = str(triggered.get('tts', '안내 멘트가 설정되어 있지 않습니다.'))
        self.get_logger().info(f'[ZONE:{triggered["name"]}] TTS: "{text}" (resume immediately)')
        self._run_tts_async(text)

        # 토픽 발행
        msg = String()
        msg.data = triggered['name']
        self.pub_zone_event.publish(msg)

        # ---- 즉시 추적 재개 ----
        self.block_goal_updates = False
        self.close_distance_hit_count = 0

        # Bool 퍼블리시 (A, C 존에서만)
        if triggered['name'] in ['gallery_A', 'gallery_C']:
            ready_msg = Bool()
            ready_msg.data = True
            self.pub_ready_to_track.publish(ready_msg)

    # ===================== 로컬 TTS 실행(비블로킹) =====================
    def _run_tts_async(self, text: str):
        try:
            # voice_processing/tts.py 안의 say_async 사용
            tts.say_async(text)
        except Exception as e:
            self.get_logger().error(f'TTS failed: {e}')

# ===================== 엔트리 포인트 =====================
def main(args=None):
    rclpy.init(args=args)         # ROS2 초기화
    node = TrackerPersonNode()    # 노드 인스턴스 생성
    try:
        rclpy.spin(node)          # 콜백/타이머를 돌리며 대기
    except KeyboardInterrupt:
        pass                      # Ctrl+C 등으로 종료
    finally:
        if rclpy.ok():
            node.destroy_node()   # 노드 정리
            rclpy.shutdown()      # ROS2 종료

if __name__ == '__main__':  # 스크립트 직접 실행 시
    main()                  # main() 호출
