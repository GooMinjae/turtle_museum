# page01_init_screen.py
import threading
import rclpy
from rclpy.node import Node
from rclpy.context import Context
from PyQt5.QtCore import QObject, pyqtSignal, Qt
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rclpy.executors import SingleThreadedExecutor

from std_msgs.msg import String, Bool, Int32MultiArray
from irobot_create_msgs.msg import DockStatus
from sensor_msgs.msg import BatteryState


class _UiBridge(QObject):
    guide_dock_text = pyqtSignal(str)
    guide_batt_text = pyqtSignal(str)

    patrol_dock_text = pyqtSignal(str)
    patrol_batt_text = pyqtSignal(str)

class _InitStatusNode(Node):
    def __init__(self, bridge: _UiBridge, *, 
                 guide_topic_dock: str, guide_topic_batt: str,
                 patrol_topic_dock: str, patrol_topic_batt: str,
                 dock_type: str = "string", context: Context = None):
        super().__init__('init_status_node', context=context)
        self.bridge = bridge
        qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                         history=HistoryPolicy.KEEP_LAST, depth=10)


        # Dock
        self.create_subscription(DockStatus, guide_topic_dock, self._on_guide_dock_status, qos)

        self.create_subscription(DockStatus, patrol_topic_dock, self._on_patrol_dock_status, qos)

        # Battery state
        self.sub_guide_batt = self.create_subscription(
            BatteryState, guide_topic_batt, self._on_guide_batt, 10
        )

        # Battery state
        self.sub_parol_batt = self.create_subscription(
            BatteryState, patrol_topic_batt, self._on_patrol_batt, 10
        )

    def _on_guide_dock_status(self, msg: DockStatus):
        self.get_logger().info("dock cb")
        state = "Docked" if getattr(msg, "is_docked", False) else "Undocked"
        self.bridge.guide_dock_text.emit(f"Dock: {state}")

    def _on_patrol_dock_status(self, msg: DockStatus):
        self.get_logger().info("dock cb")
        state = "Docked" if getattr(msg, "is_docked", False) else "Undocked"
        self.bridge.patrol_dock_text.emit(f"Dock: {state}")

    def _on_guide_batt(self, msg: BatteryState):
        # BatteryState.percentage: 0.0~1.0 범위인 경우가 일반적
        pct = msg.percentage
        if pct is None:
            self.bridge.guide_batt_text.emit("Battery: N/A")
            return
        # 일부 드라이버는 0~100으로 주기도 함 → 1.0 초과면 그대로 사용
        value = pct * 100.0 if 0.0 <= pct <= 1.0 else pct
        self.bridge.guide_batt_text.emit(f"Battery: {value:.1f}%")

    def _on_patrol_batt(self, msg: BatteryState):
        # BatteryState.percentage: 0.0~1.0 범위인 경우가 일반적
        pct = msg.percentage
        if pct is None:
            self.bridge.patrol_batt_text.emit("Battery: N/A")
            return
        # 일부 드라이버는 0~100으로 주기도 함 → 1.0 초과면 그대로 사용
        value = pct * 100.0 if 0.0 <= pct <= 1.0 else pct
        self.bridge.patrol_batt_text.emit(f"Battery: {value:.1f}%")


class Page01InitScreen:
    """
    첫 페이지 상태 라벨 세팅/해제 컨트롤러
    - set_ui(...)로 라벨과 토픽 이름, 도크 타입을 지정
    - start()로 구독 시작, close_app()로 종료
    - 별도 rclpy.Context를 사용해 다른 페이지의 rclpy와 충돌 방지
    """
    def __init__(self):
        self.bridge = _UiBridge()
        self.ctx = None
        self.node = None
        self._spin_thread = None
        self._running = False

        # Qt labels (주입)
        self._guide_dock_label = None
        self._guide_batt_label = None

        self._patrol_dock_label = None
        self._patrol_batt_label = None

        # Default topics
        self.guide_topic_dock = "/robot8/dock_status"
        self.guide_topic_batt = "/robot8/battery_state"

        self.patrol_topic_dock = "/robot9/dock_status"
        self.patrol_topic_batt = "/robot9/battery_state"
        self.dock_type = DockStatus()  # "string" 또는 "bool"

    def set_ui(self, *, guide_dock_label, guide_battery_label, patrol_dock_label, patrol_battery_label,
               guide_topic_dock="/robot8/dock_status", guide_topic_batt="/robot8/battery_state",
               patrol_topic_dock="/robot9/dock_status", patrol_topic_batt="/robot9/battery_state",
               dock_type="string"):
        self._guide_dock_label = guide_dock_label
        self._guide_batt_label = guide_battery_label
        self.guide_topic_dock = guide_topic_dock
        self.guide_topic_batt = guide_topic_batt

        self._patrol_dock_label = patrol_dock_label
        self._patrol_batt_label = patrol_battery_label
        self.patrol_topic_dock = patrol_topic_dock
        self.patrol_topic_batt = patrol_topic_batt
        self.dock_type = dock_type

        # 초기 텍스트
        self._guide_dock_label.setText("Dock: ---")
        self._guide_batt_label.setText("Battery: ---")

        self._patrol_dock_label.setText("Dock: ---")
        self._patrol_batt_label.setText("Battery: ---")

        self.bridge.guide_dock_text.connect(self._guide_dock_label.setText, Qt.QueuedConnection)
        self.bridge.guide_batt_text.connect(self._guide_batt_label.setText, Qt.QueuedConnection)
        self.bridge.patrol_dock_text.connect(self._patrol_dock_label.setText, Qt.QueuedConnection)
        self.bridge.patrol_batt_text.connect(self._patrol_batt_label.setText, Qt.QueuedConnection)

    def start(self):
        if self._running:
            return
        self.ctx = Context()
        rclpy.init(context=self.ctx)

        self.node = _InitStatusNode(
            self.bridge,
            guide_topic_dock=self.guide_topic_dock,
            guide_topic_batt=self.guide_topic_batt,
            patrol_topic_dock=self.patrol_topic_dock,
            patrol_topic_batt=self.patrol_topic_batt,
            dock_type=self.dock_type,
            context=self.ctx
        )

        # 전용 executor 준비
        self._executor = SingleThreadedExecutor(context=self.ctx)
        self._executor.add_node(self.node)

        self._running = True
        self._spin_thread = threading.Thread(target=self._spin, daemon=True)
        self._spin_thread.start()

    def _spin(self):
        try:
            # context= 키워드 인자 절대 쓰지 않습니다.
            while self._running and rclpy.ok(context=self.ctx):
                self._executor.spin_once(timeout_sec=0.1)
        finally:
            try:
                self._executor.remove_node(self.node)
            except Exception:
                pass
            try:
                self._executor.shutdown()
            except Exception:
                pass
            # node/context 정리는 close_app()에서 최종적으로 해도 됩니다.

    def close_app(self):
        if not self._running:
            return
        # 1) 루프 중지 요청
        self._running = False

        # 2) 스레드 합류 (spin 루프 종료 대기)
        if self._spin_thread:
            self._spin_thread.join(timeout=1.0)
            self._spin_thread = None

        # 3) 노드/컨텍스트 정리
        try:
            if self.node is not None:
                self.node.destroy_node()
                self.node = None
        except Exception:
            pass
        try:
            if self.ctx is not None and rclpy.ok(context=self.ctx):
                rclpy.shutdown(context=self.ctx)
        except Exception:
            pass
        self.ctx = None

