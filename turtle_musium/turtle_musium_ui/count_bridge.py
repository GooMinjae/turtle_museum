# count_bridge.py
import threading
from PyQt5.QtCore import QObject, pyqtSignal
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import Int32
import os

class CountBridge(QObject):
    countDone = pyqtSignal(int)

    def __init__(self, req_topic="/tour/count/request", done_topic="/tour/count/done"):
        super().__init__()
        self._req_topic = req_topic
        self._done_topic = done_topic
        self._running = False
        self._thread = None
        self._node = None
        self._pub_req = None
        self._sub_done = None   # ← 중요: 구독 핸들 보관

    def start(self):
        if self._running: return
        self._running = True
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def _run(self):

        # # >>> 환경 강제 통일 (중요)
        # os.environ.setdefault('ROS_DOMAIN_ID', '0')
        # os.environ.pop('ROS_DISCOVERY_SERVER', None)
        # os.environ.pop('ROS_SUPER_CLIENT', None)
        # os.environ['RMW_IMPLEMENTATION'] = 'rmw_fastrtps_cpp'   # cyclonedds 쓰면 그걸로
        # # <<<
        rclpy.init(args=None)
        qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                        history=HistoryPolicy.KEEP_LAST, depth=1)

        self._node = Node("count_bridge")
        self._node.get_logger().info("count_bridge started")  # 디버그

        self._pub_req = self._node.create_publisher(Int32, self._req_topic, qos)

        # def _cb_done(msg: Int32):
        #     self._node.get_logger().info(f"recv /tour/count/done: {msg.data}")  # 디버그
        #     self.countDone.emit(int(msg.data))

        # ← 멤버에 저장 (GC 방지)
        self._sub_done = self._node.create_subscription(Int32, self._done_topic, self._cb_done, qos)

        try:
            while self._running and rclpy.ok():
                rclpy.spin_once(self._node, timeout_sec=0.1)
        finally:
            if self._node:
                self._node.destroy_node()
            rclpy.shutdown()

    def _cb_done(self, msg: Int32):
        self._node.get_logger().info(f"recv /tour/count/done: {msg.data}")  # 디버그
        self.countDone.emit(int(msg.data))

    def publish_expected_count(self, n: int):
        if not self._node or not self._pub_req: return
        msg = Int32(); msg.data = int(n)
        self._pub_req.publish(msg)

    def stop(self):
        self._running = False
        if self._thread:
            self._thread.join()
