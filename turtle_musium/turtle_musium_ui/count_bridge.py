# count_bridge.py
import threading
from PyQt5.QtCore import QObject, pyqtSignal
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from std_msgs.msg import Int32

class CountBridge(QObject):
    countDone = pyqtSignal(int)  # /tour/count/done 수신 시 방출

    def __init__(self, req_topic="/tour/count/request", done_topic="/tour/count/done"):
        super().__init__()
        self._req_topic = req_topic
        self._done_topic = done_topic
        self._running = False
        self._thread = None
        self._node = None

    def start(self):
        if self._running: return
        self._running = True
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def _run(self):
        rclpy.init(args=None)
        qos_pub = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST, depth=1
        )
        qos_sub = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST, depth=1
        )
        self._node = Node("count_bridge")

        self._pub_req = self._node.create_publisher(Int32, self._req_topic, qos_pub)

        def _cb_done(msg: Int32):
            self.countDone.emit(int(msg.data))

        self._node.create_subscription(Int32, self._done_topic, _cb_done, qos_sub)

        try:
            while self._running and rclpy.ok():
                rclpy.spin_once(self._node, timeout_sec=0.1)
        finally:
            if self._node:
                self._node.destroy_node()
            rclpy.shutdown()

    def publish_expected_count(self, n: int):
        if not self._node: return
        msg = Int32(); msg.data = int(n)
        self._pub_req.publish(msg)

    def stop(self):
        self._running = False
        if self._thread:
            self._thread.join()
