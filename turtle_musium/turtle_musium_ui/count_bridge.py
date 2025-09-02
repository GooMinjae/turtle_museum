# count_bridge.py
import threading
from PyQt5.QtCore import QObject, pyqtSignal
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from std_msgs.msg import Int32
import os

import time
class CountBridge(QObject):
    countDone = pyqtSignal(int)

    def __init__(self, req_topic="/robot8/audience_count", done_topic="/robot8/people_check"):
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
        # 1) 이 브리지 전용 Context
        self._ctx = Context()
        rclpy.init(context=self._ctx)

        # 2) 노드 생성 (전용 context로)
        self._node = Node("count_bridge", context=self._ctx)
        self._node.get_logger().info("count_bridge started")    
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST, depth=10
        )
        self._pub_req = self._node.create_publisher(Int32, self._req_topic, qos)

        self._sub_done = self._node.create_subscription(Int32, self._done_topic, self._cb_done, qos)

        # 3) 이 스레드 전용 Executor 만들고 노드 등록
        self._executor = SingleThreadedExecutor(context=self._ctx)
        self._executor.add_node(self._node)

        try:
            # 4) 전용 executor로 spin_once 루프
            while self._running and rclpy.ok(context=self._ctx):
                self._executor.spin_once(timeout_sec=0.1)
        finally:
            # 5) 정리: executor → node → shutdown 순
            try:
                self._executor.remove_node(self._node)
            except Exception:
                pass
            try:
                self._node.destroy_node()
            except Exception:
                pass
            try:
                self._executor.shutdown()
            except Exception:
                pass
            try:
                rclpy.shutdown(context=self._ctx)
            except TypeError:
                # (버전에 따라 context 인자 없을 수도 있음)
                rclpy.shutdown()

    def _cb_done(self, msg: Int32):
        self._node.get_logger().info(f"recv /tour/count/done: {msg.data}")  # 디버그
        self.countDone.emit(int(msg.data))

    def publish_expected_count(self, n: int, repeat: int = 80, delay: float = 0.05):
        if not self._node or not self._pub_req:
            return
        msg = Int32()
        msg.data = int(n)
        for i in range(repeat):
            self._node.get_logger().info(f"Pub {i+1}/{repeat}")
            self._pub_req.publish(msg)
            time.sleep(delay)   # 너무 빠르면 subscriber가 놓칠 수 있어서 약간의 간격을 줌


    def stop(self):
        self._running = False
        if self._thread:
            self._thread.join()
