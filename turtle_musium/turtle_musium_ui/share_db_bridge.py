# pc2_bridge.py
import json, threading
from PyQt5.QtCore import QObject, pyqtSignal
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import String
from std_srvs.srv import Trigger
from data_base.visit_db import log_artwork_state, get_gift_counts_for_visit

class Pc2Bridge(QObject):
    """PC2 통신 브리지: 작품상황 구독 + 기념품 재고 질의 서비스 응답"""
    # DB 기록이 일어났을 때 UI가 필요하면 이 신호로 알림
    artworkEvent = pyqtSignal(dict)

    def __init__(self, topic_art="/pc2/artwork_state", srv_gift="/pc2/gift_counts"):
        super().__init__()
        self._topic_art = topic_art
        self._srv_gift  = srv_gift
        self._running = False
        self._thread = None
        self._node = None
        self.current_visit_id = None  # MonitoringApp에서 설정

    def start(self):
        if self._running: return
        self._running = True
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def _run(self):
        rclpy.init(args=None)
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST, depth=10
        )
        self._node = Node("pc2_bridge")

        # 1) 작품 상황 구독
        def _cb_art(msg: String):
            try:
                payload = json.loads(msg.data)
                # payload 예: {"piece_id":"gallery_A","state":"arrived","ts":"2025-09-02T09:10:11"}
                if self.current_visit_id is not None:
                    log_artwork_state(
                        int(self.current_visit_id),
                        str(payload.get("piece_id","")),
                        str(payload.get("state","")),
                        str(payload.get("ts",""))
                    )
                self.artworkEvent.emit(payload)
            except Exception as e:
                self._node.get_logger().warn(f"artwork_state parse/save error: {e}")

        self._node.create_subscription(String, self._topic_art, _cb_art, qos)

        # 2) 기념품 재고 질의 서비스(Trigger: message에 JSON 반환)
        def _on_gift_req(req, resp):
            try:
                vid = int(self.current_visit_id) if self.current_visit_id is not None else None
                if vid is None:
                    resp.success = False
                    resp.message = "no-visit"
                    return resp
                data = get_gift_counts_for_visit(vid)
                resp.success = True
                resp.message = json.dumps(data)  # {"moo":..,"pinga":..,"haowl":..,"pingu":..}
                return resp
            except Exception as e:
                resp.success = False
                resp.message = f"error:{e}"
                return resp

        self._node.create_service(Trigger, self._srv_gift, _on_gift_req)

        try:
            while self._running and rclpy.ok():
                rclpy.spin_once(self._node, timeout_sec=0.1)
        finally:
            if self._node:
                self._node.destroy_node()
            rclpy.shutdown()

    def stop(self):
        self._running = False
        if self._thread:
            self._thread.join()
