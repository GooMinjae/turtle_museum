# pc2_bridge.py
import json, threading
from datetime import datetime
from PyQt5.QtCore import QObject, pyqtSignal
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import String, Int32MultiArray
from std_srvs.srv import Trigger
from data_base.visit_db import log_artwork_state, get_gift_counts_for_visit


GIFT_KEYS = ["gift_moo", "gift_pinga", "gift_haowl", "gift_pingu"]

class Pc2Bridge(QObject):
    """PC2 통신 브리지: 작품상황 구독 + 기념품 재고 질의 서비스 응답"""
    # DB 기록이 일어났을 때 UI가 필요하면 이 신호로 알림
    artworkEvent = pyqtSignal(dict)

    def __init__(self, topic_art="/robot9/gallery_state", srv_gift="/robot9/gift_data"):
        super().__init__()
        self._topic_art = topic_art
        self._srv_gift  = srv_gift
        self._running = False
        self._thread = None
        self._node = None
        self.current_visit_id = None  # MonitoringApp에서 설정

        # 추가: 작품 ID 목록과 상태 초기화
        self.piece_ids = ["gallery_A", "gallery_B", "gallery_C"]
        self._last_present = [None] * len(self.piece_ids)

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

        # 1) 작품 상황 구독: Int32MultiArray로 [1,0,1,...]
        def _cb_art(msg: Int32MultiArray):
            try:
                vals = list(msg.data)  # 예: [1, 0, 1]
                # piece_ids 길이에 맞춰 자르거나, 부족하면 0으로 패딩
                if len(vals) < len(self.piece_ids):
                    vals = vals + [0] * (len(self.piece_ids) - len(vals))
                else:
                    vals = vals[:len(self.piece_ids)]

                now = datetime.now().isoformat(timespec="seconds")
                for i, v in enumerate(vals):
                    present = bool(v)

                    # 변화가 없으면 스킵 (로그/시그널 중복 방지)
                    if self._last_present[i] is not None and present == self._last_present[i]:
                        continue

                    self._last_present[i] = present
                    state = "arrived" if present else "missing"
                    piece_id = self.piece_ids[i]

                    payload = {
                        "piece_id": piece_id,
                        "present": present,
                        "state": state,
                        "ts": now,
                    }

                    # 방문 ID가 설정되어 있으면 DB 기록
                    if self.current_visit_id is not None:
                        try:
                            log_artwork_state(
                                int(self.current_visit_id),
                                str(piece_id),
                                str(state),
                                str(now),
                            )
                        except Exception as e:
                            self._node.get_logger().warn(f"log_artwork_state error: {e}")

                    # UI로 알림
                    self.artworkEvent.emit(payload)

            except Exception as e:
                self._node.get_logger().warn(f"artwork_state parse/save error: {e}")

        self._node.create_subscription(Int32MultiArray, self._topic_art, _cb_art, qos)

        # 2) 기념품 재고 질의 서비스(Trigger: message에 JSON 반환)
        # def _on_gift_req(req, resp):
        #     try:
        #         vid = int(self.current_visit_id) if self.current_visit_id is not None else None
        #         if vid is None:
        #             resp.success = False
        #             resp.message = "no-visit"
        #             return resp
        #         data = get_gift_counts_for_visit(vid)
        #         resp.success = True
        #         resp.message = json.dumps(data)  # {"moo":..,"pinga":..,"haowl":..,"pingu":..}
        #         return resp
        #     except Exception as e:
        #         resp.success = False
        #         resp.message = f"error:{e}"
        #         return resp

        # self._node.create_service(Trigger, self._srv_gift, _on_gift_req)

        def _cb_gift(req, resp):
            try:
                if self.current_visit_id is None:
                    resp.success = False
                    resp.message = "visit_id not set"
                    return resp

                counts = get_gift_counts_for_visit(int(self.current_visit_id))
                # dict가 아니면 dict로 변환
                if not isinstance(counts, dict):
                    counts = {k: int(counts[i]) if i < len(counts) else 0
                            for i, k in enumerate(GIFT_KEYS)}

                # 재고 > 0 → '1', 아니면 '0'
                bitstring = ''.join('1' if int(counts.get(k, 0)) > 0 else '0' for k in GIFT_KEYS)

                resp.success = True
                resp.message = bitstring           # 예: "1001"
                return resp
            except Exception as e:
                resp.success = False
                resp.message = f"gift query error: {e}"
                return resp

        self._node.create_service(Trigger, self._srv_gift, _cb_gift)

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
