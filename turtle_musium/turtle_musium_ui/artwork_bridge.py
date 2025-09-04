# artwork_bridge.py (fixed)
import threading
from PyQt5.QtCore import QObject, pyqtSignal

import rclpy
from rclpy.node import Node
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import String, Bool

from turtle_musium_ui.utils_resource import res_path

# 작품 데이터
PIECE_DB = {
    "gallery_A": {
        "image": "/home/rokey/turtlebot4_ws/src/turtle_musium/resource/images/piece1.png",
        "desc":  '이 작품은 핑구 킹입니다.\n21세기 아동 문학의 주도문화였습니다.\n그리고 대한민국에서도\n많은 사랑을 받았습니다.',
    },
    "gallery_B": {
        "image": "/home/rokey/turtlebot4_ws/src/turtle_musium/resource/images/piece2.png",
        "desc":  "이 작품은 요리하는 핑구입니다.\n에피소드 4에서 핑구가 제빵을 해,\n모든 사람에게 인정을 받는 이야기입니다.",
    },
    "gallery_C": {
        "image": "/home/rokey/turtlebot4_ws/src/turtle_musium/resource/images/piece3.jpg",
        "desc":  "마지막 작품은 화난 핑구입니다.\n이 작품은 2010년대 중반 'Noot Noot!'이라는 밈으로 더 유명한 작품입니다.",
    },
}

class ArtworkBridge(QObject):
    showArtwork = pyqtSignal(str, str)  # (image_path, description)
    trackDone  = pyqtSignal(bool)

    def __init__(self,
                topic_now="/robot8/now_loc",
                topic_done="/robot8/is_done_track",
                piece_db=None,
                use_best_effort=False):
        super().__init__()
        self._topic_now  = topic_now
        self._topic_done = topic_done
        self._db = piece_db or PIECE_DB

        self._running = False
        self._thread = None
        self._ctx = None
        self._node = None
        self._executor = None

        # QoS 선택(필요 시 BEST_EFFORT로)
        self._qos = QoSProfile(
            reliability=(ReliabilityPolicy.BEST_EFFORT if use_best_effort else ReliabilityPolicy.RELIABLE),
            history=HistoryPolicy.KEEP_LAST, depth=10
        )

    def start(self):
        if self._running:
            return
        self._running = True
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def _run(self):
        # per-bridge context/executor
        self._ctx = Context()
        rclpy.init(context=self._ctx)
        self._node = Node("artwork_bridge", context=self._ctx)

        # --- 콜백 ---
        def _cb_show(msg: String):
            piece_id = msg.data.strip()
            entry = self._db.get(piece_id)
            if entry:
                self.showArtwork.emit(entry.get("image", ""), entry.get("desc", ""))
            elif piece_id == "None" or piece_id == "":
                # ★ 이미지 경로는 빈 문자열로, 설명만 보냄(경로 자리에 문구 넣지 않기)
                self.showArtwork.emit("", f"다음 작품 이동 중입니다.\n잠시만 기다려 주세요…")
            else:
                self.showArtwork.emit("", f"알 수 없는 작품 ID: {piece_id}")

        def _cb_done(msg: Bool):
            self.trackDone.emit(bool(msg.data))

        # --- 구독 생성 ---
        self._node.create_subscription(String, self._topic_now,  _cb_show, self._qos)
        self._node.create_subscription(Bool,   self._topic_done, _cb_done, self._qos)

        # 전용 executor로 spin
        self._executor = SingleThreadedExecutor(context=self._ctx)
        self._executor.add_node(self._node)

        try:
            while self._running and rclpy.ok(context=self._ctx):
                self._executor.spin_once(timeout_sec=0.1)
        finally:
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
                rclpy.shutdown()

    def stop(self):
        if not self._running:
            return
        self._running = False
        if self._thread:
            self._thread.join(timeout=1.0)
            self._thread = None
