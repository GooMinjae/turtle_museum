# artwork_bridge.py
import json
import threading
from PyQt5.QtCore import QObject, pyqtSignal
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool

# 1) 작품 ID -> (이미지 경로, 설명) 매핑
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
    showArtwork = pyqtSignal(str, str)  # image_path, description
    trackDone  = pyqtSignal(bool)

    def __init__(self, topic_name="/robot8/now_loc", piece_db=None):
        super().__init__()
        self._topic = topic_name
        self._running = False
        self._thread = None
        self._node = None
        self._db = piece_db or PIECE_DB
        self.going_piece_idx = 2

    def start(self):
        if self._running: return
        self._running = True
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()


    def _run(self):
        rclpy.init(args=None)
        self._node = Node("artwork_bridge")

        # 작품 표시 토픽
        def _cb_show(msg: String):
            piece_id = msg.data.strip()
            entry = self._db.get(piece_id)
            if entry:
                img = entry.get("image", "")
                desc = entry.get("desc", "")
                self.showArtwork.emit(img, desc)
            elif piece_id=="None":
                self.showArtwork.emit(f"작품 {self.going_piece_idx} 이동 중입니다.", f"잠시만 기다려 주세요…")
                self.going_piece_idx += 1
                pass
            else:
                self.showArtwork.emit("", f"알 수 없는 작품 ID: {piece_id}")

        self._node.create_subscription(String, self._topic, _cb_show, 10)

        # 완료 토픽 구독: /robot8/is_done_track (Bool)
        def _cb_done(msg: Bool):
            self.trackDone.emit(bool(msg.data))  # True면 다음 페이지로 넘어감

        self._node.create_subscription(Bool, "/robot8/is_done_track", _cb_done, 10)

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
