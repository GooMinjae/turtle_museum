# page03_description_art_screen.py (수정본)
import os
import threading
import numpy as np
import cv2
from PyQt5.QtWidgets import QWidget
from PyQt5.QtGui import QPixmap, QImage
from PyQt5.QtCore import Qt, pyqtSignal

import rclpy
from rclpy.node import Node
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import CompressedImage

from turtle_musium_ui.artwork_bridge import ArtworkBridge  # 설명용 그대로 사용

class GuideTracking(QWidget):
    doneSignal = pyqtSignal()

    # 카메라 프레임을 UI 스레드로 올리는 시그널
    guideFrame = pyqtSignal(QImage)
    patrolFrame = pyqtSignal(QImage)

    def __init__(self):
        super().__init__()
        self.bridge = None   # ArtworkBridge (설명/완료 신호)
        self.description_label = None

        self.guide_cam_label = None
        self.patrol_cam_label = None

        # ROS 카메라 구독용
        self._cam_ctx = None
        self._cam_node = None
        self._cam_exec = None
        self._cam_thread = None
        self._cam_running = False

        # Qt 시그널 → 실제 라벨 갱신 연결
        self.guideFrame.connect(self._set_guide_pixmap)
        self.patrolFrame.connect(self._set_patrol_pixmap)

    def set_ui(self, *,
            guide_cam_label,
            patrol_cam_label,
            info_label,
            guide_topic="/robot8/oakd/rgb/image_raw/compressed",
            patrol_topic="/robot9/oakd/rgb/image_raw/compressed",
            now_loc_topic="/robot8/now_loc",
            done_topic="/robot8/is_done_track"):
        """라벨/토픽 주입"""
        self.guide_cam_label = guide_cam_label
        self.patrol_cam_label = patrol_cam_label
        self.description_label = info_label

        # 초기 문구
        if self.guide_cam_label:
            self.guide_cam_label.setText("가이드봇 카메라 연결 중…")
        if self.patrol_cam_label:
            self.patrol_cam_label.setText("패트롤봇 카메라 연결 중…")
        if self.description_label:
            self.description_label.setText("작품 설명 대기 중…")

        # 설명/완료 신호용 ArtworkBridge 시작 (이미지 경로는 무시)
        self.bridge = ArtworkBridge(topic_now=now_loc_topic, topic_done=done_topic)
        self.bridge.showArtwork.connect(self._on_show_artwork_text_only)
        self.bridge.trackDone.connect(self._on_done_track)
        self.bridge.start()

        # 카메라 구독 시작
        self._start_cams(guide_topic, patrol_topic)

    # -------------------------
    # 설명 텍스트 갱신 (이미지는 무시)
    # -------------------------
    def _on_show_artwork_text_only(self, image_path: str, description: str):
        if self.description_label and description:
            self.description_label.setText(description)

    def _on_done_track(self, is_done: bool):
        if is_done:
            self.doneSignal.emit()

    # -------------------------
    # 카메라 구독 스레드/노드
    # -------------------------
    def _start_cams(self, guide_topic: str, patrol_topic: str):
        if self._cam_running:
            return
        self._cam_running = True
        self._cam_thread = threading.Thread(
            target=self._cam_spin, args=(guide_topic, patrol_topic), daemon=True
        )
        self._cam_thread.start()

    def _cam_spin(self, guide_topic: str, patrol_topic: str):
        self._cam_ctx = Context()
        rclpy.init(context=self._cam_ctx)
        self._cam_node = Node("dual_cam_bridge", context=self._cam_ctx)

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )

        def _cb(img_msg: CompressedImage, is_guide: bool):
            try:
                # CompressedImage → numpy → BGR → RGB → QImage
                np_arr = np.frombuffer(img_msg.data, np.uint8)
                frame_bgr = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
                if frame_bgr is None:
                    return
                frame_rgb = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)
                h, w, ch = frame_rgb.shape
                qimg = QImage(frame_rgb.data, w, h, ch * w, QImage.Format.Format_RGB888)
                # 메인 스레드로 emit
                if is_guide:
                    self.guideFrame.emit(qimg.copy())
                else:
                    self.patrolFrame.emit(qimg.copy())
            except Exception:
                pass

        # 구독 생성
        self._cam_node.create_subscription(
            CompressedImage, guide_topic, lambda m: _cb(m, True), qos
        )
        self._cam_node.create_subscription(
            CompressedImage, patrol_topic, lambda m: _cb(m, False), qos
        )

        self._cam_exec = SingleThreadedExecutor(context=self._cam_ctx)
        self._cam_exec.add_node(self._cam_node)

        try:
            while self._cam_running and rclpy.ok(context=self._cam_ctx):
                self._cam_exec.spin_once(timeout_sec=0.1)
        finally:
            try:
                self._cam_exec.remove_node(self._cam_node)
            except Exception:
                pass
            try:
                self._cam_node.destroy_node()
            except Exception:
                pass
            try:
                self._cam_exec.shutdown()
            except Exception:
                pass
            try:
                rclpy.shutdown(context=self._cam_ctx)
            except TypeError:
                rclpy.shutdown()

    # -------------------------
    # UI 라벨 갱신 슬롯
    # -------------------------
    def _set_guide_pixmap(self, qimg: QImage):
        if not self.guide_cam_label:
            return
        pix = QPixmap.fromImage(qimg).scaled(
            self.guide_cam_label.size(), Qt.KeepAspectRatio, Qt.SmoothTransformation
        )
        self.guide_cam_label.setPixmap(pix)

    def _set_patrol_pixmap(self, qimg: QImage):
        if not self.patrol_cam_label:
            return
        pix = QPixmap.fromImage(qimg).scaled(
            self.patrol_cam_label.size(), Qt.KeepAspectRatio, Qt.SmoothTransformation
        )
        self.patrol_cam_label.setPixmap(pix)

    # -------------------------
    # 종료 처리
    # -------------------------
    def close_app(self):
        if self.bridge:
            self.bridge.stop()
        # 카메라 스레드 종료
        self._cam_running = False
        if self._cam_thread:
            self._cam_thread.join(timeout=1.0)
            self._cam_thread = None

        if self.guide_cam_label:
            self.guide_cam_label.setText("종료되었습니다.")
        if self.patrol_cam_label:
            self.patrol_cam_label.setText("")
        if self.description_label:
            self.description_label.setText("")



if __name__ == "__main__":
    import sys, os
    from PyQt5.QtWidgets import QApplication, QLabel, QPushButton
    from PyQt5.uic import loadUi
    from ament_index_python.packages import get_resource

    def dummy_callback(data: str):
        print("Scanned barcode:", data)

    app = QApplication(sys.argv)

    # .ui 로드
    pkg_name = "turtle_musium"
    ui_filename = "monitoring_ui.ui"
    ui_file = '/home/rokey/turtlebot4_ws/src/turtle_musium/resource/monitoring_ui.ui'
    main_win = loadUi(ui_file)
    main_win.stackedWidget.setCurrentWidget(main_win.page03_description)

    # 모듈 위젯 생성(부모는 main_win로)
    scanner = GuideTracking()

    # .ui의 objectName과 매칭되는 위젯 주입
    guide_cam_label = main_win.findChild(QLabel, "guide_cam")
    patrol_cam_label = main_win.findChild(QLabel, "patrol_cam")
    info_label = main_win.findChild(QLabel, "description_label")
    scanner.set_ui(guide_cam_label=guide_cam_label, patrol_cam_label=patrol_cam_label, info_label=info_label)

    # 버튼으로 시작하고 싶으면 아래 한 줄로 연결
    # scan_button.clicked.connect(scanner.start_scanning)
    # 바로 시작하려면:

    main_win.show()
    exit_code = app.exec_()

    # 종료 정리
    scanner.close_app()
    sys.exit(exit_code)
