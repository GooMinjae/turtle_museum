# guide_tracking.py
import os
from PyQt5.QtWidgets import QWidget
from PyQt5.QtGui import QPixmap
from PyQt5.QtCore import Qt, pyqtSignal
from turtle_musium_ui.artwork_bridge import ArtworkBridge
# from artwork_bridge import ArtworkBridge

class GuideTracking(QWidget):
    doneSignal = pyqtSignal()
    def __init__(self):
        super().__init__()
        self.bridge = None
        self.piece_img = None   # 이미지 표시용 QLabel
        self.description_label = None  # 설명/상태 표시용 QLabel

    def set_ui(self, *, cam_label, info_label, topic="/robot8/now_loc"):
        self.piece_img = cam_label
        self.description_label = info_label

        # 초기 문구
        self.piece_img.setText("작품 1 이동 중입니다.")
        self.description_label.setText("잠시만 기다려 주세요…")

        # ROS2 브리지 시작
        self.bridge = ArtworkBridge(topic)
        self.bridge.showArtwork.connect(self.on_show_artwork)
        self.bridge.trackDone.connect(self._on_done_track)
        self.bridge.start()

    def on_show_artwork(self, image_path: str, description: str):
        # 설명 갱신
        if description:
            self.description_label.setText(description)

        # 이미지 갱신
        if image_path and os.path.exists(image_path):
            pix = QPixmap(image_path)
            pix = pix.scaled(self.piece_img.size(),
                            Qt.KeepAspectRatio, Qt.SmoothTransformation)
            self.piece_img.setPixmap(pix)
        else:
            if not image_path:
                self.piece_img.setText("작품 이미지 대기 중…")
            elif not os.path.exists(image_path):
                self.piece_img.setText("이미지 파일을 찾을 수 없습니다.")

    def _on_done_track(self, is_done: bool):
        if is_done:
            # MonitoringApp으로 페이지 전환 신호
            self.doneSignal.emit()

    def close_app(self):
        if self.bridge:
            self.bridge.stop()
        if self.piece_img:
            self.piece_img.setText("종료되었습니다.")
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
    cam_label = main_win.findChild(QLabel, "piece_img")
    info_label = main_win.findChild(QLabel, "description_label")
    scanner.set_ui(cam_label=cam_label, info_label=info_label)

    # 버튼으로 시작하고 싶으면 아래 한 줄로 연결
    # scan_button.clicked.connect(scanner.start_scanning)
    # 바로 시작하려면:

    main_win.show()
    exit_code = app.exec_()

    # 종료 정리
    scanner.close_app()
    sys.exit(exit_code)
