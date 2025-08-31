import sys
import cv2
import os
from PyQt5.QtWidgets import (
    QApplication, QLabel, QWidget, QPushButton
)
from PyQt5.QtGui import QImage, QPixmap
from PyQt5.QtCore import Qt
from datetime import datetime
from barcode_scanner import BarcodeScannerWorker

class BarcodeScannerApp(QWidget):
    def __init__(self, on_barcode_callback):
        super().__init__()
        self.on_barcode_callback = on_barcode_callback
        self.scanner_worker = None

        # UI 요소는 .ui에서 주입됨
        self.barcode_cam_label = None
        self.barcode_info_label = None

    def set_ui(self, *, cam_label, info_label):
        """ .ui 로드 후 외부에서 라벨/버튼을 꽂아줌 """
        self.barcode_cam_label = cam_label
        self.barcode_info_label = info_label

        # 초기 문구
        self.barcode_cam_label.setText("카메라 화면")
        self.barcode_info_label.setText("바코드가 없으신가요?\n[취소] 버튼을 눌러주세요.")


    def start_scanning(self):
        self.barcode_info_label.setText("바코드 스캔 중입니다...")  # 상태 표시
        self.scanner_worker = BarcodeScannerWorker()
        self.scanner_worker.frameCaptured.connect(self.update_frame)
        self.scanner_worker.barcodeDetected.connect(self.display_barcode_info_label)
        self.scanner_worker.errorOccurred.connect(self.handle_camera_error)
        self.scanner_worker.start()

    def update_frame(self, frame):
        rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        h, w, ch = rgb_frame.shape
        bytes_per_line = ch * w
        qt_image = QImage(rgb_frame.data, w, h, bytes_per_line, QImage.Format_RGB888)
        self.barcode_cam_label.setPixmap(QPixmap.fromImage(qt_image))


    """
    gift data
    [0]: moo
    [1]: pinga
    [2]: haowl
    [3]: pingu
    """
    def display_barcode_info_label(self, barcode_data):
        # 바코드 맞지 않을 경우 예외처리
        try:
            obj_nowdate = datetime.strptime(barcode_data.split('-')[0], "%Y%m%d")
            count_people = barcode_data.split('-')[1]
            gift_data = barcode_data.split('-')[2]
        except (ValueError):
            self.barcode_info_label.setText("바코드 형식이 맞지 않습니다.")
            self.close_app()
            return

        match_idx_gift = {
            0: "moo",
            1: "pinga",
            2: "haowl",
            3: "pingu",
        }

        txt = f"날짜: {obj_nowdate.date()}\n인원 수: {count_people}명\n기념품\n"
        for idx, num_gift in enumerate(gift_data):
            if num_gift != "0":
                txt += f"{match_idx_gift[idx]} - {num_gift}개\n"

        self.barcode_info_label.setText(txt)
        print(f"[스캔 결과] {barcode_data}")

        if self.barcode_cam_label:
            self.barcode_cam_label.setText("인원 수 파악 중")
        self.on_barcode_callback(barcode_data)
        self.close_app()

    def handle_camera_error(self, error_msg):
        self.barcode_cam_label.setText(error_msg)
        self.barcode_info_label.setText("카메라 연결 실패\n잠시 후 다시 시도해주세요.")
        self.scanner_worker.stop()


    def close_app(self):
        try:
            self.scanner_worker.frameCaptured.disconnect(self.update_frame)
            self.scanner_worker.stop()
        except (AttributeError, TypeError):
            pass
        # self.barcode_info_label.setText("바코드가 없으신가요?\n[취소] 버튼을 눌러주세요.")  # 상태 표시
        # self.barcode_cam_label.setText("카메라 화면")


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
    main_win.stackedWidget.setCurrentWidget(main_win.page02_barcode)

    # 모듈 위젯 생성(부모는 main_win로)
    scanner = BarcodeScannerApp(on_barcode_callback=dummy_callback)

    # .ui의 objectName과 매칭되는 위젯 주입
    cam_label = main_win.findChild(QLabel, "barcode_cam_label")
    info_label = main_win.findChild(QLabel, "barcode_info_label")
    scanner.set_ui(cam_label=cam_label, info_label=info_label)

    # 버튼으로 시작하고 싶으면 아래 한 줄로 연결
    # scan_button.clicked.connect(scanner.start_scanning)
    # 바로 시작하려면:
    scanner.start_scanning()

    main_win.show()
    exit_code = app.exec_()

    # 종료 정리
    scanner.close_app()
    sys.exit(exit_code)
