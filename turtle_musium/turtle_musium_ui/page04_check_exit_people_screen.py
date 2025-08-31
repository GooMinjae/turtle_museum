# check_exit_cam_screen.py (수정본)
import cv2, os
from PyQt5.QtWidgets import QWidget
from PyQt5.QtGui import QImage, QPixmap
from PyQt5.QtCore import Qt
from check_exit_people import ExitPeopleWorker

class CheckExitCamScreen(QWidget):
    def __init__(self):
        super().__init__()
        self.worker = None
        self.check_exit_cam_label  = None
        self.check_exit_info_label = None

    def set_ui(self, *, cam_label, info_label):
        self.check_exit_cam_label  = cam_label
        self.check_exit_info_label = info_label
        self.check_exit_cam_label.setText("카메라 연결 중…")
        self.check_exit_info_label.setText("출구 감시 준비 중입니다.")

    def start_scanning(self):
        self.check_exit_info_label.setText("사람 감지 중…")
        self.worker = ExitPeopleWorker(cam_idx=0, stop_on_detect=True)

        # 신호 연결
        self.worker.sig.frameCaptured.connect(self.update_frame)
        self.worker.sig.detected.connect(self.on_detected)
        self.worker.sig.errorOccurred.connect(self.on_error)

        self.worker.start()

    def update_frame(self, frame_bgr):
        rgb = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)
        h, w, ch = rgb.shape
        qimg = QImage(rgb.data, w, h, w*ch, QImage.Format_RGB888)
        pix  = QPixmap.fromImage(qimg)
        pix  = pix.scaled(self.check_exit_cam_label.size(),
                        Qt.KeepAspectRatio, Qt.SmoothTransformation)
        self.check_exit_cam_label.setPixmap(pix)

    def on_detected(self, _):
        self.check_exit_info_label.setText("출구에 사람 감지됨!")
        # 필요시 자동 종료
        # self.close_app()

    def on_error(self, msg):
        self.check_exit_cam_label.setText("카메라 오류")
        self.check_exit_info_label.setText(msg)
        if self.worker: self.worker.stop()

    def close_app(self):
        try:
            if self.worker:
                self.worker.stop()
        finally:
            if self.check_exit_info_label:
                self.check_exit_info_label.setText("종료되었습니다.")
            if self.check_exit_cam_label:
                self.check_exit_cam_label.setText("카메라 화면")


if __name__ == "__main__":
    # main 예시 (지금 쓰는 __main__ 쪽에 추가)
    from PyQt5.uic import loadUi
    from PyQt5.QtWidgets import QApplication, QLabel
    import rclpy, sys

    app = QApplication(sys.argv)
    rclpy.init()  # ★ 노드 생성 전에

    ui = loadUi("/home/rokey/turtlebot4_ws/src/turtle_musium/resource/monitoring_ui.ui")
    ui.stackedWidget.setCurrentWidget(ui.page04_exit_webcam)

    screen = CheckExitCamScreen()
    cam_label  = ui.findChild(QLabel, "webcam_label")
    info_label = ui.findChild(QLabel, "condition_exit_label")
    screen.set_ui(cam_label=cam_label, info_label=info_label)
    screen.start_scanning()

    ui.show()
    code = app.exec_()

    # 종료 정리
    screen.close_app()
    rclpy.shutdown()  # ★ 종료 시
    sys.exit(code)

