import sys
import os
import threading
import cv2
import rclpy
from rclpy.qos import QoSProfile
from cv_bridge import CvBridge
from rclpy.node import Node

# from PyQt5.QtCore import pyqtSignal

from ament_index_python.resources import get_resource
from PyQt5.uic import loadUi
from PyQt5.QtCore import Qt, QTimer, QThread, pyqtSignal
from PyQt5.QtGui import QPixmap, QImage
from PyQt5.QtWidgets import QMainWindow, QApplication, QStackedWidget

# import mediapipe as mp

from std_msgs.msg import Bool

from rclpy.executors import MultiThreadedExecutor

# from voice_processing.tts import say, say_async

from sensor_msgs.msg import Image, CameraInfo


class WindowClass(QMainWindow):
    voice_signal = pyqtSignal(bool)
    qr_check_signal = pyqtSignal(list, bool, bool)
    order_done_signal = pyqtSignal()

    def __init__(self):
        super().__init__()
        rclpy.init(args=None)
        self.node = rclpy.create_node("window")

        os.environ["QT_OPENGL"] = "software"

        # UI 파일 로드
        pkg_name = 'turtle_musium'
        ui_filename = 'monitoring_ui.ui' # Ensure this UI file has a QStackedWidget
        _, package_path = get_resource('packages', pkg_name)
        ui_file = os.path.join(package_path, 'share', pkg_name, 'resource', ui_filename)
        loadUi(ui_file, self)

        self.setWindowTitle("TURTLE MUSIUM")
        self.stackedWidget.setCurrentIndex(1) # Start with the first page (index 0)

        # ROS init
        self.bridge = CvBridge()

        # self.node.create_subscription(Image, '/image/color/image_raw_ui', self.color_callback, 10) # QoS 프로필 적용)

        # self.voice_signal.connect(self.handle_voice_result)
        # self.node.create_subscription(Bool, '/voice_command_success', self.voice_command_callback, 10)

        # self.qr_check_signal.connect(self.handle_qr_check_result)
        # self.node.create_subscription(
        #     TargetList,
        #     '/qr_check_state',
        #     self.qr_check_callback,
        #     10
        # )

        # self.order_done_signal.connect(self.handle_order_done)
        # self.node.create_subscription(Bool, '/order_done', self.order_done_callback, 10)

        # ROS spin 쓰레드
        self.spin_thread = threading.Thread(target=self.ros_spin_loop, daemon=True)
        self.spin_thread.start()

        self.start_camera_thread()

        self.initialize_all()

    def initialize_all(self):
        self.stackedWidget.setCurrentIndex(0)
        self.order_label.setText('환영합니다! 무엇을 구매하시겠습니까? \n"Hello, ROKEY"로 시작해주세요')
        self.target_list = []
        if hasattr(self, 'cam_worker'):
            self.cam_worker.set_detection_enabled(True)
        self.speaking = False
        self.need_id = False
        self.switch_mp = True

    def ros_spin_loop(self):
        executor = MultiThreadedExecutor()
        executor.add_node(self.node)      # 구독 달아둔 노드
        # executor.add_node(self.img_node)  # 이미지 프레임 노드
        while rclpy.ok():
            executor.spin_once(timeout_sec=0.05)

    def stop_camera_thread(self):
        if hasattr(self, 'cam_worker') and self.cam_worker.running:
            self.cam_worker.stop()
            self.cam_thread.quit()
            self.cam_thread.wait()


    def on_face_detected(self):
        # 중복 TTS 방지
        if not self.speaking:
            self.speaking = True
            say_async("어서오세요. 무엇을 도와드릴까요?")

    def voice_command_callback(self, msg):
        self.switch_mp = False
        self.voice_signal.emit(msg.data)

    def qr_check_callback(self, msg):
        self.qr_check_signal.emit(list(msg.targets), msg.is_checking, msg.qr_status)

    def order_done_callback(self, msg):
        if msg.data:
            self.order_done_signal.emit()

    def handle_voice_result(self, success: bool):
        if success:
            # self.stackedWidget.setCurrentIndex(0)
            self.order_label.setText("음성 인식 완료")
            if hasattr(self, 'cam_worker'):
                self.cam_worker.set_detection_enabled(False)
        else:
            self.order_label.setText("음성 인식 실패")


    def calculation(self, item_list):
        total_price = 0
        for item in item_list:
            price = self.item_price.get(item, 0)
            total_price += price
        
        return total_price

    def handle_qr_check_result(self, target_list, is_checking, qr_status):
        if is_checking:
            print("[QR 페이지] QR 인식 중")
            self.order_label.setText("QR 인식 중")
            self.need_id = True
        else:
            print("[메인 페이지] QR 인식 종료")

            if qr_status and self.need_id:
                say_async("인증 되었습니다.")
                self.order_label.setText("성인 인증 성공")
                self.need_id = False
                print("성인 인증 성공")
            elif (not qr_status) and self.need_id:
                say_async("인증 되지 않았습니다.")
                self.order_label.setText("성인 인증 실패")
                self.need_id = False
                print("성인 인증 실패")

            print("품목:", target_list)
            self.target_list = target_list

    def handle_order_done(self):
        print("[결제 페이지] 모든 품목 처리 완료")
        self.stackedWidget.setCurrentIndex(2)
        self.price_list.clear()
        
        price = self.calculation(self.target_list)

        for item in self.target_list:
            self.price_list.addItem(f"{item}: {self.item_price[item]} 원")

        self.total_price_label.setText(f"총 금액: {price} 원")

        say_async("감사합니다. 안녕히 가세요.")

        QTimer.singleShot(5000, lambda: self.initialize_all())

    
    def color_callback(self, msg):
        # print('111111111')
        self.latest_frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")

    def get_color_frame(self):
        return self.latest_frame

    def timerEvent(self, event):
        frame = self.get_color_frame
        if frame is not None:
            self.update_frame(frame)

    def start_camera_thread(self):
        # 이미 실행 중이면 다시 시작 안 함
        if hasattr(self, 'cam_thread') and self.cam_thread.isRunning():
            return

        self.cam_thread = QThread()
        self.cam_worker = CameraWorker(self.get_color_frame, fps=30, min_conf=0.8)
        self.cam_worker.moveToThread(self.cam_thread)

        self.cam_thread.started.connect(self.cam_worker.run)
        self.cam_worker.frame_ready.connect(self.display_frame)
        self.cam_worker.face_detected.connect(self.on_face_detected)
        self.cam_worker.finished.connect(self.cam_thread.quit)
        self.cam_worker.finished.connect(self.cam_worker.deleteLater)
        self.cam_thread.finished.connect(self.cam_thread.deleteLater)

        self.cam_thread.start()


    def display_frame(self, qt_image):
        self.full_camera.setPixmap(QPixmap.fromImage(qt_image))

    def closeEvent(self, event):
        # 안전하게 종료
        if hasattr(self, 'cam_worker'):
            self.cam_worker.stop()
        event.accept()


def main():
    # GPU 충돌 방지용 환경 변수
    os.environ["QT_OPENGL"] = "software"

    app = QApplication(sys.argv)
    window = WindowClass()
    window.show()
    sys.exit(app.exec())
    
    rclpy.shutdown()


if __name__ == "__main__":
    main()