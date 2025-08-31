import sys
from PyQt5.QtWidgets import QApplication, QLabel
from PyQt5.uic import loadUi

# 각 페이지 컨트롤러
from page02_barcode_scanner_screen import BarcodeScannerApp
from page03_description_art_screen import GuideTracking
from page04_check_exit_people_screen import CheckExitCamScreen

import rclpy


from count_bridge import CountBridge
from visit_db import insert_visit, update_counted_count

UI_FILE = "/home/rokey/turtlebot4_ws/src/turtle_musium/resource/monitoring_ui.ui"

class MonitoringApp:
    def __init__(self):
        self.app = QApplication(sys.argv)
        self.ui  = loadUi(UI_FILE)

        # 현재 활성 페이지 인덱스 (ui 파일에서 페이지 이름은 예시: page02_barcode, page03_description, page04_exit_webcam)
        self.stacked = self.ui.stackedWidget

        # 컨트롤러 보관 (지연 생성)
        self.page02 = None  # BarcodeScannerApp
        self.page03 = None  # GuideTracking
        self.page04 = None  # CheckExitCamScreen

        # ROS init 상태 추적 (page04 전용)
        self.ros_active = False

        # 카운트 브릿지 & 현재 방문 rowid
        self.count_bridge = None
        self.current_visit_id = None

        # 페이지 변경 시그널 연결
        self.stacked.currentChanged.connect(self.on_page_changed)

        # 시작 페이지 지정 (원하는 페이지로 바꿔도 됨)
        self.stacked.setCurrentWidget(self.ui.page01_start)
        self.ui.open_btn.clicked.connect(lambda: self.stacked.setCurrentWidget(self.ui.page02_barcode))
        # 여기서는 UI가 기본으로 띄우는 페이지를 그대로 사용

        # 초기 진입 처리
        self.on_page_changed(self.stacked.currentIndex())

        # 윈도우 닫힐 때 정리
        self.ui.closeEvent = self._wrap_close_event(self.ui.closeEvent)

    # ————————————————————————
    # 페이지 엔트리/엑싯 공통 처리
    # ————————————————————————
    def on_page_changed(self, idx: int):
        # 모든 페이지 정리(나가는 페이지들 정리)
        self.stop_page02()
        self.stop_page03()
        self.stop_page04()

        # 진입한 페이지 시작
        current_widget = self.stacked.currentWidget()
        if current_widget is self.ui.page02_barcode:
            self.start_page02()
        elif current_widget is self.ui.page03_description:
            # page04에서 쓰던 ROS2가 켜져 있다면 먼저 내려서 충돌 방지
            self.ensure_ros_stopped()
            self.start_page03()
        elif current_widget is self.ui.page04_exit_webcam:
            # page04는 rclpy 필요
            self.ensure_ros_started()
            self.start_page04()

    # ————————————————————————
    # page02: 바코드
    # ————————————————————————
    def start_page02(self):
        if self.page02 is None:
            self.page02 = BarcodeScannerApp(on_barcode_callback=self.on_barcode_scanned)
            cam_label  = self.ui.findChild(QLabel, "barcode_cam_label")
            info_label = self.ui.findChild(QLabel, "barcode_info_label")
            self.page02.set_ui(cam_label=cam_label, info_label=info_label)
        self.page02.start_scanning()

        # ★ CountBridge 시작(이 시점부터 /tour/count/done 대기)
        if self.count_bridge is None:
            self.count_bridge = CountBridge()
            self.count_bridge.countDone.connect(self.on_count_done)
            self.count_bridge.start()


    def stop_page02(self):
        if self.page02:
            self.page02.close_app()
        if self.count_bridge:
            self.count_bridge.stop()
            self.count_bridge = None
        self.current_visit_id = None


    def on_barcode_scanned(self, data: str):
        # 예: "YYYYMMDD-<인원수>-<기념품코드>"
        try:
            date_str, planned_count, gift_str = data.split("-")
        except Exception:
            print("[monitoring] 잘못된 바코드 형식:", data)
            return

        # DB insert
        self.current_visit_id = insert_visit(date_str, int(planned_count), gift_str)
        print("[monitoring] DB insert visit_id:", self.current_visit_id)

        # 예상 인원 퍼블리시
        if self.count_bridge:
            self.count_bridge.publish_expected_count(int(planned_count))
            print("[monitoring] publish /tour/count/request:", planned_count)


    # /tour/count/done 수신 핸들러
    def on_count_done(self, counted: int):
        print("[monitoring] /tour/count/done:", counted)
        if self.current_visit_id is not None:
            update_counted_count(self.current_visit_id, counted)
            print("[monitoring] DB update counted_count")

        # 다음 페이지로 전환(page03)
        self.stacked.setCurrentWidget(self.ui.page03_description)

    # ————————————————————————
    # page03: 작품 설명(로봇 위치 → 작품 이미지/설명 표시)
    # ————————————————————————
    def start_page03(self):
        if self.page03 is None:
            self.page03 = GuideTracking()
            cam_label  = self.ui.findChild(QLabel, "piece_img")
            info_label = self.ui.findChild(QLabel, "description_label")
            # set_ui 안에서 ArtworkBridge가 start() 되며, 내부 스레드에서 rclpy.init()을 수행
            self.page03.set_ui(cam_label=cam_label, info_label=info_label)
        # 별도 start 없음 (set_ui에서 시작)

    def stop_page03(self):
        if self.page03:
            self.page03.close_app()  # ArtworkBridge stop() → 내부에서 rclpy.shutdown()

    # ————————————————————————
    # page04: 출구 사람 감시( YOLO + ROS Bool 퍼블리시 )
    # ————————————————————————
    def start_page04(self):
        if self.page04 is None:
            self.page04 = CheckExitCamScreen()
            cam_label  = self.ui.findChild(QLabel, "webcam_label")
            info_label = self.ui.findChild(QLabel, "condition_exit_label")
            self.page04.set_ui(cam_label=cam_label, info_label=info_label)
        self.page04.start_scanning()

    def stop_page04(self):
        if self.page04:
            self.page04.close_app()

    # ————————————————————————
    # ROS2 컨텍스트 관리 (page04 전용)
    # ————————————————————————
    def ensure_ros_started(self):
        if not self.ros_active:
            rclpy.init(args=None)
            self.ros_active = True

    def ensure_ros_stopped(self):
        if self.ros_active:
            try:
                rclpy.shutdown()
            except Exception:
                pass
            self.ros_active = False

    # ————————————————————————
    # 종료 처리
    # ————————————————————————
    def _wrap_close_event(self, orig_handler):
        def handler(event):
            # 페이지별 종료 정리
            self.stop_page02()
            self.stop_page03()
            self.stop_page04()
            # ROS 내려주기 (혹시 켜져 있으면)
            self.ensure_ros_stopped()
            # 원래 핸들러 호출
            if orig_handler:
                orig_handler(event)
        return handler

    def run(self):
        self.ui.show()
        code = self.app.exec_()
        # 안전 정리
        self.ensure_ros_stopped()
        sys.exit(code)


if __name__ == "__main__":
    MonitoringApp().run()
