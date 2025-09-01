# page05_exit_screen.py
from PyQt5.QtWidgets import QWidget, QLabel, QApplication
from PyQt5.QtCore import Qt
from PyQt5.uic import loadUi
import sys

from data_base.visit_db import get_visit_by_id

# # ---- DB 접근: 안전 임포트 (패키지/로컬 둘 다 시도) ----
# try:
#     from turtle_musium_ui.visit_db import get_visit_by_id  # 패키지 경로
# except Exception:
#     try:
#         from visit_db import get_visit_by_id               # 로컬 모듈
#     except Exception:
#         # 테스트 편의용: 임포트 실패 시 더미 함수
#         def get_visit_by_id(_visit_id: int):
#             return None


class ExitSummaryScreen(QWidget):
    def __init__(self):
        super().__init__()
        self.title_label: QLabel = None
        self.info_label: QLabel  = None
        self._current_visit_id = None

    def set_ui(self, *, title_label: QLabel, info_label: QLabel):
        """ .ui에서 주입 """
        self.title_label = title_label
        self.info_label  = info_label
        if self.title_label:
            self.title_label.setText("방문 요약")
        if self.info_label:
            self.info_label.setText("데이터를 불러오는 중입니다…")

    def show_visit(self, visit_id: int):
        """ 특정 visit_id의 정보를 로드하여 표시 """
        self._current_visit_id = visit_id
        row = get_visit_by_id(visit_id)
        if not row:
            if self.info_label:
                self.info_label.setText(f"방문 ID {visit_id} 데이터를 찾을 수 없습니다.")
            return


        date_str   = row.get("date", "")
        planned    = row.get("planned_count", None)
        counted    = row.get("counted_count", None)
        gift_moo   = row.get("gift_moo", 0)
        gift_pinga = row.get("gift_pinga", 0)
        gift_haowl = row.get("gift_haowl", 0)
        gift_pingu = row.get("gift_pingu", 0)
        created_at = row.get("created_at", "")

        # 차이/상태 계산
        diff = None
        status = "미집계"
        if counted is not None:
            if planned is not None:
                diff = int(counted) - int(planned)
            status = "완료"

        # 보기 좋게 포매팅 (HTML 허용)

        html = [
            f"<b>방문 ID</b>: {visit_id}",
            f"<b>방문일</b>: {date_str}",
            f"<b>예상 인원</b>: {planned if planned is not None else '-'} 명",
            f"<b>실집계 인원</b>: {counted if counted is not None else '-'} 명",
            f"<b>차이</b>: {('+' if (diff is not None and diff > 0) else '') + str(diff) if diff is not None else '-'}",
            f"<b>기념품 수량</b>: moo={gift_moo}, pinga={gift_pinga}, haowl={gift_haowl}, pingu={gift_pingu}",
            f"<b>생성 시각</b>: {created_at or '-'}",
            f"<b>상태</b>: {status}",
        ]
        if self.info_label:
            self.info_label.setText("<br>".join(html))
            self.info_label.setAlignment(Qt.AlignTop | Qt.AlignLeft)

    def refresh(self):
        if self._current_visit_id is not None:
            self.show_visit(self._current_visit_id)


if __name__ == "__main__":
    app = QApplication(sys.argv)

    # .ui 로드 (경로는 환경에 맞게 유지)
    ui_file = "/home/rokey/turtlebot4_ws/src/turtle_musium/resource/monitoring_ui.ui"
    main_win = loadUi(ui_file)

    # page05로 전환
    if hasattr(main_win, "stackedWidget") and hasattr(main_win, "page05_final"):
        main_win.stackedWidget.setCurrentWidget(main_win.page05_final)

    # 위젯 생성
    screen = ExitSummaryScreen()

    # 다양한 objectName 패턴을 지원 (둘 중 존재하는 걸 사용)
    # 2) 사용자가 준 테스트 패턴
    title_label = main_win.findChild(QLabel, "final_info_label")
    info_label  = main_win.findChild(QLabel, "db_label")

    # # 라벨이 없으면 에러 표시
    if title_label is None or info_label is None:
        print("[ERROR] page05 라벨을 찾을 수 없습니다. "
              "objectName을 확인하세요: final_title_label/final_info_label 또는 info_label/db_label")
        sys.exit(1)

    # UI 주입
    screen.set_ui(title_label=title_label, info_label=info_label)

    # 커맨드라인에서 visit_id 입력 가능: python3 page05_exit_screen.py 3
    visit_id = None
    if len(sys.argv) >= 2:
        try:
            visit_id = int(sys.argv[1])
        except ValueError:
            pass

    visit_id = 10
    # 테스트: visit_id가 주어지면 즉시 표시
    if visit_id is not None:
        screen.show_visit(visit_id)

    main_win.show()
    sys.exit(app.exec_())
