import os
import barcode
from barcode.writer import ImageWriter
from datetime import datetime

# 날짜
# 인원수
# 기념품

class BarcodeGenerator:
    def __init__(self, count_people="3", gift_data="1001", directory=None):
        """
        BarcodeGenerator 클래스 초기화
        :param count_people: 인원수 (기본값: 3)
        :param gift_data: 기념품 (기본값: 1001)
        :param directory: 바코드를 저장할 디렉토리 (기본값: 현재 파일 기준)
        """
        self.count_people = count_people
        self.gift_data = gift_data
        self.directory = directory or os.path.dirname(os.path.abspath(__file__))  # 디렉토리 기본값 설정

    def create_bar_code(self):
        """ 날짜, 시간, 무료 금액 정보를 포함한 바코드 생성 """
        date_data = datetime.now().strftime("%Y%m%d")
        bar_data = f"{date_data}-{self.count_people}-{self.gift_data}"

        # 바코드 생성 (Code128 바코드 형식 사용)
        code128 = barcode.get_barcode_class('code128')
        barcode_instance = code128(bar_data, writer=ImageWriter())

        # 저장할 경로 설정
        save_path = os.path.join(self.directory, "bar_codes")
        os.makedirs(save_path, exist_ok=True)  # 폴더가 없으면 생성
        file_path = os.path.join(save_path, f"bar_code_{bar_data}")  # 파일 경로 설정

        options = {
            'module_width': 0.5,     # 선 굵기 키우기
            'module_height': 30.0,   # 세로 높이 늘리기
            'quiet_zone': 6.5,       # 여백 확보
            'write_text': True       # 아래 텍스트 제거
        }
        barcode_instance.save(file_path, options)  # 바코드 저장

        # barcode_instance.save(file_path)  # 바코드 저장

        print(f"바코드 생성 완료: {file_path}")
        return file_path

# 외부에서 호출 예시
DUMMY = [
    {
        "count_people": "3",
        "gift_data": "1111",
    },
    {
        "count_people": "3",
        "gift_data": "1010",
    }
]
if __name__ == "__main__":
    dummy_idx = 0
    barcode_generator = BarcodeGenerator(
        count_people=DUMMY[dummy_idx]["count_people"], 
        gift_data=DUMMY[dummy_idx]["gift_data"],
        directory="/home/rokey/turtlebot4_ws/src/turtle_musium/turtle_musium_ui"
    )  # 원하는 금액으로 초기화
    barcode_generator.create_bar_code()  # 바코드 생성