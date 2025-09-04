import cv2
import threading
from pyzbar.pyzbar import decode
# import winsound
import os
from PyQt5.QtCore import pyqtSignal, QObject
from datetime import datetime

'''
Thread 사용 이유
간단한 프로젝트에서는 QTimer가 UI 이벤트 루프에서 동작하여 간편하고 유용하지만,
프로젝트의 확장성을 우려하고, 카메라와 UI를 독립적으로 나누기 위해 Thread 선택
또한, 명시적인 스레드 종료 및 자원 해제를 통해 라이프 사이클을 완전히 컨트롤 할 수 있다.
'''

CAM_IDX = 2


class BarcodeScannerWorker(QObject):
    frameCaptured = pyqtSignal(object)  # 프레임 업데이트 신호
    barcodeDetected = pyqtSignal(str)   # 바코드 정보 신호
    errorOccurred = pyqtSignal(str)     # 에러 발생 시그널 추가

    def __init__(self):
        super().__init__()
        self.running = False
        self.cap = cv2.VideoCapture(CAM_IDX, cv2.CAP_V4L2)
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        self.cap.set(cv2.CAP_PROP_FPS, 30)

        self.thread = None  # 스레드 객체

    def start(self):
        if self.thread is None or not self.thread.is_alive():  # 기존 스레드가 없거나 종료되었을 때만 실행
            self.running = True
            self.thread = threading.Thread(target=self.run)
            self.thread.start()

    def run(self):
        while self.running:
            ret, frame = self.cap.read()
            if not ret:
                self.errorOccurred.emit("카메라를 열 수 없습니다.")
                break

            # frame = cv2.resize(frame, (0, 0), fx=0.5, fy=0.5)
            
            processed_frame, barcode_info, detected = self.recognize_barcode(frame)
            self.frameCaptured.emit(processed_frame)
            
            if detected:
                self.barcodeDetected.emit(barcode_info)
                self.running = False
                
        self.cap.release()
    
    def recognize_barcode(self, frame):
        barcodes = decode(frame)
        barcode_data = None
        for barcode in barcodes:
            if barcode.type == 'PDF417':
                continue
            x, y, w, h = barcode.rect
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
            barcode_data = barcode.data.decode('utf-8')
            barcode_type = barcode.type
            text = f'{barcode_type}: {barcode_data}'
            cv2.putText(frame, text, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX,
                        0.9, (0, 255, 0), 2)
            # winsound.Beep(1000, 200)
            os.system('beep -f 1000 -l 200')
            return frame, barcode_data, True
        return frame, barcode_data, False
    
    def stop(self):
        self.running = False
        if self.thread:
            self.thread.join()


if __name__ == "__main__":
    # cap = cv2.VideoCapture(0)
    worker = BarcodeScannerWorker()

    while True:
        ret, frame = worker.cap.read()
        if not ret:
            print("카메라를 열 수 없습니다.")
            break

        frame, barcode_data, detected = worker.recognize_barcode(frame)
        cv2.imshow("Barcode Scanner", frame)

        if detected:
            # obj_nowdate = datetime.strptime(barcode_data.split('-')[0], "%Y%m%d")
            # count_people = barcode_data.split('-')[1]
            # gift_data = barcode_data.split('-')[2]

            obj_nowdate = datetime.strptime(barcode_data[:8], "%Y%m%d")
            count_people = barcode_data[8]
            gift_data = barcode_data[9:]
            # info = {'datetime':obj_nowdate, 'price':free_amount}
            print("바코드 감지됨:")
            print(f"날짜: {obj_nowdate}, 인원수: {count_people}, 기념품: {gift_data}")
            break

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    worker.cap.release()
    cv2.destroyAllWindows()