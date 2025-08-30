import cv2
import threading
from ultralytics import YOLO
from PyQt5.QtCore import pyqtSignal, QObject
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool

MODEL_PATH = "/home/rokey/turtlebot4_ws/src/turtle_musium/resource/exit_people.pt"
CONF_THRES = 0.9
CAM_IDX = 2
TOPIC_NAME = "/exit/people_detected"

class ExitPeopleWorker(Node):
    # frameCaptured = pyqtSignal(object)  # 프레임 업데이트 신호
    # checkYolo = pyqtSignal(bool)   # 
    # errorOccurred = pyqtSignal(str)     # 에러 발생 시그널 추가
    """
    - 카메라에서 프레임 읽기
    - YOLO로 사람(people=0) 검출
    - 검출되면 ROS2 토픽(Bool) True 발행
    """
    def __init__(self):
        super().__init__('exit_people_worker')
        self.running = False
        self.cap = cv2.VideoCapture(CAM_IDX)
        self.model = YOLO(MODEL_PATH)


        # QoS: 신뢰성 RELIABLE + TransientLocal (늦게 구독해도 마지막 값 1회 전달)
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,  # ROS1의 latch 유사
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        self.pub = self.create_publisher(Bool, TOPIC_NAME, qos)

        self.thread = None  # 스레드 객체

    def start(self):
        if self.thread is None or not self.thread.is_alive():  # 기존 스레드가 없거나 종료되었을 때만 실행
            self.running = True
            self.thread = threading.Thread(target=self.run)
            self.thread.start()

    def run(self):
        self.publish_bool(False)

        while self.running:
            ret, frame = self.cap.read()
            if not ret:
                break

            frame = cv2.resize(frame, (0, 0), fx=0.5, fy=0.5)

            processed_frame, detected = self.recognize_exit_people(frame)
            # self.frameCaptured.emit(processed_frame)

            cv2.imshow("ExitPeople Debug", processed_frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                self.get_logger().info("사용자 종료(q).")
                break

            if detected:
                self.publish_bool(True)
                # self.checkYolo.emit(True)   # bool 값으로 발행
                self.running = False

        self.cap.release()
        cv2.destroyAllWindows()

    def publish_bool(self, value: bool):
        msg = Bool()
        msg.data = value
        self.pub.publish(msg)

    def recognize_exit_people(self, frame):
        results = self.model(frame, conf=CONF_THRES, verbose=False)

        detected = False
        for result in results:
            for box in result.boxes:
                cls_id = int(box.cls[0])  # 클래스 ID
                conf = float(box.conf[0]) # 신뢰도
                if cls_id == 0 and conf >= CONF_THRES:  # 0=person
                    detected = True
                    # bbox 그리기 (디버깅용)
                    x1, y1, x2, y2 = map(int, box.xyxy[0])
                    cv2.rectangle(frame, (x1, y1), (x2, y2), (0,255,0), 2)
                    cv2.putText(frame, f"Person {conf:.2f}", (x1, y1-5),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 2)

        return frame, detected


    def stop(self):
        self.running = False
        if self.thread:
            self.thread.join()


# if __name__ == "__main__":
#     cap = cv2.VideoCapture(CAM_IDX)
#     worker = ExitPeopleWorker()

#     while True:
#         ret, frame = cap.read()
#         if not ret:
#             print("카메라를 열 수 없습니다.")
#             break

#         frame, detected = worker.recognize_exit_people(frame)
#         cv2.imshow("Barcode Scanner", frame)

#         if detected:
#             print("사람 감지됨")
#             break

#         if cv2.waitKey(1) & 0xFF == ord('q'):
#             break

#     cap.release()
#     cv2.destroyAllWindows()


def main():
    rclpy.init()
    node = ExitPeopleWorker()

    # 워커 스레드로 영상/YOLO 처리 시작
    node.start()

    # rclpy 스핀으로 ROS 통신 유지
    try:
        while rclpy.ok() and (node.thread and node.thread.is_alive()):
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()