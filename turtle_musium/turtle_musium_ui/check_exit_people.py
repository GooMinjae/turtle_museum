# check_exit_people.py (수정본)
import cv2, threading
from ultralytics import YOLO
from PyQt5.QtCore import QObject, pyqtSignal
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from std_msgs.msg import Bool

import time

MODEL_PATH  = "/home/rokey/turtlebot4_ws/src/turtle_musium/resource/exit_people.pt"
CONF_THRES  = 0.8
CAM_IDX     = 0
TOPIC_NAME  = "/exit/people_detected"

class _Signals(QObject):
    frameCaptured = pyqtSignal(object)   # np.ndarray (BGR)
    detected      = pyqtSignal(bool)     # True on detected
    errorOccurred = pyqtSignal(str)      # 에러 메시지

class ExitPeopleWorker(Node):
    def __init__(self, cam_idx=CAM_IDX, stop_on_detect=True):
        super().__init__('exit_people_worker')
        self.sig = _Signals()
        self.running = False
        self.stop_on_detect = stop_on_detect

        # 카메라
        self.cap = cv2.VideoCapture(cam_idx, cv2.CAP_V4L2)
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        self.cap.set(cv2.CAP_PROP_FPS, 30)

        if not self.cap.isOpened():
            self.sig.errorOccurred.emit(f"카메라({cam_idx})를 열 수 없습니다.")
        self.model = YOLO(MODEL_PATH)

        # QoS
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        self.pub = self.create_publisher(Bool, TOPIC_NAME, qos)
        self.thread = None

    def start(self):
        if self.thread is None or not self.thread.is_alive():
            self.running = True
            self.thread = threading.Thread(target=self.run, daemon=True)
            self.thread.start()

    def run(self):
        # 초기 False 한번
        self.publish_bool(False)

        while self.running:
            ok, frame = self.cap.read()
            if not ok:
                self.sig.errorOccurred.emit("프레임을 읽을 수 없습니다.")
                break

            processed_frame, detected = self.recognize_exit_people(frame)
            # UI로 프레임 전달
            self.sig.frameCaptured.emit(processed_frame)

            if detected:
                for _ in range(30):
                    self.publish_bool(True)
                    time.sleep(0.1)
                self.sig.detected.emit(True)
                if self.stop_on_detect:
                    self.running = False
                    break

        self.cap.release()

    def publish_bool(self, value: bool):
        msg = Bool(); msg.data = value
        self.pub.publish(msg)

    def recognize_exit_people(self, frame):
        results = self.model(frame, conf=CONF_THRES, verbose=False)
        detected = False
        for result in results:
            for box in result.boxes:
                if int(box.cls[0]) == 0 and float(box.conf[0]) >= CONF_THRES:
                    detected = True
                    x1, y1, x2, y2 = map(int, box.xyxy[0])
                    cv2.rectangle(frame, (x1,y1), (x2,y2), (0,255,0), 2)
                    cv2.putText(frame, f"Person {float(box.conf[0]):.2f}",
                                (x1, y1-5), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                                (0,255,0), 2)
        return frame, detected

    def stop(self):
        self.running = False
        if self.thread:
            self.thread.join()




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