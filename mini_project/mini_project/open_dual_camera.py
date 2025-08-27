# two_usb_cams_yolo_pub_with_service.py
import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_srvs.srv import Trigger
from std_msgs.msg import String
from cv_bridge import CvBridge
from ultralytics import YOLO
import numpy as np

# ========= 설정 =========
W, H   = 640, 480
FPS    = 20.0
IDX0, IDX1 = 2, 4  # 사용중인 /dev/video 인덱스에 맞게
MODEL_PATH = "/home/rokey/turtlebot4_ws/src/mini_project/resource/best.pt"
CONF_THRES = 0.7
DEVICE     = 0     # GPU: 0, CPU: "cpu"
# =======================

def open_cam(idx):
    cap = cv2.VideoCapture(idx, cv2.CAP_V4L2)
    if not cap.isOpened():
        raise RuntimeError(f'카메라 열기 실패: index {idx}')
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, W)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, H)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
    cap.set(cv2.CAP_PROP_FPS, FPS)
    return cap

def max_confidence(yolo_result):
    """
    Ultralytics 결과에서 가장 높은 confidence (float).
    탐지가 없으면 0.0 반환.
    """
    try:
        boxes = yolo_result.boxes
        if boxes is None or len(boxes) == 0:
            return 0.0
        # boxes.conf: tensor [N], 최고값 추출
        return float(boxes.conf.max().item())
    except Exception:
        return 0.0

class DualUSBCamYOLOPublisher(Node):
    def __init__(self):
        super().__init__('dual_usb_cam_yolo_publisher')
        self.bridge = CvBridge()

        # 탐지 결과 이미지 퍼블리셔
        self.pub0_det = self.create_publisher(Image, '/cam0/detect/image', 10)
        self.pub1_det = self.create_publisher(Image, '/cam1/detect/image', 10)

        # (옵션) 현재 상태 브로드캐스트
        self.pub_detected_cam = self.create_publisher(String, '/detected_cam', 10)

        # 서비스: 선택 결과 반환
        self.srv = self.create_service(Trigger, '/detected_cam/select', self.on_select_request)

        # 카메라
        self.cap0 = open_cam(IDX0)
        self.cap1 = open_cam(IDX1)

        # YOLO
        self.model = YOLO(MODEL_PATH)
        self.model.to(DEVICE)

        # 상태 보관 (서비스 응답용 최신값)
        self.last_n0 = 0
        self.last_n1 = 0
        self.last_c0 = 0.0
        self.last_c1 = 0.0
        self.last_choice = "none"  # "cam0" | "cam1" | "none"

        # 타이머
        period = 1.0 / FPS
        self.timer = self.create_timer(period, self.tick)

        self.get_logger().info(
            f'YOLO loaded: {MODEL_PATH}, device={DEVICE}, conf={CONF_THRES}\n'
            f'Publishing /cam0/detect/image, /cam1/detect/image; Service: /detected_cam/select'
        )

    def decide_choice(self, det0: bool, det1: bool, c0: float, c1: float) -> str:
        """
        선택 규칙:
        - 둘 다 미탐지: "none"
        - 한쪽만 탐지: 해당 cam
        - both 탐지: 더 높은 confidence 쪽 선택 (동률이면 cam0 우선)
        """
        if not det0 and not det1:
            return "none"
        if det0 and not det1:
            return "cam0"
        if det1 and not det0:
            return "cam1"
        # both
        if c0 >= c1:
            return "cam0"
        else:
            return "cam1"

    def tick(self):
        ok0, f0 = self.cap0.read()
        ok1, f1 = self.cap1.read()
        if not ok0 or f0 is None:
            self.get_logger().warn('cam0 프레임 읽기 실패')
            return
        if not ok1 or f1 is None:
            self.get_logger().warn('cam1 프레임 읽기 실패')
            return

        # ===== YOLO 추론 =====
        res0 = self.model.predict(source=f0, conf=CONF_THRES, verbose=False, device=DEVICE)[0]
        res1 = self.model.predict(source=f1, conf=CONF_THRES, verbose=False, device=DEVICE)[0]

        # 개수/최대 confidence
        n0 = 0 if res0.boxes is None else len(res0.boxes)
        n1 = 0 if res1.boxes is None else len(res1.boxes)
        c0 = max_confidence(res0)
        c1 = max_confidence(res1)

        det0 = n0 > 0
        det1 = n1 > 0

        # 선택 결정
        choice = self.decide_choice(det0, det1, c0, c1)

        # 시각화 이미지
        vis0 = res0.plot()
        vis1 = res1.plot()

        # 퍼블리시
        now = self.get_clock().now().to_msg()

        msg0_det = self.bridge.cv2_to_imgmsg(vis0, encoding='bgr8')
        msg0_det.header.stamp = now
        msg0_det.header.frame_id = 'cam0_frame'
        self.pub0_det.publish(msg0_det)

        msg1_det = self.bridge.cv2_to_imgmsg(vis1, encoding='bgr8')
        msg1_det.header.stamp = now
        msg1_det.header.frame_id = 'cam1_frame'
        self.pub1_det.publish(msg1_det)

        # 상태 브로드캐스트(문자열)
        s = String()
        s.data = choice
        self.pub_detected_cam.publish(s)

        # 최신 상태 저장 (서비스 응답용)
        self.last_n0, self.last_n1 = n0, n1
        self.last_c0, self.last_c1 = c0, c1
        self.last_choice = choice

        # (선택) 로그
        self.get_logger().info(f'cam0: n={n0}, max_conf={c0:.3f} | cam1: n={n1}, max_conf={c1:.3f} -> choice={choice}')

    # -------- 서비스 콜백 --------
    def on_select_request(self, request, response):
        """
        Trigger 응답:
        success: 선택 가능 여부 (none이면 False)
        message: "cam0|cam1|none max_conf=.. (cam0=.., cam1=..)"
        """
        if self.last_choice == "none":
            response.success = False
            response.message = f'none (cam0={self.last_c0:.3f}, cam1={self.last_c1:.3f})'
        else:
            max_conf = max(self.last_c0, self.last_c1)
            response.success = True
            response.message = (
                f'{self.last_choice} max_conf={max_conf:.3f} '
                f'(cam0={self.last_c0:.3f}, cam1={self.last_c1:.3f})'
            )
        return response

    def destroy_node(self):
        try:
            if hasattr(self, 'cap0') and self.cap0: self.cap0.release()
            if hasattr(self, 'cap1') and self.cap1: self.cap1.release()
        except Exception:
            pass
        super().destroy_node()

def main():
    rclpy.init()
    node = DualUSBCamYOLOPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
