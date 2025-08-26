from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO
import rclpy
from rclpy.node import Node
import time
class yolo_camera_detect(Node):
    
    def __init__(self):
        self.inference_time = []
        super().__init__('turtlebot4_first_python_node')

        self.sub_cam = self.create_subscription(Image, '/robot8/oakd/rgb/preview/image_raw', self.callback_cam, 1)
        self.pub_yolo = self.create_publisher(Image, '/robot8/image', 1)

        self.bridge = CvBridge()
        self.model = YOLO("/home/rokey/Downloads/train/train_8s_patient20_avg83/weights/best.pt")
        # self.model = YOLO("/home/rokey/Downloads/train/train_8n_patient30_avg80/weights/best.pt")
        # self.model = YOLO("/home/rokey/Downloads/train/train_8n_patient20_avg80/weights/best.pt")
        # self.model = YOLO("/home/rokey/Downloads/train/train_11s_patient20_avg77/weights/best.pt")

    def callback_cam(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        start = time.time()


        results = self.model.predict(source=frame, conf=0.75, verbose=False)
        result = results[0]
        end = time.time()


        for box in result.boxes:
            cls_id = int(box.cls[0])
            conf = float(box.conf[0]) * 100
            label = f"{result.names[cls_id]} {conf:.1f}%"

            x1, y1, x2, y2 = map(int, box.xyxy[0])
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.putText(frame, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        msg_out = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')

        self.pub_yolo.publish(msg_out)
        self.inference_time.append(end-start)
        if len(self.inference_time) == 100:
            inference_time = sum(self.inference_time) / 100
        
            print(f"Average inference time: {(inference_time):.4f} seconds")
            print(f"FPS: {1/(inference_time):.4f}")
            self.inference_time = []


def main(args=None):
    rclpy.init(args=args)
    node = yolo_camera_detect()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
