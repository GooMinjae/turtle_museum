import cv2
from ultralytics import YOLO

class person_count_detect():
    def __init__(self):
        self.model = YOLO("/home/rokey/turtlebot4_ws/src/training/runs/detect/web_8n/weights/best.pt")  # COCO 기본 가중치
        self.cap = cv2.VideoCapture(2)    # 0=웹캠. 파일이면 "video.mp4"

    def person_count_detect(self):


        results = self.model.predict(source=frame, classes=[0], conf=0.5, verbose=False)

        while True:
            ok, frame = cap.read()
            if not ok:
                break

            # 사람만 감지 (COCO class 0)
            results = model.predict(source=frame, classes=[0], conf=0.5, verbose=False)
            r = results[0]
            n_people = 0

            if r.boxes is not None:
                n_people = len(r.boxes)

                # 바운딩박스 그리기
                for box in r.boxes:
                    x1, y1, x2, y2 = box.xyxy[0].int().tolist()
                    cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                    cv2.putText(frame, "person", (x1, max(0, y1-5)),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,0), 2)

            # 사람 수 오버레이
            cv2.putText(frame, f"People: {n_people}", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 255), 2)

            cv2.imshow("YOLOv8n - person count", frame)
            if cv2.waitKey(1) & 0xFF == 27:  # ESC로 종료
                break

        cap.release()
        cv2.destroyAllWindows()

def main():
    person_count_detect()
    try:
        pass
    except:
        pass

if __name__ == '__main__':
    main()