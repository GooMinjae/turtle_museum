from ultralytics import YOLO
import cv2

model = YOLO("yolov8n.pt")
cap = cv2.VideoCapture("video.mp4")

# 임계선(y=300 예시). 이 선을 아래→위로 지나가면 카운트 증가.
line_y = 300
seen_ids = set()
count_up = 0

while True:
    ok, frame = cap.read()
    if not ok: break

    # tracker는 strongsort.yaml 같은 기본 트래커 설정 사용 가능
    # 사람만 추적하려면 persist=True + classes=[0]
    results = model.track(source=frame, persist=True, classes=[0], conf=0.5, verbose=False)
    r = results[0]

    if hasattr(r, "boxes") and r.boxes is not None and r.boxes.id is not None:
        ids = r.boxes.id.int().tolist()
        y_centers = r.boxes.xywh[:,1].tolist()

        for tid, cy in zip(ids, y_centers):
            # 선 통과 체크 로직(간단 예시): 처음 본 ID가 선 위쪽에 있으면 카운트
            if tid not in seen_ids and cy < line_y:
                count_up += 1
                seen_ids.add(tid)

        # 시각화
        for box, tid in zip(r.boxes.xyxy, ids):
            x1,y1,x2,y2 = box.int().tolist()
            cv2.rectangle(frame,(x1,y1),(x2,y2),(0,255,0),2)
            cv2.putText(frame,f"id:{tid}",(x1,y1-5),cv2.FONT_HERSHEY_SIMPLEX,0.6,(0,255,0),2)

    cv2.line(frame,(0,line_y),(frame.shape[1],line_y),(255,255,255),2)
    cv2.putText(frame, f"Unique crossed: {count_up}", (10,30),
                cv2.FONT_HERSHEY_SIMPLEX,1,(0,255,255),2)

    cv2.imshow("track & count", frame)
    if cv2.waitKey(1) & 0xFF == 27:
        break

cap.release()
cv2.destroyAllWindows()
