import cv2

def list_available_cameras(max_index=10):
    print("🔍 사용 가능한 카메라 인덱스 찾는 중...")
    for i in range(max_index):
        cap = cv2.VideoCapture(i)
        if cap.isOpened():
            print(f"✅ 카메라 발견: 인덱스 {i}")
            cap.release()
        else:
            print(f"❌ 사용 불가: 인덱스 {i}")

# list_available_cameras()
import cv2

cap = cv2.VideoCapture(2)  # 기본 카메라: 0
if not cap.isOpened():
    raise RuntimeError("카메라를 열 수 없습니다. 인덱스/권한/점유 상태를 확인하세요.")

while True:
    ok, frame = cap.read()
    if not ok:
        print("프레임을 읽지 못했습니다.")
        break

    cv2.imshow("Webcam", frame)     # 화면 표시 (BGR)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
