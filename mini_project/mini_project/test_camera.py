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

list_available_cameras()