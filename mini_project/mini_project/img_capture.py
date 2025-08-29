import cv2
import os

# 저장할 폴더
save_dir = "captured_images"
os.makedirs(save_dir, exist_ok=True)

# 카메라 열기 (0번 카메라)
cap = cv2.VideoCapture(2)

img_count = 201

while True:
    ret, frame = cap.read()
    if not ret:
        print("카메라를 열 수 없습니다.")
        break

    # 화면에 표시
    cv2.imshow("Camera", frame)

    # 키 입력 대기 (1ms)
    key = cv2.waitKey(1) & 0xFF

    # d 키를 누르면 현재 프레임 저장
    if key == ord('d'):
        filename = os.path.join(save_dir, f"capture_{img_count:03d}.png")
        cv2.imwrite(filename, frame)
        print(f"{filename} 저장 완료")
        img_count += 1

    # q 키를 누르면 종료
    elif key == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
