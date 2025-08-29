#!/usr/bin/env python3
import os
import cv2
import numpy as np
import rosbag2_py
from cv_bridge import CvBridge
from rclpy.serialization import deserialize_message

# ===== 사용자 설정 =====
BAG_PATH   = os.path.expanduser("~/rosbag2_2025_08_29-15_29_17")  # 폴더 경로(*.db3가 들어있는 디렉토리)
TOPIC_NAME = "/robot9/oakd/rgb/image_raw/compressed"                          # 변환할 이미지 토픽
OUTPUT_MP4 = "output.mp4"                                          # 결과 파일명
FPS        = 30                                                    # 모르면 30으로 고정 (필요시 조정)

# ===== 내부 =====
bridge = CvBridge()
writer = None
frame_size = None

def list_topics(reader):
    ts = reader.get_all_topics_and_types()
    return {t.name: t.type for t in ts}

def ensure_bgr8(img):
    """VideoWriter에 넣을 수 있도록 8-bit 3채널로 맞춘다."""
    if img is None:
        return None
    if img.dtype == np.uint16:
        # depth(16UC1) → 보기 좋게 normalize + 컬러맵
        norm = cv2.normalize(img, None, 0, 255, cv2.NORM_MINMAX)
        u8 = norm.astype(np.uint8)
        color = cv2.applyColorMap(u8, cv2.COLORMAP_JET)
        return color
    if len(img.shape) == 2:
        # GRAY → BGR
        return cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
    if img.shape[2] == 4:
        # BGRA → BGR
        return cv2.cvtColor(img, cv2.COLOR_BGRA2BGR)
    # 이미 BGR일 가능성
    return img

def open_reader(bag_path):
    storage_options = rosbag2_py.StorageOptions(uri=bag_path, storage_id="sqlite3")
    converter_options = rosbag2_py.ConverterOptions(input_serialization_format="cdr",
                                                    output_serialization_format="cdr")
    reader = rosbag2_py.SequentialReader()
    reader.open(storage_options, converter_options)
    return reader

def main():
    global writer, frame_size

    reader = open_reader(BAG_PATH)
    type_map = list_topics(reader)

    if TOPIC_NAME not in type_map:
        print("❌ 지정한 토픽이 bag에 없습니다. 사용 가능한 이미지 토픽:")
        for name, typ in type_map.items():
            if "Image" in typ:  # Image 또는 CompressedImage만 표시
                print(f"  - {name} ({typ})")
        return

    msg_type_name = type_map[TOPIC_NAME]
    if msg_type_name == "sensor_msgs/msg/Image":
        from sensor_msgs.msg import Image as MsgT
        is_compressed = False
    elif msg_type_name == "sensor_msgs/msg/CompressedImage":
        from sensor_msgs.msg import CompressedImage as MsgT
        is_compressed = True
    else:
        print(f"❌ 지원하지 않는 메시지 타입: {msg_type_name}")
        return

    # 다시 처음부터 읽도록 새로 open (SequentialReader는 한 번 훑으면 되돌리기 번거로움)
    reader = open_reader(BAG_PATH)

    count = 0
    while reader.has_next():
        topic, data, t = reader.read_next()
        if topic != TOPIC_NAME:
            continue

        msg = deserialize_message(data, MsgT)

        # 메시지 타입에 따라 디코딩
        if not is_compressed:
            # sensor_msgs/Image
            # RGB/Mono/Depth 모두 처리
            try:
                # bgr8로 바로 변환 시도 (Mono16 등은 실패할 수 있음)
                frame = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            except Exception:
                # 실패하면 원본 그대로 받고 후처리
                frame = bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        else:
            # sensor_msgs/CompressedImage (jpeg, png, compressedDepth 등)
            # cv_bridge가 포맷에 맞춰 자동 디코드 (passthrough로 받아 후처리)
            try:
                frame = bridge.compressed_imgmsg_to_cv2(msg, desired_encoding="passthrough")
            except Exception as e:
                # 드물게 compressedDepth에서 문제가 될 경우 수동 디코딩 시도
                # PNG로 인코딩된 depth가 대부분이라 imdecode 가능
                arr = np.frombuffer(msg.data, dtype=np.uint8)
                frame = cv2.imdecode(arr, cv2.IMREAD_UNCHANGED)

        # VideoWriter에 맞게 정규화(BGR8, 3채널)
        frame = ensure_bgr8(frame)
        if frame is None:
            continue

        if writer is None:
            h, w = frame.shape[:2]
            frame_size = (w, h)
            fourcc = cv2.VideoWriter_fourcc(*"mp4v")  # 필요시 'avc1' 등으로 교체
            writer = cv2.VideoWriter(OUTPUT_MP4, fourcc, FPS, frame_size)
            if not writer.isOpened():
                print("❌ VideoWriter 열기 실패. 코덱/경로 확인 필요.")
                return
            print(f"▶ Writing to {OUTPUT_MP4} ({w}x{h} @ {FPS}fps)")

        # 크기가 변하는 토픽이라면 첫 프레임 크기에 맞춰 리사이즈
        if (frame.shape[1], frame.shape[0]) != frame_size:
            frame = cv2.resize(frame, frame_size, interpolation=cv2.INTER_AREA)

        writer.write(frame)
        count += 1

    if writer:
        writer.release()

    print(f"✅ 변환 완료: {OUTPUT_MP4} (총 {count} 프레임)")
    if count == 0:
        print("⚠️ 선택한 토픽에서 프레임을 찾지 못했습니다. 토픽명/타입을 다시 확인하세요.")

if __name__ == "__main__":
    main()
