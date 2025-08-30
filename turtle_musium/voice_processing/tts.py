# voice_processing/tts_service.py
from __future__ import annotations
from gtts import gTTS
from io import BytesIO
from pydub import AudioSegment
import simpleaudio as sa
import threading
import time

_tts_lock = threading.Lock()      # gTTS 호출(네트워크) 직렬화
_play_lock = threading.Lock()     # 동시 재생 시 오디오 장치 충돌 방지(필요 시 해제 가능)
_BASE_RATE = 220                  # 재생 속도 환산 기준 (말 빠르기 근사)

def _gen_tts_mp3_bytes(text: str, lang: str = "ko") -> BytesIO:
    """
    gTTS로 MP3를 메모리 버퍼(BytesIO)에 생성하여 반환.
    """
    buf = BytesIO()
    # gTTS는 네트워크 호출이므로 잠깐 직렬화(동시 대량요청 예방)
    with _tts_lock:
        tts = gTTS(text=text, lang=lang)
        tts.write_to_fp(buf)
    buf.seek(0)
    return buf

def _speed_adjust(seg: AudioSegment, rate: int | None) -> AudioSegment:
    """
    gTTS는 rate(말 빠르기)를 직접 지원하지 않으므로,
    pydub의 frame_rate 트릭으로 재생 속도를 근사 조절 (피치도 함께 변함).
    - rate 기준은 _BASE_RATE(200). rate가 300이면 1.5배속, 100이면 0.5배속.
    """
    if not rate:
        return seg
    factor = max(0.25, min(4.0, rate / float(_BASE_RATE)))
    if abs(factor - 1.0) < 1e-3:
        return seg
    new_frame_rate = int(seg.frame_rate * factor)
    # 속도 변경(피치 변화 동반) 후, 원래 frame_rate로 재설정
    seg_fast = seg._spawn(seg.raw_data, overrides={"frame_rate": new_frame_rate})
    return seg_fast.set_frame_rate(seg.frame_rate)

def _play_segment(seg: AudioSegment, block: bool = True):
    """
    simpleaudio로 메모리에서 바로 재생.
    block=True면 재생 완료까지 대기.
    """
    # 필요 시 재생을 직렬화(동시 재생 원치 않으면 유지, 동시 허용하려면 락 제거)
    with _play_lock:
        play_obj = sa.play_buffer(
            seg.raw_data,
            num_channels=seg.channels,
            bytes_per_sample=seg.sample_width,
            sample_rate=seg.frame_rate
        )
    if block:
        play_obj.wait_done()
    return play_obj

def say(text: str, wait_sec: float = 0.0, block: bool = True, rate: int | None = None, lang: str = "ko"):
    """
    텍스트를 말함(메모리 재생).
    - text: 말할 텍스트
    - wait_sec: 말한 뒤 추가로 기다릴 시간(초)
    - block: True면 재생 완료까지 대기, False면 비동기 재생 후 바로 반환
    - rate: (선택) 말 빠르기 근사. 기준 200. ex) 180, 220
    - lang: gTTS 언어 코드(기본 'ko')
    """
    # 1) gTTS → MP3(BytesIO)
    mp3_buf = _gen_tts_mp3_bytes(text, lang=lang)

    # 2) MP3(BytesIO) → AudioSegment
    seg = AudioSegment.from_file(mp3_buf, format="mp3")

    # 3) 속도 조절(근사)
    seg = _speed_adjust(seg, rate)

    # 4) 재생
    _play_segment(seg, block=block)

    # 5) 후행 대기
    if wait_sec and wait_sec > 0:
        time.sleep(wait_sec)

def _say_worker(text: str, wait_sec: float, rate: int | None, lang: str):
    try:
        say(text=text, wait_sec=wait_sec, block=True, rate=rate, lang=lang)
    except Exception:
        # 비동기 스레드 오류는 로깅만 하는 편이 안전
        # 필요하다면 여기서 logging.exception(...) 추가
        pass

def say_async(text: str, wait_sec: float = 0.0, rate: int | None = None, lang: str = "ko"):
    """
    비동기로 말함. 현재 스레드를 막고 싶지 않을 때 사용.
    내부적으로 say(block=True)를 새 스레드에서 실행(재생은 해당 스레드에서 완료).
    """
    threading.Thread(
        target=_say_worker,
        args=(text, wait_sec, rate, lang),
        daemon=True
    ).start()

if __name__ == "__main__":
    # 블로킹(끝까지 기다림)
    say("안녕하세요.")
