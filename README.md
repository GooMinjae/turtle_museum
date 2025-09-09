# 🏛️ turtle_museum

**무인 박물관 보조 로봇 시스템 (Docent & Guard Robot)**  
본 프로젝트는 TurtleBot4와 ROS2 기반으로 **도슨트 안내**와 **박물관 경비** 역할을 수행하는 자율주행 로봇 시스템입니다.  

- 📌 SLAM 기반 실내 지도 작성 및 자율주행
- 📌 멀티 로봇 제어 (Guide Bot & Patrol Bot)
- 📌 YOLO 기반 객체 인식 및 추적
- 📌 관람객 안내, 전시 해설, 기념품 전달
- 📌 무인 박물관 운영 시나리오 구현

---

## 🚀 프로젝트 개요

- **목적**: 안내 해설사(도슨트) 인력 부족 문제 해결 및 무인 박물관 관리 자동화
- **핵심 기능**:
  - 관람객 인식 및 인원수 확인 (바코드 + YOLO)
  - 작품 위치 안내 및 설명
  - 순찰 및 전시물 이상 여부 확인
  - 기념품 탐지 및 전달
- **활용 로봇**: TurtleBot4 × 2 (Guide, Patrol)

---

## 🛠️ 사용 기술 스택

### Hardware
- TurtleBot4 (Create 3 + LiDAR + Depth Camera)
- Raspberry Pi 4 제어 모듈
- 2D LiDAR (360°)
- RGB-D 카메라 / USB 웹캠
- Laptop PC (개발 및 제어용)

### Software
- **ROS2 Humble** – SLAM, Navigation, 노드 통신
- **Python3** – 주요 로직 구현
- **YOLOv8** – 객체 탐지 및 사람 인식
- **OpenCV** – 이미지 처리
- **PyQt5** – 모니터링 UI/대시보드
- **SQLite3** – 방문객/기념품 DB 기록

---

## 📂 시스템 구조

### Flow Chart
- 관람객 입장 → 바코드 확인 → 인원 파악  
- 가이드 로봇 이동 → 작품 설명 → 관람객 동행  
- 순찰 로봇 이동 → 전시물 점검 → 기념품 전달  

### System Architecture
- Robot8 (Guide Bot) – 안내 및 설명
- Robot9 (Patrol Bot) – 순찰 및 보안
- PC1 – Guide Robot Control
- PC2 – Patrol Robot Control
- PC3 – 통합 모니터링 및 DB 관리

---

## 🎯 주요 기능

- **가이드 로봇**
  - 바코드 인식 → 관람객 인원 확인
  - 관람객 동선을 따라 작품 설명
  - 작품별 안내 멀티미디어 출력
- **순찰 로봇**
  - 전시물 유무 감지 및 순찰
  - 기념품 샵 이동 → 기념품 인식/수령 → 전달
- **데이터베이스**
  - 날짜별 관람객 수, 기념품 기록 저장
- **모니터링**
  - PyQt 기반 GUI로 로봇 상태 및 카메라 영상 확인

---

## 📊 성능 및 개선

- **YOLOv8n** 모델 최종 선택 (YOLOv11n 대비 mAP 4% ↑)
- QoS BEST_EFFORT 설정으로 이미지 지연 1000ms → 200ms 단축
- FPS 동기화 개선 (15Hz 처리로 프레임 드랍 최소화)
- 추후 개선: 다중 로봇 협업 시 안정적인 통신 및 부드러운 Tracking 구현

---

## 📹 시연 영상

▶️ [프로젝트 영상 보기](https://drive.google.com/file/d/1DuDNQV7jyd9tyvowtBA9Rhn2f8gMsLjb/view?usp=drive_link)

---

## 📌 License
MIT License
