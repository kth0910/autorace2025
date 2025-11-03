# Vision + Planning 연동 실행 가이드

## 📋 실행 순서 (필수!)

### 사전 준비 (한 번만)

```bash
# 1. 워크스페이스 빌드 확인
cd ~/autorace2025/autorace2025
source /opt/ros/noetic/setup.bash
catkin_make
source devel/setup.bash

# 2. 실행 권한 부여
chmod +x src/wego_vision/scripts/*.py
chmod +x src/wego_planning/scripts/*.py

# 3. 카메라 권한
sudo chmod 666 /dev/video0

# 4. bashrc에 추가 (선택사항 - 매번 소싱 안 하려면)
echo "source ~/autorace2025/autorace2025/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

---

## 🚀 실행 순서 (매번)

### 터미널 1️⃣: roscore (기본)

```bash
roscore
```

**역할:** ROS 마스터 노드 (모든 노드의 중앙 관리자)  
**유지:** 계속 실행 상태 유지 (절대 종료하지 마세요!)  
**확인:** `started core service [/rosout]` 메시지 나오면 OK

---

### 터미널 2️⃣: 카메라 (Vision)

```bash
# 소싱
source ~/autorace2025/autorace2025/devel/setup.bash

# 카메라 퍼블리셔 실행
rosrun wego_vision simple_camera_publisher.py
```

**역할:** 카메라에서 이미지를 읽어서 `/usb_cam/image_raw` 토픽으로 발행  
**확인:** `[Simple Camera] 초기화 완료` 메시지  
**유지:** 계속 실행 상태 유지

**확인 방법 (새 터미널):**
```bash
rostopic hz /usb_cam/image_raw
# 출력: average rate: 30.0
```

---

### 터미널 3️⃣: 이미지 보정 (Vision)

```bash
# 소싱
source ~/autorace2025/autorace2025/devel/setup.bash

# 이미지 보정 노드 실행
rosrun wego_vision camera_node.py
```

**역할:** `/usb_cam/image_raw` → `/vision/image_rect` 변환 (왜곡 보정)  
**확인:** `[Camera Node] 초기화 완료` 메시지  
**유지:** 계속 실행 상태 유지

**확인 방법 (새 터미널):**
```bash
rostopic hz /vision/image_rect
# 출력: average rate: 30.0
```

---

### 터미널 4️⃣: 차선 검출 (Vision)

```bash
# 소싱
source ~/autorace2025/autorace2025/devel/setup.bash

# 차선 검출 노드 실행
rosrun wego_vision lane_detection_simple.py
```

**역할:** `/vision/image_rect` → `/vision/lane_info` 차선 정보 생성  
**확인:** `[Simple Lane Detection] 초기화 완료` 메시지  
**유지:** 계속 실행 상태 유지

**확인 방법 (새 터미널):**
```bash
rostopic echo /vision/lane_info
# left_lane_detected: True
# confidence: 0.9
# center_offset: 0.023
```

---

### 터미널 5️⃣: Odometry (Planning 필요)

```bash
# 소싱
source ~/autorace2025/autorace2025/devel/setup.bash

# Dummy Odometry 실행 (실제 로봇 없을 때)
rosrun wego_bringup dummy_odom_node.py
```

**역할:** `/odom` 토픽 발행 (차량 위치 정보)  
**확인:** `[Dummy Odom] 초기화 완료` 메시지  
**유지:** 계속 실행 상태 유지

**실제 로봇 있으면:** 실제 odometry 노드 실행

---

### 터미널 6️⃣: Path Planner (Planning)

```bash
# 소싱
source ~/autorace2025/autorace2025/devel/setup.bash

# 차선 기반 경로 계획
rosrun wego_planning path_planner_with_lane.py
```

**역할:** `/vision/lane_info` + `/odom` → `/planning/path` 경로 생성  
**확인:** `[Path Planner with Lane] 초기화 완료` 메시지  
**유지:** 계속 실행 상태 유지

**확인 방법 (새 터미널):**
```bash
rostopic echo /planning/path
# path with waypoints
```

---

### 터미널 7️⃣: 시각화 (선택사항)

```bash
# RViz 실행 (별도 소싱 불필요)
rviz
```

**RViz에서 추가:**
- **Image** → `/vision/lane_image` (차선 검출)
- **Path** → `/planning/path` (계획된 경로)
- **Odometry** → `/odom` (차량 위치)

**또는 간단하게:**
```bash
# 이미지만 확인
rosrun rqt_image_view rqt_image_view /vision/lane_image
```

---

## 📊 데이터 흐름 확인

```
카메라 → 이미지 보정 → 차선 검출 ┐
                                 ├─→ 경로 계획 → /planning/path
Odometry ────────────────────────┘
```

### 토픽 확인

```bash
rostopic list

# 필수 토픽:
# /usb_cam/image_raw       ← 터미널 2 (카메라)
# /vision/image_rect       ← 터미널 3 (이미지 보정)
# /vision/lane_info        ← 터미널 4 (차선 검출)
# /odom                    ← 터미널 5 (Odometry)
# /planning/path           ← 터미널 6 (경로 계획)
```

### 주파수 확인

```bash
# 각 토픽 주파수 확인 (새 터미널)
rostopic hz /usb_cam/image_raw    # 30 Hz
rostopic hz /vision/lane_info     # 30 Hz
rostopic hz /odom                 # 50 Hz
rostopic hz /planning/path        # 1-10 Hz
```

---

## 🎯 간단 실행 (launch 파일 사용)

### Vision + Planning 통합 실행

```bash
# 터미널 1: roscore
roscore

# 터미널 2: 전체 실행
source ~/autorace2025/autorace2025/devel/setup.bash
roslaunch wego_bringup autonomous_stack.launch simulation:=true rviz:=true
```

**장점:** 모든 노드가 한 번에 실행됨!

---

## ⚙️ 개별 노드 디버깅 시

특정 노드만 재시작하려면:

```bash
# 노드 종료
rosnode kill /lane_detection_simple

# 재시작
rosrun wego_vision lane_detection_simple.py
```

---

## 🐛 문제 해결

### 문제 1: "waiting for /vision/lane_info"

**원인:** 차선 검출 노드가 실행 안 됨  
**해결:** 터미널 4 확인 (lane_detection_simple 실행 중?)

### 문제 2: "waiting for /odom"

**원인:** Odometry 노드가 실행 안 됨  
**해결:** 터미널 5 확인 (dummy_odom_node 실행 중?)

### 문제 3: /planning/path가 발행 안 됨

**원인:** path_planner_with_lane 노드가 데이터를 못 받음  
**확인:**
```bash
rostopic echo /vision/lane_info
# confidence > 0.5 인가요?

rostopic echo /odom
# 데이터 나오나요?
```

---

## ✅ 전체 체크리스트

실행 순서대로:

- [ ] 1️⃣ roscore 실행
- [ ] 2️⃣ simple_camera_publisher 실행
- [ ] 3️⃣ camera_node 실행
- [ ] 4️⃣ lane_detection_simple 실행
- [ ] 5️⃣ dummy_odom_node 실행
- [ ] 6️⃣ path_planner_with_lane 실행
- [ ] 7️⃣ (선택) RViz 또는 rqt_image_view

### 각 단계 확인

```bash
# 2번 후:
rostopic hz /usb_cam/image_raw      # 30 Hz?

# 3번 후:
rostopic hz /vision/image_rect      # 30 Hz?

# 4번 후:
rostopic echo /vision/lane_info     # confidence > 0?

# 5번 후:
rostopic hz /odom                   # 50 Hz?

# 6번 후:
rostopic echo /planning/path        # path 나오나요?
```

---

## 💡 권장: tmux 사용

여러 터미널 관리가 편해집니다:

```bash
# tmux 세션 시작
tmux new -s wego

# 창 분할 (6개 창 만들기)
# Ctrl+b " : 수평 분할
# Ctrl+b % : 수직 분할
# Ctrl+b 방향키 : 창 이동

# 각 창에서 순서대로 실행!
```

---

## 🎯 빠른 참조

**최소 실행 (Vision만):**
```bash
roscore &
rosrun wego_vision simple_camera_publisher.py &
rosrun wego_vision camera_node.py &
rosrun wego_vision lane_detection_simple.py
```

**Vision + Planning:**
```bash
roscore &
rosrun wego_vision simple_camera_publisher.py &
rosrun wego_vision camera_node.py &
rosrun wego_vision lane_detection_simple.py &
rosrun wego_bringup dummy_odom_node.py &
rosrun wego_planning path_planner_with_lane.py
```

---

**이 문서를 저장해서 매번 참고하세요!** 📋



