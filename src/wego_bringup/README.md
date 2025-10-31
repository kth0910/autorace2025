# WeGO Bringup Package

## 📋 개요

`wego_bringup`은 WeGO 자율주행 시스템의 **통합 실행** 패키지입니다.  
전체 자율주행 스택(VISION → PLANNING → CONTROL)을 한 번에 실행하고 관리합니다.

## 🎯 주요 기능

### Launch Files

#### 1. **autonomous_stack.launch** (전체 시스템)
전체 자율주행 스택을 실행합니다.

```bash
roslaunch wego_bringup autonomous_stack.launch
```

**Arguments:**
- `rviz:=true/false` - RViz 시각화 실행 여부 (기본: false)
- `use_camera:=true/false` - USB 카메라 실행 여부 (기본: true)
- `simulation:=true/false` - 시뮬레이션 모드 (기본: false)

**예시:**
```bash
# RViz와 함께 실행
roslaunch wego_bringup autonomous_stack.launch rviz:=true

# 시뮬레이션 모드로 실행
roslaunch wego_bringup autonomous_stack.launch simulation:=true rviz:=true

# 카메라 없이 실행
roslaunch wego_bringup autonomous_stack.launch use_camera:=false
```

#### 2. **minimal.launch** (개별 모듈)
특정 모듈만 실행합니다 (개발 및 테스트용).

```bash
# 비전 모듈만 실행
roslaunch wego_bringup minimal.launch node_group:=vision

# 계획 모듈만 실행
roslaunch wego_bringup minimal.launch node_group:=planning

# 제어 모듈만 실행
roslaunch wego_bringup minimal.launch node_group:=control

# 모두 실행 (기본값)
roslaunch wego_bringup minimal.launch
```

## 📊 시스템 아키텍처

```
┌─────────────────────────────────────────────────────────────────┐
│                         WeGO Autonomous Stack                    │
└─────────────────────────────────────────────────────────────────┘

┌──────────────┐     ┌──────────────┐     ┌──────────────┐
│   VISION     │────▶│   PLANNING   │────▶│   CONTROL    │
└──────────────┘     └──────────────┘     └──────────────┘
       │                    │                     │
       ▼                    ▼                     ▼
  /vision/*          /planning/*           /ackermann_cmd
                                                  │
                                                  ▼
                                           /commands/*
```

### 데이터 흐름

```
/usb_cam/image_raw
       │
       ▼
[camera_node] → /vision/image_rect
       │
       ▼
[detection_node] → /vision/obstacles
       │
       ▼
[fusion_node] → /vision/fused_objects
       │
       ├─────────────────────────────┐
       ▼                             ▼
[path_planner_node] ───▶ /planning/path
                              │
                              ▼
                    [local_planner_node] ───▶ /planning/trajectory
                                                      │
                                                      ▼
                                              [controller_node] ───▶ /ackermann_cmd
                                                                          │
                                                                          ▼
                                                                  [vesc_bridge_node]
                                                                          │
                                                      ┌───────────────────┴────────────────┐
                                                      ▼                                    ▼
                                            /commands/motor/speed            /commands/servo/position
```

## 🚀 빠른 시작

### 1. 워크스페이스 빌드

```bash
cd ~/wego_ws
catkin_make
source devel/setup.bash
```

### 2. 전체 시스템 실행

```bash
# 기본 실행
roslaunch wego_bringup autonomous_stack.launch

# RViz 시각화 포함
roslaunch wego_bringup autonomous_stack.launch rviz:=true
```

### 3. 시뮬레이션 모드

```bash
# 시뮬레이션 모드 (실제 하드웨어 없이 테스트)
roslaunch wego_bringup autonomous_stack.launch simulation:=true rviz:=true
```

## 🛠️ 개발 및 디버깅

### 개별 모듈 테스트

```bash
# 비전 시스템만 테스트
roslaunch wego_bringup minimal.launch node_group:=vision

# 계획 시스템만 테스트
roslaunch wego_bringup minimal.launch node_group:=planning

# 제어 시스템만 테스트
roslaunch wego_bringup minimal.launch node_group:=control
```

### 토픽 모니터링

```bash
# 모든 토픽 확인
rostopic list

# 특정 토픽 모니터링
rostopic echo /vision/fused_objects
rostopic echo /planning/trajectory
rostopic echo /ackermann_cmd

# 토픽 주파수 확인
rostopic hz /vision/image_rect
rostopic hz /odom
```

### 노드 그래프 시각화

```bash
# RQt 그래프
rqt_graph

# TF 트리 확인
rosrun rqt_tf_tree rqt_tf_tree
```

## 📦 의존성

### 필수 패키지
- `wego_vision`
- `wego_planning`
- `wego_control`

### ROS 패키지
- `rviz`
- `tf`
- `rospy`

## ⚙️ 설정

### RViz 설정 파일
- `config/wego_autonomous.rviz`: 전체 시스템 시각화 설정

### 주요 토픽

**입력:**
- `/usb_cam/image_raw`: 카메라 이미지
- `/odom`: 차량 Odometry

**출력:**
- `/commands/motor/speed`: 모터 속도
- `/commands/servo/position`: 서보 위치

**내부 토픽:**
- `/vision/image_rect`: 보정된 이미지
- `/vision/obstacles`: 감지된 장애물
- `/vision/fused_objects`: 융합된 객체
- `/planning/path`: 전역 경로
- `/planning/trajectory`: 지역 궤적
- `/ackermann_cmd`: Ackermann 제어 명령

## 🔧 트러블슈팅

### 카메라가 인식되지 않을 때
```bash
# 카메라 디바이스 확인
ls /dev/video*

# 카메라 없이 실행
roslaunch wego_bringup autonomous_stack.launch use_camera:=false
```

### Odometry 데이터가 없을 때
```bash
# 시뮬레이션 모드 사용
roslaunch wego_bringup autonomous_stack.launch simulation:=true
```

### 빌드 오류
```bash
# 의존성 설치
rosdep install --from-paths src --ignore-src -r -y

# 클린 빌드
cd ~/wego_ws
catkin_make clean
catkin_make
```

## 📝 추후 개발 계획

- [ ] 로깅 시스템 통합 (rosbag 자동 저장)
- [ ] 시스템 모니터링 노드 추가
- [ ] 비상 정지 시스템 통합
- [ ] 웹 인터페이스 추가
- [ ] Docker 컨테이너 지원

## 📝 라이센스

MIT License

