# WeGO - 자율주행 ROS 워크스페이스

<div align="center">

**ROS Noetic 기반 자율주행 차량 소프트웨어 스택**

[![ROS Version](https://img.shields.io/badge/ROS-Noetic-blue.svg)](http://wiki.ros.org/noetic)
[![Python Version](https://img.shields.io/badge/Python-3.8+-green.svg)](https://www.python.org/)
[![License](https://img.shields.io/badge/License-MIT-yellow.svg)](LICENSE)

</div>

---

## 📋 프로젝트 개요

**WeGO**는 ROS Noetic 기반의 자율주행 차량 소프트웨어 플랫폼입니다.  
환경 인식(Vision), 경로 계획(Planning), 차량 제어(Control)의 3가지 핵심 모듈로 구성되어 있으며,  
확장 가능하고 모듈화된 아키텍처를 제공합니다.

## 🎯 주요 특징

- ✅ **모듈화된 아키텍처**: Vision, Planning, Control이 독립적으로 동작
- ✅ **ROS Noetic 완벽 지원**: Python 3 네이티브 지원
- ✅ **실시간 처리**: 센서 융합 및 제어 루프 최적화
- ✅ **시뮬레이션 지원**: 하드웨어 없이도 개발 및 테스트 가능
- ✅ **확장 가능**: 새로운 알고리즘 및 센서 쉽게 추가 가능

## 🏗️ 시스템 아키텍처

```
┌─────────────────────────────────────────────────────────────────┐
│                         WeGO Autonomous Stack                    │
└─────────────────────────────────────────────────────────────────┘

  VISION              PLANNING             CONTROL
┌──────────┐       ┌──────────┐        ┌──────────┐
│ Camera   │──────▶│  Path    │───────▶│Controller│
│ Node     │       │ Planner  │        │   Node   │
└──────────┘       └──────────┘        └──────────┘
     │                   │                    │
┌──────────┐       ┌──────────┐        ┌──────────┐
│Detection │       │  Local   │        │   VESC   │
│  Node    │──────▶│ Planner  │        │  Bridge  │
└──────────┘       └──────────┘        └──────────┘
     │
┌──────────┐
│ Fusion   │
│  Node    │
└──────────┘
```

### 데이터 흐름

```
camera/image → detection → fusion → path_planner → local_planner → controller → vesc_bridge → motors
```

## 📦 패키지 구조

```
wego_ws/
├── src/
│   ├── wego_vision/          # 환경 인식 모듈
│   │   ├── scripts/
│   │   │   ├── camera_node.py
│   │   │   ├── detection_node.py
│   │   │   └── fusion_node.py
│   │   ├── launch/
│   │   ├── config/
│   │   └── README.md
│   │
│   ├── wego_planning/        # 경로 계획 모듈
│   │   ├── scripts/
│   │   │   ├── path_planner_node.py
│   │   │   └── local_planner_node.py
│   │   ├── launch/
│   │   ├── config/
│   │   └── README.md
│   │
│   ├── wego_control/         # 차량 제어 모듈
│   │   ├── scripts/
│   │   │   ├── controller_node.py
│   │   │   └── vesc_bridge_node.py
│   │   ├── launch/
│   │   ├── config/
│   │   └── README.md
│   │
│   └── wego_bringup/         # 통합 실행 패키지
│       ├── launch/
│       │   ├── autonomous_stack.launch
│       │   └── minimal.launch
│       ├── config/
│       └── README.md
│
├── README.md                 # 이 파일
└── requirements.txt          # Python 의존성
```

## 🚀 빠른 시작

### 1. 시스템 요구사항

- **OS**: Ubuntu 20.04 LTS
- **ROS**: Noetic Ninjemys
- **Python**: 3.8 이상
- **필수 패키지**: `python3-opencv`, `python3-numpy`, `python3-scipy`

### 2. 설치

#### Step 1: ROS Noetic 설치

```bash
# ROS Noetic 설치 (이미 설치된 경우 건너뛰기)
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
sudo apt update
sudo apt install ros-noetic-desktop-full
```

#### Step 2: 워크스페이스 빌드

```bash
cd ~/wego_ws
catkin_make
source devel/setup.bash
```

#### Step 3: 의존성 설치

```bash
# Python 패키지 설치
pip3 install -r requirements.txt

# ROS 의존성 설치
rosdep install --from-paths src --ignore-src -r -y
```

#### Step 4: Python 스크립트 실행 권한 부여

```bash
# 모든 Python 노드에 실행 권한 부여
find src -name "*.py" -type f -exec chmod +x {} \;
```

### 3. 실행

#### 전체 자율주행 스택 실행

```bash
roslaunch wego_bringup autonomous_stack.launch
```

#### RViz 시각화 포함 실행

```bash
roslaunch wego_bringup autonomous_stack.launch rviz:=true
```

#### 시뮬레이션 모드 (하드웨어 없이)

```bash
roslaunch wego_bringup autonomous_stack.launch simulation:=true rviz:=true
```

#### 개별 모듈 실행

```bash
# Vision만 실행
roslaunch wego_vision vision_pipeline.launch

# Planning만 실행
roslaunch wego_planning planner.launch

# Control만 실행
roslaunch wego_control control.launch
```

## 📊 모듈 상세 설명

### 🔍 Vision (wego_vision)

**목적**: 센서 데이터로부터 환경 정보 추출

**핵심 노드**:
- `camera_node`: 카메라 이미지 보정
- `detection_node`: 객체 감지 (ArUco, YOLOv8)
- `fusion_node`: 다중 센서 융합

**주요 토픽**:
- 입력: `/usb_cam/image_raw`
- 출력: `/vision/fused_objects`

[자세한 내용 →](src/wego_vision/README.md)

---

### 🗺️ Planning (wego_planning)

**목적**: 최적 경로 계획 및 궤적 생성

**핵심 노드**:
- `path_planner_node`: 전역 경로 계획 (A*, RRT)
- `local_planner_node`: 지역 궤적 생성 (DWA, Pure Pursuit)

**주요 토픽**:
- 입력: `/vision/fused_objects`, `/odom`
- 출력: `/planning/trajectory`

[자세한 내용 →](src/wego_planning/README.md)

---

### 🎮 Control (wego_control)

**목적**: 경로 추종 및 차량 제어

**핵심 노드**:
- `controller_node`: 경로 추종 제어 (Stanley)
- `vesc_bridge_node`: VESC 모터 인터페이스

**주요 토픽**:
- 입력: `/planning/trajectory`, `/odom`
- 출력: `/commands/motor/speed`, `/commands/servo/position`

[자세한 내용 →](src/wego_control/README.md)

---

### 🚀 Bringup (wego_bringup)

**목적**: 전체 시스템 통합 실행

**주요 런치 파일**:
- `autonomous_stack.launch`: 전체 스택 실행
- `minimal.launch`: 개별 모듈 실행

[자세한 내용 →](src/wego_bringup/README.md)

## 🧪 개발 및 테스트

### 토픽 모니터링

```bash
# 모든 활성 토픽 확인
rostopic list

# 특정 토픽 데이터 확인
rostopic echo /vision/fused_objects
rostopic echo /planning/trajectory
rostopic echo /ackermann_cmd

# 토픽 발행 주기 확인
rostopic hz /vision/image_rect
```

### 노드 그래프 시각화

```bash
# RQt 노드 그래프
rqt_graph

# TF 트리 확인
rosrun rqt_tf_tree rqt_tf_tree
```

### 로그 확인

```bash
# rosout 로그 확인
rostopic echo /rosout

# 특정 노드 로그 레벨 변경
rosservice call /camera_node/set_logger_level "logger: 'rosout'
level: 'debug'"
```

## 📚 주요 토픽 목록

| 토픽 이름 | 메시지 타입 | 설명 |
|----------|------------|------|
| `/usb_cam/image_raw` | sensor_msgs/Image | 원본 카메라 이미지 |
| `/vision/image_rect` | sensor_msgs/Image | 보정된 이미지 |
| `/vision/obstacles` | geometry_msgs/PoseArray | 감지된 장애물 |
| `/vision/fused_objects` | geometry_msgs/PoseArray | 융합된 객체 정보 |
| `/odom` | nav_msgs/Odometry | 차량 위치 및 속도 |
| `/planning/path` | nav_msgs/Path | 전역 경로 |
| `/planning/trajectory` | nav_msgs/Path | 지역 궤적 |
| `/ackermann_cmd` | ackermann_msgs/AckermannDriveStamped | Ackermann 제어 명령 |
| `/commands/motor/speed` | std_msgs/Float64 | 모터 속도 (ERPM) |
| `/commands/servo/position` | std_msgs/Float64 | 서보 위치 |

## 🔧 설정 및 튜닝

### 카메라 캘리브레이션

```bash
# 카메라 캘리브레이션 도구 실행
rosrun camera_calibration cameracalibrator.py \
  --size 8x6 \
  --square 0.025 \
  image:=/usb_cam/image_raw
```

### 제어기 파라미터 튜닝

제어 파라미터는 `wego_control/config/control_params.yaml`에서 수정:

```yaml
stanley:
  k: 1.0              # Cross-track error gain
  k_soft: 2.5         # Softening constant
```

### 경로 계획 설정

경로 계획 파라미터는 `wego_planning/config/planner_params.yaml`에서 수정:

```yaml
path_planner:
  algorithm: astar    # 'astar', 'rrt', 'dijkstra'
  frequency: 1.0      # Hz
```

## 🐛 트러블슈팅

### 카메라가 인식되지 않을 때

```bash
# 카메라 디바이스 확인
ls /dev/video*

# 권한 확인
sudo chmod 666 /dev/video0

# 카메라 없이 실행
roslaunch wego_bringup autonomous_stack.launch use_camera:=false
```

### 빌드 오류

```bash
# 워크스페이스 클린
cd ~/wego_ws
catkin_make clean
rm -rf build/ devel/

# 의존성 재설치
rosdep update
rosdep install --from-paths src --ignore-src -r -y

# 재빌드
catkin_make
source devel/setup.bash
```

### Python import 오류

```bash
# setup.py 확인 및 재생성
cd ~/wego_ws
catkin_make
source devel/setup.bash

# Python 경로 확인
echo $PYTHONPATH
```

## 🤝 기여 가이드

1. Fork the repository
2. Create your feature branch (`git checkout -b feature/AmazingFeature`)
3. Commit your changes (`git commit -m 'Add some AmazingFeature'`)
4. Push to the branch (`git push origin feature/AmazingFeature`)
5. Open a Pull Request

## 📝 개발 로드맵

### v0.1.0 (현재)
- [x] 기본 Vision 파이프라인
- [x] 경로 계획 알고리즘 (A*)
- [x] Stanley Controller 구현
- [x] VESC 인터페이스

### v0.2.0 (계획)
- [ ] YOLOv8 객체 감지 통합
- [ ] RRT* 경로 계획
- [ ] MPC 제어기 추가
- [ ] 동적 장애물 회피

### v0.3.0 (계획)
- [ ] LiDAR 통합
- [ ] SLAM 기능
- [ ] 주차 기능
- [ ] 웹 대시보드

## 📄 라이센스

이 프로젝트는 MIT 라이센스 하에 배포됩니다. 자세한 내용은 [LICENSE](LICENSE) 파일을 참조하세요.

## 👥 개발팀

**WeGO Team**
- Email: wego@example.com
- GitHub: [github.com/wego](https://github.com/wego)

## 🙏 감사의 말

이 프로젝트는 다음 오픈소스 프로젝트들의 도움을 받았습니다:
- [ROS (Robot Operating System)](https://www.ros.org/)
- [OpenCV](https://opencv.org/)
- [MIT RACECAR](https://mit-racecar.github.io/)

---

<div align="center">

**Made with ❤️ by WeGO Team**

[문서](src/) · [이슈 신고](https://github.com/wego/issues) · [기여하기](#-기여-가이드)

</div>

