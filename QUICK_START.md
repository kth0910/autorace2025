# WeGO 빠른 시작 가이드

## 📋 사전 준비 (Ubuntu 20.04)

### ROS Noetic 설치 확인
```bash
# ROS 버전 확인
rosversion -d
# 출력: noetic

# ROS 환경 소싱
source /opt/ros/noetic/setup.bash
```

### 의존성 설치
```bash
# ROS 패키지 의존성 설치
sudo apt update
sudo apt install -y \
  ros-noetic-cv-bridge \
  ros-noetic-image-transport \
  ros-noetic-ackermann-msgs \
  ros-noetic-usb-cam \
  python3-opencv \
  python3-numpy \
  python3-scipy

# rosdep으로 자동 의존성 설치
cd ~/autorace2025/autorace2025
rosdep install --from-paths src --ignore-src -r -y
```

---

## ⚡ 5분 만에 시작하기

### 1️⃣ 워크스페이스 빌드

```bash
# 워크스페이스로 이동
cd ~/autorace2025/autorace2025

# ROS 환경 소싱 (중요!)
source /opt/ros/noetic/setup.bash

# 빌드 실행
catkin_make

# 성공 메시지 확인:
# [100%] Built target ...

# 워크스페이스 소싱
source devel/setup.bash
```

### 2️⃣ 실행 권한 부여

```bash
# 모든 Python 스크립트에 실행 권한 부여
find src -name "*.py" -type f -exec chmod +x {} \;

# 패키지 인식 확인
rospack list | grep wego
# 출력 예시:
# wego_bringup /home/.../autorace2025/autorace2025/src/wego_bringup
# wego_control /home/.../autorace2025/autorace2025/src/wego_control
# wego_planning /home/.../autorace2025/autorace2025/src/wego_planning
# wego_vision /home/.../autorace2025/autorace2025/src/wego_vision
```

### 3️⃣ 실행

> **⚠️ 주의**: 새 터미널을 열 때마다 아래 명령어 실행 필수
> ```bash
> source ~/autorace2025/autorace2025/devel/setup.bash
> ```

#### 방법 1: 시뮬레이션 모드 (하드웨어 없이) ⭐ 권장

```bash
# 터미널 1: roscore
roscore

# 터미널 2: 자율주행 스택 실행
source ~/autorace2025/autorace2025/devel/setup.bash
roslaunch wego_bringup autonomous_stack.launch simulation:=true rviz:=true
```

#### 방법 2: 실제 차량 (카메라 연결 필요)

```bash
# 카메라 연결 확인
ls /dev/video*

# 권한 설정
sudo chmod 666 /dev/video0

# 실행
roslaunch wego_bringup autonomous_stack.launch rviz:=true
```

#### 방법 3: 개별 모듈 테스트 (디버깅용)

> **⚠️ 중요**: roscore를 먼저 실행해야 합니다!

```bash
# 터미널 1: roscore 실행 (필수!)
roscore

# 터미널 2: 개별 모듈 테스트
source ~/autorace2025/autorace2025/devel/setup.bash

# Vision만 테스트
roslaunch wego_bringup minimal.launch node_group:=vision

# Planning만 테스트
roslaunch wego_bringup minimal.launch node_group:=planning

# Control만 테스트
roslaunch wego_bringup minimal.launch node_group:=control
```

#### 방법 4: 빠른 테스트 (개별 노드)

```bash
# 터미널 1: roscore
roscore

# 터미널 2: 카메라 노드만 실행
rosrun wego_vision camera_node.py

# 정상 실행 시 출력:
# [INFO] [...]: [Camera Node] 초기화 완료
```

---

## 🔍 실행 확인

### 활성 노드 확인
```bash
rosnode list
```

**예상 출력:**
```
/vision/camera_node
/vision/detection_node
/vision/fusion_node
/planning/path_planner_node
/planning/local_planner_node
/control/controller_node
/control/vesc_bridge_node
```

### 토픽 확인
```bash
rostopic list
```

**주요 토픽:**
- `/vision/image_rect` - 보정된 이미지
- `/vision/fused_objects` - 감지된 객체
- `/planning/path` - 계획된 경로
- `/planning/trajectory` - 생성된 궤적
- `/ackermann_cmd` - 제어 명령

### 토픽 데이터 확인
```bash
# 이미지 확인
rostopic echo /vision/image_rect

# 경로 확인
rostopic echo /planning/path

# 제어 명령 확인
rostopic echo /ackermann_cmd
```

---

## 🛠️ 문제 해결

### 1. CMake/빌드 에러

#### "Could not find a package configuration file"
```bash
# ROS 환경 소싱 확인
source /opt/ros/noetic/setup.bash

# 의존성 재설치
sudo apt install -y ros-noetic-cv-bridge ros-noetic-ackermann-msgs

# 클린 빌드
cd ~/autorace2025/autorace2025
rm -rf build/ devel/
catkin_make
```

#### "catkin_install_python: command not found"
```bash
# catkin-tools 설치
sudo apt install python3-catkin-tools

# 또는 표준 catkin_make 사용 (이미 사용 중)
catkin_make
```

### 2. 런타임 에러

#### "/home/wego/.ros/log/latest가 없다" 또는 "Unable to contact ROS master"
```bash
# roscore가 실행되지 않았을 때 발생
# 해결: 별도 터미널에서 roscore 실행

# 터미널 1: roscore 실행
roscore

# 터미널 2: 노드/launch 파일 실행
source ~/autorace2025/autorace2025/devel/setup.bash
roslaunch wego_bringup autonomous_stack.launch simulation:=true

# ROS 마스터 확인
echo $ROS_MASTER_URI
# 정상 출력: http://localhost:11311
```

#### "RLException: [autonomous_stack.launch] is neither a launch file"
```bash
# 워크스페이스 소싱 확인
source ~/autorace2025/autorace2025/devel/setup.bash

# 패키지 경로 확인
rospack find wego_bringup

# bashrc에 영구 추가
echo "source ~/autorace2025/autorace2025/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

#### Python import 에러 (ImportError: No module named ...)
```bash
# cv_bridge 설치 확인
sudo apt install ros-noetic-cv-bridge python3-opencv

# Python 경로 확인
python3 -c "import cv2; print(cv2.__version__)"
```

### 3. 카메라 문제

#### 카메라 인식 실패
```bash
# 카메라 디바이스 확인
ls /dev/video*

# 카메라 권한 설정
sudo chmod 666 /dev/video0

# 사용자를 video 그룹에 추가
sudo usermod -a -G video $USER
# 로그아웃 후 재로그인

# 카메라 없이 시뮬레이션 모드 사용
roslaunch wego_bringup autonomous_stack.launch simulation:=true rviz:=true
```

#### USB 카메라 패키지 오류
```bash
# usb_cam 패키지 설치
sudo apt install ros-noetic-usb-cam

# 카메라 테스트
rosrun usb_cam usb_cam_node
```

### 4. RViz 문제

#### RViz가 실행되지 않음
```bash
# RViz 설치 확인
sudo apt install ros-noetic-rviz

# RViz 없이 실행
roslaunch wego_bringup autonomous_stack.launch simulation:=true rviz:=false
```

### 5. 노드 실행 권한 오류

#### "Permission denied" 에러
```bash
cd ~/autorace2025/autorace2025
find src -name "*.py" -type f -exec chmod +x {} \;
```

---

## ✅ 완전 체크리스트 (처음 설정 시)

### 단계별 실행
```bash
# 1. ROS 환경 소싱
source /opt/ros/noetic/setup.bash

# 2. 의존성 설치
cd ~/autorace2025/autorace2025
sudo apt install -y ros-noetic-cv-bridge ros-noetic-image-transport \
  ros-noetic-ackermann-msgs ros-noetic-usb-cam python3-opencv

# 3. rosdep 의존성 자동 설치
rosdep install --from-paths src --ignore-src -r -y

# 4. 빌드
catkin_make

# 5. 워크스페이스 소싱
source devel/setup.bash

# 6. 실행 권한 부여
find src -name "*.py" -type f -exec chmod +x {} \;

# 7. 패키지 확인
rospack list | grep wego

# 8. 시뮬레이션 실행 (새 터미널에서)
# 터미널 1:
roscore

# 터미널 2:
source ~/autorace2025/autorace2025/devel/setup.bash
roslaunch wego_bringup autonomous_stack.launch simulation:=true rviz:=true

# 9. bashrc에 영구 추가 (선택사항)
echo "source ~/autorace2025/autorace2025/devel/setup.bash" >> ~/.bashrc
```

---

## 📚 다음 단계

1. **튜닝**: [각 패키지의 config/ 폴더](src/)에서 파라미터 조정
2. **개발**: 새로운 알고리즘 추가 및 커스터마이징
3. **문서**: [README.md](README.md) 및 각 패키지 문서 참조
4. **시각화**: RViz에서 `/planning/path`, `/planning/trajectory` 토픽 확인

---

## 🆘 도움말

- **전체 문서**: [README.md](README.md)
- **Vision 문서**: [wego_vision/README.md](src/wego_vision/README.md)
- **Planning 문서**: [wego_planning/README.md](src/wego_planning/README.md)
- **Control 문서**: [wego_control/README.md](src/wego_control/README.md)
- **Bringup 문서**: [wego_bringup/README.md](src/wego_bringup/README.md)

---

**즐거운 개발 되세요! 🚗💨**

