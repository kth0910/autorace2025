# WeGO 빌드 가이드

## 📋 사전 요구사항

### 시스템 요구사항
- **OS**: Ubuntu 20.04 LTS
- **ROS**: Noetic Ninjemys
- **Python**: 3.8 이상
- **Disk Space**: 최소 5GB

### ROS Noetic 설치 확인

```bash
# ROS 버전 확인
rosversion -d

# 예상 출력: noetic
```

ROS가 설치되어 있지 않다면:

```bash
# ROS Noetic 설치
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
sudo apt update
sudo apt install -y ros-noetic-desktop-full

# 환경 설정
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
source ~/.bashrc

# rosdep 초기화
sudo rosdep init
rosdep update
```

## 🔧 의존성 설치

### 1. ROS 패키지 의존성

```bash
cd ~/autorace2025/wego_ws

# rosdep을 통한 자동 의존성 설치
rosdep install --from-paths src --ignore-src -r -y
```

필요한 주요 ROS 패키지:
- `ros-noetic-cv-bridge`
- `ros-noetic-image-transport`
- `ros-noetic-ackermann-msgs`
- `ros-noetic-tf`
- `ros-noetic-tf2-ros`
- `ros-noetic-usb-cam`

### 2. Python 패키지 의존성

```bash
# pip 업그레이드
python3 -m pip install --upgrade pip

# 프로젝트 의존성 설치
pip3 install -r requirements.txt
```

또는 개별 설치:

```bash
pip3 install opencv-python>=4.5.0
pip3 install numpy>=1.19.0
pip3 install scipy>=1.5.0
pip3 install pyyaml>=5.3.0
pip3 install matplotlib>=3.3.0
```

## 🏗️ 빌드

### Step 1: 워크스페이스 초기화

```bash
cd ~/autorace2025/wego_ws

# 첫 빌드 전 확인
ls src/
# 출력: wego_vision  wego_planning  wego_control  wego_bringup
```

### Step 2: Catkin Build

```bash
# catkin_make 실행
catkin_make

# 성공 메시지 확인
# [100%] Built target <target_name>
```

**문제 발생 시:**

```bash
# 클린 빌드
catkin_make clean
rm -rf build/ devel/
catkin_make
```

### Step 3: 환경 소싱

```bash
# 빌드 결과물 소싱
source devel/setup.bash

# 확인
echo $ROS_PACKAGE_PATH
# 출력에 ~/autorace2025/wego_ws/src가 포함되어야 함
```

**영구 적용:**

```bash
# bashrc에 추가
echo "source ~/autorace2025/wego_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### Step 4: 실행 권한 설정

```bash
# 모든 Python 스크립트에 실행 권한 부여
find src -name "*.py" -type f -exec chmod +x {} \;
```

## ✅ 빌드 확인

### 패키지 확인

```bash
# WeGO 패키지들이 인식되는지 확인
rospack list | grep wego
```

**예상 출력:**
```
wego_bringup /home/<user>/autorace2025/wego_ws/src/wego_bringup
wego_control /home/<user>/autorace2025/wego_ws/src/wego_control
wego_planning /home/<user>/autorace2025/wego_ws/src/wego_planning
wego_vision /home/<user>/autorace2025/wego_ws/src/wego_vision
```

### 런치 파일 확인

```bash
# 런치 파일 경로 확인
rospack find wego_bringup
roscd wego_bringup/launch
ls
```

**예상 출력:**
```
autonomous_stack.launch  minimal.launch
```

## 🧪 빌드 테스트

### 최소 테스트

```bash
# roscore 실행 (별도 터미널)
roscore

# 개별 노드 테스트 (새 터미널)
rosrun wego_vision camera_node.py
```

정상 실행 시:
```
[INFO] [<timestamp>]: [Camera Node] 초기화 완료
```

### 전체 스택 테스트

```bash
# 시뮬레이션 모드로 전체 스택 실행
roslaunch wego_bringup autonomous_stack.launch simulation:=true
```

정상 실행 시 여러 노드의 초기화 메시지 출력:
```
[INFO] [...]: [Camera Node] 초기화 완료
[INFO] [...]: [Detection Node] 초기화 완료
[INFO] [...]: [Fusion Node] 초기화 완료
[INFO] [...]: [Path Planner] 초기화 완료
...
```

## 🐛 빌드 문제 해결

### 문제 1: CMake 버전 오류

```
CMake Error: CMake 3.0.2 or higher is required
```

**해결:**
```bash
sudo apt install cmake
cmake --version  # 3.0.2 이상인지 확인
```

### 문제 2: Python import 오류

```
ImportError: No module named 'cv_bridge'
```

**해결:**
```bash
# cv_bridge 재설치
sudo apt install ros-noetic-cv-bridge

# Python 경로 확인
python3 -c "import sys; print(sys.path)"
```

### 문제 3: catkin_pkg 오류

```
ModuleNotFoundError: No module named 'catkin_pkg'
```

**해결:**
```bash
pip3 install catkin_pkg
```

### 문제 4: ackermann_msgs 없음

```
Could not find a package configuration file provided by "ackermann_msgs"
```

**해결:**
```bash
sudo apt install ros-noetic-ackermann-msgs
```

### 문제 5: 권한 오류

```
Permission denied: '/dev/video0'
```

**해결:**
```bash
# 카메라 권한 부여
sudo chmod 666 /dev/video0

# 또는 사용자를 video 그룹에 추가
sudo usermod -a -G video $USER
# 로그아웃 후 재로그인
```

## 📊 빌드 출력 구조

성공적인 빌드 후:

```
wego_ws/
├── build/              # 빌드 아티팩트
├── devel/              # 개발 환경 설정
│   ├── setup.bash
│   ├── setup.sh
│   └── lib/
│       ├── wego_vision/
│       ├── wego_planning/
│       ├── wego_control/
│       └── wego_bringup/
└── src/                # 소스 코드
```

## 🔄 재빌드

코드 변경 후:

```bash
cd ~/autorace2025/wego_ws

# 빠른 재빌드
catkin_make

# 클린 재빌드 (문제 발생 시)
catkin_make clean
catkin_make
```

특정 패키지만 빌드:

```bash
catkin_make --only-pkg-with-deps wego_vision
```

## 📝 다음 단계

빌드 완료 후:

1. **테스트 실행**: [QUICK_START.md](QUICK_START.md) 참조
2. **설정 조정**: 각 패키지의 `config/` 폴더 확인
3. **개발 시작**: 새로운 기능 추가 및 커스터마이징

---

**빌드 성공을 축하합니다! 🎉**

