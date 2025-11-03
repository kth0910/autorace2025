# 센서 설치 가이드

실제 로봇에서 사용할 센서들의 설치 방법입니다.

---

## 📦 필수 센서

### 1. 카메라
- USB 카메라 (기본 지원)
- `/dev/video0` 접근 권한 필요

```bash
sudo chmod 666 /dev/video0
```

---

## 🔧 선택 센서 (Odometry용)

### 2. IMU (관성 측정 장치)

#### 지원 모델
- myAHRS+ (권장)
- MPU6050
- BNO055
- 기타 ROS 호환 IMU

#### myAHRS+ 드라이버 설치

**방법 1: apt 설치 (간단)**
```bash
sudo apt update
sudo apt install ros-noetic-myahrs-driver
```

**방법 2: 소스 빌드**
```bash
cd ~/catkin_ws/src
git clone https://github.com/withrobot/myahrs_driver.git
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

#### 실행
```bash
roslaunch myahrs_driver myahrs_driver.launch
```

#### 확인
```bash
rostopic echo /imu/data
# angular_velocity.z 값이 변하는지 확인 (로봇 회전시)
```

---

### 3. VESC (모터 컨트롤러)

#### VESC 드라이버 설치

```bash
cd ~/catkin_ws/src
git clone https://github.com/mit-racecar/vesc.git
cd ~/catkin_ws
rosdep install --from-paths src --ignore-src -r -y
catkin_make
source devel/setup.bash
```

#### 설정

`vesc_driver/config/vesc.yaml` 파일 수정:
```yaml
vesc_driver_node:
  port: /dev/ttyACM1  # VESC USB 포트
  duty_cycle_min: -1.0
  duty_cycle_max: 1.0
  current_min: -20.0
  current_max: 20.0
  speed_min: -20000  # ERPM
  speed_max: 20000   # ERPM
```

#### 실행
```bash
roslaunch vesc_driver vesc_driver_node.launch
```

#### 확인
```bash
rostopic echo /sensors/core
# speed (ERPM) 값 확인
```

---

## 🔌 USB 포트 확인

### 장치 연결 확인
```bash
# USB 시리얼 장치 확인
ls -l /dev/ttyUSB* /dev/ttyACM*

# 예시 출력:
# /dev/ttyACM0 → IMU
# /dev/ttyACM1 → VESC
# /dev/video0  → 카메라
```

### 권한 설정
```bash
# 일시적 (재부팅 시 사라짐)
sudo chmod 666 /dev/ttyACM0
sudo chmod 666 /dev/ttyACM1

# 영구적 (udev 규칙 추가)
sudo usermod -a -G dialout $USER
# 로그아웃 후 재로그인 필요
```

---

## 📊 센서별 토픽

| 센서 | 토픽 | 메시지 타입 | 주기 |
|------|------|-------------|------|
| 카메라 | `/usb_cam/image_raw` | sensor_msgs/Image | ~30 Hz |
| IMU | `/imu/data` | sensor_msgs/Imu | ~100 Hz |
| VESC | `/sensors/core` | vesc_msgs/VescStateStamped | ~50 Hz |

---

## 🧪 센서 테스트

### 카메라 테스트
```bash
rosrun wego_vision simple_camera_publisher.py
rqt_image_view /usb_cam/image_raw
```

### IMU 테스트
```bash
roslaunch myahrs_driver myahrs_driver.launch
rostopic echo /imu/data

# 로봇을 회전시켜보고 angular_velocity.z 값 확인
```

### VESC 테스트
```bash
roslaunch vesc_driver vesc_driver_node.launch
rostopic echo /sensors/core

# 모터를 돌려보고 speed (ERPM) 값 확인
```

---

## 🔧 트러블슈팅

### IMU를 못 찾음
```bash
# USB 연결 확인
ls /dev/ttyACM* /dev/ttyUSB*

# 드라이버 재설치
sudo apt install ros-noetic-myahrs-driver

# launch 파일에서 포트 확인
# myahrs_driver/launch/myahrs_driver.launch
# <param name="port" value="/dev/ttyACM0"/>
```

### VESC를 못 찾음
```bash
# USB 연결 확인
ls /dev/ttyACM*

# 권한 확인
sudo chmod 666 /dev/ttyACM1

# VESC Tool로 펌웨어 확인
```

### 센서가 없을 때
**시뮬레이션 모드를 사용하세요!**

```bash
# Odometry 시뮬레이션
roslaunch wego_bringup odometry.launch simulation:=true

# 전체 스택 시뮬레이션
roslaunch wego_bringup autonomous_stack.launch simulation:=true
```

---

## 📚 관련 문서

- **QUICK_START_SIMPLE.md** - 센서 없이 빠른 시작
- **ODOMETRY_SETUP.md** - Odometry 상세 설정
- **QUICK_RUN.txt** - 전체 실행 가이드

---

## 🛒 센서 구매 링크

### 추천 센서
- **IMU**: myAHRS+ (WithRobot)
- **모터 컨트롤러**: VESC 4.12 또는 6.x
- **카메라**: Logitech C270, C920

### 대체 센서
- **IMU**: MPU6050, BNO055 (저가형)
- **모터 컨트롤러**: Arduino + L298N (학습용)

---

**센서 없이도 시뮬레이션으로 모든 알고리즘을 테스트할 수 있습니다!**

