# Odometry 설정 가이드

IMU와 VESC를 사용한 실제 odometry 시스템 설정 가이드입니다.

## 📋 목차

1. [하드웨어 요구사항](#하드웨어-요구사항)
2. [소프트웨어 설치](#소프트웨어-설치)
3. [센서 연결 확인](#센서-연결-확인)
4. [파라미터 캘리브레이션](#파라미터-캘리브레이션)
5. [실행 방법](#실행-방법)
6. [검증 및 튜닝](#검증-및-튜닝)

---

## 하드웨어 요구사항

### 필수 센서
- **IMU (Inertial Measurement Unit)**
  - 3축 각속도 센서 (Gyroscope)
  - ROS 드라이버: `myahrs_driver` 또는 호환 드라이버
  - 토픽: `/imu/data` (sensor_msgs/Imu)
  
- **VESC (Vehicle Electronic Speed Controller)**
  - 모터 속도 제어 및 피드백
  - ROS 드라이버: `vesc_driver` (선택사항)
  - 토픽: `/sensors/core` (피드백) 또는 `/commands/motor/speed` (명령)

### 연결 방식
```
┌─────────────┐
│   로봇 PC   │
└─────┬───────┘
      │
      ├───USB──► IMU (/dev/ttyUSB0 또는 /dev/ttyACM0)
      │
      └───USB──► VESC (/dev/ttyACM1)
```

---

## 소프트웨어 설치

### 1. IMU 드라이버 설치 (예: myAHRS+)

```bash
cd ~/catkin_ws/src
git clone https://github.com/withrobot/myahrs_driver.git
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

### 2. VESC 드라이버 설치 (선택사항)

```bash
cd ~/catkin_ws/src
git clone https://github.com/mit-racecar/vesc.git
cd ~/catkin_ws
rosdep install --from-paths src --ignore-src -r -y
catkin_make
source devel/setup.bash
```

### 3. 패키지 빌드

```bash
cd ~/autorace2025/autorace2025
catkin_make
source devel/setup.bash
```

---

## 센서 연결 확인

### 1. IMU 연결 확인

```bash
# USB 장치 확인
ls -l /dev/ttyUSB* /dev/ttyACM*

# IMU 드라이버 실행
roslaunch myahrs_driver myahrs_driver.launch

# 다른 터미널에서 확인
rostopic list | grep imu
rostopic echo /imu/data

# 예상 출력:
# header:
#   stamp: ...
# orientation: ...
# angular_velocity:
#   x: ...
#   y: ...
#   z: 0.05  # ← 이 값이 중요 (yaw rate)
```

**확인 포인트:**
- `/imu/data` 토픽이 발행되는가?
- `angular_velocity.z` 값이 변하는가? (로봇을 회전시켜보기)
- 주기가 충분한가? (최소 50Hz 권장)

### 2. VESC 연결 확인

#### 방법 A: VESC 피드백 사용 (권장)

```bash
# VESC 드라이버 실행
roslaunch vesc_driver vesc_driver_node.launch

# 확인
rostopic list | grep sensors
rostopic echo /sensors/core

# 예상 출력:
# state:
#   voltage_input: 12.5
#   temperature_pcb: 25.0
#   current_motor: 1.2
#   current_input: 0.5
#   speed: 1500.0  # ← ERPM 값
#   duty_cycle: 0.2
#   charge_drawn: 0.0
#   charge_regen: 0.0
#   energy_drawn: 0.0
#   energy_regen: 0.0
#   displacement: 0
#   distance_traveled: 0
```

**확인 포인트:**
- `/sensors/core` 토픽이 발행되는가?
- `speed` (ERPM) 값이 모터 구동 시 변하는가?
- 주기가 충분한가? (최소 20Hz 권장)

#### 방법 B: 모터 명령값 사용 (피드백 없음)

```bash
# Control 노드를 실행하면 /commands/motor/speed 발행됨
rostopic echo /commands/motor/speed

# 또는 수동으로 테스트
rostopic pub /commands/motor/speed std_msgs/Float64 "data: 1000.0"
```

---

## 파라미터 캘리브레이션

### 1. ERPM → m/s 변환 계수 측정

VESC의 ERPM을 실제 속도(m/s)로 변환하는 계수를 측정합니다.

```bash
# 테스트 준비
# 1. 로봇을 일정 거리(예: 1m)에 마크 표시
# 2. VESC에 일정 ERPM 명령 전송
# 3. 1m 이동하는 시간 측정

# 예시 계산:
# - ERPM: 4000
# - 거리: 1.0 m
# - 시간: 1.0 s
# → 속도: 1.0 m/s
# → 계수: 1.0 / 4000 = 0.00025

# launch 파일에서 설정
# <param name="erpm_to_speed_gain" value="0.00025"/>
```

**정확한 측정 방법:**

```python
# 간단한 캘리브레이션 스크립트
#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry

distance = 0.0
start_time = None

def odom_callback(msg):
    global distance, start_time
    if start_time is None:
        start_time = rospy.Time.now()
    
    # 거리 누적 (간단히 x 변화량만)
    # 실제로는 이전 위치와 비교 필요
    
    elapsed = (rospy.Time.now() - start_time).to_sec()
    if elapsed > 5.0:  # 5초 후 결과 출력
        print(f"거리: {distance:.2f} m")
        print(f"시간: {elapsed:.2f} s")
        print(f"속도: {distance/elapsed:.3f} m/s")
        rospy.signal_shutdown("완료")

rospy.init_node('calibration')
pub = rospy.Publisher('/commands/motor/speed', Float64, queue_size=10)
sub = rospy.Subscriber('/odom', Odometry, odom_callback)

rospy.sleep(1.0)
pub.publish(Float64(4000.0))  # 4000 ERPM
rospy.spin()
```

### 2. IMU 각속도 검증

```bash
# IMU를 360도 회전시키고 각도 변화 확인
rostopic echo /odom | grep -A 10 orientation

# 또는 간단한 테스트
# 로봇을 제자리에서 한 바퀴 돌리고
# /odom의 yaw 각도가 2π (6.28) 정도 변해야 함
```

### 3. 파라미터 조정

`src/wego_bringup/launch/odometry.launch` 파일 수정:

```xml
<node name="real_odom_node" pkg="wego_bringup" type="real_odom_node.py" output="screen">
  <!-- ... -->
  
  <!-- ERPM 변환 계수 조정 -->
  <param name="erpm_to_speed_gain" value="0.00025"/>  <!-- ← 측정값으로 수정 -->
  
  <!-- 차량 파라미터 -->
  <param name="wheelbase" value="0.32"/>  <!-- ← 실제 측정값 -->
  <param name="wheel_radius" value="0.05"/>  <!-- ← 실제 측정값 -->
  
  <!-- ... -->
</node>
```

---

## 실행 방법

### 모드 1: 시뮬레이션 (센서 없이 테스트)

```bash
# Terminal 1
roscore

# Terminal 2
source ~/autorace2025/autorace2025/devel/setup.bash
roslaunch wego_bringup odometry.launch simulation:=true

# Terminal 3 (확인)
rostopic hz /odom
rostopic echo /odom
rviz  # Add -> Odometry -> /odom
```

### 모드 2: 실제 센서 (VESC 명령값 사용)

```bash
# Terminal 1
roscore

# Terminal 2: IMU 드라이버
roslaunch myahrs_driver myahrs_driver.launch

# Terminal 3: Real Odometry
source ~/autorace2025/autorace2025/devel/setup.bash
roslaunch wego_bringup odometry.launch simulation:=false

# Terminal 4: Control (모터 명령 발생)
roslaunch wego_control control.launch

# Terminal 5 (확인)
rostopic hz /imu/data  # IMU
rostopic hz /commands/motor/speed  # VESC 명령
rostopic hz /odom  # Odometry
```

### 모드 3: 실제 센서 (VESC 피드백 사용)

```bash
# Terminal 1
roscore

# Terminal 2: IMU 드라이버
roslaunch myahrs_driver myahrs_driver.launch

# Terminal 3: VESC 드라이버
roslaunch vesc_driver vesc_driver_node.launch

# Terminal 4: Real Odometry (피드백 모드)
source ~/autorace2025/autorace2025/devel/setup.bash
roslaunch wego_bringup odometry.launch \
  simulation:=false \
  use_vesc_feedback:=true

# Terminal 5 (확인)
rostopic hz /imu/data  # IMU
rostopic hz /sensors/core  # VESC 피드백
rostopic hz /odom  # Odometry
```

### 전체 스택 실행

```bash
# Terminal 1
roscore

# Terminal 2: IMU + VESC 드라이버
roslaunch myahrs_driver myahrs_driver.launch &
roslaunch vesc_driver vesc_driver_node.launch &

# Terminal 3: 전체 자율주행 스택
source ~/autorace2025/autorace2025/devel/setup.bash
roslaunch wego_bringup autonomous_stack.launch \
  simulation:=false \
  use_vesc_feedback:=true \
  rviz:=true
```

---

## 검증 및 튜닝

### 1. 직선 주행 테스트

```bash
# 로봇을 1m 직선으로 이동
# /odom의 x 또는 y 변화가 실제 거리와 일치하는지 확인

rostopic echo /odom | grep -A 3 "position:"

# 예상:
# position:
#   x: 1.05  # ← 실제 1m와 비교
#   y: 0.02  # ← 직진이면 거의 0
#   z: 0.0
```

**문제:**
- x가 너무 크다 → `erpm_to_speed_gain` 감소
- x가 너무 작다 → `erpm_to_speed_gain` 증가

### 2. 회전 테스트

```bash
# 로봇을 제자리에서 360도 회전
# /odom의 orientation (yaw)이 2π 변하는지 확인

rostopic echo /odom | grep -A 4 "orientation:"

# 또는 RPY로 변환하여 확인
rosrun tf tf_echo odom base_link
```

**문제:**
- 회전각이 너무 크다 → IMU 캘리브레이션 필요
- 회전각이 너무 작다 → IMU 캘리브레이션 필요

### 3. Bag 파일로 데이터 분석

```bash
# 데이터 기록
rosbag record /odom /imu/data /sensors/core /commands/motor/speed -O test_odom.bag

# 데이터 재생 및 분석
rosbag play test_odom.bag
rqt_plot /odom/pose/pose/position/x /odom/pose/pose/position/y

# Python으로 분석
rostopic echo -b test_odom.bag -p /odom > odom_data.csv
```

### 4. 파라미터 실시간 조정

```bash
# 현재 값 확인
rosparam get /real_odom_node/erpm_to_speed_gain

# 실시간 변경
rosparam set /real_odom_node/erpm_to_speed_gain 0.0003

# 노드 재시작
rosnode kill /real_odom_node
rosrun wego_bringup real_odom_node.py
```

### 5. Covariance 튜닝

정확도에 따라 covariance 값을 조정합니다.

`real_odom_node.py` 수정:

```python
# Pose covariance (x, y의 불확실성)
odom.pose.covariance[0] = 0.01  # x의 분산 (작을수록 신뢰도 높음)
odom.pose.covariance[7] = 0.01  # y의 분산
odom.pose.covariance[35] = 0.01  # yaw의 분산

# Twist covariance (vx, vyaw의 불확실성)
odom.twist.covariance[0] = 0.01  # vx의 분산
odom.twist.covariance[35] = 0.01  # vyaw의 분산
```

---

## 고급 옵션

### robot_localization 패키지 사용

더 정확한 odometry를 위해 EKF 필터 사용:

```bash
sudo apt install ros-noetic-robot-localization

# ekf_config.yaml 작성
# ekf_localization_node 실행
```

### GPS 융합

옥외 환경에서 GPS를 추가로 융합:

```bash
# GPS 드라이버 실행
roslaunch ublox_gps ublox_gps.launch

# navsat_transform_node로 GPS를 odometry 좌표계로 변환
```

---

## 문제 해결

### 문제: Odometry가 계속 증가만 함

**원인:** 타임아웃으로 속도가 0이 안 됨

**해결:**
```bash
rosparam get /real_odom_node/vesc_timeout
rosparam set /real_odom_node/vesc_timeout 0.2  # 더 짧게
```

### 문제: Odometry가 튀는 현상

**원인:** IMU 노이즈 또는 VESC 속도 변화

**해결:**
1. 로우패스 필터 추가
2. Covariance 증가
3. `robot_localization` EKF 사용

### 문제: 로봇이 멈춰도 Odometry가 움직임

**원인:** IMU drift 또는 VESC 잔류 명령

**해결:**
```bash
# 속도 임계값 추가 (코드 수정 필요)
if abs(self.current_linear_velocity) < 0.01:
    self.current_linear_velocity = 0.0
```

---

## 참고 자료

- [ROS Odometry](http://wiki.ros.org/navigation/Tutorials/RobotSetup/Odom)
- [robot_localization](http://docs.ros.org/en/noetic/api/robot_localization/html/index.html)
- [VESC Project](https://vesc-project.com/)
- [myAHRS+](http://withrobot.com/en/sensor/myahrsplus/)

---

**작성:** WeGO 자율주행 팀  
**업데이트:** 2025-11-03

