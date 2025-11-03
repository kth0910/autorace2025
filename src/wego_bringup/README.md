# WeGO Bringup Package

ROS Noetic 기반 자율주행 스택의 통합 실행 패키지입니다.

## 📦 패키지 개요

`wego_bringup`는 Vision, Planning, Control 모듈을 통합하여 전체 자율주행 시스템을 실행합니다.

## 🚀 주요 기능

### 1. Odometry 시스템

#### 시뮬레이션 모드 (Dummy Odometry)
- **노드**: `dummy_odom_node.py`
- **기능**: 원형 궤적의 더미 odometry 데이터 발행
- **사용 시나리오**: 센서 없이 알고리즘 테스트

#### 실제 모드 (IMU + VESC Fusion)
- **노드**: `real_odom_node.py`
- **기능**: IMU 각속도와 VESC 속도를 융합한 실제 odometry 계산
- **센서**:
  - IMU: `/imu/data` (angular velocity → yaw 계산)
  - VESC: `/sensors/core` (실제 피드백) 또는 `/commands/motor/speed` (명령값)
- **알고리즘**: Dead Reckoning 기반 위치 추정

### 2. 통합 런치 파일

#### `autonomous_stack.launch`
전체 자율주행 스택을 실행합니다.

```bash
# 시뮬레이션 모드 (더미 센서)
roslaunch wego_bringup autonomous_stack.launch simulation:=true

# 실제 모드 (실제 센서, VESC 명령값 사용)
roslaunch wego_bringup autonomous_stack.launch simulation:=false

# 실제 모드 (실제 센서, VESC 피드백 사용)
roslaunch wego_bringup autonomous_stack.launch simulation:=false use_vesc_feedback:=true

# RViz 포함
roslaunch wego_bringup autonomous_stack.launch rviz:=true

# 카메라 없이
roslaunch wego_bringup autonomous_stack.launch use_camera:=false
```

#### `odometry.launch`
Odometry 노드만 개별 실행합니다.

```bash
# 시뮬레이션 모드
roslaunch wego_bringup odometry.launch simulation:=true

# 실제 모드 (VESC 명령값 사용)
roslaunch wego_bringup odometry.launch simulation:=false

# 실제 모드 (VESC 피드백 사용)
roslaunch wego_bringup odometry.launch simulation:=false use_vesc_feedback:=true
```

## 📊 토픽 구조

### Odometry 관련 토픽

#### 입력 (실제 모드)
- `/imu/data` (`sensor_msgs/Imu`): IMU 센서 데이터
- `/sensors/core` (`vesc_msgs/VescStateStamped`): VESC 상태 피드백
- `/commands/motor/speed` (`std_msgs/Float64`): 모터 속도 명령 (피드백 없을 시)

#### 출력
- `/odom` (`nav_msgs/Odometry`): Odometry 데이터
- `/odom_debug` (`geometry_msgs/PoseStamped`): 디버깅용 위치 정보

### TF 트리
```
map
 └─ odom
     └─ base_link
         ├─ camera_link
         ├─ imu_link
         └─ laser_link
```

## 🔧 설정 파라미터

### Real Odometry Node 파라미터

| 파라미터 | 기본값 | 설명 |
|---------|--------|------|
| `rate` | 50.0 | Odometry 발행 주기 (Hz) |
| `frame_id` | "odom" | Odometry 프레임 ID |
| `child_frame_id` | "base_link" | 로봇 프레임 ID |
| `wheelbase` | 0.32 | 휠베이스 (m) |
| `wheel_radius` | 0.05 | 바퀴 반지름 (m) |
| `erpm_to_speed_gain` | 0.00025 | ERPM → m/s 변환 계수 |
| `use_vesc_feedback` | false | VESC 피드백 사용 여부 |
| `use_imu_orientation` | true | IMU 방향 사용 여부 |
| `imu_timeout` | 0.5 | IMU 타임아웃 (s) |
| `vesc_timeout` | 0.5 | VESC 타임아웃 (s) |

### Dummy Odometry Node 파라미터

| 파라미터 | 기본값 | 설명 |
|---------|--------|------|
| `rate` | 50.0 | 발행 주기 (Hz) |
| `radius` | 5.0 | 궤적 반경 (m) |
| `angular_velocity` | 0.1 | 각속도 (rad/s) |

## 📝 실행 예시

### 1. 개발/테스트 환경 (시뮬레이션)

```bash
# Terminal 1: roscore
roscore

# Terminal 2: 전체 스택 (시뮬레이션)
source ~/autorace2025/autorace2025/devel/setup.bash
roslaunch wego_bringup autonomous_stack.launch simulation:=true rviz:=true

# 확인
rostopic hz /odom  # 50 Hz
rostopic echo /odom
```

### 2. 실제 로봇 환경

```bash
# Terminal 1: roscore
roscore

# Terminal 2: IMU 드라이버 실행
roslaunch myahrs_driver myahrs_driver.launch

# Terminal 3: VESC 드라이버 실행 (피드백 있음)
roslaunch vesc_driver vesc_driver_node.launch

# 또는 VESC 명령만 사용 (피드백 없음)
# (control 노드가 /commands/motor/speed 발행)

# Terminal 4: 전체 스택 실행
source ~/autorace2025/autorace2025/devel/setup.bash
roslaunch wego_bringup autonomous_stack.launch \
  simulation:=false \
  use_vesc_feedback:=true \
  rviz:=true

# 확인
rostopic hz /imu/data  # IMU 확인
rostopic hz /sensors/core  # VESC 피드백 확인
rostopic hz /odom  # 50 Hz
rostopic echo /odom
```

### 3. Odometry만 테스트

```bash
# 실제 odometry만 실행
roslaunch wego_bringup odometry.launch simulation:=false

# 시각화
rviz
# Add -> Odometry -> Topic: /odom
# Add -> TF
```

## 🐛 트러블슈팅

### IMU 데이터가 안 들어올 때

```bash
# IMU 토픽 확인
rostopic list | grep imu
rostopic hz /imu/data
rostopic echo /imu/data

# IMU 드라이버 재시작
rosnode kill /myahrs_driver
roslaunch myahrs_driver myahrs_driver.launch
```

### VESC 데이터가 안 들어올 때

```bash
# VESC 토픽 확인
rostopic list | grep -E "sensors|commands"
rostopic hz /sensors/core  # 피드백 모드
rostopic hz /commands/motor/speed  # 명령 모드

# use_vesc_feedback 설정 확인
rosparam get /real_odom_node/use_vesc_feedback
```

### Odometry가 이상하게 나올 때

```bash
# 파라미터 확인
rosparam list | grep real_odom

# ERPM 변환 계수 조정
rosparam set /real_odom_node/erpm_to_speed_gain 0.0003

# 노드 재시작
rosnode kill /real_odom_node
rosrun wego_bringup real_odom_node.py

# 디버그 출력 활성화
rosservice call /real_odom_node/set_logger_level "logger: 'rosout'
level: 'debug'"

# 디버그 토픽 확인
rostopic echo /odom_debug
```

### TF 오류

```bash
# TF 트리 확인
rosrun rqt_tf_tree rqt_tf_tree
rosrun tf view_frames
evince frames.pdf

# static_transform_publisher 확인
rosnode list | grep static
rosnode info /map_to_odom
```

## 📈 성능 모니터링

```bash
# Odometry 주기 확인
rostopic hz /odom

# Odometry 지연 확인
rostopic delay /odom

# 센서 주기 확인
rostopic hz /imu/data
rostopic hz /sensors/core

# 노드 상태 확인
rosnode info /real_odom_node
rosnode list
```

## 🔍 디버깅 팁

### 1. RViz에서 확인
```bash
rviz
# Fixed Frame: odom
# Add -> Odometry -> Topic: /odom
# Add -> TF
# Add -> Axes (base_link 위치 확인)
```

### 2. rqt_graph로 토픽 연결 확인
```bash
rqt_graph
```

### 3. Bag 파일로 데이터 기록
```bash
# 기록
rosbag record /odom /imu/data /sensors/core -O odom_test.bag

# 재생
rosbag play odom_test.bag
```

## 📚 관련 문서

- [QUICK_RUN.txt](../../QUICK_RUN.txt): 빠른 실행 가이드
- [wego_vision/README.md](../wego_vision/README.md): Vision 모듈
- [wego_planning/README.md](../wego_planning/README.md): Planning 모듈
- [wego_control/README.md](../wego_control/README.md): Control 모듈

## 📦 의존성

### ROS 패키지
- `rospy`
- `nav_msgs`
- `sensor_msgs`
- `geometry_msgs`
- `std_msgs`
- `tf`
- `ackermann_msgs`

### 선택적 패키지
- `vesc_msgs` (VESC 피드백 사용 시)
- `myahrs_driver` (IMU 사용 시)
- `rviz` (시각화)

### Python 패키지
- `numpy`

## 🎯 다음 단계

1. IMU 캘리브레이션
2. VESC 파라미터 튜닝
3. Odometry 정확도 검증
4. EKF/UKF 필터 추가 (robot_localization 패키지)
5. GPS 융합 (옥외 주행)

## 📄 라이센스

이 패키지는 WeGO 자율주행 프로젝트의 일부입니다.
