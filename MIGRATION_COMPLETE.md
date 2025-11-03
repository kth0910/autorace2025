# ✅ 패키지 병합 및 Odometry 업그레이드 완료

**작업 완료일**: 2025-11-03

---

## 🎉 완료된 작업

### 1. ✅ IMU + VESC 기반 실제 Odometry 구현

#### 새로 추가된 파일
- **`wego_bringup/scripts/real_odom_node.py`**
  - IMU 각속도 + VESC 속도 융합
  - Dead Reckoning 알고리즘
  - TF 발행 (odom → base_link)
  - 두 가지 모드: VESC 피드백 / VESC 명령값

- **`wego_bringup/launch/odometry.launch`**
  - 시뮬레이션 모드: `dummy_odom_node.py`
  - 실제 모드: `real_odom_node.py`
  - 파라미터 설정 지원

- **`ODOMETRY_SETUP.md`**
  - 하드웨어 설정 가이드
  - 캘리브레이션 방법
  - 트러블슈팅

#### 주요 기능
```python
# 입력
/imu/data              # IMU 각속도 (yaw rate)
/sensors/core          # VESC 피드백 (ERPM)
/commands/motor/speed  # VESC 명령 (대체)

# 출력
/odom                  # nav_msgs/Odometry
/odom_debug            # geometry_msgs/PoseStamped
```

#### 사용 방법
```bash
# 시뮬레이션
roslaunch wego_bringup odometry.launch simulation:=true

# 실제 센서 (VESC 명령값)
roslaunch wego_bringup odometry.launch simulation:=false

# 실제 센서 (VESC 피드백)
roslaunch wego_bringup odometry.launch simulation:=false use_vesc_feedback:=true
```

---

### 2. ✅ autorace_planning → wego_planning 병합

#### 병합된 파일

**Config 파일 (6개)**
```
wego_planning/config/move_base/
├── costmap_common.yaml
├── costmap_common_vision.yaml
├── costmap_common_vision_scan.yaml
├── global_costmap.yaml
├── local_costmap.yaml
└── dwa_local_planner.yaml
```

**Launch 파일 (3개)**
```
wego_planning/launch/move_base/
├── move_base_core.launch
├── move_base_vision.launch
└── planning_viz.launch
```

**Scripts (1개)**
```
wego_planning/scripts/
└── send_goal.py
```

#### 의존성 추가
```xml
<depend>move_base</depend>
<depend>move_base_msgs</depend>
<depend>sensor_msgs</depend>
<depend>tf2_geometry_msgs</depend>
<depend>cv_bridge</depend>
<depend>image_transport</depend>
<exec_depend>topic_tools</exec_depend>
```

#### 사용 방법
```bash
# 차선 기반 Planning (기존)
rosrun wego_planning path_planner_with_lane.py

# Move Base Navigation (신규)
roslaunch wego_planning move_base/move_base_vision.launch
rosrun wego_planning send_goal.py _x:=2.0 _y:=1.0
```

---

### 3. ✅ autorace_planning 패키지 삭제

**삭제됨**: `src/autorace_planning/`

**현재 패키지 구조**:
```
src/
├── wego_bringup/     (Odometry 통합)
├── wego_control/     
├── wego_planning/    (Move Base 통합)
└── wego_vision/      
```

---

## 📚 업데이트된 문서

### 신규 문서
1. **`ODOMETRY_SETUP.md`** - IMU + VESC Odometry 설정 가이드
2. **`PLANNING_MIGRATION.md`** - Planning 패키지 병합 가이드
3. **`MIGRATION_COMPLETE.md`** - 이 문서

### 수정된 문서
1. **`QUICK_RUN.txt`** - Odometry 및 Move Base 사용법 추가
2. **`wego_bringup/README.md`** - Real Odometry 설명 추가
3. **`wego_planning/README.md`** - 두 가지 Planning 모드 설명

---

## 🔄 마이그레이션 체크리스트

### Odometry
- [x] real_odom_node.py 구현
- [x] odometry.launch 생성
- [x] CMakeLists.txt 업데이트
- [x] autonomous_stack.launch에 통합
- [x] ODOMETRY_SETUP.md 작성
- [x] QUICK_RUN.txt 업데이트

### Planning
- [x] Config 파일 복사
- [x] Launch 파일 복사
- [x] Scripts 복사
- [x] package.xml 의존성 추가
- [x] CMakeLists.txt 업데이트
- [x] README.md 재작성
- [x] QUICK_RUN.txt 업데이트
- [x] autorace_planning 삭제
- [x] DEPRECATED.md 작성

---

## 🚀 사용자 작업 필요

### 1. 의존성 설치
```bash
# Move Base 관련 패키지
sudo apt update
sudo apt install -y \
  ros-noetic-move-base \
  ros-noetic-move-base-msgs \
  ros-noetic-dwa-local-planner \
  ros-noetic-topic-tools

# IMU 드라이버 (필요 시)
# sudo apt install ros-noetic-myahrs-driver

# VESC 드라이버 (필요 시)
# git clone https://github.com/mit-racecar/vesc.git
```

### 2. 빌드
```bash
cd ~/autorace2025/autorace2025
catkin_make
source devel/setup.bash
```

### 3. 권한 설정 (Ubuntu/Linux)
```bash
chmod +x src/wego_vision/scripts/*.py
chmod +x src/wego_planning/scripts/*.py
chmod +x src/wego_bringup/scripts/*.py
chmod +x src/wego_control/scripts/*.py
```

---

## 🧪 테스트 가이드

### 1. Odometry 테스트

#### 시뮬레이션
```bash
# Terminal 1
roscore

# Terminal 2
source devel/setup.bash
roslaunch wego_bringup odometry.launch simulation:=true

# Terminal 3
rostopic hz /odom        # 50 Hz 확인
rostopic echo /odom      # 데이터 확인
rviz                     # Odometry 시각화
```

#### 실제 센서
```bash
# Terminal 1: roscore
roscore

# Terminal 2: IMU
roslaunch myahrs_driver myahrs_driver.launch

# Terminal 3: Real Odometry
source devel/setup.bash
roslaunch wego_bringup odometry.launch simulation:=false

# Terminal 4: 확인
rostopic hz /imu/data
rostopic hz /odom
rostopic echo /odom
```

### 2. Planning 테스트

#### 차선 기반
```bash
# Terminal 1: roscore
roscore

# Terminal 2: Vision + Odometry
source devel/setup.bash
roslaunch wego_bringup autonomous_stack.launch simulation:=true

# Terminal 3: Planning
rosrun wego_planning path_planner_with_lane.py

# Terminal 4: 확인
rostopic echo /planning/path
```

#### Move Base
```bash
# Terminal 1: roscore + Odometry
roscore
roslaunch wego_bringup odometry.launch simulation:=true

# Terminal 2: Move Base
source devel/setup.bash
roslaunch wego_planning move_base/move_base_vision.launch

# Terminal 3: 목표 전송
rosrun wego_planning send_goal.py _x:=2.0 _y:=1.0

# Terminal 4: 확인
rostopic echo /cmd_vel
rostopic echo /move_base/status
```

---

## 📊 Before & After

### Before
```
src/
├── autorace_planning/      ← Move Base만
├── wego_bringup/           ← Dummy Odometry만
├── wego_control/
├── wego_planning/          ← 차선 기반만
└── wego_vision/
```

### After
```
src/
├── wego_bringup/           ← Dummy + Real Odometry
├── wego_control/
├── wego_planning/          ← 차선 기반 + Move Base 통합
└── wego_vision/
```

---

## 🎯 기능 비교

| 기능 | Before | After |
|------|--------|-------|
| Dummy Odometry | ✅ | ✅ |
| Real Odometry (IMU+VESC) | ❌ | ✅ |
| 차선 기반 Planning | ✅ | ✅ |
| Move Base Navigation | ✅ (별도 패키지) | ✅ (통합) |
| Vision 기반 Costmap | ✅ | ✅ |
| 패키지 수 | 5개 | 4개 |

---

## 📖 주요 문서 요약

### 실행 가이드
- **`QUICK_RUN.txt`** - 빠른 실행 명령어 모음 ⭐

### 상세 가이드
- **`ODOMETRY_SETUP.md`** - Odometry 하드웨어 설정 및 캘리브레이션
- **`wego_bringup/README.md`** - Bringup 패키지 전체 문서
- **`wego_planning/README.md`** - Planning 패키지 전체 문서

### 마이그레이션
- **`PLANNING_MIGRATION.md`** - Planning 패키지 병합 상세 가이드
- **`MIGRATION_COMPLETE.md`** - 이 문서 (작업 완료 요약)

---

## ⚠️ 주의사항

### 1. autorace_planning 패키지
**완전히 삭제되었습니다.** 모든 기능이 `wego_planning`으로 이전되었습니다.

### 2. Odometry 모드
- **시뮬레이션**: 센서 없이 알고리즘 테스트
- **실제**: IMU + VESC 필수

### 3. Planning 모드
- **차선 기반**: Vision만으로 간단한 주행
- **Move Base**: 복잡한 환경에서 내비게이션

---

## 🐛 문제 해결

### Odometry 관련
```bash
# IMU 데이터 안 들어옴
rostopic hz /imu/data
roslaunch myahrs_driver myahrs_driver.launch

# Odometry가 이상함
rosparam get /real_odom_node/erpm_to_speed_gain
# 값 조정 후 재시작
```

### Move Base 관련
```bash
# 목표에 도달 못함
rosparam set /move_base/DWAPlannerROS/xy_goal_tolerance 0.5

# 제자리 맴돔
rosparam set /move_base/DWAPlannerROS/path_distance_bias 64.0

# Costmap에 장애물 안 보임
rostopic echo /vision/obstacles
rosrun tf tf_echo odom camera_link
```

---

## 🎓 학습 자료

### ROS Navigation
- [ROS Navigation Tuning Guide](http://wiki.ros.org/navigation/Tutorials/Navigation%20Tuning%20Guide)
- [DWA Local Planner](http://wiki.ros.org/dwa_local_planner)

### Odometry
- [ROS Odometry Tutorial](http://wiki.ros.org/navigation/Tutorials/RobotSetup/Odom)
- [robot_localization](http://docs.ros.org/en/noetic/api/robot_localization/html/)

---

**작업 완료**: 2025-11-03  
**작업자**: WeGO 자율주행 팀  
**상태**: ✅ 완료

모든 기능이 정상적으로 통합되었으며, 테스트 준비가 완료되었습니다! 🚀

