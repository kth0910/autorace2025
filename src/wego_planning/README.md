# WeGO Planning Package

## 📋 개요

`wego_planning`은 WeGO 자율주행 시스템의 **경로 계획** 모듈입니다.  
차선 기반 경로 계획과 Move Base 기반 내비게이션을 모두 지원합니다.

## 🎯 두 가지 Planning 모드

### 모드 1: 차선 기반 경로 계획 (Lane-based Planning)
Vision에서 감지한 차선 정보를 기반으로 경로를 생성합니다.

- **Path Planner**: 전역 경로 생성 (A*, RRT, Dijkstra)
- **Local Planner**: 지역 궤적 생성 (DWA, Pure Pursuit)
- **Lane Following**: 차선 정보 기반 경로 추종

### 모드 2: Move Base 내비게이션 (Move Base Navigation)
ROS Navigation Stack을 사용한 표준 내비게이션입니다.

- **Move Base**: ROS 표준 내비게이션 노드
- **DWA Local Planner**: 동적 창 접근법 지역 경로 계획
- **Costmap**: 장애물 회피 및 경로 최적화
- **Vision Integration**: 카메라 기반 장애물 감지 지원

---

## 🚀 사용 방법

### 모드 1: 차선 기반 경로 계획

#### 전체 계획 파이프라인 실행
```bash
roslaunch wego_planning planner.launch

# RViz 포함
roslaunch wego_planning planner.launch rviz:=true
```

#### 개별 노드 실행
```bash
# 전역 경로 계획 노드만 실행
rosrun wego_planning path_planner_node.py

# 지역 궤적 생성 노드만 실행
rosrun wego_planning local_planner_node.py

# 차선 정보를 사용한 경로 계획
rosrun wego_planning path_planner_with_lane.py
```

### 모드 2: Move Base 내비게이션

#### 기본 Move Base 실행 (LaserScan 사용)
```bash
roslaunch wego_planning move_base/move_base_core.launch
```

#### Vision 기반 Move Base 실행 (카메라 장애물 사용)
```bash
# Vision PointCloud 사용
roslaunch wego_planning move_base/move_base_vision.launch

# Vision LaserScan 사용
roslaunch wego_planning move_base/move_base_vision.launch use_laserscan:=true
```

#### 목표 지점 전송
```bash
# 기본 목표 (2.0, 0.0)
rosrun wego_planning send_goal.py

# 커스텀 목표
rosrun wego_planning send_goal.py _x:=3.0 _y:=2.0

# 프레임 지정
rosrun wego_planning send_goal.py _x:=1.5 _y:=1.0 _frame:=odom
```

#### 시각화
```bash
roslaunch wego_planning move_base/planning_viz.launch
```

---

## 📊 노드 그래프

### 차선 기반 Planning
```
/vision/fused_objects ─┐
                       ├─→ [path_planner_node] → /planning/path ─┐
/odom ─────────────────┘                                          │
                                                                  ├─→ [local_planner_node] → /planning/trajectory
/odom ────────────────────────────────────────────────────────────┤
/vision/fused_objects ────────────────────────────────────────────┘

또는

/vision/lane_info ─┐
                   ├─→ [path_planner_with_lane] → /planning/path
/odom ─────────────┘
```

### Move Base Navigation
```
/move_base_simple/goal ─→ [move_base] ─→ /cmd_vel
                              ↑
                              ├─ /odom
                              ├─ /scan (또는 /vision/obstacles)
                              └─ /map
```

---

## ⚙️ 설정

### 차선 기반 Planning 설정
설정 파일: `config/planner_params.yaml`

**주요 파라미터:**
- `path_planner/algorithm`: 계획 알고리즘 선택 (astar, rrt, dijkstra)
- `path_planner/frequency`: 계획 주기 (Hz)
- `local_planner/planner_type`: 플래너 타입 (dwa, pure_pursuit)
- `local_planner/lookahead_distance`: 전방 주시 거리 (m)

### Move Base 설정
설정 디렉토리: `config/move_base/`

**설정 파일들:**
- `costmap_common.yaml`: 공통 costmap 설정 (LaserScan)
- `costmap_common_vision.yaml`: Vision PointCloud 기반 설정
- `costmap_common_vision_scan.yaml`: Vision LaserScan 기반 설정
- `global_costmap.yaml`: 전역 costmap 설정
- `local_costmap.yaml`: 지역 costmap 설정
- `dwa_local_planner.yaml`: DWA 플래너 파라미터

**주요 파라미터:**

```yaml
# DWA Local Planner
DWAPlannerROS:
  max_vel_x: 0.6          # 최대 선속도 (m/s)
  min_vel_x: 0.05         # 최소 선속도 (m/s)
  max_vel_theta: 1.0      # 최대 각속도 (rad/s)
  
  # 궤도 방지: 글로벌 플랜을 더 따르게
  path_distance_bias: 48.0
  goal_distance_bias: 16.0
  
  # 목표 도달 허용 오차
  xy_goal_tolerance: 0.30
  yaw_goal_tolerance: 0.35

# Costmap
obstacle_range: 4.0       # 장애물 감지 범위 (m)
inflation_radius: 0.35    # 장애물 팽창 반경 (m)

# Footprint (로봇 크기)
footprint: [[-0.18, -0.14], [-0.18, 0.14], [0.18, 0.14], [0.18, -0.14]]
```

---

## 📦 의존성

### ROS 패키지
- `rospy`
- `std_msgs`
- `geometry_msgs`
- `nav_msgs`
- `sensor_msgs`
- `tf`, `tf2_ros`, `tf2_geometry_msgs`
- `move_base`, `move_base_msgs`
- `cv_bridge`, `image_transport`
- `topic_tools`

### Python 패키지
- `python3-numpy`
- `python3-scipy`

---

## 🧮 알고리즘 설명

### 차선 기반 Planning

#### A* (A-star)
- 휴리스틱 기반 최적 경로 탐색
- 빠르고 효율적인 전역 경로 계획

#### DWA (Dynamic Window Approach)
- 동적 제약 조건을 고려한 지역 궤적 생성
- 속도와 가속도 제한 내에서 최적 경로 선택

#### Pure Pursuit
- 전방 주시점 추종 알고리즘
- 부드러운 곡선 주행에 적합

### Move Base Navigation

#### NavfnROS (Global Planner)
- Dijkstra 기반 전역 경로 계획
- 정적 맵에서 최단 경로 탐색

#### DWA Local Planner
- 동적 창 접근법으로 지역 경로 생성
- 장애물 회피 및 속도 최적화
- 로봇의 동역학 제약 고려

---

## 🔍 파일 구조

```
wego_planning/
├── CMakeLists.txt
├── package.xml
├── setup.py
├── README.md
├── scripts/
│   ├── path_planner_node.py          # 전역 경로 계획
│   ├── local_planner_node.py         # 지역 궤적 생성
│   ├── path_planner_with_lane.py     # 차선 기반 경로 계획
│   └── send_goal.py                  # Move Base 목표 전송
├── launch/
│   ├── planner.launch                # 차선 기반 플래너
│   └── move_base/
│       ├── move_base_core.launch     # 기본 Move Base
│       ├── move_base_vision.launch   # Vision 기반 Move Base
│       └── planning_viz.launch       # 시각화
└── config/
    ├── planner_params.yaml           # 차선 기반 파라미터
    └── move_base/
        ├── costmap_common.yaml
        ├── costmap_common_vision.yaml
        ├── costmap_common_vision_scan.yaml
        ├── global_costmap.yaml
        ├── local_costmap.yaml
        └── dwa_local_planner.yaml
```

---

## 🐛 트러블슈팅

### Move Base가 목표에 도달하지 못함

**원인:** Costmap 파라미터 또는 goal tolerance 설정 문제

**해결:**
```bash
# Goal tolerance 확대
rosparam set /move_base/DWAPlannerROS/xy_goal_tolerance 0.5
rosparam set /move_base/DWAPlannerROS/yaw_goal_tolerance 0.5

# 장애물 팽창 반경 줄이기
rosparam set /move_base/local_costmap/inflation_radius 0.2
```

### 로봇이 제자리에서 맴돎 (Oscillation)

**원인:** path_distance_bias가 너무 작음

**해결:**
```bash
rosparam set /move_base/DWAPlannerROS/path_distance_bias 64.0
```

### Costmap에 장애물이 안 보임

**원인:** 센서 데이터가 안 들어옴 또는 sensor_frame 설정 오류

**해결:**
```bash
# 센서 토픽 확인
rostopic echo /vision/obstacles
rostopic echo /scan

# TF 확인
rosrun tf tf_echo odom camera_link
```

---

## 🔧 추후 개발 계획

- [ ] RRT* 알고리즘 구현
- [ ] 동적 장애물 예측 및 회피
- [ ] MPC (Model Predictive Control) 추가
- [ ] 주차 경로 계획 기능
- [ ] SLAM과 통합
- [ ] 다중 목표 경로 계획

---

## 📝 라이센스

MIT License

