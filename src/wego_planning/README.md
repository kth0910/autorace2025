# WeGO Planning Package

## 📋 개요

`wego_planning`은 WeGO 자율주행 시스템의 **경로 계획** 모듈입니다.  
전역 경로 계획과 지역 궤적 생성을 담당하여 안전하고 효율적인 주행 경로를 제공합니다.

## 🎯 주요 기능

### 1. **Path Planner Node** (`path_planner_node.py`)
- **역할**: 전역 경로 계획 (시작점 → 목표점)
- **입력**:
  - `/vision/fused_objects` (geometry_msgs/PoseArray): 장애물 정보
  - `/odom` (nav_msgs/Odometry): 현재 위치
- **출력**: `/planning/path` (nav_msgs/Path)
- **지원 알고리즘**:
  - A* (A-star)
  - RRT (Rapidly-exploring Random Tree)
  - Dijkstra

### 2. **Local Planner Node** (`local_planner_node.py`)
- **역할**: 지역 궤적 생성 및 장애물 회피
- **입력**:
  - `/planning/path` (nav_msgs/Path): 전역 경로
  - `/odom` (nav_msgs/Odometry): 현재 위치 및 속도
  - `/vision/fused_objects` (geometry_msgs/PoseArray): 장애물
- **출력**: `/planning/trajectory` (nav_msgs/Path)
- **지원 알고리즘**:
  - DWA (Dynamic Window Approach)
  - TEB (Timed Elastic Band)
  - Pure Pursuit

## 🚀 사용 방법

### 전체 계획 파이프라인 실행
```bash
roslaunch wego_planning planner.launch
```

### 개별 노드 실행
```bash
# 전역 경로 계획 노드만 실행
rosrun wego_planning path_planner_node.py

# 지역 궤적 생성 노드만 실행
rosrun wego_planning local_planner_node.py
```

### RViz 시각화 포함 실행
```bash
roslaunch wego_planning planner.launch rviz:=true
```

## 📊 노드 그래프

```
/vision/fused_objects ─┐
                       ├─→ [path_planner_node] → /planning/path ─┐
/odom ─────────────────┘                                          │
                                                                  ├─→ [local_planner_node] → /planning/trajectory
/odom ────────────────────────────────────────────────────────────┤
/vision/fused_objects ────────────────────────────────────────────┘
```

## ⚙️ 설정

설정 파일: `config/planner_params.yaml`

### 주요 파라미터

**전역 경로 계획:**
- `path_planner/algorithm`: 계획 알고리즘 선택
- `path_planner/frequency`: 계획 주기

**지역 궤적 생성:**
- `local_planner/planner_type`: 플래너 타입 선택
- `local_planner/lookahead_distance`: 전방 주시 거리
- `local_planner/dwa/max_vel_x`: 최대 선속도

## 📦 의존성

- `rospy`
- `std_msgs`
- `geometry_msgs`
- `nav_msgs`
- `tf`, `tf2_ros`
- `python3-numpy`
- `python3-scipy`

## 🧮 알고리즘 설명

### A* (A-star)
- 휴리스틱 기반 최적 경로 탐색
- 빠르고 효율적인 전역 경로 계획

### DWA (Dynamic Window Approach)
- 동적 제약 조건을 고려한 지역 궤적 생성
- 속도와 가속도 제한 내에서 최적 경로 선택

### Pure Pursuit
- 전방 주시점 추종 알고리즘
- 부드러운 곡선 주행에 적합

## 🔧 추후 개발 계획

- [ ] RRT* 알고리즘 구현
- [ ] 동적 장애물 예측 및 회피
- [ ] 비용 맵(Costmap) 통합
- [ ] MPC (Model Predictive Control) 추가
- [ ] 주차 경로 계획 기능

## 📝 라이센스

MIT License

