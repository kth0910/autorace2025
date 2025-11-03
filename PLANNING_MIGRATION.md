# Planning 패키지 병합 완료

`autorace_planning` 패키지가 `wego_planning` 패키지로 성공적으로 병합되었습니다.

## 📦 변경 사항

### 1. 패키지 통합

기존의 두 planning 패키지를 하나로 통합했습니다:
- ✅ `autorace_planning` → `wego_planning`으로 병합
- ✅ Move Base 기능 추가
- ✅ 차선 기반 planning 유지

### 2. 추가된 파일

#### Scripts
- `wego_planning/scripts/send_goal.py` - Move Base 목표 전송 노드

#### Launch Files
- `wego_planning/launch/move_base/move_base_core.launch` - 기본 Move Base
- `wego_planning/launch/move_base/move_base_vision.launch` - Vision 기반 Move Base
- `wego_planning/launch/move_base/planning_viz.launch` - 시각화

#### Config Files (move_base/)
- `costmap_common.yaml` - 기본 costmap (LaserScan)
- `costmap_common_vision.yaml` - Vision PointCloud 기반
- `costmap_common_vision_scan.yaml` - Vision LaserScan 기반
- `global_costmap.yaml` - 전역 costmap 설정
- `local_costmap.yaml` - 지역 costmap 설정
- `dwa_local_planner.yaml` - DWA 플래너 파라미터

### 3. 업데이트된 파일

#### `package.xml`
새로운 의존성 추가:
```xml
<depend>move_base</depend>
<depend>move_base_msgs</depend>
<depend>sensor_msgs</depend>
<depend>tf2_geometry_msgs</depend>
<depend>cv_bridge</depend>
<depend>image_transport</depend>
<exec_depend>topic_tools</exec_depend>
```

#### `CMakeLists.txt`
- Move Base 관련 패키지 추가
- `send_goal.py` 스크립트 추가

#### `README.md`
- 두 가지 planning 모드 설명 추가
- Move Base 사용 방법 추가
- 트러블슈팅 섹션 추가

---

## 🚀 사용 방법

### 방법 1: 차선 기반 Planning (기존 방식)

```bash
# 차선 정보 기반 경로 계획
rosrun wego_planning path_planner_with_lane.py
```

### 방법 2: Move Base Navigation (신규 추가)

```bash
# Vision 기반 Move Base
roslaunch wego_planning move_base/move_base_vision.launch

# 목표 지점 전송
rosrun wego_planning send_goal.py _x:=2.0 _y:=1.0
```

---

## ⚠️ 주의 사항

### 1. autorace_planning 패키지

`autorace_planning` 패키지는 이제 사용하지 않습니다.
모든 기능이 `wego_planning`으로 이전되었습니다.

**옵션 A: 패키지 제거 (권장)**
```bash
cd ~/autorace2025/autorace2025/src
rm -rf autorace_planning
cd ~/autorace2025/autorace2025
catkin_make
```

**옵션 B: 백업 후 제거**
```bash
cd ~/autorace2025/autorace2025/src
mv autorace_planning autorace_planning.backup
cd ~/autorace2025/autorace2025
catkin_make
```

### 2. 의존성 설치

Move Base를 사용하려면 다음 패키지가 필요합니다:

```bash
sudo apt update
sudo apt install -y \
  ros-noetic-move-base \
  ros-noetic-move-base-msgs \
  ros-noetic-dwa-local-planner \
  ros-noetic-topic-tools
```

### 3. 빌드

병합 후 반드시 빌드해야 합니다:

```bash
cd ~/autorace2025/autorace2025
catkin_make
source devel/setup.bash
```

---

## 📊 패키지 구조 비교

### Before (분리)
```
src/
├── autorace_planning/    ← Move Base 전용
│   ├── config/
│   ├── launch/
│   └── scripts/
└── wego_planning/        ← 차선 기반 전용
    ├── config/
    ├── launch/
    └── scripts/
```

### After (통합)
```
src/
└── wego_planning/        ← 통합 패키지
    ├── config/
    │   ├── planner_params.yaml      (차선 기반)
    │   └── move_base/               (Move Base)
    ├── launch/
    │   ├── planner.launch           (차선 기반)
    │   └── move_base/               (Move Base)
    └── scripts/
        ├── path_planner_node.py          (차선 기반)
        ├── local_planner_node.py         (차선 기반)
        ├── path_planner_with_lane.py     (차선 기반)
        └── send_goal.py                  (Move Base)
```

---

## 🔄 마이그레이션 체크리스트

- [x] Config 파일 복사
- [x] Launch 파일 복사 및 경로 수정
- [x] Script 파일 복사
- [x] package.xml 의존성 업데이트
- [x] CMakeLists.txt 업데이트
- [x] README 업데이트
- [x] QUICK_RUN.txt 업데이트
- [ ] autorace_planning 패키지 제거 (사용자 수동)
- [ ] 의존성 패키지 설치 (사용자 수동)
- [ ] 빌드 (사용자 수동)

---

## 🧪 테스트

### 1. 패키지 확인
```bash
rospack find wego_planning
rospack list | grep planning
```

### 2. Launch 파일 확인
```bash
roslaunch wego_planning planner.launch --screen
roslaunch wego_planning move_base/move_base_vision.launch --screen
```

### 3. 노드 확인
```bash
rosrun wego_planning path_planner_with_lane.py
rosrun wego_planning send_goal.py
```

---

## 📝 문서

- `wego_planning/README.md`: 통합 패키지 전체 문서
- `QUICK_RUN.txt`: 빠른 실행 가이드 (업데이트됨)
- `ODOMETRY_SETUP.md`: Odometry 설정 가이드 (신규)

---

## 🎯 다음 단계

1. **의존성 설치**
   ```bash
   sudo apt install ros-noetic-move-base ros-noetic-dwa-local-planner
   ```

2. **빌드**
   ```bash
   cd ~/autorace2025/autorace2025
   catkin_make
   source devel/setup.bash
   ```

3. **autorace_planning 제거** (선택사항)
   ```bash
   rm -rf src/autorace_planning
   ```

4. **테스트**
   - 차선 기반 planning 테스트
   - Move Base navigation 테스트

---

**병합 완료일**: 2025-11-03  
**작업자**: WeGO 자율주행 팀

