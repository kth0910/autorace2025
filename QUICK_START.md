# WeGO 빠른 시작 가이드

## ⚡ 5분 만에 시작하기

### 1️⃣ 워크스페이스 빌드

```bash
cd ~/autorace2025/wego_ws
catkin_make
source devel/setup.bash
```

### 2️⃣ 실행 권한 부여

```bash
# 모든 Python 스크립트에 실행 권한 부여
find src -name "*.py" -type f -exec chmod +x {} \;
```

### 3️⃣ 실행

#### 방법 1: 시뮬레이션 모드 (하드웨어 없이)

```bash
roslaunch wego_bringup autonomous_stack.launch simulation:=true rviz:=true
```

#### 방법 2: 실제 차량 (카메라 연결 필요)

```bash
roslaunch wego_bringup autonomous_stack.launch rviz:=true
```

#### 방법 3: 개별 모듈 테스트

```bash
# Vision만 테스트
roslaunch wego_bringup minimal.launch node_group:=vision

# Planning만 테스트
roslaunch wego_bringup minimal.launch node_group:=planning

# Control만 테스트
roslaunch wego_bringup minimal.launch node_group:=control
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

### 빌드 에러
```bash
# 클린 빌드
cd ~/autorace2025/wego_ws
catkin_make clean
catkin_make
source devel/setup.bash
```

### 카메라 인식 실패
```bash
# 카메라 디바이스 확인
ls /dev/video*

# 없으면 시뮬레이션 모드 사용
roslaunch wego_bringup autonomous_stack.launch simulation:=true rviz:=true
```

### Python import 에러
```bash
# 워크스페이스 소싱 확인
source ~/autorace2025/wego_ws/devel/setup.bash

# bashrc에 추가 (영구 적용)
echo "source ~/autorace2025/wego_ws/devel/setup.bash" >> ~/.bashrc
```

---

## 📚 다음 단계

1. **튜닝**: [각 패키지의 config/ 폴더](src/)에서 파라미터 조정
2. **개발**: 새로운 알고리즘 추가 및 커스터마이징
3. **문서**: [README.md](README.md) 및 각 패키지 문서 참조

---

## 🆘 도움말

- **전체 문서**: [README.md](README.md)
- **Vision 문서**: [wego_vision/README.md](src/wego_vision/README.md)
- **Planning 문서**: [wego_planning/README.md](src/wego_planning/README.md)
- **Control 문서**: [wego_control/README.md](src/wego_control/README.md)
- **Bringup 문서**: [wego_bringup/README.md](src/wego_bringup/README.md)

---

**즐거운 개발 되세요! 🚗💨**

