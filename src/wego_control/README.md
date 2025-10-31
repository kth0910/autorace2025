# WeGO Control Package

## 📋 개요

`wego_control`은 WeGO 자율주행 시스템의 **차량 제어** 모듈입니다.  
계획된 궤적을 추종하기 위한 조향 및 속도 제어를 수행하고, VESC 모터 컨트롤러와의 인터페이스를 제공합니다.

## 🎯 주요 기능

### 1. **Controller Node** (`controller_node.py`)
- **역할**: 경로 추종 제어기
- **입력**:
  - `/planning/trajectory` (nav_msgs/Path): 목표 궤적
  - `/odom` (nav_msgs/Odometry): 현재 위치 및 속도
- **출력**: `/ackermann_cmd` (ackermann_msgs/AckermannDriveStamped)
- **지원 알고리즘**:
  - **Stanley Controller** (기본)
  - PID Controller (추후 구현)
  - MPC (Model Predictive Control) (추후 구현)

### 2. **VESC Bridge Node** (`vesc_bridge_node.py`)
- **역할**: Ackermann 명령을 VESC 명령으로 변환
- **입력**: `/ackermann_cmd` (ackermann_msgs/AckermannDriveStamped)
- **출력**:
  - `/commands/motor/speed` (std_msgs/Float64): 모터 ERPM
  - `/commands/servo/position` (std_msgs/Float64): 서보 위치
- **기능**:
  - 조향각 → 서보 위치 매핑
  - 속도 → ERPM 변환
  - 안전 제한 적용

## 🚀 사용 방법

### 전체 제어 시스템 실행
```bash
roslaunch wego_control control.launch
```

### 개별 노드 실행
```bash
# 제어기 노드만 실행
rosrun wego_control controller_node.py

# VESC 브릿지 노드만 실행
rosrun wego_control vesc_bridge_node.py
```

### RViz 시각화 포함 실행
```bash
roslaunch wego_control control.launch rviz:=true
```

## 📊 노드 그래프

```
/planning/trajectory ─┐
                      ├─→ [controller_node] → /ackermann_cmd → [vesc_bridge_node] ─┬─→ /commands/motor/speed
/odom ────────────────┘                                                              └─→ /commands/servo/position
```

## ⚙️ 설정

설정 파일: `config/control_params.yaml`

### 주요 파라미터

**차량 파라미터:**
- `vehicle/wheelbase`: 축간 거리 (0.32m)
- `vehicle/max_steering_angle`: 최대 조향각 (0.5 rad)
- `vehicle/max_speed`: 최대 속도 (2.0 m/s)

**Stanley Controller:**
- `stanley/k`: Cross-track error 게인 (1.0)
- `stanley/k_soft`: Softening constant (2.5)

**VESC 변환:**
- `vesc/speed_to_erpm_gain`: 속도-ERPM 변환 게인 (4000.0)
- `vesc/steering_to_servo_gain`: 조향-서보 변환 게인 (0.5)

## 🎛️ Stanley Controller

### 알고리즘 설명

Stanley Controller는 경로 추종을 위한 비선형 제어기로, 두 가지 에러를 동시에 보정합니다:

1. **Heading Error (θₑ)**: 차량 방향과 경로 방향의 차이
2. **Cross-track Error (eₜ)**: 차량과 경로 간의 수직 거리

**제어 법칙:**
```
δ = θₑ + arctan(k * eₜ / (k_soft + v))
```

여기서:
- `δ`: 조향각
- `k`: Cross-track error 게인
- `v`: 현재 속도
- `k_soft`: 저속에서의 안정성을 위한 상수

### 장점
- 저속과 고속 모두에서 안정적
- 간단한 구현과 빠른 계산
- 비홀로노믹 제약 조건 고려

## 🔌 VESC 인터페이스

### 조향 변환
```
servo_position = steering_offset + (steering_angle * gain)
```
- 중앙: 0.5
- 좌회전: > 0.5
- 우회전: < 0.5

### 속도 변환
```
ERPM = speed (m/s) * gain + offset
```
- 기본 게인: 4000 (1 m/s = 4000 ERPM)

## 📦 의존성

- `rospy`
- `std_msgs`
- `geometry_msgs`
- `nav_msgs`
- `ackermann_msgs`
- `tf` (좌표 변환)
- `python3-numpy`

## 🛡️ 안전 기능

### 제한 사항
- 최대 조향각 제한
- 최대 속도 제한
- 서보 위치 범위 제한 (0.0 ~ 1.0)
- 모터 ERPM 제한

### 비상 정지
- 명령 타임아웃 감지
- Odometry 데이터 손실 감지
- 장애물 최소 거리 위반 시 (추후 구현)

## 🔧 추후 개발 계획

- [ ] PID Controller 구현
- [ ] MPC (Model Predictive Control) 구현
- [ ] 적응형 속도 제어 (곡률 기반)
- [ ] 긴급 제동 시스템
- [ ] 차량 상태 모니터링
- [ ] 실시간 파라미터 튜닝 인터페이스

## 📝 라이센스

MIT License

