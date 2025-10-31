# WeGO Vision Package

## 📋 개요

`wego_vision`은 WeGO 자율주행 시스템의 **환경 인식** 모듈입니다.  
카메라, LiDAR, IMU 등의 센서 데이터를 처리하여 주변 환경의 장애물과 주행 가능 영역을 인식합니다.

## 🎯 주요 기능

### 1. **Camera Node** (`camera_node.py`)
- **역할**: 카메라 이미지 수신 및 전처리
- **입력**: `/usb_cam/image_raw` (sensor_msgs/Image)
- **출력**: `/vision/image_rect` (sensor_msgs/Image)
- **기능**:
  - 카메라 캘리브레이션
  - 왜곡 보정 (Undistortion)
  - 이미지 품질 향상

### 2. **Detection Node** (`detection_node.py`)
- **역할**: 객체 감지 및 장애물 인식
- **입력**: `/vision/image_rect` (sensor_msgs/Image)
- **출력**: `/vision/obstacles` (geometry_msgs/PoseArray)
- **지원 알고리즘**:
  - ArUco 마커 감지
  - YOLOv8 객체 감지 (추후 구현)

### 3. **Fusion Node** (`fusion_node.py`)
- **역할**: 다중 센서 데이터 융합
- **입력**:
  - `/vision/obstacles` (geometry_msgs/PoseArray)
  - `/scan` (sensor_msgs/LaserScan) - 선택적
  - `/imu` (sensor_msgs/Imu) - 선택적
- **출력**: `/vision/fused_objects` (geometry_msgs/PoseArray)
- **기능**:
  - Kalman Filter 기반 센서 융합
  - 노이즈 제거 및 정확도 향상

## 🚀 사용 방법

### 전체 비전 파이프라인 실행
```bash
roslaunch wego_vision vision_pipeline.launch
```

### 개별 노드 실행
```bash
# 카메라 노드만 실행
rosrun wego_vision camera_node.py

# 객체 감지 노드만 실행
rosrun wego_vision detection_node.py

# 센서 융합 노드만 실행
rosrun wego_vision fusion_node.py
```

### RViz 시각화 포함 실행
```bash
roslaunch wego_vision vision_pipeline.launch rviz:=true
```

## 📊 노드 그래프

```
/usb_cam/image_raw → [camera_node] → /vision/image_rect → [detection_node] → /vision/obstacles → [fusion_node] → /vision/fused_objects
                                                                                                    ↑
                                                                                    /scan (LiDAR) ──┤
                                                                                    /imu (IMU) ─────┘
```

## ⚙️ 설정

설정 파일: `config/camera_params.yaml`

주요 파라미터:
- `camera/width`, `camera/height`: 카메라 해상도
- `detection/method`: 감지 알고리즘 선택 (`aruco` 또는 `yolo`)
- `fusion/use_lidar`, `fusion/use_imu`: 추가 센서 사용 여부

## 📦 의존성

- `rospy`
- `cv_bridge`
- `sensor_msgs`
- `geometry_msgs`
- `image_transport`
- `message_filters`
- `python3-opencv`
- `usb_cam` (카메라 드라이버)

## 🔧 추후 개발 계획

- [ ] YOLOv8 객체 감지 통합
- [ ] LiDAR 포인트 클라우드 처리
- [ ] 딥러닝 기반 차선 인식
- [ ] 실시간 SLAM 통합

## 📝 라이센스

MIT License

