# 차선 인식 (Lane Detection) 가이드

## 📋 개요

`lane_detection_node.py`는 카메라 이미지에서 차선을 검출하고, Planning 노드가 사용할 수 있는 형태로 차선 정보를 발행합니다.

## 🎯 주요 기능

### 1. 차선 검출
- **색상 기반 검출**: 흰색 + 노란색 차선
- **Edge Detection**: Canny edge detector
- **Line Detection**: Hough Line Transform
- **Lane Fitting**: 2차 다항식 피팅 (y = ax² + bx + c)

### 2. 제공 정보
- 좌/우 차선 검출 여부
- 차선 방정식 계수 (a, b, c)
- 차선 중심으로부터의 오프셋 (center_offset)
- 차량 heading error
- 차선 곡률 (curvature)
- 신뢰도 (confidence)

## 🚀 빌드 및 실행

### 1. 빌드 (메시지 포함)

```bash
cd ~/autorace2025/autorace2025
source /opt/ros/noetic/setup.bash

# 빌드 (LaneInfo.msg 포함)
catkin_make

# 소싱
source devel/setup.bash

# 메시지 확인
rosmsg show wego_vision/LaneInfo
```

### 2. 실행 권한 부여

```bash
chmod +x src/wego_vision/scripts/lane_detection_node.py
```

### 3. 실행

#### 옵션 A: Vision 파이프라인 전체 실행
```bash
# 터미널 1: roscore
roscore

# 터미널 2: Vision 파이프라인
source ~/autorace2025/autorace2025/devel/setup.bash
roslaunch wego_vision vision_pipeline.launch
```

#### 옵션 B: 차선 인식 노드만 실행
```bash
# 터미널 1: roscore
roscore

# 터미널 2: 카메라 노드
rosrun wego_vision camera_node.py

# 터미널 3: 차선 인식 노드
rosrun wego_vision lane_detection_node.py
```

## 📊 토픽 구조

### 입력 토픽
- `/vision/image_rect` (sensor_msgs/Image)
  - 보정된 카메라 이미지

### 출력 토픽
- `/vision/lane_info` (wego_vision/LaneInfo)
  - 차선 정보 (Planning 노드가 사용)
- `/vision/lane_image` (sensor_msgs/Image)
  - 차선 검출 결과 시각화 이미지

## 🔍 토픽 확인

```bash
# 차선 정보 확인
rostopic echo /vision/lane_info

# 출력 예시:
# header: ...
# left_lane_detected: True
# right_lane_detected: True
# center_offset: 0.023  # 왼쪽으로 2.3cm
# heading_error: 0.05   # 5도
# curvature: 0.0001
# confidence: 0.9

# 토픽 주파수 확인
rostopic hz /vision/lane_info

# 차선 이미지 확인 (RQt Image View)
rosrun rqt_image_view rqt_image_view /vision/lane_image
```

## ⚙️ 파라미터 튜닝

### 설정 파일
`src/wego_vision/config/lane_detection_params.yaml`

### 주요 파라미터

#### 1. ROI 설정
```yaml
roi_top_ratio: 0.5      # 이미지 상단 50%부터 (0.0~1.0)
roi_bottom_ratio: 1.0   # 이미지 하단 100%까지
```

#### 2. 색상 필터 (실외 밝은 환경)
```yaml
white_lower: [0, 0, 200]     # 밝은 흰색
white_upper: [180, 30, 255]
```

#### 3. 색상 필터 (실내 어두운 테이프)
```yaml
white_lower: [0, 0, 0]       # 어두운 검은색 테이프
white_upper: [180, 255, 50]
```

#### 4. Edge 검출 민감도
```yaml
canny_low: 50     # 낮추면 더 많은 edge 검출
canny_high: 150   # 높이면 강한 edge만 검출
```

#### 5. Hough Line 파라미터
```yaml
hough_threshold: 20          # 낮추면 더 많은 선 검출
hough_min_line_length: 20    # 최소 선 길이
hough_max_line_gap: 300      # 선 사이 간격
```

## 🎨 시각화

### RViz 설정

```bash
# RViz 실행
rviz

# Add 버튼 클릭 후 추가:
# 1. Image → Topic: /vision/lane_image
# 2. Marker → Topic: /visualization_marker (차선 3D 표시)
```

### Image View
```bash
# 차선 검출 결과 이미지
rosrun rqt_image_view rqt_image_view /vision/lane_image

# 원본 이미지 비교
rosrun rqt_image_view rqt_image_view /vision/image_rect
```

## 🔧 Planning 노드 연동

Planning 노드에서 차선 정보 사용 예시:

```python
#!/usr/bin/env python3
import rospy
from wego_vision.msg import LaneInfo

class PathPlannerNode:
    def __init__(self):
        # 차선 정보 구독
        rospy.Subscriber('/vision/lane_info', LaneInfo, self.lane_callback)
    
    def lane_callback(self, msg):
        # 차선 중심으로부터의 오프셋
        offset = msg.center_offset
        
        # Heading error
        heading_err = msg.heading_error
        
        # 양쪽 차선 모두 검출되었는지 확인
        if msg.left_lane_detected and msg.right_lane_detected:
            if abs(offset) > 0.1:  # 10cm 이상 벗어남
                rospy.logwarn(f"차선 이탈 위험: {offset:.3f}m")
            
            # 차선 중심을 따라가는 경로 생성
            # ... 경로 계획 로직
```

## 📈 성능 최적화

### 1. ROI 최적화
```yaml
# 차선이 주로 보이는 영역만 처리
roi_top_ratio: 0.6    # 하단 40%만 처리
```

### 2. 해상도 조정
```xml
<!-- vision_pipeline.launch -->
<param name="image_width" value="320"/>   <!-- 낮은 해상도로 빠른 처리 -->
<param name="image_height" value="240"/>
```

### 3. 디버그 이미지 끄기
```yaml
publish_debug_image: false  # 프로덕션 환경
```

## 🧪 테스트

### 1. 정지 이미지 테스트
```bash
# 이미지 발행 (테스트용)
rosrun image_publisher image_publisher test_lane.jpg

# 차선 검출 확인
rostopic echo /vision/lane_info
```

### 2. 실시간 테스트
```bash
# Vision 파이프라인 실행
roslaunch wego_vision vision_pipeline.launch

# 차선 정보 모니터링
watch -n 0.5 rostopic echo /vision/lane_info
```

### 3. 녹화 및 재생
```bash
# 차선 주행 녹화
rosbag record /vision/image_rect /vision/lane_info

# 재생
rosbag play <파일명>.bag
```

## 🐛 트러블슈팅

### 문제 1: 차선이 검출되지 않음
```bash
# 디버그 이미지 확인
rosrun rqt_image_view rqt_image_view /vision/lane_image

# 해결:
# 1. white_lower/upper 조정 (색상 범위)
# 2. canny_low/high 조정 (edge 민감도)
# 3. roi_top_ratio 조정 (ROI 확장)
```

### 문제 2: 너무 많은 false positive
```bash
# 해결:
# 1. hough_threshold 증가 (선 검출 임계값)
# 2. hough_min_line_length 증가 (짧은 선 제거)
# 3. ROI 좁히기
```

### 문제 3: 신뢰도 낮음
```yaml
# 파라미터 튜닝
canny_low: 30           # 낮춤
canny_high: 100         # 낮춤
hough_threshold: 15     # 낮춤
```

## 📚 알고리즘 설명

### 1. 전처리
1. ROI 적용 (사다리꼴 영역)
2. HSV 색공간 변환
3. 색상 필터링 (흰색 + 노란색)

### 2. Edge 검출
1. Canny Edge Detection
2. Binary 이미지 생성

### 3. Line 검출
1. Hough Line Transform
2. 선 분류 (좌/우)
3. 기울기 기반 필터링

### 4. Lane Fitting
1. 점들을 수집
2. 2차 다항식 피팅: `y = ax² + bx + c`
3. 좌/우 차선 방정식 생성

### 5. 정보 계산
1. 차선 중심 계산
2. Offset 및 Heading error 계산
3. 곡률 계산

## 🎓 다음 단계

### 개선 사항
- [ ] Kalman Filter로 차선 추적 안정화
- [ ] 딥러닝 기반 차선 검출 (LaneNet)
- [ ] 곡선 도로 대응 개선
- [ ] 다중 차선 검출
- [ ] 차선 변경 감지

### Planning 연동
- [ ] 차선 중심을 따라가는 경로 생성
- [ ] 차선 이탈 경고 시스템
- [ ] 차선 변경 경로 계획

---

**문제가 있거나 개선사항이 있다면 언제든 알려주세요!** 🚗💨

