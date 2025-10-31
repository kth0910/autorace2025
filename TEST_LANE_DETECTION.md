# 차선 인식 테스트 가이드

## 🚀 빠른 테스트 (5분)

### 1단계: 빌드

```bash
cd ~/autorace2025/autorace2025
source /opt/ros/noetic/setup.bash

# 메시지 포함 빌드
catkin_make

# 소싱
source devel/setup.bash

# 메시지 확인
rosmsg show wego_vision/LaneInfo
```

예상 출력:
```
std_msgs/Header header
bool left_lane_detected
bool right_lane_detected
float32 left_a
float32 left_b
float32 left_c
...
```

### 2단계: 실행 권한

```bash
chmod +x src/wego_vision/scripts/lane_detection_node.py
```

### 3단계: 실행

```bash
# 터미널 1: roscore
roscore

# 터미널 2: Vision 파이프라인 (차선 인식 포함)
source ~/autorace2025/autorace2025/devel/setup.bash
roslaunch wego_vision vision_pipeline.launch
```

### 4단계: 확인

```bash
# 터미널 3: 토픽 확인
rostopic list | grep lane

# 출력:
# /vision/lane_info
# /vision/lane_image

# 차선 정보 확인
rostopic echo /vision/lane_info

# 차선 이미지 확인
rosrun rqt_image_view rqt_image_view /vision/lane_image
```

---

## 🎨 시각화 테스트

### RQt Image View로 확인

```bash
# 4개 창을 동시에
rosrun rqt_image_view rqt_image_view /usb_cam/image_raw &
rosrun rqt_image_view rqt_image_view /vision/image_rect &
rosrun rqt_image_view rqt_image_view /vision/lane_image &
```

**확인할 것:**
- 원본 이미지에 차선이 보이는가?
- 차선 검출 이미지에 녹색(좌), 파란색(우), 빨간색(중심) 선이 그려졌는가?
- 화면 상단에 Offset, Heading Error, Confidence 값이 표시되는가?

### RViz로 3D 시각화

```bash
# RViz 실행
rviz

# Add 버튼 클릭:
# 1. Image → Topic: /vision/lane_image
# 2. Path → Topic: /planning/path (Planning 노드 연동 시)
```

---

## 📊 데이터 모니터링

### 실시간 데이터 확인

```bash
# 차선 정보 실시간 모니터링
watch -n 0.5 "rostopic echo /vision/lane_info | head -20"
```

### 주파수 확인

```bash
# 차선 인식 주파수 (카메라 FPS에 따라 다름)
rostopic hz /vision/lane_info

# 예상: 10-30 Hz
```

### 메시지 내용 확인

```bash
rostopic echo /vision/lane_info

# 예상 출력:
# header:
#   seq: 123
#   stamp: ...
# left_lane_detected: True
# right_lane_detected: True
# left_a: 0.0001
# left_b: -0.5
# left_c: 320.0
# center_offset: 0.023    # 왼쪽으로 2.3cm
# heading_error: 0.087    # 약 5도
# curvature: 0.0002
# confidence: 0.9
```

---

## 🎯 파라미터 튜닝 테스트

### 1. 실내 검은 테이프 차선 (어두운 환경)

```yaml
# config/lane_detection_params.yaml 수정
lane_detection_node:
  white_lower: [0, 0, 0]       # 검은색 검출
  white_upper: [180, 255, 50]
  roi_top_ratio: 0.6
  canny_low: 30
  canny_high: 100
```

재실행:
```bash
# 파라미터 변경 후 노드 재시작
rosnode kill /lane_detection_node
rosrun wego_vision lane_detection_node.py
```

### 2. 실외 흰색 차선 (밝은 환경)

```yaml
lane_detection_node:
  white_lower: [0, 0, 200]     # 밝은 흰색
  white_upper: [180, 30, 255]
  roi_top_ratio: 0.5
  canny_low: 50
  canny_high: 150
```

### 3. 노란색 + 흰색 차선

기본 설정 사용 (양쪽 색상 모두 검출)

---

## 🧪 정지 이미지 테스트

### 테스트 이미지 준비

```bash
# 차선이 있는 이미지 저장
rostopic echo -n 1 /usb_cam/image_raw > /tmp/lane_test.jpg

# 또는 웹에서 차선 이미지 다운로드
wget https://example.com/lane_image.jpg -O ~/test_lane.jpg
```

### 이미지 발행 노드 사용

```bash
# image_publisher 설치
sudo apt install ros-noetic-image-publisher

# 터미널 1: roscore
roscore

# 터미널 2: 이미지 발행
rosrun image_publisher image_publisher ~/test_lane.jpg \
  _publish_rate:=10 \
  image:=/vision/image_rect

# 터미널 3: 차선 인식
rosrun wego_vision lane_detection_node.py

# 터미널 4: 결과 확인
rosrun rqt_image_view rqt_image_view /vision/lane_image
```

---

## 📹 녹화 및 재생 테스트

### 차선 주행 녹화

```bash
# 주행 중 녹화
rosbag record -O lane_test.bag \
  /usb_cam/image_raw \
  /vision/image_rect \
  /vision/lane_info \
  /vision/lane_image

# Ctrl+C로 종료
```

### 재생 및 분석

```bash
# 터미널 1: roscore
roscore

# 터미널 2: bag 파일 재생
rosbag play lane_test.bag

# 터미널 3: 이미지 확인
rosrun rqt_image_view rqt_image_view /vision/lane_image

# 터미널 4: 데이터 분석
rostopic echo /vision/lane_info
```

### bag 파일 정보 확인

```bash
# bag 파일 정보
rosbag info lane_test.bag

# 특정 토픽만 재생
rosbag play lane_test.bag --topics /vision/image_rect
```

---

## 🐛 문제 해결

### 문제 1: 차선이 전혀 검출되지 않음

**원인:**
- 색상 범위가 맞지 않음
- ROI 설정이 잘못됨
- 카메라 이미지 품질 문제

**해결:**

1. 디버그 이미지 확인:
```bash
rosrun rqt_image_view rqt_image_view /vision/lane_image
```

2. 파라미터 조정:
```yaml
# 더 넓은 색상 범위
white_lower: [0, 0, 150]   # 임계값 낮춤
white_upper: [180, 50, 255]

# Edge 검출 민감도 증가
canny_low: 30              # 낮춤
canny_high: 100            # 낮춤

# ROI 확장
roi_top_ratio: 0.3         # 더 많은 영역
```

### 문제 2: False positive가 너무 많음

**해결:**

```yaml
# 더 엄격한 필터링
hough_threshold: 30         # 증가
hough_min_line_length: 30   # 증가
canny_low: 70               # 증가

# ROI 축소
roi_top_ratio: 0.6
```

### 문제 3: Confidence가 낮음

**원인:**
- 한쪽 차선만 검출됨
- 차선이 불규칙함

**해결:**

```bash
# confidence 값 확인
rostopic echo /vision/lane_info | grep confidence

# 0.5 이하면 파라미터 튜닝 필요
```

### 문제 4: 메시지가 발행되지 않음

**확인:**

```bash
# 노드 실행 확인
rosnode list | grep lane

# 토픽 발행 확인
rostopic hz /vision/lane_info

# 로그 확인
rosnode info /lane_detection_node
```

---

## ✅ 성공 기준

다음 조건이 모두 만족되면 성공:

1. ✅ `rostopic list`에 `/vision/lane_info`, `/vision/lane_image` 토픽 존재
2. ✅ `rostopic hz /vision/lane_info` 주파수 10Hz 이상
3. ✅ `confidence` 값 0.7 이상
4. ✅ 디버그 이미지에 차선이 명확하게 표시됨
5. ✅ `center_offset`이 실제 차선 위치와 일치

---

## 📈 성능 벤치마크

```bash
# CPU 사용률 확인
top -p $(pgrep -f lane_detection)

# 메모리 사용량
ps aux | grep lane_detection

# 토픽 latency 측정
rostopic delay /vision/lane_info
```

**권장 성능:**
- CPU: < 30%
- 메모리: < 200MB
- Latency: < 100ms
- 주파수: 10-30 Hz

---

## 🎓 다음 단계

테스트 성공 후:

1. **Planning 노드 연동**
   ```bash
   # path_planner_with_lane.py 실행
   rosrun wego_planning path_planner_with_lane.py
   
   # 경로 확인
   rostopic echo /planning/path
   ```

2. **파라미터 최적화**
   - 다양한 환경에서 테스트
   - 최적 파라미터 찾기
   - config 파일에 저장

3. **실제 차량 테스트**
   - 저속 주행 테스트
   - 차선 추종 성능 확인
   - 안전 기능 테스트

---

**문제가 있거나 추가 질문이 있으면 언제든 알려주세요!** 🚗💨

