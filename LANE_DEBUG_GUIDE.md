# 차선 인식 디버깅 가이드

## 🚨 차선이 검출되지 않을 때

### 1단계: 현재 상태 확인

```bash
# 터미널 1: Vision 파이프라인 실행
roslaunch wego_vision vision_pipeline.launch

# 터미널 2: 디버그 이미지 확인
rosrun rqt_image_view rqt_image_view
```

**rqt_image_view에서:**
1. `/usb_cam/image_raw` 선택 → 원본 이미지 확인
2. `/vision/lane_image` 선택 → 차선 검출 결과 확인

**결과가 완전히 검은색이면:** 아무것도 검출 못함!

---

## 🎯 해결 방법

### 방법 1: 실내 환경용 파라미터 (검은 테이프)

```bash
# 1. 파라미터 파일 변경
cd ~/autorace2025/autorace2025
cp src/wego_vision/config/lane_detection_params_indoor.yaml \
   src/wego_vision/config/lane_detection_params.yaml

# 2. 재실행
roslaunch wego_vision vision_pipeline.launch

# 3. 확인
rosrun rqt_image_view rqt_image_view /vision/lane_image
```

**실내용 파라미터 특징:**
- 검은색/어두운 색 검출: `white_lower: [0, 0, 0]`
- Edge 더 민감: `canny_low: 30`
- 선 검출 더 관대: `hough_threshold: 15`

### 방법 2: Edge 기반 검출 (색상 무관!) ⭐ 권장

```bash
# 실행 권한
chmod +x ~/autorace2025/autorace2025/src/wego_vision/scripts/lane_detection_edge_based.py

# 터미널 1: roscore
roscore

# 터미널 2: 카메라
rosrun wego_vision camera_node.py

# 터미널 3: Edge 기반 차선 검출
rosrun wego_vision lane_detection_edge_based.py

# 터미널 4: 결과 확인
rosrun rqt_image_view rqt_image_view /vision/lane_image
```

**장점:**
- 흰색/검은색 상관없이 작동
- Edge(모양)만 검출
- 조명 변화에 강함

### 방법 3: 파라미터 실시간 튜닝

```bash
# rqt_reconfigure 사용
rosrun rqt_reconfigure rqt_reconfigure
```

또는 수동 튜닝:

```bash
# 파라미터 파일 열기
vim ~/autorace2025/autorace2025/src/wego_vision/config/lane_detection_params.yaml
```

**튜닝 포인트:**

1. **ROI (관심 영역)**
   ```yaml
   roi_top_ratio: 0.3   # 더 넓게 (위쪽까지)
   ```

2. **색상 범위 (실내 검은 테이프)**
   ```yaml
   white_lower: [0, 0, 0]      # 검은색
   white_upper: [180, 255, 80]  # 어두운 회색까지
   ```

3. **Canny Edge (민감도)**
   ```yaml
   canny_low: 30     # 낮을수록 더 많은 edge
   canny_high: 100   # 낮을수록 약한 edge도 검출
   ```

4. **Hough Line (선 검출)**
   ```yaml
   hough_threshold: 15          # 낮을수록 더 많은 선
   hough_min_line_length: 15    # 짧은 선도 검출
   ```

---

## 🔍 단계별 디버깅

### Step 1: 원본 이미지 확인

```bash
rosrun rqt_image_view rqt_image_view /usb_cam/image_raw
```

**확인:**
- 차선이 눈으로 보이는가?
- 조명은 충분한가?
- 차선이 명확한가?

### Step 2: Edge 확인

원본 이미지에서 수동으로 Edge 확인:

```python
import cv2
import numpy as np

# 이미지 로드
img = cv2.imread('test.jpg')
gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

# Canny Edge
edges = cv2.Canny(gray, 30, 100)

# 확인
cv2.imshow('Edges', edges)
cv2.waitKey(0)
```

### Step 3: Hough Line 확인

```python
lines = cv2.HoughLinesP(edges, 1, np.pi/180, 15, 15, 200)
print(f"검출된 선: {len(lines) if lines is not None else 0}개")
```

### Step 4: 차선 정보 확인

```bash
rostopic echo /vision/lane_info

# 출력 확인:
# left_lane_detected: True/False
# right_lane_detected: True/False
# confidence: 0.0 ~ 1.0
```

---

## 📊 환경별 권장 설정

### 실내 (검은 테이프)

```yaml
white_lower: [0, 0, 0]
white_upper: [180, 255, 80]
canny_low: 30
canny_high: 100
hough_threshold: 15
```

### 실외 (흰색 도로 차선)

```yaml
white_lower: [0, 0, 200]
white_upper: [180, 30, 255]
canny_low: 50
canny_high: 150
hough_threshold: 20
```

### 어두운 환경

```yaml
roi_top_ratio: 0.4        # 더 넓게
canny_low: 20             # 더 민감
hough_threshold: 10       # 더 관대
```

### 밝은 환경

```yaml
white_lower: [0, 0, 180]  # 더 밝은 색만
canny_low: 60
canny_high: 180
```

---

## 🛠️ 빠른 테스트 스크립트

```bash
#!/bin/bash
# test_lane_detection.sh

echo "=== 차선 검출 테스트 ==="

# 1. 노드 확인
echo "1. 노드 확인..."
rosnode list | grep lane || echo "  ❌ 차선 노드 없음!"

# 2. 토픽 확인
echo "2. 토픽 확인..."
rostopic list | grep lane

# 3. 토픽 주파수
echo "3. 토픽 주파수..."
timeout 3 rostopic hz /vision/lane_info

# 4. 차선 정보
echo "4. 차선 검출 상태..."
timeout 2 rostopic echo /vision/lane_info | grep "detected\|confidence"

echo "=== 테스트 완료 ==="
```

---

## 🎨 시각화 팁

### 원본 + 검출 결과 동시 보기

```bash
# 창 4개 동시 실행
rosrun rqt_image_view rqt_image_view /usb_cam/image_raw &
rosrun rqt_image_view rqt_image_view /vision/image_rect &
rosrun rqt_image_view rqt_image_view /vision/lane_image &
```

### RViz에서 확인

```bash
rviz

# Add:
# - Image → /vision/lane_image
# - Path → /planning/path (Planning 연동 시)
```

---

## 🚀 성공 기준

다음이 모두 만족되면 성공:

1. ✅ `/vision/lane_image`에 녹색/파란색 선이 그려짐
2. ✅ `rostopic echo /vision/lane_info`에서 `confidence > 0.5`
3. ✅ `left_lane_detected: True` 또는 `right_lane_detected: True`
4. ✅ 차선이 실제 위치와 일치

---

## 💡 최종 권장사항

### 실내 테스트 (검은 테이프)

```bash
# Edge 기반 사용 (가장 안정적!)
rosrun wego_vision lane_detection_edge_based.py \
  _canny_low:=30 \
  _canny_high:=100 \
  _hough_threshold:=15
```

### 실외 테스트 (흰색 차선)

```bash
# 색상 기반 사용
roslaunch wego_vision vision_pipeline.launch
```

---

**여전히 안 되면 알려주세요! 함께 디버깅합시다.** 🔧

