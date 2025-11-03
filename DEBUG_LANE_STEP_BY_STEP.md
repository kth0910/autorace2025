# 차선 검출 단계별 디버깅 가이드

## 🔴 그라데이션만 나올 때

**증상:** `/vision/lane_image`가 검은색-흰색 그라데이션만 표시

**원인:** 차선이 전혀 검출되지 않음

## 🔍 단계별 확인

### Step 1: 원본 이미지 확인 ⭐ 가장 중요!

```bash
rosrun rqt_image_view rqt_image_view
```

**드롭다운에서 차례로 확인:**

1. `/usb_cam/image_raw` → 원본 카메라 이미지
2. `/vision/image_rect` → 보정된 이미지

**질문:**
- 이미지가 제대로 나오나요?
- 흰 차선이 눈으로 보이나요?
- 화면이 너무 어둡거나 밝지 않나요?

### Step 2: 디버그 마스크 확인 (새로 추가!)

```bash
rosrun rqt_image_view rqt_image_view
```

**드롭다운에서:**

1. `/vision/debug/edges` → Edge 검출 결과 (흰색 부분이 edge)
2. `/vision/debug/color_mask` → 흰색 필터 결과
3. `/vision/debug/combined_mask` → 결합된 마스크 (이게 중요!)

**확인:**
- Edge 이미지에 차선 모양이 보이나요?
- Color mask에 차선이 보이나요?
- Combined에 선이 보이나요?

### Step 3: 차선 정보 확인

```bash
rostopic echo /vision/lane_info

# 확인:
# left_lane_detected: True/False
# right_lane_detected: True/False
# confidence: ???
```

## 🛠️ 문제별 해결

### 문제 1: 원본 이미지가 너무 어둡거나 밝음

```bash
# 카메라 밝기 조정
v4l2-ctl -d /dev/video0 -c brightness=150
v4l2-ctl -d /dev/video0 -c contrast=130

# 또는 launch 파일에 추가
```

### 문제 2: Edge가 전혀 안 보임

```bash
# Canny 임계값을 대폭 낮춤
rosrun wego_vision lane_detection_hybrid.py \
  _canny_low:=20 \
  _canny_high:=80
```

### 문제 3: Color mask가 비어있음

**흰색 범위 확대:**

```bash
rosrun wego_vision lane_detection_hybrid.py \
  _white_lower:="[0, 0, 150]" \
  _white_upper:="[180, 50, 255]"
```

### 문제 4: Combined mask에 선이 있는데 차선 검출 실패

```bash
# Hough Line 임계값 낮춤
rosrun wego_vision lane_detection_hybrid.py \
  _hough_threshold:=15 \
  _hough_min_line_length:=10
```

## 🎯 초보자용 완전 디버깅

### 터미널 1: roscore
```bash
roscore
```

### 터미널 2: 카메라
```bash
source ~/autorace2025/autorace2025/devel/setup.bash
rosrun wego_vision camera_node.py
```

### 터미널 3: 하이브리드 차선 검출 (매우 민감하게!)
```bash
source ~/autorace2025/autorace2025/devel/setup.bash

# 모든 파라미터 최대한 민감하게
rosrun wego_vision lane_detection_hybrid.py \
  _canny_low:=20 \
  _canny_high:=80 \
  _white_lower:="[0, 0, 120]" \
  _white_upper:="[180, 60, 255]" \
  _color_weight:=0.5 \
  _hough_threshold:=10 \
  _hough_min_line_length:=10 \
  _roi_top_ratio:=0.3
```

### 터미널 4: 디버그 창 4개 열기

```bash
# 1. 원본
rosrun rqt_image_view rqt_image_view /usb_cam/image_raw &

# 2. Edge 검출 결과
rosrun rqt_image_view rqt_image_view /vision/debug/edges &

# 3. 흰색 검출 결과
rosrun rqt_image_view rqt_image_view /vision/debug/color_mask &

# 4. 최종 결과
rosrun rqt_image_view rqt_image_view /vision/lane_image &
```

## 📸 예상 결과

### 정상인 경우:

**원본:** 검은 바닥 + 흰 차선 보임  
**Edges:** 차선 경계가 흰 선으로 보임  
**Color mask:** 흰 차선 부분만 흰색  
**Lane image:** 녹색(좌), 파란색(우), 빨간색(중심) 선

### 그라데이션만 나오는 경우:

**원인:** Edge도 안 나오고, Color mask도 비어있음  
**해결:** 파라미터를 극단적으로 민감하게 설정

## 🚨 긴급 테스트 스크립트

```bash
#!/bin/bash
# quick_lane_test.sh

echo "=== 차선 검출 디버깅 ==="

# 원본 이미지 확인
echo "1. 원본 이미지 창 열기..."
rosrun rqt_image_view rqt_image_view /usb_cam/image_raw &
sleep 2

# Edge 확인
echo "2. Edge 검출 창 열기..."
rosrun rqt_image_view rqt_image_view /vision/debug/edges &
sleep 2

# 색상 확인
echo "3. 색상 마스크 창 열기..."
rosrun rqt_image_view rqt_image_view /vision/debug/color_mask &
sleep 2

# 최종 결과
echo "4. 최종 결과 창 열기..."
rosrun rqt_image_view rqt_image_view /vision/lane_image &

echo ""
echo "=== 4개 창 확인하세요! ==="
echo "1. 원본: 차선이 보이나요?"
echo "2. Edges: 하얀 선이 보이나요?"
echo "3. Color: 차선 부분이 하얗나요?"
echo "4. Lane: 녹색/파란색 선이 보이나요?"
```

## 💡 즉시 시도: 극단적 파라미터

```bash
# 노드 재시작
rosnode kill /lane_detection_hybrid_node

# 초민감 설정으로 실행
rosrun wego_vision lane_detection_hybrid.py \
  _canny_low:=10 \
  _canny_high:=50 \
  _white_lower:="[0, 0, 100]" \
  _white_upper:="[180, 80, 255]" \
  _color_weight:=0.5 \
  _hough_threshold:=5 \
  _hough_min_line_length:=5 \
  _roi_top_ratio:=0.2
```

**이제 `/vision/debug/edges`를 확인하세요!**

차선 모양이 조금이라도 보이나요?

## 📊 체크리스트

다음을 확인하고 알려주세요:

- [ ] `/usb_cam/image_raw`에 이미지가 나오나요?
- [ ] 이미지에 흰 차선이 눈으로 보이나요?
- [ ] `/vision/debug/edges`에 무언가 보이나요?
- [ ] `/vision/debug/color_mask`에 무언가 보이나요?

결과를 알려주시면 정확히 해결해드릴게요! 🔍

