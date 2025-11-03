# Vision만 사용하기 (Planning 없이)

## 📋 개요

Planning/Control 없이 **Vision 차선 검출만** 개발하는 가이드입니다.

## 🎯 단순 버전: lane_detection_simple.py

- ✅ 좌/우 차선 구분 **없음**
- ✅ ROI **하단 35%만** 사용
- ✅ Edge + 흰색 검출
- ✅ 간단한 offset 계산

## 🚀 빠른 실행

### Step 1: 빌드 (처음 한 번만)

```bash
cd ~/autorace2025/autorace2025

# Git pull (Windows에서 수정한 내용)
git pull origin main

# usb_cam 삭제 (의존성 문제)
rm -rf src/usb_cam

# 빌드
rm -rf build/ devel/
source /opt/ros/noetic/setup.bash
catkin_make
source devel/setup.bash

# 실행 권한
chmod +x src/wego_vision/scripts/*.py
```

### Step 2: 실행

```bash
# 터미널 1: roscore
roscore

# 터미널 2: 카메라 (자체 제작)
source ~/autorace2025/autorace2025/devel/setup.bash
rosrun wego_vision simple_camera_publisher.py

# 터미널 3: 단순 차선 검출 ⭐
source ~/autorace2025/autorace2025/devel/setup.bash
rosrun wego_vision lane_detection_simple.py

# 터미널 4: 시각화
rosrun rqt_image_view rqt_image_view
```

### Step 3: 확인

**rqt_image_view 드롭다운에서:**

1. `/usb_cam/image_raw` → 원본 (차선 보이나요?)
2. `/vision/debug/roi` → ROI 영역 (녹색으로 표시)
3. `/vision/debug/edges` → Edge 검출 (선 보이나요?)
4. `/vision/debug/color_mask` → 흰색 검출
5. `/vision/lane_image` → 최종 결과 (빨간 점들 + 정보)

## 📐 ROI 하단 35% 확인

`/vision/debug/roi`를 보면:
- 화면 **상단 65%**: 검은색 (무시)
- 화면 **하단 35%**: 녹색 반투명 (여기만 처리)
- **노란색 선**: ROI 경계

## 🔍 디버그 토픽

| 토픽 | 내용 |
|------|------|
| `/usb_cam/image_raw` | 원본 카메라 |
| `/vision/debug/roi` | ROI 영역 표시 |
| `/vision/debug/edges` | Edge 검출 결과 |
| `/vision/debug/color_mask` | 흰색 검출 결과 |
| `/vision/lane_image` | 최종 차선 검출 |
| `/vision/lane_info` | 차선 데이터 |

## 📊 차선 정보 확인

```bash
rostopic echo /vision/lane_info

# 출력:
# left_lane_detected: True    ← "차선 있음" 의미만
# right_lane_detected: False  ← 사용 안 함
# center_offset: 0.023        ← 중심에서 얼마나 벗어났나
# confidence: 0.9             ← 0.5 이상이면 OK
# center_lane_points: [...]   ← 검출된 점들
```

## ⚙️ 파라미터 조정

```bash
# 파라미터 파일
vim ~/autorace2025/autorace2025/src/wego_vision/config/lane_simple_params.yaml
```

**주요 파라미터:**

### 1. ROI 크기 조정
```yaml
roi_top_ratio: 0.65   # 하단 35% (현재)
roi_top_ratio: 0.5    # 하단 50% (더 넓게)
roi_top_ratio: 0.7    # 하단 30% (더 좁게)
```

### 2. Edge 민감도
```yaml
canny_low: 50     # 기본
canny_low: 30     # 더 민감 (더 많은 edge)
canny_low: 70     # 덜 민감 (강한 edge만)
```

### 3. 선 검출
```yaml
hough_threshold: 30   # 기본
hough_threshold: 20   # 더 많은 선
hough_threshold: 40   # 적은 선 (노이즈 감소)
```

### 4. 색상 가중치
```yaml
color_weight: 0.3   # Edge 70% + 색상 30%
color_weight: 0.5   # Edge 50% + 색상 50% (균형)
color_weight: 0.1   # Edge 90% + 색상 10% (Edge 위주)
```

## 🎨 실시간 파라미터 조정

```bash
# 노드 종료 (Ctrl+C)

# 새 파라미터로 재실행
rosrun wego_vision lane_detection_simple.py \
  _roi_top_ratio:=0.65 \
  _canny_low:=50 \
  _canny_high:=150 \
  _white_lower:="[0, 0, 180]" \
  _white_upper:="[180, 30, 255]" \
  _color_weight:=0.3 \
  _hough_threshold:=30
```

## 💡 차선이 안 보이면

### 1. ROI 확대
```bash
rosrun wego_vision lane_detection_simple.py _roi_top_ratio:=0.5
# 하단 50%로 확대
```

### 2. 파라미터 초민감
```bash
rosrun wego_vision lane_detection_simple.py \
  _canny_low:=30 \
  _canny_high:=100 \
  _hough_threshold:=15 \
  _color_weight:=0.5
```

### 3. 흰색 범위 확대
```bash
rosrun wego_vision lane_detection_simple.py \
  _white_lower:="[0, 0, 150]" \
  _white_upper:="[180, 50, 255]"
```

## 📋 완전 체크리스트

```bash
# 1. roscore
roscore

# 2. 카메라 (새 터미널)
source ~/autorace2025/autorace2025/devel/setup.bash
rosrun wego_vision simple_camera_publisher.py

# 3. 차선 검출 (새 터미널)
source ~/autorace2025/autorace2025/devel/setup.bash
rosrun wego_vision lane_detection_simple.py

# 4. 시각화 (새 터미널)
rosrun rqt_image_view rqt_image_view

# 드롭다운에서 순서대로:
# /usb_cam/image_raw     → 원본
# /vision/debug/roi      → ROI (녹색 영역)
# /vision/debug/edges    → Edge
# /vision/lane_image     → 최종
```

## ✅ 성공 기준

1. `/vision/debug/roi`에 **하단 35%만 녹색**
2. `/vision/debug/edges`에 **차선 모양** 보임
3. `/vision/lane_image`에 **빨간 점들** 보임
4. `rostopic echo /vision/lane_info`에서 `confidence > 0.5`

## 🎯 Git 업로드 (Windows)

```powershell
cd c:\dev\autorace2025\autorace2025

git add .
git commit -m "feat(vision): 단순 차선 검출 추가 (Vision only)

- 좌/우 구분 없음
- ROI 하단 35%만 사용
- Simple camera publisher 추가
- 디버그 토픽 추가 (roi, edges, color_mask)
- Vision만 개발용"

git push origin main
```

## 🚀 Ubuntu 실행

```bash
# Pull
git pull origin main

# usb_cam 삭제
rm -rf src/usb_cam

# 빌드
catkin_make
source devel/setup.bash

# 실행
chmod +x src/wego_vision/scripts/*.py
sudo chmod 666 /dev/video0

# 간단 실행!
rosrun wego_vision simple_camera_publisher.py &
rosrun wego_vision lane_detection_simple.py
```

---

**완전 단순화!** 좌/우 없고, ROI 하단 35%만! 🎯

```bash
rosrun wego_vision lane_detection_simple.py
```

이제 되나요? 🚗💨


