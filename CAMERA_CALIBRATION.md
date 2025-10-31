# 카메라 캘리브레이션 가이드

## 📋 개요

카메라 캘리브레이션은 렌즈 왜곡을 보정하여 정확한 이미지를 얻기 위해 필요합니다.  
특히 차선 인식, 거리 측정에 중요합니다.

## 🎯 현재 상태

현재는 **기본 캘리브레이션 파일**이 제공되어 경고 없이 실행됩니다:
- `src/wego_vision/calibration/wego_camera.yaml`
- 왜곡 계수가 모두 0 (보정 없음)

**실제 사용 시 반드시 자신의 카메라로 캘리브레이션을 수행하세요!**

## 🚀 캘리브레이션 방법

### 1. 체크보드 패턴 준비

**A4 용지에 인쇄:**
```bash
# 8x6 체크보드 다운로드
wget https://github.com/opencv/opencv/raw/master/doc/pattern.png
```

또는 직접 생성:
```bash
# OpenCV로 체크보드 생성
python3 << EOF
import cv2
import numpy as np

# 8x6 체크보드 (각 칸 25mm)
board = cv2.drawChessboardCorners((800, 600), (8, 6), None, False)
cv2.imwrite('checkerboard.png', board)
EOF
```

**인쇄 시 주의사항:**
- 실제 크기로 인쇄 (비율 조정 안 함)
- 평평한 판에 부착
- 각 사각형 크기 측정 (예: 25mm)

### 2. ROS 캘리브레이션 도구 설치

```bash
sudo apt install ros-noetic-camera-calibration
```

### 3. 카메라 실행

```bash
# 터미널 1: roscore
roscore

# 터미널 2: 카메라만 실행
roslaunch wego_vision vision_pipeline.launch start_camera:=true
```

### 4. 캘리브레이션 실행

```bash
# 터미널 3: 캘리브레이션 도구
rosrun camera_calibration cameracalibrator.py \
  --size 8x6 \
  --square 0.025 \
  image:=/usb_cam/image_raw \
  camera:=/usb_cam
```

**파라미터 설명:**
- `--size 8x6`: 체크보드 내부 코너 수 (가로x세로)
- `--square 0.025`: 각 사각형 크기 (미터 단위, 25mm = 0.025m)
- `image:=/usb_cam/image_raw`: 이미지 토픽
- `camera:=/usb_cam`: 카메라 네임스페이스

### 5. 캘리브레이션 수행

**창이 열리면:**

1. **체크보드를 카메라에 보여주기**
   - 다양한 각도로 이동
   - 좌/우/상/하 이동
   - 앞/뒤 이동
   - 기울이기

2. **진행 상황 확인**
   - 화면 오른쪽에 막대 그래프
   - X, Y, Size, Skew가 모두 초록색이 되면 OK

3. **CALIBRATE 버튼 클릭**
   - 계산 시작 (몇 분 소요)
   - "Calibration Complete!" 메시지 대기

4. **SAVE 버튼 클릭**
   - `/tmp/calibrationdata.tar.gz` 저장
   - 압축 해제

5. **파일 복사**
   ```bash
   # 압축 해제
   cd /tmp
   tar -xzf calibrationdata.tar.gz
   
   # 캘리브레이션 파일 복사
   cp ost.yaml ~/autorace2025/autorace2025/src/wego_vision/calibration/wego_camera.yaml
   ```

### 6. 확인

```bash
# 캘리브레이션 파일 확인
cat ~/autorace2025/autorace2025/src/wego_vision/calibration/wego_camera.yaml

# camera_matrix와 distortion_coefficients 값이 0이 아니어야 함
```

## 🔧 캘리브레이션 파일 형식

```yaml
image_width: 640
image_height: 480
camera_name: wego_camera

camera_matrix:
  rows: 3
  cols: 3
  data: [fx, 0, cx,
         0, fy, cy,
         0, 0, 1]

distortion_coefficients:
  rows: 1
  cols: 5
  data: [k1, k2, p1, p2, k3]
```

## ⚙️ USB 카메라 경고 해결

### "Unknown control" 경고

```
white balance temperature, auto
exposure, auto
exposure absolute
unknown control
```

**원인:** 일부 저가 카메라는 자동 제어 기능을 지원하지 않음

**해결:** 이미 적용됨! `vision_pipeline.launch`에서:
```xml
<param name="autofocus" value="false"/>
<param name="autoexposure" value="false"/>
<param name="auto_white_balance" value="false"/>
```

**경고가 계속 나와도 무시해도 됩니다!** 카메라는 정상 작동합니다.

### 완전히 없애려면

```bash
# v4l2-ctl 설치
sudo apt install v4l-utils

# 카메라 지원 기능 확인
v4l2-ctl -d /dev/video0 --list-ctrls

# 지원 안 하는 기능 확인
# "white_balance_temperature_auto" 없으면 지원 안 함
```

## 🎨 캘리브레이션 품질 확인

### Before vs After

```bash
# 터미널 1: 원본 이미지
rosrun rqt_image_view rqt_image_view /usb_cam/image_raw

# 터미널 2: 보정된 이미지
rosrun rqt_image_view rqt_image_view /vision/image_rect
```

**확인 사항:**
- 이미지 가장자리가 덜 왜곡되었는가?
- 직선이 더 직선으로 보이는가?

## 🔄 LiDAR/IMU 설정

### Launch 파일에서 제어

```bash
# LiDAR/IMU 사용 (기본값)
roslaunch wego_vision vision_pipeline.launch use_lidar:=true use_imu:=true

# 비활성화
roslaunch wego_vision vision_pipeline.launch use_lidar:=false use_imu:=false
```

### 현재 상태

이미 `vision_pipeline.launch`에서 기본값이 `true`로 설정됨:
```xml
<arg name="use_lidar" default="true"/>
<arg name="use_imu" default="true"/>
```

**주의:** LiDAR/IMU 토픽이 발행되지 않으면 경고만 표시되고 계속 실행됩니다.

## ✅ 체크리스트

### 필수 (지금)
- [x] 기본 캘리브레이션 파일 생성 → **완료**
- [x] USB 카메라 경고 무시 설정 → **완료**
- [x] LiDAR/IMU true 설정 → **완료**

### 선택 (나중에)
- [ ] 실제 카메라 캘리브레이션 수행
- [ ] 캘리브레이션 품질 검증
- [ ] 다양한 조명에서 테스트

## 🐛 문제 해결

### 문제 1: "Camera calibration file not found"

**해결됨!** 이제 기본 파일이 제공됩니다.

### 문제 2: USB 카메라 경고가 계속 나옴

**정상입니다!** 카메라가 해당 기능을 지원하지 않아서 나오는 경고일 뿐, 작동에는 문제 없습니다.

### 문제 3: LiDAR/IMU 토픽 없음

```bash
# fusion_node에서 경고만 나오고 계속 실행됨
[WARN] [timestamp]: LiDAR 데이터 없음
[WARN] [timestamp]: IMU 데이터 없음

# 정상! Vision 데이터만으로 계속 실행
```

## 🎓 다음 단계

1. **지금 바로 실행**
   ```bash
   roslaunch wego_vision vision_pipeline.launch
   ```
   - 경고 무시하고 정상 작동 확인

2. **차선 인식 테스트**
   ```bash
   rosrun rqt_image_view rqt_image_view /vision/lane_image
   ```

3. **나중에 캘리브레이션**
   - 체크보드 준비
   - 정확한 캘리브레이션 수행

---

**지금은 기본 설정으로 테스트하고, 성능 개선이 필요하면 나중에 캘리브레이션하세요!** 📸

