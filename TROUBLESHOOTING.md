# WeGO Vision 트러블슈팅 가이드

## 🔴 자주 발생하는 오류

### 1. cv2.Mat 오류

**오류 메시지:**
```
AttributeError: module 'cv2' has no attribute 'Mat'
AttributeError: module 'cv2' has no attribute 'CV_64F'
```

**원인:** Python OpenCV에서는 `cv2.Mat` 대신 numpy 배열을 사용해야 함

**해결:** ✅ 이미 수정됨! camera_node.py에서 numpy 배열로 변경

**확인:**
```bash
# camera_node.py 재실행
rosnode kill /camera_node
rosrun wego_vision camera_node.py
```

---

### 2. USB 카메라 권한 오류

**오류 메시지:**
```
VIDIOC_S_CTRL: Permission denied
```

**빠른 해결:**
```bash
# 임시 권한 부여
sudo chmod 666 /dev/video0

# 영구 권한 (재로그인 필요)
sudo usermod -a -G video $USER
# 로그아웃 후 재로그인
```

**영구 해결 (udev 규칙):**
```bash
# udev 규칙 생성
sudo bash -c 'cat > /etc/udev/rules.d/99-camera.rules << EOF
KERNEL=="video[0-9]*", MODE="0666"
EOF'

# 규칙 적용
sudo udevadm control --reload-rules
sudo udevadm trigger
```

**자세한 내용:** `FIX_CAMERA_PERMISSION.md` 참조

---

### 3. Calibration 파일 로드 실패

**오류 메시지:**
```
[ERROR] [Camera Node] 캘리브레이션 로드 실패: ...
```

**원인:** calibration 디렉토리 또는 파일이 없음

**해결:**
```bash
# calibration 디렉토리 생성
mkdir -p ~/autorace2025/autorace2025/src/wego_vision/calibration

# 기본 파일 생성 (FIX_CAMERA_PERMISSION.md 참조)
cat > ~/autorace2025/autorace2025/src/wego_vision/calibration/wego_camera.yaml << 'EOF'
image_width: 640
image_height: 480
camera_name: wego_camera
camera_matrix:
  rows: 3
  cols: 3
  data: [500.0, 0.0, 320.0, 0.0, 500.0, 240.0, 0.0, 0.0, 1.0]
distortion_coefficients:
  rows: 1
  cols: 5
  data: [0.0, 0.0, 0.0, 0.0, 0.0]
distortion_model: plumb_bob
EOF
```

---

### 4. "Unknown control" 경고

**경고 메시지:**
```
unknown control 'white_balance_temperature_auto'
unknown control 'exposure_auto'
```

**원인:** 카메라가 해당 제어 기능을 지원하지 않음

**해결:** ✅ 이미 처리됨!
- launch 파일에서 `output="log"`로 변경 → 화면에 출력 안 됨
- 카메라는 정상 작동함 (무시해도 됨)

**로그 확인:**
```bash
# 필요하면 로그 파일 확인
cat ~/.ros/log/latest/usb_cam-*.log
```

---

### 5. 카메라 이미지가 안 나옴

**증상:** `/usb_cam/image_raw` 토픽이 발행되지 않음

**확인:**
```bash
# 카메라 디바이스 확인
ls /dev/video*

# 토픽 확인
rostopic list | grep image

# 토픽 주파수 확인
rostopic hz /usb_cam/image_raw
```

**해결:**
```bash
# 1. 카메라 재연결
# USB 케이블 뽑았다 꽂기

# 2. 카메라 권한 확인
ls -l /dev/video0
# crw-rw-rw- 여야 함

# 3. usb_cam 노드 재시작
rosnode kill /usb_cam
roslaunch wego_vision vision_pipeline.launch
```

---

### 6. 차선이 검출되지 않음

**증상:** `/vision/lane_info`의 confidence가 계속 0

**확인:**
```bash
# 디버그 이미지 확인
rosrun rqt_image_view rqt_image_view /vision/lane_image

# 원본 이미지 확인
rosrun rqt_image_view rqt_image_view /usb_cam/image_raw
```

**해결:**
```bash
# 파라미터 튜닝
vim ~/autorace2025/autorace2025/src/wego_vision/config/lane_detection_params.yaml

# 주요 파라미터:
# - white_lower/upper: 색상 범위
# - canny_low/high: Edge 검출 민감도
# - roi_top_ratio: ROI 영역

# 재실행
roslaunch wego_vision vision_pipeline.launch
```

**자세한 내용:** `LANE_DETECTION_GUIDE.md` 참조

---

### 7. LiDAR/IMU 경고

**경고 메시지:**
```
[WARN] [Fusion Node] LiDAR 데이터 대기 중...
[WARN] [Fusion Node] IMU 데이터 대기 중...
```

**원인:** 실제 센서가 연결되지 않음

**해결:** 
**정상입니다!** Vision 데이터만으로도 계속 실행됩니다.

센서 없이 실행하려면:
```bash
roslaunch wego_vision vision_pipeline.launch use_lidar:=false use_imu:=false
```

---

### 8. catkin_make 오류

**오류:** `Could not find package`

**해결:**
```bash
# ROS 환경 소싱
source /opt/ros/noetic/setup.bash

# 클린 빌드
cd ~/autorace2025/autorace2025
rm -rf build/ devel/
catkin_make

# 워크스페이스 소싱
source devel/setup.bash
```

---

### 9. Python import 오류

**오류 메시지:**
```
ImportError: No module named 'wego_vision.msg'
```

**원인:** 메시지가 빌드되지 않았거나 소싱 안 됨

**해결:**
```bash
# 메시지 포함 빌드
cd ~/autorace2025/autorace2025
catkin_make

# 소싱
source devel/setup.bash

# 메시지 확인
rosmsg show wego_vision/LaneInfo
```

---

### 10. 노드가 시작되지 않음

**오류:** `Permission denied`

**해결:**
```bash
# 실행 권한 부여
cd ~/autorace2025/autorace2025
find src -name "*.py" -type f -exec chmod +x {} \;

# 특정 노드만
chmod +x src/wego_vision/scripts/lane_detection_node.py
```

---

## 🛠️ 일반적인 디버깅 절차

### 1. 로그 확인
```bash
# rosout 로그
rostopic echo /rosout

# 특정 노드 로그
rosnode info /camera_node
```

### 2. 토픽 확인
```bash
# 모든 토픽
rostopic list

# 토픽 타입
rostopic type /vision/lane_info

# 토픽 주파수
rostopic hz /vision/lane_info

# 토픽 내용
rostopic echo /vision/lane_info
```

### 3. 노드 확인
```bash
# 실행 중인 노드
rosnode list

# 노드 정보
rosnode info /lane_detection_node

# 노드 재시작
rosnode kill /lane_detection_node
rosrun wego_vision lane_detection_node.py
```

### 4. 그래프 확인
```bash
# 노드 연결 그래프
rqt_graph

# TF 트리
rosrun rqt_tf_tree rqt_tf_tree
```

---

## 🆘 도움 요청 시 포함할 정보

1. **오류 메시지** (전체)
2. **실행 명령어**
3. **ROS 버전**: `rosversion -d`
4. **토픽 리스트**: `rostopic list`
5. **노드 리스트**: `rosnode list`
6. **로그**: `rostopic echo /rosout | grep ERROR`

---

**여전히 문제가 있으면 알려주세요!** 🚗💨

