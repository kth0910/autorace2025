# 🚀 초간단 실행 가이드 (센서 없이)

IMU나 VESC 같은 센서가 없어도 알고리즘 테스트를 할 수 있습니다!

---

## 📋 사전 준비 (처음 한 번만)

```bash
cd ~/autorace2025/autorace2025
source /opt/ros/noetic/setup.bash
catkin_make
source devel/setup.bash
chmod +x src/*/scripts/*.py
sudo chmod 666 /dev/video0
```

---

## 🎯 실행 (5개 터미널)

### Terminal 1: roscore
```bash
roscore
```

### Terminal 2: 카메라
```bash
source ~/autorace2025/autorace2025/devel/setup.bash
rosrun wego_vision simple_camera_publisher.py
```

### Terminal 3: 차선 검출
```bash
source ~/autorace2025/autorace2025/devel/setup.bash
rosrun wego_vision lane_detection_simple.py
```

### Terminal 4: Odometry (시뮬레이션)
```bash
source ~/autorace2025/autorace2025/devel/setup.bash
roslaunch wego_bringup odometry.launch simulation:=true
```

### Terminal 5: 경로 계획
```bash
source ~/autorace2025/autorace2025/devel/setup.bash
rosrun wego_planning path_planner_with_lane.py
```

---

## ✅ 확인

```bash
# 모든 토픽이 나와야 함
rostopic list

# 데이터 확인
rostopic hz /usb_cam/image_raw   # ~30 Hz
rostopic hz /vision/lane_info    # ~30 Hz
rostopic hz /odom                # 50 Hz
rostopic hz /planning/path       # ~10 Hz
```

---

## 🎨 시각화 (선택사항)

```bash
# 차선 이미지 보기
rqt_image_view /vision/lane_image

# 또는 RViz
rviz
# Add -> Image -> /vision/lane_image
# Add -> Path -> /planning/path
# Add -> Odometry -> /odom
```

---

## 🛑 종료

각 터미널에서 `Ctrl+C`

---

## ❓ 문제 해결

### Q: 카메라가 안 보여요
```bash
ls /dev/video*
sudo chmod 666 /dev/video0
```

### Q: 차선이 안 잡혀요
```bash
# 파라미터 조정
rosnode kill /lane_detection_simple
rosrun wego_vision lane_detection_simple.py \
  _roi_top_ratio:=0.6 \
  _canny_low:=30 \
  _canny_high:=100
```

### Q: planning/path가 안 나와요
```bash
# lane_info 확인
rostopic echo /vision/lane_info
# confidence가 0.5 이상이어야 함
```

### Q: 패키지를 못 찾아요
```bash
source ~/autorace2025/autorace2025/devel/setup.bash
```

---

## 📚 더 자세한 가이드

- **QUICK_RUN.txt** - 모든 실행 옵션
- **ODOMETRY_SETUP.md** - 실제 센서 사용법
- **wego_planning/README.md** - Move Base 사용법

---

**이 가이드는 센서 없이 시뮬레이션으로 실행하는 방법입니다.**  
**실제 로봇에서 사용하려면 ODOMETRY_SETUP.md를 참고하세요!**

