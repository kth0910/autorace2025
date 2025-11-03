# IMU 실제 데이터 사용법

myahrs_driver 저장소가 없어도 실제 IMU를 사용할 수 있습니다!

---

## 🎯 가장 쉬운 방법들 (apt 설치)

### 방법 1: imu-tools (범용, 권장) ⭐⭐⭐

```bash
sudo apt update
sudo apt install -y ros-noetic-imu-tools
```

**지원 센서:**
- MPU6050, MPU9250
- BNO055
- ADIS16448
- 대부분의 I2C/SPI IMU

**실행:**
```bash
# Terminal 1
rosrun imu_tools imu_filter_node

# Terminal 2
rostopic echo /imu/data
```

---

### 방법 2: 특정 센서별 드라이버

#### A. Raspberry Pi Sense HAT
```bash
sudo apt install -y ros-noetic-rtimulib-ros
rosrun rtimulib_ros rtimulib_ros
```

#### B. Xsens IMU
```bash
sudo apt install -y ros-noetic-xsens-driver
roslaunch xsens_driver xsens.launch
```

#### C. Phidgets IMU
```bash
sudo apt install -y ros-noetic-phidgets-imu
roslaunch phidgets_imu imu.launch
```

#### D. Bosch BNO055
```bash
sudo apt install -y ros-noetic-bno055
rosrun bno055 bno055_node
```

---

### 방법 3: Arduino + IMU 센서

Arduino에 MPU6050이나 다른 IMU를 연결한 경우:

```bash
# rosserial 설치
sudo apt install -y ros-noetic-rosserial-arduino
sudo apt install -y ros-noetic-rosserial-python

# 실행
rosrun rosserial_python serial_node.py /dev/ttyACM0
```

**Arduino 코드 (간단한 예시):**
```cpp
#include <ros.h>
#include <sensor_msgs/Imu.h>
#include <Wire.h>
#include <MPU6050.h>

ros::NodeHandle nh;
sensor_msgs::Imu imu_msg;
ros::Publisher imu_pub("imu/data", &imu_msg);

MPU6050 mpu;

void setup() {
  Wire.begin();
  mpu.initialize();
  nh.initNode();
  nh.advertise(imu_pub);
}

void loop() {
  int16_t gx, gy, gz;
  mpu.getRotation(&gx, &gy, &gz);
  
  imu_msg.angular_velocity.x = gx * 0.001;  // 스케일 조정
  imu_msg.angular_velocity.y = gy * 0.001;
  imu_msg.angular_velocity.z = gz * 0.001;
  
  imu_pub.publish(&imu_msg);
  nh.spinOnce();
  delay(10);
}
```

---

### 방법 4: 우리가 만든 간단한 IMU 퍼블리셔 ⭐

USB-시리얼로 연결된 IMU 센서용:

```bash
# 실행
source ~/autorace2025/autorace2025/devel/setup.bash
rosrun wego_bringup simple_imu_publisher.py

# 포트 지정
rosrun wego_bringup simple_imu_publisher.py _port:=/dev/ttyUSB0
rosrun wego_bringup simple_imu_publisher.py _port:=/dev/ttyACM0

# 보레이트 지정
rosrun wego_bringup simple_imu_publisher.py _baudrate:=9600
```

**지원 형식:**
- 쉼표로 구분된 값: `gx,gy,gz,ax,ay,az`
- Z축 각속도만: `0.05`

**센서 데이터 형식 확인:**
```bash
# 시리얼 모니터로 확인
sudo apt install minicom
minicom -D /dev/ttyUSB0 -b 115200
```

---

## 🔌 연결 확인

### 1. USB 포트 확인
```bash
# 연결된 USB 장치 확인
ls -l /dev/ttyUSB* /dev/ttyACM*

# 예시 출력:
# /dev/ttyUSB0  ← IMU 센서
# /dev/ttyACM0  ← Arduino
```

### 2. 권한 설정
```bash
# 임시 (재부팅 시 사라짐)
sudo chmod 666 /dev/ttyUSB0

# 영구 (사용자를 dialout 그룹에 추가)
sudo usermod -a -G dialout $USER
# 로그아웃 후 재로그인 필요
```

### 3. 센서 테스트
```bash
# IMU 데이터 확인
rostopic echo /imu/data

# 각속도 확인 (로봇 회전시켜보기)
rostopic echo /imu/data/angular_velocity/z
```

---

## 🚀 Real Odometry와 함께 사용

### 전체 실행 순서

```bash
# Terminal 1: roscore
roscore

# Terminal 2: IMU 드라이버
rosrun imu_tools imu_filter_node
# 또는
rosrun wego_bringup simple_imu_publisher.py _port:=/dev/ttyUSB0

# Terminal 3: Real Odometry
source ~/autorace2025/autorace2025/devel/setup.bash
roslaunch wego_bringup odometry.launch simulation:=false

# Terminal 4: 확인
rostopic hz /imu/data    # IMU 주기
rostopic hz /odom        # Odometry 주기
rostopic echo /odom      # 위치 데이터
```

---

## 📊 센서별 비교

| 센서 | 설치 | 정확도 | 가격 | 권장도 |
|------|------|--------|------|--------|
| MPU6050 | apt | ⭐⭐⭐ | $ | ⭐⭐⭐⭐⭐ |
| BNO055 | apt | ⭐⭐⭐⭐⭐ | $$ | ⭐⭐⭐⭐ |
| Xsens | apt | ⭐⭐⭐⭐⭐ | $$$$ | ⭐⭐⭐ |
| Arduino+IMU | rosserial | ⭐⭐⭐ | $ | ⭐⭐⭐⭐ |

---

## ⚙️ IMU 캘리브레이션

### imu-tools 사용
```bash
# 캘리브레이션 실행
rosrun imu_tools imu_calib

# 지시에 따라 센서를 6방향으로 회전
# 완료되면 캘리브레이션 파일 생성됨
```

### 수동 캘리브레이션
```bash
# 1. 센서를 평평한 곳에 놓기
# 2. 바이어스 측정
rostopic echo /imu/data > imu_raw.txt
# 10초 대기 후 Ctrl+C
# 3. 평균값 계산하여 오프셋 설정
```

---

## 🐛 문제 해결

### Q: /dev/ttyUSB0이 없어요
```bash
# USB 재연결 또는 다른 포트 확인
ls /dev/tty*

# dmesg로 확인
dmesg | grep tty
```

### Q: Permission denied
```bash
sudo chmod 666 /dev/ttyUSB0
# 또는
sudo usermod -a -G dialout $USER
```

### Q: IMU 데이터가 안 나와요
```bash
# 센서 연결 확인
rostopic list | grep imu

# 데이터 확인
rostopic echo /imu/data

# 주기 확인
rostopic hz /imu/data
```

### Q: angular_velocity.z 값이 이상해요
```bash
# 캘리브레이션 실행
rosrun imu_tools imu_calib

# 또는 스케일 조정
rosparam set /imu_filter_node/gain 0.1
```

### Q: 이것저것 다 안 돼요!
```bash
# 시뮬레이션 모드 사용 ⭐
roslaunch wego_bringup odometry.launch simulation:=true

# 센서 없이도 완벽하게 작동합니다!
```

---

## 📝 추천 순서

### 초보자
1. **시뮬레이션 모드** (센서 불필요)
2. **imu-tools** (MPU6050)
3. **Arduino + IMU**

### 중급자
1. **BNO055** (높은 정확도)
2. **Phidgets IMU**
3. **커스텀 IMU 퍼블리셔**

### 전문가
1. **Xsens IMU** (최고 정확도)
2. **EKF 필터 추가** (robot_localization)
3. **멀티센서 융합**

---

## 🎓 더 나아가기

### robot_localization으로 센서 융합
```bash
sudo apt install ros-noetic-robot-localization

# EKF 설정
roslaunch robot_localization ekf.launch
```

### RViz로 시각화
```bash
rviz
# Add -> Imu -> /imu/data
# Add -> Odometry -> /odom
# Add -> TF
```

---

**센서가 없어도 시뮬레이션 모드로 완벽하게 개발할 수 있습니다!** 🚀

하지만 실제 센서를 사용하면 더 정확한 결과를 얻을 수 있습니다.

