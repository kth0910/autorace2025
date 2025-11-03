# GitHub 계정 없이 패키지 다운로드하는 방법

`git clone` 시 계정을 요구하거나 `apt install`이 안 될 때 사용하세요!

---

## 🎯 가장 간단한 방법: 센서 없이 실행

**센서가 없거나 설치가 복잡하면 시뮬레이션 모드를 사용하세요!**

```bash
# IMU, VESC 없이 실행
roslaunch wego_bringup odometry.launch simulation:=true
```

✅ 센서 불필요  
✅ 설치 불필요  
✅ 모든 알고리즘 테스트 가능

---

## 📥 방법 1: 웹 브라우저로 ZIP 다운로드 (추천) ⭐

### IMU 드라이버 (myahrs_driver)

1. **웹 브라우저 열기**
   - https://github.com/withrobot/myahrs_driver

2. **ZIP 다운로드**
   - 녹색 **"Code"** 버튼 클릭
   - **"Download ZIP"** 클릭

3. **압축 풀기**
   ```bash
   cd ~/Downloads
   unzip myahrs_driver-master.zip
   ```

4. **catkin_ws로 복사**
   ```bash
   cp -r myahrs_driver-master ~/catkin_ws/src/myahrs_driver
   ```

5. **빌드**
   ```bash
   cd ~/catkin_ws
   catkin_make
   source devel/setup.bash
   ```

### VESC 드라이버

1. **웹 브라우저 열기**
   - https://github.com/mit-racecar/vesc

2. **ZIP 다운로드**
   - 녹색 **"Code"** 버튼 클릭
   - **"Download ZIP"** 클릭

3. **압축 풀기 및 복사**
   ```bash
   cd ~/Downloads
   unzip vesc-master.zip
   cp -r vesc-master ~/catkin_ws/src/vesc
   ```

4. **빌드**
   ```bash
   cd ~/catkin_ws
   rosdep install --from-paths src --ignore-src -r -y
   catkin_make
   source devel/setup.bash
   ```

---

## 🔧 방법 2: Git 설정 초기화 후 재시도

```bash
# Git 자격 증명 헬퍼 비활성화
git config --global credential.helper ""

# 다시 시도
cd ~/catkin_ws/src
git clone https://github.com/withrobot/myahrs_driver.git
```

**계정 요구하면:**
- 그냥 **Enter 여러 번** 눌러서 취소
- 방법 1 사용 (ZIP 다운로드)

---

## 🚫 방법 3: Git 완전히 안 쓰기

### Ubuntu에서 wget으로 다운로드

```bash
# myahrs_driver
cd ~/catkin_ws/src
wget https://github.com/withrobot/myahrs_driver/archive/refs/heads/master.zip
unzip master.zip
mv myahrs_driver-master myahrs_driver
rm master.zip

# VESC
wget https://github.com/mit-racecar/vesc/archive/refs/heads/master.zip -O vesc.zip
unzip vesc.zip
mv vesc-master vesc
rm vesc.zip

# 빌드
cd ~/catkin_ws
catkin_make
```

---

## ❓ 자주 묻는 질문

### Q1: apt install이 "unable to locate package" 나와요
**A:** 해당 패키지가 공식 저장소에 없습니다. 위 방법 1 사용하세요.

### Q2: git clone이 계속 계정을 요구해요
**A:** 
1. Ctrl+C로 취소
2. 방법 1 (ZIP 다운로드) 사용
3. 또는 방법 2 (Git 설정 초기화)

### Q3: 이것도 저것도 안 돼요
**A:** 센서 없이 시뮬레이션 모드 사용!
```bash
roslaunch wego_bringup odometry.launch simulation:=true
```

### Q4: catkin_make 시 에러가 나요
**A:** 
```bash
# 의존성 설치
cd ~/catkin_ws
rosdep install --from-paths src --ignore-src -r -y

# 클린 빌드
catkin_make clean
catkin_make
```

### Q5: 센서가 꼭 필요한가요?
**A:** 아니요! 시뮬레이션 모드로 모든 알고리즘을 테스트할 수 있습니다.

---

## 📸 스크린샷 가이드

### GitHub에서 ZIP 다운로드하는 방법

```
1. 브라우저에서 GitHub 저장소 열기
   
2. 화면 중간쯤에 녹색 "Code" 버튼 찾기
   
3. 클릭하면 나오는 메뉴에서 "Download ZIP" 선택
   
4. Downloads 폴더에 저장됨
```

---

## ✅ 확인 방법

### 설치 확인
```bash
# myahrs_driver 확인
rospack find myahrs_driver

# VESC 확인
rospack find vesc_driver

# 정상이면 경로가 출력됨
# 예: /home/user/catkin_ws/src/myahrs_driver
```

### 실행 확인
```bash
# IMU
roslaunch myahrs_driver myahrs_driver.launch
# 다른 터미널에서
rostopic list | grep imu

# VESC
roslaunch vesc_driver vesc_driver_node.launch
# 다른 터미널에서
rostopic list | grep vesc
```

---

## 🎉 결론

**가장 쉬운 방법:**

1. **센서가 없거나 설치 복잡** → 시뮬레이션 모드 ⭐
   ```bash
   roslaunch wego_bringup odometry.launch simulation:=true
   ```

2. **센서는 있는데 설치만 안 됨** → ZIP 다운로드
   - 웹 브라우저로 GitHub 접속
   - Download ZIP
   - 압축 풀고 catkin_ws/src로 복사
   - catkin_make

3. **다 귀찮음** → 시뮬레이션 모드 사용!

---

**계정 없이도 충분히 가능합니다!** 🚀

