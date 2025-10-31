# USB 카메라 권한 문제 해결

## 🔴 오류 메시지

```
VIDIOC_S_CTRL: Permission denied
```

## 🛠️ 해결 방법

### 방법 1: 임시 권한 부여 (빠름)

```bash
# /dev/video0에 권한 부여
sudo chmod 666 /dev/video0

# 여러 카메라가 있으면
sudo chmod 666 /dev/video*
```

**단점:** 재부팅하면 다시 설정해야 함

### 방법 2: udev 규칙 추가 (영구적) ⭐ 권장

```bash
# udev 규칙 파일 생성
sudo nano /etc/udev/rules.d/99-camera.rules
```

**파일 내용:**
```
# USB 카메라 권한 설정
KERNEL=="video[0-9]*", MODE="0666"
SUBSYSTEM=="video4linux", MODE="0666"
```

**저장 후:**
```bash
# udev 규칙 재로드
sudo udevadm control --reload-rules
sudo udevadm trigger

# 카메라 재연결 또는 재부팅
```

### 방법 3: 사용자를 video 그룹에 추가 (권장)

```bash
# 현재 사용자를 video 그룹에 추가
sudo usermod -a -G video $USER

# 그룹 확인
groups $USER

# 로그아웃 후 재로그인 (필수!)
# 또는 재부팅
```

**재로그인 후 확인:**
```bash
groups
# 출력에 'video'가 있어야 함
```

## ✅ 빠른 해결 (지금 바로)

```bash
# 1. 권한 부여
sudo chmod 666 /dev/video0

# 2. 사용자를 video 그룹에 추가 (영구적)
sudo usermod -a -G video $USER

# 3. 로그아웃 후 재로그인

# 4. Vision 파이프라인 실행
roslaunch wego_vision vision_pipeline.launch
```

## 🧪 확인

```bash
# 카메라 권한 확인
ls -l /dev/video0

# 정상:
# crw-rw-rw- 1 root video ... /dev/video0
#        ^^^ 읽기/쓰기 가능

# 그룹 확인
groups
# 출력에 'video' 있어야 함

# 카메라 테스트
v4l2-ctl -d /dev/video0 --list-formats-ext
```

## 📝 요약

| 방법 | 속도 | 영구성 | 권장 |
|------|------|--------|------|
| chmod 666 | ⚡ 즉시 | ❌ 재부팅 시 초기화 | 테스트용 |
| udev 규칙 | 🔄 재부팅 필요 | ✅ 영구적 | ⭐ 권장 |
| video 그룹 | 🔄 재로그인 필요 | ✅ 영구적 | ⭐ 권장 |

**가장 좋은 방법: video 그룹 추가 + udev 규칙**

