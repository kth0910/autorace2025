# Git 워크플로우 가이드

## 📋 개요

Windows에서 개발하고 Ubuntu에서 테스트하는 워크플로우입니다.

## 🔄 기본 워크플로우

### Windows에서 (개발)

```powershell
# 1. 파일 수정 (VS Code 등)
# 예: lane_detection_node.py 수정

# 2. Git 상태 확인
git status

# 3. 변경사항 추가
git add .

# 4. 커밋
git commit -m "feat(vision): 차선 인식 알고리즘 개선"

# 5. Push
git push origin main
```

### Ubuntu에서 (테스트)

```bash
# 1. Pull
cd ~/autorace2025/autorace2025
git pull origin main

# 2. 소싱 (변경사항 적용)
source devel/setup.bash

# 3. 실행
roslaunch wego_vision vision_pipeline.launch

# 4. 테스트
rostopic echo /vision/lane_info
```

## 🎯 지금 상황 (calibration 파일 추가)

### Windows에서

```powershell
cd c:\dev\autorace2025\autorace2025

# Git 상태 확인
git status

# 새 파일들 확인:
# - src/wego_vision/calibration/wego_camera.yaml
# - src/wego_vision/scripts/camera_node.py (수정됨)
# - src/wego_vision/launch/vision_pipeline.launch (수정됨)
# - 기타 문서들

# 모두 추가
git add .

# 커밋
git commit -m "feat(vision): 카메라 캘리브레이션 파일 추가 및 권한 문제 해결

- calibration/wego_camera.yaml 기본 파일 추가
- camera_node.py cv2.Mat 오류 수정 (numpy 배열 사용)
- USB 카메라 권한 오류 해결 (output=log)
- 문서 추가: CAMERA_CALIBRATION.md, FIX_CAMERA_PERMISSION.md, TROUBLESHOOTING.md"

# Push
git push origin main
```

### Ubuntu에서

```bash
cd ~/autorace2025/autorace2025

# Pull (최신 변경사항 받기)
git pull origin main

# 이제 calibration 파일이 자동으로 생성됨!
ls -l src/wego_vision/calibration/wego_camera.yaml

# 권한 부여 (한 번만)
sudo chmod 666 /dev/video0
sudo usermod -a -G video $USER

# 실행
source devel/setup.bash
roslaunch wego_vision vision_pipeline.launch
```

## 📝 Git 커밋 메시지 규칙

### 형식
```
<type>(<scope>): <subject>

<body>

<footer>
```

### Type
- `feat`: 새 기능
- `fix`: 버그 수정
- `docs`: 문서만 변경
- `style`: 코드 포맷팅 (기능 변화 없음)
- `refactor`: 리팩토링
- `test`: 테스트 추가
- `chore`: 빌드, 설정 파일 변경

### Scope
- `vision`: wego_vision 패키지
- `planning`: wego_planning 패키지
- `control`: wego_control 패키지
- `bringup`: wego_bringup 패키지

### 예시

```bash
# 새 기능
git commit -m "feat(vision): 차선 인식 노드 추가"

# 버그 수정
git commit -m "fix(vision): cv2.Mat 오류 수정"

# 문서
git commit -m "docs: 트러블슈팅 가이드 추가"

# 설정
git commit -m "chore: calibration 파일 추가"
```

## 🔀 브랜치 전략 (선택사항)

### 기본 (간단)
```bash
# main 브랜치에서 직접 작업
git add .
git commit -m "..."
git push origin main
```

### 고급 (안전)
```bash
# 기능별 브랜치 생성
git checkout -b feature/lane-detection

# 작업 후 커밋
git add .
git commit -m "feat(vision): 차선 인식 구현"

# Push
git push origin feature/lane-detection

# GitHub에서 Pull Request 생성
# 리뷰 후 main에 merge
```

## 🚫 주의사항

### .gitignore 확인

```bash
# 빌드 파일은 커밋하지 않음
build/
devel/
*.pyc
__pycache__/
.catkin_workspace
```

### 큰 파일 제외

```bash
# rosbag 파일 등은 제외
*.bag
*.bag.active
```

## 🔄 자주 사용하는 명령어

### 변경사항 확인
```bash
git status              # 변경된 파일 목록
git diff               # 변경 내용 상세
git log --oneline      # 커밋 히스토리
```

### 되돌리기
```bash
# 작업 디렉토리 변경 취소
git checkout -- <file>

# 스테이징 취소
git reset HEAD <file>

# 마지막 커밋 취소 (변경사항 유지)
git reset --soft HEAD~1

# 마지막 커밋 완전 취소
git reset --hard HEAD~1
```

### 동기화
```bash
# 원격 변경사항 확인
git fetch origin

# Pull (fetch + merge)
git pull origin main

# Push
git push origin main
```

## 💡 팁

### 1. Python 코드만 수정하면

```bash
# Windows에서
git add src/wego_vision/scripts/lane_detection_node.py
git commit -m "feat(vision): 차선 검출 알고리즘 개선"
git push

# Ubuntu에서
git pull
# catkin_make 불필요! 바로 재실행
roslaunch wego_vision vision_pipeline.launch
```

### 2. 파라미터만 수정하면

```bash
# Windows에서
git add src/wego_vision/config/lane_detection_params.yaml
git commit -m "chore(vision): 차선 검출 파라미터 튜닝"
git push

# Ubuntu에서
git pull
# 바로 재실행
roslaunch wego_vision vision_pipeline.launch
```

### 3. 메시지 파일 수정하면

```bash
# Windows에서
git add src/wego_vision/msg/LaneInfo.msg
git commit -m "feat(vision): LaneInfo 메시지 필드 추가"
git push

# Ubuntu에서
git pull
catkin_make  # ⚠️ 이 경우는 빌드 필요!
source devel/setup.bash
roslaunch wego_vision vision_pipeline.launch
```

## 📊 현재 파일 구조

```
autorace2025/
├── .git/
├── src/
│   ├── wego_vision/
│   │   ├── calibration/          # ✅ 새로 추가
│   │   │   └── wego_camera.yaml  # ✅ 새로 추가
│   │   ├── scripts/
│   │   │   ├── camera_node.py    # 🔧 수정됨
│   │   │   └── lane_detection_node.py
│   │   ├── launch/
│   │   │   └── vision_pipeline.launch  # 🔧 수정됨
│   │   └── ...
│   └── ...
├── CAMERA_CALIBRATION.md         # 📄 새 문서
├── FIX_CAMERA_PERMISSION.md      # 📄 새 문서
├── TROUBLESHOOTING.md            # 📄 새 문서
├── LANE_DETECTION_GUIDE.md       # 📄 기존
└── GIT_WORKFLOW.md               # 📄 이 파일
```

## ✅ 지금 해야 할 것

```powershell
# Windows에서
cd c:\dev\autorace2025\autorace2025

git add .
git commit -m "feat(vision): 카메라 캘리브레이션 및 권한 문제 해결

- calibration 파일 추가로 경고 제거
- cv2.Mat 오류 수정 (numpy 배열 사용)
- USB 카메라 권한 문제 해결
- 트러블슈팅 문서 추가"

git push origin main
```

```bash
# Ubuntu에서
cd ~/autorace2025/autorace2025

git pull origin main

# 권한 설정 (한 번만)
sudo chmod 666 /dev/video0

# 실행!
source devel/setup.bash
roslaunch wego_vision vision_pipeline.launch
```

---

**이제 Windows에서 개발하고 Git으로 Ubuntu에 배포하는 워크플로우 완성!** 🎉

