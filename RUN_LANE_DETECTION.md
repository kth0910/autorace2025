# 차선 인식 실행 가이드 (검은 길 + 흰 차선)

## 🎯 추천: Hybrid 모드 (Edge + 색상)

**검은 길에 흰 차선**에 최적화된 하이브리드 모드입니다.

### 빠른 실행

```bash
# 실행 권한
chmod +x ~/autorace2025/autorace2025/src/wego_vision/scripts/lane_detection_hybrid.py

# 터미널 1: roscore
roscore

# 터미널 2: 카메라
source ~/autorace2025/autorace2025/devel/setup.bash
rosrun wego_vision camera_node.py

# 터미널 3: 하이브리드 차선 검출 ⭐
rosrun wego_vision lane_detection_hybrid.py

# 터미널 4: 결과 확인
rosrun rqt_image_view rqt_image_view /vision/lane_image
```

## 🎨 하이브리드 모드 작동 원리

```
입력 이미지
    │
    ├─→ Edge 검출 (70%) ─┐
    │   - Canny Edge      │
    │   - 모양 기반        │
    │                     │
    └─→ 색상 검출 (30%) ─┤─→ 결합 → Hough Line → 차선!
        - 흰색 필터        │
        - 노이즈 제거      │
```

**장점:**
- ✅ Edge 메인 → 조명 변화에 강함
- ✅ 색상 보조 → 흰 차선만 강조
- ✅ 노이즈 감소 → 검은 길은 무시

## ⚙️ 파라미터 튜닝

### 기본 설정 (추천)

```yaml
# Edge (메인, 70%)
canny_low: 50
canny_high: 150

# 색상 (보조, 30%)
white_lower: [0, 0, 180]     # 밝은 흰색
white_upper: [180, 30, 255]
color_weight: 0.3            # 30% 가중치
```

### 환경별 조정

#### 1. 밝은 실외 (햇빛)
```yaml
canny_low: 60
canny_high: 180
white_lower: [0, 0, 200]     # 더 밝은 흰색만
color_weight: 0.2            # Edge 비중 증가
```

#### 2. 어두운 실내 (조명 약함)
```yaml
canny_low: 40
canny_high: 120
white_lower: [0, 0, 150]     # 약간 어두운 흰색도
color_weight: 0.4            # 색상 비중 증가
```

#### 3. 차선이 흐릿할 때
```yaml
hough_threshold: 20          # 낮춤 (더 많은 선 검출)
color_weight: 0.5            # 색상 비중 증가
```

#### 4. 차선이 명확할 때
```yaml
hough_threshold: 40          # 높임 (노이즈 감소)
color_weight: 0.2            # Edge 비중 증가
```

## 🚀 실행 비교

### 모드 1: 색상 기반 (기본)
```bash
rosrun wego_vision lane_detection_node.py
```
- 흰색/노란색 검출
- 조명에 민감

### 모드 2: Edge 기반
```bash
rosrun wego_vision lane_detection_edge_based.py
```
- 색상 무관
- 노이즈 많을 수 있음

### 모드 3: Hybrid (추천!) ⭐
```bash
rosrun wego_vision lane_detection_hybrid.py
```
- Edge 메인 + 색상 보조
- **검은 길 + 흰 차선에 최적**

## 📊 성능 비교

| 모드 | 조명 변화 | 노이즈 | 검은 길 + 흰 차선 | 추천도 |
|------|----------|--------|------------------|--------|
| 색상 기반 | ⚠️ 민감 | ✅ 적음 | ⚠️ 보통 | ⭐⭐ |
| Edge 기반 | ✅ 강함 | ⚠️ 많음 | ✅ 좋음 | ⭐⭐⭐ |
| **Hybrid** | ✅ 강함 | ✅ 적음 | ✅ **최고** | ⭐⭐⭐⭐⭐ |

## 🎬 완전한 테스트 절차

### Step 1: 원본 이미지 확인
```bash
rosrun rqt_image_view rqt_image_view /usb_cam/image_raw
```
- 흰 차선이 명확하게 보이나요?
- 검은 길과 대비가 좋은가요?

### Step 2: Hybrid 실행
```bash
rosrun wego_vision lane_detection_hybrid.py
```

### Step 3: 결과 확인
```bash
# 시각적 확인
rosrun rqt_image_view rqt_image_view /vision/lane_image

# 데이터 확인
rostopic echo /vision/lane_info
```

**성공 기준:**
- 녹색(좌) + 파란색(우) 선이 보임
- `confidence > 0.7`
- 차선 중앙에 빨간색 선

### Step 4: 파라미터 튜닝 (필요시)
```bash
# 파라미터 파일 수정
vim ~/autorace2025/autorace2025/src/wego_vision/config/lane_detection_params_hybrid.yaml

# 재실행
rosnode kill /lane_detection_hybrid_node
rosrun wego_vision lane_detection_hybrid.py
```

## 💡 문제 해결

### 차선이 안 보일 때

```bash
# 1. 색상 비중 증가
rosrun wego_vision lane_detection_hybrid.py \
  _color_weight:=0.5

# 2. Edge 민감도 증가
rosrun wego_vision lane_detection_hybrid.py \
  _canny_low:=40 \
  _canny_high:=120

# 3. 선 검출 관대하게
rosrun wego_vision lane_detection_hybrid.py \
  _hough_threshold:=20
```

### 노이즈가 많을 때

```bash
# Edge 비중 증가 + 임계값 증가
rosrun wego_vision lane_detection_hybrid.py \
  _color_weight:=0.2 \
  _hough_threshold:=40
```

## ✅ 최종 권장 설정

### 검은 아스팔트 + 흰 차선 (일반적)

```bash
rosrun wego_vision lane_detection_hybrid.py \
  _canny_low:=50 \
  _canny_high:=150 \
  _white_lower:="[0, 0, 180]" \
  _white_upper:="[180, 30, 255]" \
  _color_weight:=0.3 \
  _hough_threshold:=30
```

### 검은 바닥 + 흰 테이프 (실내)

```bash
rosrun wego_vision lane_detection_hybrid.py \
  _canny_low:=40 \
  _canny_high:=120 \
  _white_lower:="[0, 0, 150]" \
  _white_upper:="[180, 50, 255]" \
  _color_weight:=0.4 \
  _hough_threshold:=25
```

## 🎯 Integration with Planning

Planning 노드에서 차선 정보 사용:

```python
from wego_vision.msg import LaneInfo

def lane_callback(self, msg):
    if msg.confidence > 0.7:  # 신뢰도 확인
        # 차선 중심 오프셋
        offset = msg.center_offset  # meters
        
        # 경로 계획에 사용
        self.follow_lane(offset, msg.heading_error)
```

---

**검은 길 + 흰 차선 → Hybrid 모드가 최고입니다!** 🎯

```bash
# 바로 실행!
chmod +x ~/autorace2025/autorace2025/src/wego_vision/scripts/lane_detection_hybrid.py
rosrun wego_vision lane_detection_hybrid.py
```

