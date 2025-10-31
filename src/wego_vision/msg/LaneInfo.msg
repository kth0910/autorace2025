# Lane Detection Information
# 차선 인식 정보를 Planning 노드에 전달하기 위한 메시지

Header header

# 차선 검출 상태
bool left_lane_detected     # 왼쪽 차선 검출 여부
bool right_lane_detected    # 오른쪽 차선 검출 여부

# 차선 방정식 계수 (2차 다항식: y = ax^2 + bx + c)
float32 left_a
float32 left_b
float32 left_c

float32 right_a
float32 right_b
float32 right_c

# 차선 중심선 정보
float32 center_offset       # 차량 중심으로부터 차선 중심까지의 거리 (meter, 왼쪽 +)
float32 heading_error       # 차선 방향과 차량 방향의 각도 차이 (radian)

# 차선 곡률
float32 curvature           # 차선의 곡률 (1/meter)

# 차선 포인트들 (시각화용)
geometry_msgs/Point[] left_lane_points
geometry_msgs/Point[] right_lane_points
geometry_msgs/Point[] center_lane_points

# 신뢰도
float32 confidence          # 차선 검출 신뢰도 (0.0 ~ 1.0)

