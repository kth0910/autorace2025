#!/bin/bash
# Vision + Planning 일괄 실행 스크립트

# 색상 코드
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

echo -e "${GREEN}========================================${NC}"
echo -e "${GREEN}WeGO Vision + Planning 실행${NC}"
echo -e "${GREEN}========================================${NC}"

# 워크스페이스 소싱
cd ~/autorace2025/autorace2025
source devel/setup.bash

# 실행
echo -e "\n${YELLOW}[1/5] 카메라 시작...${NC}"
rosrun wego_vision simple_camera_publisher.py &
CAMERA_PID=$!
sleep 2

echo -e "${YELLOW}[2/5] 이미지 보정 시작...${NC}"
rosrun wego_vision camera_node.py &
CAMERA_NODE_PID=$!
sleep 2

echo -e "${YELLOW}[3/5] 차선 검출 시작...${NC}"
rosrun wego_vision lane_detection_simple.py &
LANE_PID=$!
sleep 2

echo -e "${YELLOW}[4/5] Odometry 시작...${NC}"
rosrun wego_bringup dummy_odom_node.py &
ODOM_PID=$!
sleep 2

echo -e "${YELLOW}[5/5] 경로 계획 시작...${NC}"
rosrun wego_planning path_planner_with_lane.py &
PLANNER_PID=$!
sleep 2

echo -e "\n${GREEN}========================================${NC}"
echo -e "${GREEN}모든 노드 실행 완료!${NC}"
echo -e "${GREEN}========================================${NC}"

echo -e "\n${YELLOW}실행 중인 노드:${NC}"
rosnode list | grep -E "camera|lane|odom|planner"

echo -e "\n${YELLOW}주요 토픽:${NC}"
rostopic list | grep -E "image_raw|lane_info|odom|path"

echo -e "\n${YELLOW}시각화 명령어:${NC}"
echo "rosrun rqt_image_view rqt_image_view /vision/lane_image"

echo -e "\n${YELLOW}종료하려면:${NC}"
echo "rosnode kill -a"

# 종료 대기
wait



