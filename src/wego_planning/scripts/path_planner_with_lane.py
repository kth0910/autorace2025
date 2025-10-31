#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
WeGO Planning - Path Planner with Lane Info
차선 정보를 활용한 경로 계획 예제

Topic Subscriptions:
  - /vision/lane_info (wego_vision/LaneInfo): 차선 정보
  - /odom (nav_msgs/Odometry): 현재 위치

Topic Publications:
  - /planning/path (nav_msgs/Path): 계획된 경로
"""

import rospy
import numpy as np
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import PoseStamped
from wego_vision.msg import LaneInfo


class PathPlannerWithLane:
    def __init__(self):
        rospy.init_node('path_planner_with_lane', anonymous=False)
        
        # Parameters
        self.lane_topic = rospy.get_param('~lane_topic', '/vision/lane_info')
        self.odom_topic = rospy.get_param('~odom_topic', '/odom')
        self.path_topic = rospy.get_param('~path_topic', '/planning/path')
        
        self.planning_frequency = rospy.get_param('~frequency', 10.0)  # Hz
        self.lookahead_distance = rospy.get_param('~lookahead_distance', 2.0)  # meters
        
        # State
        self.latest_lane_info = None
        self.current_pose = None
        
        # Publishers
        self.path_pub = rospy.Publisher(self.path_topic, Path, queue_size=10)
        
        # Subscribers
        self.lane_sub = rospy.Subscriber(self.lane_topic, LaneInfo, self.lane_callback)
        self.odom_sub = rospy.Subscriber(self.odom_topic, Odometry, self.odom_callback)
        
        # Timer
        self.planning_timer = rospy.Timer(
            rospy.Duration(1.0 / self.planning_frequency),
            self.planning_callback
        )
        
        rospy.loginfo("[Path Planner with Lane] 초기화 완료")
        rospy.loginfo(f"  - 차선 정보: {self.lane_topic}")
        rospy.loginfo(f"  - 경로 출력: {self.path_topic}")
    
    def lane_callback(self, msg):
        """차선 정보 업데이트"""
        self.latest_lane_info = msg
        
        if msg.left_lane_detected and msg.right_lane_detected:
            rospy.logdebug(f"[Lane] Offset: {msg.center_offset:.3f}m, "
                          f"Heading: {np.degrees(msg.heading_error):.1f}°, "
                          f"Confidence: {msg.confidence:.2f}")
    
    def odom_callback(self, msg):
        """현재 위치 업데이트"""
        self.current_pose = msg.pose.pose
    
    def planning_callback(self, event):
        """주기적으로 차선 기반 경로 생성"""
        if self.latest_lane_info is None:
            rospy.logwarn_throttle(5.0, "[Path Planner] 차선 정보 대기 중...")
            return
        
        if self.current_pose is None:
            rospy.logwarn_throttle(5.0, "[Path Planner] Odometry 대기 중...")
            return
        
        try:
            # 차선 정보로부터 경로 생성
            path = self.generate_path_from_lane(self.latest_lane_info)
            
            if path:
                self.path_pub.publish(path)
                rospy.logdebug(f"[Path Planner] 경로 발행: {len(path.poses)}개 웨이포인트")
        
        except Exception as e:
            rospy.logerr(f"[Path Planner] 경로 생성 오류: {e}")
    
    def generate_path_from_lane(self, lane_info):
        """
        차선 중심선을 따라가는 경로 생성
        
        Args:
            lane_info: LaneInfo 메시지
        
        Returns:
            nav_msgs/Path
        """
        path = Path()
        path.header.stamp = rospy.Time.now()
        path.header.frame_id = "map"
        
        # 신뢰도 확인
        if lane_info.confidence < 0.5:
            rospy.logwarn_throttle(2.0, f"[Path Planner] 낮은 신뢰도: {lane_info.confidence:.2f}")
            return None
        
        # 양쪽 차선이 모두 검출되었을 때
        if lane_info.left_lane_detected and lane_info.right_lane_detected:
            # 중심선 포인트 사용
            if lane_info.center_lane_points:
                for i, point in enumerate(lane_info.center_lane_points):
                    pose = PoseStamped()
                    pose.header = path.header
                    
                    # 이미지 좌표를 월드 좌표로 변환 (간단한 예시)
                    # 실제로는 카메라 캘리브레이션 매트릭스 사용
                    pose.pose.position.x = self.current_pose.position.x + (i * 0.2)
                    pose.pose.position.y = self.current_pose.position.y + lane_info.center_offset
                    pose.pose.position.z = 0.0
                    
                    # 방향은 차선 heading error 반영
                    pose.pose.orientation.w = 1.0
                    
                    path.poses.append(pose)
        
        # 한쪽 차선만 검출되었을 때
        elif lane_info.left_lane_detected or lane_info.right_lane_detected:
            rospy.logwarn_throttle(2.0, "[Path Planner] 한쪽 차선만 검출됨")
            # 검출된 차선을 기준으로 경로 생성 (추정)
            # TODO: 구현
        
        return path if len(path.poses) > 0 else None
    
    def run(self):
        rospy.spin()


if __name__ == '__main__':
    try:
        node = PathPlannerWithLane()
        node.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("[Path Planner with Lane] 종료")

