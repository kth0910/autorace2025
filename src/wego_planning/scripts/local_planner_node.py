#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
WeGO Planning - Local Planner Node
전역 경로를 기반으로 지역 궤적을 생성합니다 (DWA, TEB 등)

Topic Subscriptions:
  - /planning/path (nav_msgs/Path): 전역 경로
  - /odom (nav_msgs/Odometry): 현재 위치 및 속도
  - /vision/fused_objects (geometry_msgs/PoseArray): 장애물 정보

Topic Publications:
  - /planning/trajectory (nav_msgs/Path): 지역 궤적
"""

import rospy
import numpy as np
from geometry_msgs.msg import PoseArray, PoseStamped
from nav_msgs.msg import Path, Odometry


class LocalPlannerNode:
    def __init__(self):
        rospy.init_node('local_planner_node', anonymous=False)
        
        # Parameters
        self.global_path_topic = rospy.get_param('~global_path_topic', '/planning/path')
        self.odom_topic = rospy.get_param('~odom_topic', '/odom')
        self.obstacles_topic = rospy.get_param('~obstacles_topic', '/vision/fused_objects')
        self.output_topic = rospy.get_param('~output_topic', '/planning/trajectory')
        self.planner_type = rospy.get_param('~planner_type', 'dwa')  # 'dwa', 'teb', 'pure_pursuit'
        self.planning_frequency = rospy.get_param('~frequency', 10.0)  # Hz
        self.lookahead_distance = rospy.get_param('~lookahead_distance', 2.0)  # meters
        
        # State variables
        self.global_path = None
        self.current_pose = None
        self.current_velocity = None
        self.obstacles = None
        
        # Publisher & Subscribers
        self.trajectory_pub = rospy.Publisher(self.output_topic, Path, queue_size=10)
        self.path_sub = rospy.Subscriber(self.global_path_topic, Path, self.path_callback)
        self.odom_sub = rospy.Subscriber(self.odom_topic, Odometry, self.odom_callback)
        self.obstacles_sub = rospy.Subscriber(self.obstacles_topic, PoseArray, self.obstacles_callback)
        
        # Timer for periodic planning
        self.planning_timer = rospy.Timer(
            rospy.Duration(1.0 / self.planning_frequency),
            self.planning_callback
        )
        
        rospy.loginfo("[Local Planner] 초기화 완료")
        rospy.loginfo(f"  - 전역 경로 입력: {self.global_path_topic}")
        rospy.loginfo(f"  - Odometry 입력: {self.odom_topic}")
        rospy.loginfo(f"  - 궤적 출력: {self.output_topic}")
        rospy.loginfo(f"  - 플래너 타입: {self.planner_type}")
        rospy.loginfo(f"  - 계획 주기: {self.planning_frequency} Hz")
    
    def path_callback(self, msg):
        """전역 경로 업데이트"""
        self.global_path = msg
        rospy.logdebug(f"[Local Planner] 전역 경로 업데이트: {len(msg.poses)}개 웨이포인트")
    
    def odom_callback(self, msg):
        """현재 위치 및 속도 업데이트"""
        self.current_pose = msg.pose.pose
        self.current_velocity = msg.twist.twist
    
    def obstacles_callback(self, msg):
        """장애물 정보 업데이트"""
        self.obstacles = msg
    
    def planning_callback(self, event):
        """
        주기적으로 지역 궤적을 계획하고 발행
        """
        if self.current_pose is None or self.global_path is None:
            rospy.logwarn_throttle(5.0, "[Local Planner] 경로 또는 Odometry 데이터 대기 중...")
            return
        
        if len(self.global_path.poses) == 0:
            rospy.logwarn_throttle(5.0, "[Local Planner] 전역 경로가 비어있습니다")
            return
        
        try:
            # 지역 궤적 생성
            trajectory = self.generate_local_trajectory(
                self.global_path,
                self.current_pose,
                self.current_velocity,
                self.obstacles
            )
            
            # 궤적 발행
            if trajectory:
                self.trajectory_pub.publish(trajectory)
                rospy.logdebug(f"[Local Planner] 궤적 발행: {len(trajectory.poses)}개 포인트")
            
        except Exception as e:
            rospy.logerr(f"[Local Planner] 궤적 생성 오류: {e}")
    
    def generate_local_trajectory(self, global_path, current_pose, velocity, obstacles):
        """
        지역 궤적 생성 (Placeholder)
        실제 구현: DWA, TEB, Pure Pursuit 등
        
        Args:
            global_path: 전역 경로
            current_pose: 현재 위치
            velocity: 현재 속도
            obstacles: 장애물 리스트
        
        Returns:
            nav_msgs/Path: 지역 궤적
        """
        # Placeholder: 전역 경로에서 가까운 부분만 추출
        trajectory = Path()
        trajectory.header.stamp = rospy.Time.now()
        trajectory.header.frame_id = global_path.header.frame_id
        
        # 현재 위치에서 lookahead_distance 내의 경로만 추출
        for pose in global_path.poses:
            dx = pose.pose.position.x - current_pose.position.x
            dy = pose.pose.position.y - current_pose.position.y
            distance = np.sqrt(dx**2 + dy**2)
            
            if distance <= self.lookahead_distance:
                trajectory.poses.append(pose)
        
        # 최소한 하나의 포인트는 포함
        if len(trajectory.poses) == 0 and len(global_path.poses) > 0:
            trajectory.poses.append(global_path.poses[0])
        
        return trajectory
    
    def run(self):
        rospy.spin()


if __name__ == '__main__':
    try:
        node = LocalPlannerNode()
        node.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("[Local Planner] 종료")

