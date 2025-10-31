#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
WeGO Planning - Path Planner Node
전역 경로를 계획합니다 (A*, Dijkstra, RRT 등)

Topic Subscriptions:
  - /vision/fused_objects (geometry_msgs/PoseArray): 융합된 장애물 정보
  - /odom (nav_msgs/Odometry): 현재 위치 및 속도

Topic Publications:
  - /planning/path (nav_msgs/Path): 계획된 전역 경로
"""

import rospy
import numpy as np
from geometry_msgs.msg import PoseArray, PoseStamped
from nav_msgs.msg import Path, Odometry


class PathPlannerNode:
    def __init__(self):
        rospy.init_node('path_planner_node', anonymous=False)
        
        # Parameters
        self.obstacles_topic = rospy.get_param('~obstacles_topic', '/vision/fused_objects')
        self.odom_topic = rospy.get_param('~odom_topic', '/odom')
        self.output_topic = rospy.get_param('~output_topic', '/planning/path')
        self.planning_algorithm = rospy.get_param('~algorithm', 'astar')  # 'astar', 'rrt', 'dijkstra'
        self.planning_frequency = rospy.get_param('~frequency', 1.0)  # Hz
        
        # State variables
        self.current_pose = None
        self.obstacles = None
        self.goal_pose = None  # TODO: 목표 위치 설정 방법 구현
        
        # Publisher & Subscribers
        self.path_pub = rospy.Publisher(self.output_topic, Path, queue_size=10)
        self.obstacles_sub = rospy.Subscriber(self.obstacles_topic, PoseArray, self.obstacles_callback)
        self.odom_sub = rospy.Subscriber(self.odom_topic, Odometry, self.odom_callback)
        
        # Timer for periodic planning
        self.planning_timer = rospy.Timer(
            rospy.Duration(1.0 / self.planning_frequency), 
            self.planning_callback
        )
        
        rospy.loginfo("[Path Planner] 초기화 완료")
        rospy.loginfo(f"  - 장애물 입력: {self.obstacles_topic}")
        rospy.loginfo(f"  - Odometry 입력: {self.odom_topic}")
        rospy.loginfo(f"  - 경로 출력: {self.output_topic}")
        rospy.loginfo(f"  - 계획 알고리즘: {self.planning_algorithm}")
        rospy.loginfo(f"  - 계획 주기: {self.planning_frequency} Hz")
    
    def obstacles_callback(self, msg):
        """장애물 정보 업데이트"""
        self.obstacles = msg
        rospy.logdebug(f"[Path Planner] 장애물 업데이트: {len(msg.poses)}개")
    
    def odom_callback(self, msg):
        """현재 위치 업데이트"""
        self.current_pose = msg.pose.pose
        rospy.logdebug(f"[Path Planner] 위치 업데이트: ({msg.pose.pose.position.x:.2f}, {msg.pose.pose.position.y:.2f})")
    
    def planning_callback(self, event):
        """
        주기적으로 경로를 계획하고 발행
        """
        if self.current_pose is None:
            rospy.logwarn_throttle(5.0, "[Path Planner] Odometry 데이터 대기 중...")
            return
        
        try:
            # TODO: 목표 위치 설정 (웨이포인트, 맵 등에서 가져오기)
            # 현재는 더미 목표 위치 사용
            if self.goal_pose is None:
                self.goal_pose = PoseStamped()
                self.goal_pose.header.frame_id = "map"
                self.goal_pose.pose.position.x = 10.0
                self.goal_pose.pose.position.y = 0.0
                self.goal_pose.pose.orientation.w = 1.0
            
            # 경로 계획
            path = self.plan_path(self.current_pose, self.goal_pose.pose, self.obstacles)
            
            # 경로 발행
            if path:
                self.path_pub.publish(path)
                rospy.logdebug(f"[Path Planner] 경로 발행: {len(path.poses)}개 웨이포인트")
            
        except Exception as e:
            rospy.logerr(f"[Path Planner] 경로 계획 오류: {e}")
    
    def plan_path(self, start_pose, goal_pose, obstacles):
        """
        경로 계획 알고리즘 (Placeholder)
        실제 구현: A*, RRT, Dijkstra 등
        
        Args:
            start_pose: 시작 위치
            goal_pose: 목표 위치
            obstacles: 장애물 리스트
        
        Returns:
            nav_msgs/Path: 계획된 경로
        """
        # Placeholder: 직선 경로 생성
        path = Path()
        path.header.stamp = rospy.Time.now()
        path.header.frame_id = "map"
        
        # 시작점과 목표점 사이에 중간 웨이포인트 생성
        num_waypoints = 10
        for i in range(num_waypoints + 1):
            t = i / float(num_waypoints)
            
            pose_stamped = PoseStamped()
            pose_stamped.header = path.header
            
            # 선형 보간
            pose_stamped.pose.position.x = (1 - t) * start_pose.position.x + t * goal_pose.position.x
            pose_stamped.pose.position.y = (1 - t) * start_pose.position.y + t * goal_pose.position.y
            pose_stamped.pose.position.z = 0.0
            pose_stamped.pose.orientation.w = 1.0
            
            path.poses.append(pose_stamped)
        
        return path
    
    def run(self):
        rospy.spin()


if __name__ == '__main__':
    try:
        node = PathPlannerNode()
        node.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("[Path Planner] 종료")

