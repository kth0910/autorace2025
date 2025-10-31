#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
WeGO Vision - Fusion Node
비전, LiDAR, IMU 센서 데이터를 융합하여 통합된 환경 인식 정보를 제공합니다.

Topic Subscriptions:
  - /vision/obstacles (geometry_msgs/PoseArray): 비전 장애물 정보
  - /scan (sensor_msgs/LaserScan): LiDAR 데이터 (선택적)
  - /imu (sensor_msgs/Imu): IMU 데이터 (선택적)

Topic Publications:
  - /vision/fused_objects (geometry_msgs/PoseArray): 융합된 객체 정보
"""

import rospy
from geometry_msgs.msg import PoseArray, Pose
from sensor_msgs.msg import LaserScan, Imu
import message_filters


class FusionNode:
    def __init__(self):
        rospy.init_node('fusion_node', anonymous=False)
        
        # Parameters
        self.vision_topic = rospy.get_param('~vision_topic', '/vision/obstacles')
        self.output_topic = rospy.get_param('~output_topic', '/vision/fused_objects')
        self.use_lidar = rospy.get_param('~use_lidar', False)
        self.use_imu = rospy.get_param('~use_imu', False)
        
        # 최신 센서 데이터 저장
        self.latest_vision = None
        self.latest_lidar = None
        self.latest_imu = None
        
        # Publisher
        self.fused_pub = rospy.Publisher(self.output_topic, PoseArray, queue_size=10)
        
        # Subscribers
        self.vision_sub = rospy.Subscriber(self.vision_topic, PoseArray, self.vision_callback)
        
        if self.use_lidar:
            self.lidar_sub = rospy.Subscriber('/scan', LaserScan, self.lidar_callback)
        
        if self.use_imu:
            self.imu_sub = rospy.Subscriber('/imu', Imu, self.imu_callback)
        
        rospy.loginfo("[Fusion Node] 초기화 완료")
        rospy.loginfo(f"  - 비전 입력: {self.vision_topic}")
        rospy.loginfo(f"  - LiDAR 사용: {self.use_lidar}")
        rospy.loginfo(f"  - IMU 사용: {self.use_imu}")
        rospy.loginfo(f"  - 출력 토픽: {self.output_topic}")
    
    def vision_callback(self, msg):
        """비전 장애물 데이터 수신"""
        self.latest_vision = msg
        self.fuse_and_publish()
    
    def lidar_callback(self, msg):
        """LiDAR 데이터 수신"""
        self.latest_lidar = msg
    
    def imu_callback(self, msg):
        """IMU 데이터 수신"""
        self.latest_imu = msg
    
    def fuse_and_publish(self):
        """
        센서 데이터를 융합하여 발행
        실제 구현: Kalman Filter, Particle Filter 등
        """
        if self.latest_vision is None:
            return
        
        try:
            # TODO: 실제 센서 융합 알고리즘 구현
            # 현재는 비전 데이터를 그대로 전달 (Placeholder)
            fused_objects = self.latest_vision
            
            # 융합 결과 발행
            self.fused_pub.publish(fused_objects)
            
            rospy.logdebug(f"[Fusion Node] 융합 완료: {len(fused_objects.poses)}개 객체")
            
        except Exception as e:
            rospy.logerr(f"[Fusion Node] 센서 융합 오류: {e}")
    
    def run(self):
        rospy.spin()


if __name__ == '__main__':
    try:
        node = FusionNode()
        node.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("[Fusion Node] 종료")

