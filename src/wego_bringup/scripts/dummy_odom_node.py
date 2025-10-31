#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
WeGO Bringup - Dummy Odometry Node
시뮬레이션용 더미 Odometry 데이터를 발행합니다.

Topic Publications:
  - /odom (nav_msgs/Odometry): 더미 Odometry 데이터
"""

import rospy
import math
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
from std_msgs.msg import Header
import tf.transformations as tft


class DummyOdomNode:
    def __init__(self):
        rospy.init_node('dummy_odom_node', anonymous=False)
        
        # Parameters
        self.publish_rate = rospy.get_param('~rate', 50.0)  # Hz
        self.frame_id = rospy.get_param('~frame_id', 'odom')
        self.child_frame_id = rospy.get_param('~child_frame_id', 'base_link')
        
        # Motion parameters (원형 궤적)
        self.radius = rospy.get_param('~radius', 5.0)  # meters
        self.angular_velocity = rospy.get_param('~angular_velocity', 0.1)  # rad/s
        
        # State
        self.angle = 0.0
        
        # Publisher
        self.odom_pub = rospy.Publisher('/odom', Odometry, queue_size=50)
        
        # Timer
        self.timer = rospy.Timer(
            rospy.Duration(1.0 / self.publish_rate),
            self.publish_odom
        )
        
        rospy.loginfo("[Dummy Odom] 초기화 완료")
        rospy.loginfo(f"  - 발행 주기: {self.publish_rate} Hz")
        rospy.loginfo(f"  - 궤적 반경: {self.radius} m")
    
    def publish_odom(self, event):
        """더미 Odometry 발행"""
        # 시간 업데이트
        dt = 1.0 / self.publish_rate
        self.angle += self.angular_velocity * dt
        
        # 원형 궤적 계산
        x = self.radius * math.cos(self.angle)
        y = self.radius * math.sin(self.angle)
        theta = self.angle + math.pi / 2
        
        # 속도 계산
        vx = -self.radius * self.angular_velocity * math.sin(self.angle)
        vy = self.radius * self.angular_velocity * math.cos(self.angle)
        vtheta = self.angular_velocity
        
        # Odometry 메시지 생성
        odom = Odometry()
        odom.header.stamp = rospy.Time.now()
        odom.header.frame_id = self.frame_id
        odom.child_frame_id = self.child_frame_id
        
        # Position
        odom.pose.pose.position = Point(x, y, 0.0)
        
        # Orientation (쿼터니언)
        q = tft.quaternion_from_euler(0, 0, theta)
        odom.pose.pose.orientation = Quaternion(*q)
        
        # Velocity
        odom.twist.twist.linear = Vector3(vx, vy, 0.0)
        odom.twist.twist.angular = Vector3(0.0, 0.0, vtheta)
        
        # Covariance (간단한 예시)
        odom.pose.covariance = [0.1] * 36
        odom.twist.covariance = [0.1] * 36
        
        # 발행
        self.odom_pub.publish(odom)
    
    def run(self):
        rospy.spin()


if __name__ == '__main__':
    try:
        node = DummyOdomNode()
        node.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("[Dummy Odom] 종료")

