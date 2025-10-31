#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
WeGO Control - Controller Node
경로를 추종하기 위한 제어 명령을 생성합니다 (Stanley, PID, MPC 등)

Topic Subscriptions:
  - /planning/trajectory (nav_msgs/Path): 목표 궤적
  - /odom (nav_msgs/Odometry): 현재 위치 및 속도

Topic Publications:
  - /ackermann_cmd (ackermann_msgs/AckermannDriveStamped): 제어 명령
"""

import rospy
import numpy as np
from nav_msgs.msg import Path, Odometry
from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import PoseStamped
import tf.transformations as tft


class ControllerNode:
    def __init__(self):
        rospy.init_node('controller_node', anonymous=False)
        
        # Parameters
        self.trajectory_topic = rospy.get_param('~trajectory_topic', '/planning/trajectory')
        self.odom_topic = rospy.get_param('~odom_topic', '/odom')
        self.output_topic = rospy.get_param('~output_topic', '/ackermann_cmd')
        self.controller_type = rospy.get_param('~controller_type', 'stanley')  # 'stanley', 'pid', 'mpc'
        self.control_frequency = rospy.get_param('~frequency', 20.0)  # Hz
        
        # Vehicle parameters
        self.wheelbase = rospy.get_param('~wheelbase', 0.32)  # meters
        self.max_steering_angle = rospy.get_param('~max_steering_angle', 0.5)  # radians
        self.max_speed = rospy.get_param('~max_speed', 2.0)  # m/s
        
        # Controller parameters
        self.k_stanley = rospy.get_param('~k_stanley', 1.0)  # Stanley gain
        self.k_soft = rospy.get_param('~k_soft', 2.5)  # Softening constant
        
        # State variables
        self.trajectory = None
        self.current_pose = None
        self.current_velocity = None
        
        # Publisher & Subscribers
        self.cmd_pub = rospy.Publisher(self.output_topic, AckermannDriveStamped, queue_size=10)
        self.trajectory_sub = rospy.Subscriber(self.trajectory_topic, Path, self.trajectory_callback)
        self.odom_sub = rospy.Subscriber(self.odom_topic, Odometry, self.odom_callback)
        
        # Timer for periodic control
        self.control_timer = rospy.Timer(
            rospy.Duration(1.0 / self.control_frequency),
            self.control_callback
        )
        
        rospy.loginfo("[Controller] 초기화 완료")
        rospy.loginfo(f"  - 궤적 입력: {self.trajectory_topic}")
        rospy.loginfo(f"  - Odometry 입력: {self.odom_topic}")
        rospy.loginfo(f"  - 명령 출력: {self.output_topic}")
        rospy.loginfo(f"  - 제어기 타입: {self.controller_type}")
        rospy.loginfo(f"  - 제어 주기: {self.control_frequency} Hz")
    
    def trajectory_callback(self, msg):
        """궤적 업데이트"""
        self.trajectory = msg
        rospy.logdebug(f"[Controller] 궤적 업데이트: {len(msg.poses)}개 포인트")
    
    def odom_callback(self, msg):
        """현재 위치 및 속도 업데이트"""
        self.current_pose = msg.pose.pose
        self.current_velocity = msg.twist.twist
    
    def control_callback(self, event):
        """
        주기적으로 제어 명령을 계산하고 발행
        """
        if self.current_pose is None or self.trajectory is None:
            rospy.logwarn_throttle(5.0, "[Controller] 궤적 또는 Odometry 데이터 대기 중...")
            return
        
        if len(self.trajectory.poses) == 0:
            rospy.logwarn_throttle(5.0, "[Controller] 궤적이 비어있습니다")
            # 정지 명령 발행
            self.publish_stop_command()
            return
        
        try:
            # 제어 명령 계산
            steering_angle, speed = self.compute_control(
                self.trajectory,
                self.current_pose,
                self.current_velocity
            )
            
            # Ackermann 메시지 생성 및 발행
            self.publish_command(steering_angle, speed)
            
        except Exception as e:
            rospy.logerr(f"[Controller] 제어 계산 오류: {e}")
            self.publish_stop_command()
    
    def compute_control(self, trajectory, current_pose, velocity):
        """
        제어 명령 계산 (Stanley Controller)
        
        Args:
            trajectory: 목표 궤적
            current_pose: 현재 위치
            velocity: 현재 속도
        
        Returns:
            (steering_angle, speed): 조향각(rad), 속도(m/s)
        """
        # 가장 가까운 경로 포인트 찾기
        target_pose = self.find_nearest_point(trajectory, current_pose)
        
        if target_pose is None:
            return 0.0, 0.0
        
        # Stanley Controller
        steering_angle = self.stanley_control(current_pose, target_pose, velocity)
        
        # 속도 제어 (간단한 일정 속도)
        speed = min(self.max_speed, 1.0)  # TODO: 적응형 속도 제어
        
        return steering_angle, speed
    
    def stanley_control(self, current_pose, target_pose, velocity):
        """
        Stanley 제어기
        
        Returns:
            float: 조향각 (radians)
        """
        # 현재 위치와 방향
        current_x = current_pose.position.x
        current_y = current_pose.position.y
        
        # 쿼터니언을 오일러 각으로 변환
        orientation = current_pose.orientation
        euler = tft.euler_from_quaternion([
            orientation.x, orientation.y, orientation.z, orientation.w
        ])
        current_yaw = euler[2]
        
        # 목표 위치
        target_x = target_pose.pose.position.x
        target_y = target_pose.pose.position.y
        
        # 목표 방향
        target_orientation = target_pose.pose.orientation
        target_euler = tft.euler_from_quaternion([
            target_orientation.x, target_orientation.y,
            target_orientation.z, target_orientation.w
        ])
        target_yaw = target_euler[2]
        
        # Heading error
        heading_error = self.normalize_angle(target_yaw - current_yaw)
        
        # Cross track error
        dx = target_x - current_x
        dy = target_y - current_y
        cross_track_error = -dx * np.sin(current_yaw) + dy * np.cos(current_yaw)
        
        # 현재 속도 (최소값 설정하여 division by zero 방지)
        current_speed = max(abs(velocity.linear.x), 0.1)
        
        # Stanley control law
        steering_angle = heading_error + np.arctan2(
            self.k_stanley * cross_track_error,
            self.k_soft + current_speed
        )
        
        # 조향각 제한
        steering_angle = np.clip(steering_angle, -self.max_steering_angle, self.max_steering_angle)
        
        rospy.logdebug(f"[Controller] Heading error: {heading_error:.3f}, "
                      f"Cross track error: {cross_track_error:.3f}, "
                      f"Steering: {steering_angle:.3f}")
        
        return steering_angle
    
    def find_nearest_point(self, trajectory, current_pose):
        """
        궤적에서 가장 가까운 포인트 찾기
        """
        min_distance = float('inf')
        nearest_pose = None
        
        for pose in trajectory.poses:
            dx = pose.pose.position.x - current_pose.position.x
            dy = pose.pose.position.y - current_pose.position.y
            distance = np.sqrt(dx**2 + dy**2)
            
            if distance < min_distance:
                min_distance = distance
                nearest_pose = pose
        
        return nearest_pose
    
    def normalize_angle(self, angle):
        """각도를 -pi ~ pi 범위로 정규화"""
        while angle > np.pi:
            angle -= 2.0 * np.pi
        while angle < -np.pi:
            angle += 2.0 * np.pi
        return angle
    
    def publish_command(self, steering_angle, speed):
        """Ackermann 제어 명령 발행"""
        msg = AckermannDriveStamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "base_link"
        
        msg.drive.steering_angle = steering_angle
        msg.drive.speed = speed
        
        self.cmd_pub.publish(msg)
        rospy.logdebug(f"[Controller] 명령 발행 - 조향: {steering_angle:.3f} rad, 속도: {speed:.2f} m/s")
    
    def publish_stop_command(self):
        """정지 명령 발행"""
        self.publish_command(0.0, 0.0)
    
    def run(self):
        rospy.spin()


if __name__ == '__main__':
    try:
        node = ControllerNode()
        node.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("[Controller] 종료")

