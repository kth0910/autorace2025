#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
WeGO Bringup - Real Odometry Node
IMU와 VESC 데이터를 융합하여 실제 Odometry를 발행합니다.

Topic Subscriptions:
  - /imu/data (sensor_msgs/Imu): IMU 센서 데이터 (각속도, 가속도)
  - /sensors/core (vesc_msgs/VescStateStamped): VESC 센서 데이터 (속도, 전류 등)
  또는
  - /commands/motor/speed (std_msgs/Float64): 모터 속도 명령 (피드백 없을 시)

Topic Publications:
  - /odom (nav_msgs/Odometry): 실제 Odometry 데이터
  - /odom_debug (geometry_msgs/PoseStamped): 디버깅용 위치 정보
"""

import rospy
import math
import numpy as np
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3, PoseStamped
import tf
import tf.transformations as tft


class RealOdomNode:
    def __init__(self):
        rospy.init_node('real_odom_node', anonymous=False)
        
        # Parameters
        self.publish_rate = rospy.get_param('~rate', 50.0)  # Hz
        self.frame_id = rospy.get_param('~frame_id', 'odom')
        self.child_frame_id = rospy.get_param('~child_frame_id', 'base_link')
        
        # Vehicle parameters
        self.wheelbase = rospy.get_param('~wheelbase', 0.32)  # meters
        self.wheel_radius = rospy.get_param('~wheel_radius', 0.05)  # meters
        
        # VESC conversion parameters
        self.erpm_to_speed_gain = rospy.get_param('~erpm_to_speed_gain', 1.0 / 4000.0)  # ERPM to m/s
        self.use_vesc_feedback = rospy.get_param('~use_vesc_feedback', False)  # VESC 피드백 사용 여부
        
        # IMU parameters
        self.use_imu_orientation = rospy.get_param('~use_imu_orientation', True)
        self.imu_angular_velocity_covariance = rospy.get_param('~imu_angular_velocity_covariance', 0.01)
        
        # Timeout parameters
        self.imu_timeout = rospy.get_param('~imu_timeout', 0.5)  # seconds
        self.vesc_timeout = rospy.get_param('~vesc_timeout', 0.5)  # seconds
        
        # State variables
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.vx = 0.0
        self.vy = 0.0
        self.vtheta = 0.0
        
        # Sensor data
        self.last_imu_time = None
        self.last_vesc_time = None
        self.current_angular_velocity = 0.0  # from IMU
        self.current_linear_velocity = 0.0   # from VESC
        
        # TF Broadcaster
        self.tf_broadcaster = tf.TransformBroadcaster()
        
        # Publishers
        self.odom_pub = rospy.Publisher('/odom', Odometry, queue_size=50)
        self.debug_pub = rospy.Publisher('/odom_debug', PoseStamped, queue_size=10)
        
        # Subscribers
        self.imu_sub = rospy.Subscriber('/imu/data', Imu, self.imu_callback, queue_size=10)
        
        if self.use_vesc_feedback:
            rospy.loginfo("[Real Odom] VESC 피드백 모드 사용 - /sensors/core 구독")
            # VESC 실제 피드백 사용 (vesc_msgs 필요)
            try:
                from vesc_msgs.msg import VescStateStamped
                self.vesc_sub = rospy.Subscriber('/sensors/core', VescStateStamped, 
                                                self.vesc_state_callback, queue_size=10)
            except ImportError:
                rospy.logwarn("[Real Odom] vesc_msgs 패키지가 없습니다. 명령 모드로 전환합니다.")
                self.use_vesc_feedback = False
                self.vesc_sub = rospy.Subscriber('/commands/motor/speed', Float64, 
                                                self.vesc_command_callback, queue_size=10)
        else:
            rospy.loginfo("[Real Odom] 모터 명령 모드 사용 - /commands/motor/speed 구독")
            # VESC 명령값 사용 (피드백 없을 때)
            self.vesc_sub = rospy.Subscriber('/commands/motor/speed', Float64, 
                                            self.vesc_command_callback, queue_size=10)
        
        # Timer
        self.last_time = rospy.Time.now()
        self.timer = rospy.Timer(
            rospy.Duration(1.0 / self.publish_rate),
            self.update_odometry
        )
        
        rospy.loginfo("[Real Odom] 초기화 완료")
        rospy.loginfo(f"  - 발행 주기: {self.publish_rate} Hz")
        rospy.loginfo(f"  - Wheelbase: {self.wheelbase} m")
        rospy.loginfo(f"  - Wheel radius: {self.wheel_radius} m")
        rospy.loginfo(f"  - IMU 방향 사용: {self.use_imu_orientation}")
    
    def imu_callback(self, msg):
        """
        IMU 데이터 콜백
        각속도(angular velocity)를 사용하여 방향 계산
        """
        self.last_imu_time = rospy.Time.now()
        
        # Z축 각속도 (yaw rate) 추출
        self.current_angular_velocity = msg.angular_velocity.z
        
        rospy.logdebug(f"[Real Odom] IMU angular velocity: {self.current_angular_velocity:.3f} rad/s")
    
    def vesc_state_callback(self, msg):
        """
        VESC 상태 피드백 콜백 (실제 센서 데이터)
        """
        self.last_vesc_time = rospy.Time.now()
        
        # VESC ERPM을 선속도로 변환
        # speed (m/s) = rpm * wheel_circumference / 60
        # 또는 ERPM을 직접 변환
        erpm = msg.state.speed  # VESC의 실제 ERPM
        self.current_linear_velocity = erpm * self.erpm_to_speed_gain
        
        rospy.logdebug(f"[Real Odom] VESC ERPM: {erpm:.0f}, Speed: {self.current_linear_velocity:.3f} m/s")
    
    def vesc_command_callback(self, msg):
        """
        VESC 명령 콜백 (피드백 없을 시)
        명령값을 실제 속도로 간주
        """
        self.last_vesc_time = rospy.Time.now()
        
        # ERPM 명령을 선속도로 변환
        erpm = msg.data
        self.current_linear_velocity = erpm * self.erpm_to_speed_gain
        
        rospy.logdebug(f"[Real Odom] VESC Command ERPM: {erpm:.0f}, Speed: {self.current_linear_velocity:.3f} m/s")
    
    def update_odometry(self, event):
        """
        Odometry 업데이트 및 발행
        Dead Reckoning 기반 위치 추정
        """
        current_time = rospy.Time.now()
        dt = (current_time - self.last_time).to_sec()
        self.last_time = current_time
        
        # 센서 타임아웃 체크
        if self.last_imu_time is None or (current_time - self.last_imu_time).to_sec() > self.imu_timeout:
            rospy.logwarn_throttle(2.0, "[Real Odom] IMU 데이터 타임아웃!")
            self.current_angular_velocity = 0.0
        
        if self.last_vesc_time is None or (current_time - self.last_vesc_time).to_sec() > self.vesc_timeout:
            rospy.logwarn_throttle(2.0, "[Real Odom] VESC 데이터 타임아웃!")
            self.current_linear_velocity = 0.0
        
        # Dead Reckoning으로 위치 업데이트
        # 각속도로 방향 업데이트
        self.theta += self.current_angular_velocity * dt
        self.theta = self.normalize_angle(self.theta)
        
        # 속도 성분 계산 (로봇 좌표계에서 전진 방향)
        delta_x = self.current_linear_velocity * math.cos(self.theta) * dt
        delta_y = self.current_linear_velocity * math.sin(self.theta) * dt
        
        # 위치 업데이트
        self.x += delta_x
        self.y += delta_y
        
        # 속도 저장 (글로벌 좌표계)
        self.vx = self.current_linear_velocity * math.cos(self.theta)
        self.vy = self.current_linear_velocity * math.sin(self.theta)
        self.vtheta = self.current_angular_velocity
        
        # Odometry 메시지 발행
        self.publish_odometry(current_time)
        
        # TF 발행
        self.publish_tf(current_time)
        
        # 디버그 정보 발행
        self.publish_debug(current_time)
    
    def publish_odometry(self, timestamp):
        """Odometry 메시지 발행"""
        odom = Odometry()
        odom.header.stamp = timestamp
        odom.header.frame_id = self.frame_id
        odom.child_frame_id = self.child_frame_id
        
        # Position
        odom.pose.pose.position = Point(self.x, self.y, 0.0)
        
        # Orientation (쿼터니언)
        q = tft.quaternion_from_euler(0, 0, self.theta)
        odom.pose.pose.orientation = Quaternion(*q)
        
        # Velocity (로봇 좌표계)
        # 로봇 좌표계에서는 항상 전진 방향이 x축
        odom.twist.twist.linear = Vector3(self.current_linear_velocity, 0.0, 0.0)
        odom.twist.twist.angular = Vector3(0.0, 0.0, self.vtheta)
        
        # Covariance 설정
        # Pose covariance (x, y, z, roll, pitch, yaw)
        odom.pose.covariance = [
            0.01, 0.0,  0.0, 0.0, 0.0, 0.0,  # x
            0.0,  0.01, 0.0, 0.0, 0.0, 0.0,  # y
            0.0,  0.0,  0.0, 0.0, 0.0, 0.0,  # z (unused)
            0.0,  0.0,  0.0, 0.0, 0.0, 0.0,  # roll (unused)
            0.0,  0.0,  0.0, 0.0, 0.0, 0.0,  # pitch (unused)
            0.0,  0.0,  0.0, 0.0, 0.0, self.imu_angular_velocity_covariance  # yaw
        ]
        
        # Twist covariance (vx, vy, vz, vroll, vpitch, vyaw)
        odom.twist.covariance = [
            0.01, 0.0,  0.0, 0.0, 0.0, 0.0,  # vx
            0.0,  0.01, 0.0, 0.0, 0.0, 0.0,  # vy
            0.0,  0.0,  0.0, 0.0, 0.0, 0.0,  # vz (unused)
            0.0,  0.0,  0.0, 0.0, 0.0, 0.0,  # vroll (unused)
            0.0,  0.0,  0.0, 0.0, 0.0, 0.0,  # vpitch (unused)
            0.0,  0.0,  0.0, 0.0, 0.0, self.imu_angular_velocity_covariance  # vyaw
        ]
        
        # 발행
        self.odom_pub.publish(odom)
    
    def publish_tf(self, timestamp):
        """TF 발행 (odom -> base_link)"""
        self.tf_broadcaster.sendTransform(
            (self.x, self.y, 0.0),
            tft.quaternion_from_euler(0, 0, self.theta),
            timestamp,
            self.child_frame_id,
            self.frame_id
        )
    
    def publish_debug(self, timestamp):
        """디버그 정보 발행"""
        debug_msg = PoseStamped()
        debug_msg.header.stamp = timestamp
        debug_msg.header.frame_id = self.frame_id
        debug_msg.pose.position = Point(self.x, self.y, 0.0)
        q = tft.quaternion_from_euler(0, 0, self.theta)
        debug_msg.pose.orientation = Quaternion(*q)
        
        self.debug_pub.publish(debug_msg)
    
    def normalize_angle(self, angle):
        """각도를 -π ~ π 범위로 정규화"""
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle
    
    def run(self):
        rospy.spin()


if __name__ == '__main__':
    try:
        node = RealOdomNode()
        node.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("[Real Odom] 종료")

