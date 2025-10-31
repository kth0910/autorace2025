#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
WeGO Control - VESC Bridge Node
Ackermann 명령을 VESC 모터 컨트롤러 명령으로 변환합니다

Topic Subscriptions:
  - /ackermann_cmd (ackermann_msgs/AckermannDriveStamped): Ackermann 제어 명령

Topic Publications:
  - /commands/motor/speed (std_msgs/Float64): 모터 속도 명령
  - /commands/servo/position (std_msgs/Float64): 서보 위치 명령
"""

import rospy
import numpy as np
from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import Float64


class VESCBridgeNode:
    def __init__(self):
        rospy.init_node('vesc_bridge_node', anonymous=False)
        
        # Parameters
        self.ackermann_topic = rospy.get_param('~ackermann_topic', '/ackermann_cmd')
        self.motor_topic = rospy.get_param('~motor_topic', '/commands/motor/speed')
        self.servo_topic = rospy.get_param('~servo_topic', '/commands/servo/position')
        
        # Vehicle parameters
        self.wheelbase = rospy.get_param('~wheelbase', 0.32)  # meters
        self.steering_angle_to_servo_gain = rospy.get_param('~steering_to_servo_gain', 1.0)
        self.steering_offset = rospy.get_param('~steering_offset', 0.5)  # Servo neutral position
        
        # Speed parameters
        self.speed_to_erpm_gain = rospy.get_param('~speed_to_erpm_gain', 4000.0)
        self.speed_to_erpm_offset = rospy.get_param('~speed_to_erpm_offset', 0.0)
        
        # Safety limits
        self.max_servo_position = rospy.get_param('~max_servo_position', 1.0)
        self.min_servo_position = rospy.get_param('~min_servo_position', 0.0)
        self.max_motor_speed = rospy.get_param('~max_motor_speed', 20000.0)  # ERPM
        
        # Publishers
        self.motor_pub = rospy.Publisher(self.motor_topic, Float64, queue_size=10)
        self.servo_pub = rospy.Publisher(self.servo_topic, Float64, queue_size=10)
        
        # Subscriber
        self.ackermann_sub = rospy.Subscriber(
            self.ackermann_topic,
            AckermannDriveStamped,
            self.ackermann_callback
        )
        
        rospy.loginfo("[VESC Bridge] 초기화 완료")
        rospy.loginfo(f"  - Ackermann 입력: {self.ackermann_topic}")
        rospy.loginfo(f"  - 모터 출력: {self.motor_topic}")
        rospy.loginfo(f"  - 서보 출력: {self.servo_topic}")
        rospy.loginfo(f"  - 속도 게인: {self.speed_to_erpm_gain}")
        rospy.loginfo(f"  - 조향 게인: {self.steering_angle_to_servo_gain}")
    
    def ackermann_callback(self, msg):
        """
        Ackermann 명령을 VESC 명령으로 변환
        """
        try:
            # 조향각 → 서보 위치
            servo_position = self.steering_to_servo(msg.drive.steering_angle)
            
            # 속도 → 모터 ERPM
            motor_speed = self.speed_to_erpm(msg.drive.speed)
            
            # 명령 발행
            self.publish_motor_command(motor_speed)
            self.publish_servo_command(servo_position)
            
            rospy.logdebug(f"[VESC Bridge] 조향: {msg.drive.steering_angle:.3f} rad → {servo_position:.3f}, "
                          f"속도: {msg.drive.speed:.2f} m/s → {motor_speed:.0f} ERPM")
            
        except Exception as e:
            rospy.logerr(f"[VESC Bridge] 변환 오류: {e}")
    
    def steering_to_servo(self, steering_angle):
        """
        조향각을 서보 위치로 변환
        
        Args:
            steering_angle: 조향각 (radians), 왼쪽이 양수
        
        Returns:
            float: 서보 위치 (0.0 ~ 1.0)
        """
        # 조향각을 서보 위치로 변환
        # 중앙: 0.5, 좌회전: > 0.5, 우회전: < 0.5
        servo_position = self.steering_offset + (steering_angle * self.steering_angle_to_servo_gain)
        
        # 안전 제한
        servo_position = np.clip(servo_position, self.min_servo_position, self.max_servo_position)
        
        return servo_position
    
    def speed_to_erpm(self, speed):
        """
        속도를 ERPM (Electric RPM)으로 변환
        
        Args:
            speed: 속도 (m/s)
        
        Returns:
            float: ERPM
        """
        # 속도를 ERPM으로 변환
        erpm = speed * self.speed_to_erpm_gain + self.speed_to_erpm_offset
        
        # 안전 제한
        erpm = np.clip(erpm, -self.max_motor_speed, self.max_motor_speed)
        
        return erpm
    
    def publish_motor_command(self, motor_speed):
        """모터 속도 명령 발행"""
        msg = Float64()
        msg.data = motor_speed
        self.motor_pub.publish(msg)
    
    def publish_servo_command(self, servo_position):
        """서보 위치 명령 발행"""
        msg = Float64()
        msg.data = servo_position
        self.servo_pub.publish(msg)
    
    def run(self):
        rospy.spin()


if __name__ == '__main__':
    try:
        node = VESCBridgeNode()
        node.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("[VESC Bridge] 종료")

