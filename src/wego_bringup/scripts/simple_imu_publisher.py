#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
WeGO Bringup - Simple IMU Publisher
시리얼 포트로 연결된 일반적인 IMU 센서 데이터를 발행합니다.

Topic Publications:
  - /imu/data (sensor_msgs/Imu): IMU 센서 데이터

사용법:
  rosrun wego_bringup simple_imu_publisher.py
  rosrun wego_bringup simple_imu_publisher.py _port:=/dev/ttyUSB0
"""

import rospy
import serial
import math
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion, Vector3


class SimpleIMUPublisher:
    def __init__(self):
        rospy.init_node('simple_imu_publisher', anonymous=False)
        
        # Parameters
        self.port = rospy.get_param('~port', '/dev/ttyUSB0')
        self.baudrate = rospy.get_param('~baudrate', 115200)
        self.publish_rate = rospy.get_param('~rate', 100.0)  # Hz
        self.frame_id = rospy.get_param('~frame_id', 'imu_link')
        
        # Serial connection
        self.ser = None
        self.connect_serial()
        
        # Publisher
        self.imu_pub = rospy.Publisher('/imu/data', Imu, queue_size=50)
        
        # State
        self.gyro_z = 0.0  # rad/s
        
        rospy.loginfo(f"[Simple IMU] 초기화 완료")
        rospy.loginfo(f"  - 포트: {self.port}")
        rospy.loginfo(f"  - 보레이트: {self.baudrate}")
        rospy.loginfo(f"  - 발행 주기: {self.publish_rate} Hz")
    
    def connect_serial(self):
        """시리얼 포트 연결"""
        try:
            self.ser = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                timeout=1.0
            )
            rospy.loginfo(f"[Simple IMU] {self.port} 연결 성공")
        except serial.SerialException as e:
            rospy.logerr(f"[Simple IMU] 시리얼 연결 실패: {e}")
            rospy.logerr(f"[Simple IMU] 포트를 확인하세요: ls /dev/ttyUSB* /dev/ttyACM*")
            rospy.logerr(f"[Simple IMU] 권한 확인: sudo chmod 666 {self.port}")
            self.ser = None
    
    def parse_imu_data(self, line):
        """
        IMU 데이터 파싱
        
        예상 형식 (쉼표 구분):
        gx,gy,gz,ax,ay,az
        또는
        gyro_z
        
        각 센서마다 형식이 다르므로 수정 필요
        """
        try:
            parts = line.strip().split(',')
            
            if len(parts) >= 3:
                # 각속도 (gyroscope) - rad/s
                gx = float(parts[0])
                gy = float(parts[1])
                gz = float(parts[2])
                self.gyro_z = gz
                
                return (gx, gy, gz, 0, 0, 0)
            
            elif len(parts) == 1:
                # 각속도 Z축만
                self.gyro_z = float(parts[0])
                return (0, 0, self.gyro_z, 0, 0, 0)
            
        except (ValueError, IndexError) as e:
            rospy.logdebug(f"[Simple IMU] 파싱 오류: {e}, 데이터: {line}")
            return None
        
        return None
    
    def read_and_publish(self):
        """시리얼 데이터 읽고 발행"""
        rate = rospy.Rate(self.publish_rate)
        
        while not rospy.is_shutdown():
            if self.ser is None or not self.ser.is_open:
                rospy.logwarn_throttle(5.0, "[Simple IMU] 시리얼 포트가 닫혀있습니다")
                rate.sleep()
                continue
            
            try:
                # 시리얼 데이터 읽기
                if self.ser.in_waiting > 0:
                    line = self.ser.readline().decode('utf-8', errors='ignore')
                    data = self.parse_imu_data(line)
                    
                    if data:
                        gx, gy, gz, ax, ay, az = data
                        
                        # IMU 메시지 생성
                        imu_msg = Imu()
                        imu_msg.header.stamp = rospy.Time.now()
                        imu_msg.header.frame_id = self.frame_id
                        
                        # Angular velocity
                        imu_msg.angular_velocity = Vector3(gx, gy, gz)
                        imu_msg.angular_velocity_covariance = [
                            0.01, 0, 0,
                            0, 0.01, 0,
                            0, 0, 0.01
                        ]
                        
                        # Linear acceleration
                        imu_msg.linear_acceleration = Vector3(ax, ay, az)
                        imu_msg.linear_acceleration_covariance = [
                            0.01, 0, 0,
                            0, 0.01, 0,
                            0, 0, 0.01
                        ]
                        
                        # Orientation (unknown)
                        imu_msg.orientation = Quaternion(0, 0, 0, 1)
                        imu_msg.orientation_covariance[0] = -1  # unknown
                        
                        # 발행
                        self.imu_pub.publish(imu_msg)
                        
                        rospy.logdebug(f"[Simple IMU] gz={gz:.3f} rad/s")
                
            except serial.SerialException as e:
                rospy.logerr(f"[Simple IMU] 시리얼 읽기 오류: {e}")
                self.ser.close()
                self.ser = None
            
            except Exception as e:
                rospy.logerr(f"[Simple IMU] 오류: {e}")
            
            rate.sleep()
    
    def run(self):
        """메인 루프"""
        if self.ser is None:
            rospy.logerr("[Simple IMU] 시리얼 포트 연결 실패. 시뮬레이션 모드를 사용하세요:")
            rospy.logerr("  roslaunch wego_bringup odometry.launch simulation:=true")
            return
        
        try:
            self.read_and_publish()
        except rospy.ROSInterruptException:
            pass
        finally:
            if self.ser and self.ser.is_open:
                self.ser.close()
            rospy.loginfo("[Simple IMU] 종료")


if __name__ == '__main__':
    try:
        node = SimpleIMUPublisher()
        node.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("[Simple IMU] 종료")

