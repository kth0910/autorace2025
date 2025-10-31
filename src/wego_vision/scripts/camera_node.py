#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
WeGO Vision - Camera Node
카메라 이미지를 구독하고 전처리된 이미지를 발행합니다.

Topic Subscriptions:
  - /usb_cam/image_raw (sensor_msgs/Image): 원본 카메라 이미지

Topic Publications:
  - /vision/image_rect (sensor_msgs/Image): 보정된 카메라 이미지
"""

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import yaml
import os


class CameraNode:
    def __init__(self):
        rospy.init_node('camera_node', anonymous=False)
        
        # Parameters
        self.camera_topic = rospy.get_param('~camera_topic', '/usb_cam/image_raw')
        self.output_topic = rospy.get_param('~output_topic', '/vision/image_rect')
        self.calibration_file = rospy.get_param('~calibration_file', '')
        
        # CV Bridge
        self.bridge = CvBridge()
        
        # Camera calibration
        self.camera_matrix = None
        self.dist_coeffs = None
        self.load_calibration()
        
        # Publisher & Subscriber
        self.image_pub = rospy.Publisher(self.output_topic, Image, queue_size=10)
        self.image_sub = rospy.Subscriber(self.camera_topic, Image, self.image_callback)
        
        rospy.loginfo("[Camera Node] 초기화 완료")
        rospy.loginfo(f"  - 입력 토픽: {self.camera_topic}")
        rospy.loginfo(f"  - 출력 토픽: {self.output_topic}")
        rospy.loginfo(f"  - 캘리브레이션: {'사용' if self.camera_matrix is not None else '미사용'}")
    
    def load_calibration(self):
        """카메라 캘리브레이션 파일 로드"""
        if not self.calibration_file:
            rospy.logwarn("[Camera Node] 캘리브레이션 파일이 지정되지 않음. 왜곡 보정 없이 실행")
            return
        
        if not os.path.exists(self.calibration_file):
            rospy.logwarn(f"[Camera Node] 캘리브레이션 파일 없음: {self.calibration_file}")
            return
        
        try:
            with open(self.calibration_file, 'r') as f:
                calib_data = yaml.safe_load(f)
            
            # Camera matrix
            if 'camera_matrix' in calib_data:
                matrix_data = calib_data['camera_matrix']['data']
                self.camera_matrix = cv2.Mat(3, 3, cv2.CV_64F, matrix_data).reshape(3, 3)
            
            # Distortion coefficients
            if 'distortion_coefficients' in calib_data:
                dist_data = calib_data['distortion_coefficients']['data']
                self.dist_coeffs = cv2.Mat(1, 5, cv2.CV_64F, dist_data)
            
            rospy.loginfo("[Camera Node] 캘리브레이션 파일 로드 성공")
        
        except Exception as e:
            rospy.logerr(f"[Camera Node] 캘리브레이션 로드 실패: {e}")
    
    def image_callback(self, msg):
        """
        카메라 이미지 콜백 함수
        캘리브레이션이 있으면 왜곡 보정 수행
        """
        try:
            # ROS Image → OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # 왜곡 보정 (캘리브레이션이 있는 경우)
            if self.camera_matrix is not None and self.dist_coeffs is not None:
                h, w = cv_image.shape[:2]
                new_camera_matrix, roi = cv2.getOptimalNewCameraMatrix(
                    self.camera_matrix, self.dist_coeffs, (w, h), 1, (w, h)
                )
                rectified_image = cv2.undistort(
                    cv_image, self.camera_matrix, self.dist_coeffs, 
                    None, new_camera_matrix
                )
            else:
                # 캘리브레이션 없으면 원본 그대로
                rectified_image = cv_image
            
            # OpenCV → ROS Image
            output_msg = self.bridge.cv2_to_imgmsg(rectified_image, encoding='bgr8')
            output_msg.header = msg.header
            
            # 발행
            self.image_pub.publish(output_msg)
            
            rospy.logdebug("[Camera Node] 이미지 처리 및 발행 완료")
            
        except Exception as e:
            rospy.logerr(f"[Camera Node] 이미지 처리 오류: {e}")
    
    def run(self):
        rospy.spin()


if __name__ == '__main__':
    try:
        node = CameraNode()
        node.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("[Camera Node] 종료")

