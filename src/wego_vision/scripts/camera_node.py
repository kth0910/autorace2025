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


class CameraNode:
    def __init__(self):
        rospy.init_node('camera_node', anonymous=False)
        
        # Parameters
        self.camera_topic = rospy.get_param('~camera_topic', '/usb_cam/image_raw')
        self.output_topic = rospy.get_param('~output_topic', '/vision/image_rect')
        
        # CV Bridge
        self.bridge = CvBridge()
        
        # Publisher & Subscriber
        self.image_pub = rospy.Publisher(self.output_topic, Image, queue_size=10)
        self.image_sub = rospy.Subscriber(self.camera_topic, Image, self.image_callback)
        
        rospy.loginfo("[Camera Node] 초기화 완료")
        rospy.loginfo(f"  - 입력 토픽: {self.camera_topic}")
        rospy.loginfo(f"  - 출력 토픽: {self.output_topic}")
    
    def image_callback(self, msg):
        """
        카메라 이미지 콜백 함수
        실제 구현에서는 카메라 캘리브레이션, 왜곡 보정 등을 수행
        """
        try:
            # ROS Image → OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # TODO: 이미지 전처리 (캘리브레이션, 왜곡 보정)
            # rectified_image = self.undistort(cv_image)
            rectified_image = cv_image  # Placeholder
            
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

