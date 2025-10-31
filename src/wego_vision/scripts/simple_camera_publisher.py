#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
WeGO Vision - Simple Camera Publisher
usb_cam 대신 사용하는 간단한 카메라 퍼블리셔

Topic Publications:
  - /usb_cam/image_raw (sensor_msgs/Image): 카메라 이미지
"""

import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class SimpleCameraPublisher:
    def __init__(self):
        rospy.init_node('simple_camera_publisher', anonymous=False)
        
        # Parameters
        self.video_device = rospy.get_param('~video_device', '/dev/video0')
        self.frame_rate = rospy.get_param('~frame_rate', 30)
        self.image_width = rospy.get_param('~image_width', 640)
        self.image_height = rospy.get_param('~image_height', 480)
        
        # OpenCV VideoCapture
        self.cap = cv2.VideoCapture(self.video_device)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.image_width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.image_height)
        self.cap.set(cv2.CAP_PROP_FPS, self.frame_rate)
        
        if not self.cap.isOpened():
            rospy.logerr(f"[Simple Camera] 카메라 열기 실패: {self.video_device}")
            rospy.signal_shutdown("Camera open failed")
            return
        
        # CV Bridge
        self.bridge = CvBridge()
        
        # Publisher
        self.image_pub = rospy.Publisher('/usb_cam/image_raw', Image, queue_size=10)
        
        # Timer
        self.timer = rospy.Timer(
            rospy.Duration(1.0 / self.frame_rate),
            self.publish_image
        )
        
        rospy.loginfo("[Simple Camera] 초기화 완료")
        rospy.loginfo(f"  - 디바이스: {self.video_device}")
        rospy.loginfo(f"  - 해상도: {self.image_width}x{self.image_height}")
        rospy.loginfo(f"  - FPS: {self.frame_rate}")
    
    def publish_image(self, event):
        """이미지 캡처 및 발행"""
        ret, frame = self.cap.read()
        
        if not ret:
            rospy.logwarn_throttle(5.0, "[Simple Camera] 프레임 읽기 실패")
            return
        
        try:
            # OpenCV → ROS Image
            msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            msg.header.stamp = rospy.Time.now()
            msg.header.frame_id = 'camera_link'
            
            # 발행
            self.image_pub.publish(msg)
            
        except Exception as e:
            rospy.logerr(f"[Simple Camera] 이미지 변환 오류: {e}")
    
    def cleanup(self):
        """정리"""
        if self.cap.isOpened():
            self.cap.release()
        rospy.loginfo("[Simple Camera] 종료")
    
    def run(self):
        rospy.spin()


if __name__ == '__main__':
    try:
        node = SimpleCameraPublisher()
        node.run()
        node.cleanup()
    except rospy.ROSInterruptException:
        rospy.loginfo("[Simple Camera] 종료")

