#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
WeGO Vision - Detection Node
객체 감지를 수행하고 장애물 정보를 발행합니다.

Topic Subscriptions:
  - /vision/image_rect (sensor_msgs/Image): 보정된 카메라 이미지

Topic Publications:
  - /vision/obstacles (geometry_msgs/PoseArray): 감지된 장애물 위치
"""

import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseArray, Pose, Point, Quaternion
from cv_bridge import CvBridge
import cv2


class DetectionNode:
    def __init__(self):
        rospy.init_node('detection_node', anonymous=False)
        
        # Parameters
        self.image_topic = rospy.get_param('~image_topic', '/vision/image_rect')
        self.output_topic = rospy.get_param('~output_topic', '/vision/obstacles')
        self.detection_method = rospy.get_param('~detection_method', 'aruco')  # 'aruco' or 'yolo'
        
        # CV Bridge
        self.bridge = CvBridge()
        
        # Publisher & Subscriber
        self.obstacle_pub = rospy.Publisher(self.output_topic, PoseArray, queue_size=10)
        self.image_sub = rospy.Subscriber(self.image_topic, Image, self.image_callback)
        
        rospy.loginfo("[Detection Node] 초기화 완료")
        rospy.loginfo(f"  - 입력 토픽: {self.image_topic}")
        rospy.loginfo(f"  - 출력 토픽: {self.output_topic}")
        rospy.loginfo(f"  - 감지 방법: {self.detection_method}")
    
    def image_callback(self, msg):
        """
        이미지에서 객체를 감지하고 장애물 정보를 발행
        """
        try:
            # ROS Image → OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # TODO: 실제 객체 감지 알고리즘 구현 (YOLOv8, ArUco 등)
            obstacles = self.detect_objects(cv_image)
            
            # PoseArray 메시지 생성
            pose_array = PoseArray()
            pose_array.header = msg.header
            pose_array.poses = obstacles
            
            # 발행
            self.obstacle_pub.publish(pose_array)
            
            rospy.logdebug(f"[Detection Node] {len(obstacles)}개의 장애물 감지")
            
        except Exception as e:
            rospy.logerr(f"[Detection Node] 객체 감지 오류: {e}")
    
    def detect_objects(self, image):
        """
        객체 감지 함수 (Placeholder)
        실제 구현: YOLOv8, ArUco 마커 등
        
        Returns:
            List[Pose]: 감지된 객체들의 위치
        """
        # Placeholder: 더미 데이터 반환
        obstacles = []
        
        # 예시: 하나의 더미 장애물
        pose = Pose()
        pose.position = Point(x=2.0, y=0.0, z=0.0)
        pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        obstacles.append(pose)
        
        return obstacles
    
    def run(self):
        rospy.spin()


if __name__ == '__main__':
    try:
        node = DetectionNode()
        node.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("[Detection Node] 종료")

