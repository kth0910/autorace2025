#!/usr/bin/env python3
#-*- coding: utf-8 -*-

import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import Float64

from warper import Warper
from slidewindow import SlideWindow

class LaneDetectionNode:
    
    def __init__(self):
        self.warper = Warper()
        self.slidewindow = SlideWindow()
        self.bridge = CvBridge()
        
        self.initialized = False
        
        # slide window return variable initialization
        self.slide_img = None 
        self.slide_x_location = 0
        self.current_lane_window = ""
        
        # Publisher: 차선 중심 위치 퍼블리시
        self.lane_center_pub = rospy.Publisher("/lane_center_x", Float64, queue_size=1)
        self.lane_image_pub = rospy.Publisher("/lane_detection/image", Image, queue_size=1)
        
        # Subscriber: 보정된 카메라 이미지 구독
        rospy.Subscriber("usb_cam/image_rect_color", Image, self.laneCallback)
        
        rospy.loginfo("Lane Detection Node 초기화 완료")
    
    def laneCallback(self, _data):
        # detect lane
        if self.initialized == False:
            cv2.namedWindow("Simulator_Image", cv2.WINDOW_NORMAL) 
            cv2.createTrackbar('low_H', 'Simulator_Image', 128, 360, nothing)
            cv2.createTrackbar('low_L', 'Simulator_Image', 134, 255, nothing)
            cv2.createTrackbar('low_S', 'Simulator_Image', 87, 255, nothing)
            cv2.createTrackbar('high_H', 'Simulator_Image', 334, 360, nothing)
            cv2.createTrackbar('high_L', 'Simulator_Image', 255, 255, nothing)
            cv2.createTrackbar('high_S', 'Simulator_Image', 251, 255, nothing)
            self.initialized = True
        
        cv2_image = self.bridge.imgmsg_to_cv2(_data)
        original_img = cv2_image.copy()
        
        # cv2.imshow("original", cv2_image) 

        low_H = cv2.getTrackbarPos('low_H', 'Simulator_Image')
        low_L = cv2.getTrackbarPos('low_L', 'Simulator_Image')
        low_S = cv2.getTrackbarPos('low_S', 'Simulator_Image')
        high_H = cv2.getTrackbarPos('high_H', 'Simulator_Image')
        high_L = cv2.getTrackbarPos('high_L', 'Simulator_Image')
        high_S = cv2.getTrackbarPos('high_S', 'Simulator_Image')

        cv2.cvtColor(cv2_image, cv2.COLOR_BGR2HLS) # BGR to HLS

        lower_lane = np.array([low_H, low_L, low_S]) 
        upper_lane = np.array([high_H, high_L, high_S])

        lane_image = cv2.inRange(cv2_image, lower_lane, upper_lane)

        cv2.imshow("Lane Image", lane_image)
        self.laneDetection(lane_image)
        
        # 차선 중심 위치 퍼블리시
        center_msg = Float64()
        center_msg.data = float(self.slide_x_location)
        self.lane_center_pub.publish(center_msg)
        
        # 슬라이딩 윈도우 결과 이미지 퍼블리시
        if self.slide_img is not None:
            try:
                slide_img_msg = self.bridge.cv2_to_imgmsg(self.slide_img, "rgb8")
                self.lane_image_pub.publish(slide_img_msg)
            except:
                pass

        cv2.waitKey(1)
    
    def laneDetection(self, lane_image):
        kernel_size = 5
        blur_img = cv2.GaussianBlur(lane_image, (kernel_size, kernel_size), 0)
        warped_img = self.warper.warp(blur_img)
        # cv2.imshow("warped_img", warped_img)
        self.slide_img, self.slide_x_location, self.current_lane_window = self.slidewindow.slidewindow(warped_img)
        cv2.imshow("slide_img", self.slide_img)
        rospy.loginfo("차선 중심 위치: {}, 현재 차선: {}".format(self.slide_x_location, self.current_lane_window))


def nothing(x):
    pass


def run():
    rospy.init_node("lane_detection_node")
    lane_detector = LaneDetectionNode()
    rospy.spin()


if __name__ == '__main__':
    run()


