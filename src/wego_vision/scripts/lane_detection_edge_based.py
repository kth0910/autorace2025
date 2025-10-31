#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
WeGO Vision - Edge-Based Lane Detection Node (색상 무관)
Canny Edge만 사용하는 모양 기반 차선 검출

Topic Subscriptions:
  - /vision/image_rect (sensor_msgs/Image): 보정된 카메라 이미지

Topic Publications:
  - /vision/lane_info (wego_vision/LaneInfo): 차선 정보
  - /vision/lane_image (sensor_msgs/Image): 차선 검출 결과 이미지
"""

import rospy
import numpy as np
import cv2
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from cv_bridge import CvBridge
from wego_vision.msg import LaneInfo


class EdgeBasedLaneDetectionNode:
    def __init__(self):
        rospy.init_node('lane_detection_edge_node', anonymous=False)
        
        # Parameters
        self.image_topic = rospy.get_param('~image_topic', '/vision/image_rect')
        self.lane_info_topic = rospy.get_param('~lane_info_topic', '/vision/lane_info')
        self.debug_image_topic = rospy.get_param('~debug_image_topic', '/vision/lane_image')
        
        # Edge detection parameters (색상 무관!)
        self.roi_top_ratio = rospy.get_param('~roi_top_ratio', 0.5)
        self.roi_bottom_ratio = rospy.get_param('~roi_bottom_ratio', 1.0)
        
        # Gaussian blur
        self.blur_kernel = rospy.get_param('~blur_kernel', 5)
        
        # Canny edge
        self.canny_low = rospy.get_param('~canny_low', 50)
        self.canny_high = rospy.get_param('~canny_high', 150)
        
        # Hough transform
        self.hough_threshold = rospy.get_param('~hough_threshold', 30)
        self.hough_min_line_length = rospy.get_param('~hough_min_line_length', 20)
        self.hough_max_line_gap = rospy.get_param('~hough_max_line_gap', 200)
        
        # Lane parameters
        self.lane_width_meters = rospy.get_param('~lane_width_meters', 0.3)
        self.pixels_per_meter = rospy.get_param('~pixels_per_meter', 500)
        
        # Debugging
        self.publish_debug_image = rospy.get_param('~publish_debug_image', True)
        
        # CV Bridge
        self.bridge = CvBridge()
        
        # Image size
        self.image_width = None
        self.image_height = None
        
        # Publishers
        self.lane_info_pub = rospy.Publisher(self.lane_info_topic, LaneInfo, queue_size=10)
        if self.publish_debug_image:
            self.debug_image_pub = rospy.Publisher(self.debug_image_topic, Image, queue_size=10)
        
        # Subscriber
        self.image_sub = rospy.Subscriber(self.image_topic, Image, self.image_callback)
        
        rospy.loginfo("[Edge-Based Lane Detection] 초기화 완료")
        rospy.loginfo("  ⭐ 색상 무관 Edge 기반 검출 모드")
        rospy.loginfo(f"  - Canny: {self.canny_low} ~ {self.canny_high}")
        rospy.loginfo(f"  - Hough Threshold: {self.hough_threshold}")
    
    def image_callback(self, msg):
        """카메라 이미지에서 차선 검출"""
        try:
            # ROS Image → OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            if self.image_height is None:
                self.image_height, self.image_width = cv_image.shape[:2]
                rospy.loginfo(f"[Edge Lane] 이미지 크기: {self.image_width}x{self.image_height}")
            
            # 차선 검출
            lane_info = self.detect_lanes_edge_based(cv_image, msg.header)
            
            # LaneInfo 발행
            self.lane_info_pub.publish(lane_info)
            
            # 디버그 이미지
            if self.publish_debug_image:
                debug_image = self.draw_lanes(cv_image, lane_info)
                debug_msg = self.bridge.cv2_to_imgmsg(debug_image, encoding='bgr8')
                debug_msg.header = msg.header
                self.debug_image_pub.publish(debug_msg)
            
        except Exception as e:
            rospy.logerr(f"[Edge Lane] 오류: {e}")
    
    def detect_lanes_edge_based(self, image, header):
        """Edge 기반 차선 검출 (색상 무관)"""
        lane_info = LaneInfo()
        lane_info.header = header
        
        # 1. Grayscale 변환
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        
        # 2. ROI 적용
        roi_mask = self.get_roi_mask(image.shape)
        gray_roi = cv2.bitwise_and(gray, gray, mask=roi_mask)
        
        # 3. Gaussian Blur (노이즈 제거)
        blurred = cv2.GaussianBlur(gray_roi, (self.blur_kernel, self.blur_kernel), 0)
        
        # 4. Canny Edge Detection (핵심!)
        edges = cv2.Canny(blurred, self.canny_low, self.canny_high)
        
        # 5. Morphological operations (선을 더 명확하게)
        kernel = np.ones((3, 3), np.uint8)
        edges = cv2.dilate(edges, kernel, iterations=1)
        edges = cv2.erode(edges, kernel, iterations=1)
        
        # 6. Hough Line Transform
        lines = cv2.HoughLinesP(
            edges,
            rho=1,
            theta=np.pi/180,
            threshold=self.hough_threshold,
            minLineLength=self.hough_min_line_length,
            maxLineGap=self.hough_max_line_gap
        )
        
        # 7. 차선 분류 (좌/우)
        left_lines, right_lines = self.classify_lines(lines)
        
        # 8. 차선 피팅
        if left_lines:
            lane_info.left_lane_detected = True
            lane_info.left_a, lane_info.left_b, lane_info.left_c, lane_info.left_lane_points = \
                self.fit_lane(left_lines)
        
        if right_lines:
            lane_info.right_lane_detected = True
            lane_info.right_a, lane_info.right_b, lane_info.right_c, lane_info.right_lane_points = \
                self.fit_lane(right_lines)
        
        # 9. 중심선 계산
        if lane_info.left_lane_detected and lane_info.right_lane_detected:
            lane_info.center_offset, lane_info.heading_error, lane_info.center_lane_points = \
                self.calculate_center_offset(lane_info)
            lane_info.curvature = self.calculate_curvature(lane_info)
            lane_info.confidence = 0.9
        elif lane_info.left_lane_detected or lane_info.right_lane_detected:
            lane_info.confidence = 0.5
        else:
            lane_info.confidence = 0.0
        
        return lane_info
    
    def get_roi_mask(self, shape):
        """ROI 마스크 생성"""
        mask = np.zeros(shape[:2], dtype=np.uint8)
        
        height, width = shape[:2]
        roi_top = int(height * self.roi_top_ratio)
        roi_bottom = int(height * self.roi_bottom_ratio)
        
        # 사다리꼴 ROI
        vertices = np.array([[
            (0, roi_bottom),
            (0, roi_top),
            (width, roi_top),
            (width, roi_bottom)
        ]], dtype=np.int32)
        
        cv2.fillPoly(mask, vertices, 255)
        return mask
    
    def classify_lines(self, lines):
        """라인을 좌/우로 분류"""
        if lines is None:
            return [], []
        
        left_lines = []
        right_lines = []
        image_center = self.image_width / 2
        
        for line in lines:
            x1, y1, x2, y2 = line[0]
            
            if x2 - x1 == 0:
                continue
            
            slope = (y2 - y1) / (x2 - x1)
            
            # 너무 수평인 선 제거
            if abs(slope) < 0.3:
                continue
            
            # 너무 수직인 선 제거
            if abs(slope) > 3.0:
                continue
            
            center_x = (x1 + x2) / 2
            
            # 좌/우 분류
            if center_x < image_center and slope < 0:
                left_lines.append(line[0])
            elif center_x > image_center and slope > 0:
                right_lines.append(line[0])
        
        return left_lines, right_lines
    
    def fit_lane(self, lines):
        """차선 피팅"""
        x_points = []
        y_points = []
        
        for line in lines:
            x1, y1, x2, y2 = line
            x_points.extend([x1, x2])
            y_points.extend([y1, y2])
        
        x_points = np.array(x_points)
        y_points = np.array(y_points)
        
        if len(x_points) >= 3:
            coeffs = np.polyfit(y_points, x_points, 2)
            a, b, c = coeffs
        else:
            a, b, c = 0.0, 0.0, 0.0
        
        # 차선 포인트 생성
        lane_points = []
        if len(x_points) >= 3:
            y_range = np.linspace(min(y_points), max(y_points), 10)
            for y in y_range:
                x = a * y**2 + b * y + c
                point = Point()
                point.x = float(x)
                point.y = float(y)
                point.z = 0.0
                lane_points.append(point)
        
        return float(a), float(b), float(c), lane_points
    
    def calculate_center_offset(self, lane_info):
        """중심 오프셋 계산"""
        y_eval = self.image_height - 1
        
        left_x = lane_info.left_a * y_eval**2 + lane_info.left_b * y_eval + lane_info.left_c
        right_x = lane_info.right_a * y_eval**2 + lane_info.right_b * y_eval + lane_info.right_c
        
        lane_center = (left_x + right_x) / 2
        vehicle_center = self.image_width / 2
        pixel_offset = vehicle_center - lane_center
        center_offset = pixel_offset / self.pixels_per_meter
        
        left_slope = 2 * lane_info.left_a * y_eval + lane_info.left_b
        right_slope = 2 * lane_info.right_a * y_eval + lane_info.right_b
        avg_slope = (left_slope + right_slope) / 2
        heading_error = np.arctan(1.0 / avg_slope) if avg_slope != 0 else 0.0
        
        center_points = []
        if lane_info.left_lane_points and lane_info.right_lane_points:
            for left_pt, right_pt in zip(lane_info.left_lane_points, lane_info.right_lane_points):
                center_pt = Point()
                center_pt.x = (left_pt.x + right_pt.x) / 2
                center_pt.y = (left_pt.y + right_pt.y) / 2
                center_pt.z = 0.0
                center_points.append(center_pt)
        
        return float(center_offset), float(heading_error), center_points
    
    def calculate_curvature(self, lane_info):
        """곡률 계산"""
        left_curve = abs(2 * lane_info.left_a)
        right_curve = abs(2 * lane_info.right_a)
        avg_curve = (left_curve + right_curve) / 2
        curvature = avg_curve / (self.pixels_per_meter ** 2)
        return float(curvature)
    
    def draw_lanes(self, image, lane_info):
        """차선 그리기"""
        output = image.copy()
        overlay = np.zeros_like(output)
        
        # 좌측 차선 (녹색)
        if lane_info.left_lane_detected and lane_info.left_lane_points:
            pts = [(int(p.x), int(p.y)) for p in lane_info.left_lane_points]
            for i in range(len(pts) - 1):
                cv2.line(overlay, pts[i], pts[i+1], (0, 255, 0), 5)
        
        # 우측 차선 (파란색)
        if lane_info.right_lane_detected and lane_info.right_lane_points:
            pts = [(int(p.x), int(p.y)) for p in lane_info.right_lane_points]
            for i in range(len(pts) - 1):
                cv2.line(overlay, pts[i], pts[i+1], (255, 0, 0), 5)
        
        # 중심선 (빨간색)
        if lane_info.center_lane_points:
            pts = [(int(p.x), int(p.y)) for p in lane_info.center_lane_points]
            for i in range(len(pts) - 1):
                cv2.line(overlay, pts[i], pts[i+1], (0, 0, 255), 3)
        
        cv2.addWeighted(overlay, 0.7, output, 1.0, 0, output)
        
        # 정보 텍스트
        y_offset = 30
        cv2.putText(output, f"Left: {'YES' if lane_info.left_lane_detected else 'NO'}", 
                   (10, y_offset), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        cv2.putText(output, f"Right: {'YES' if lane_info.right_lane_detected else 'NO'}", 
                   (10, y_offset + 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 0), 2)
        cv2.putText(output, f"Offset: {lane_info.center_offset:.3f} m", 
                   (10, y_offset + 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        cv2.putText(output, f"Confidence: {lane_info.confidence:.2f}", 
                   (10, y_offset + 90), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        
        return output
    
    def run(self):
        rospy.spin()


if __name__ == '__main__':
    try:
        node = EdgeBasedLaneDetectionNode()
        node.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("[Edge Lane] 종료")

