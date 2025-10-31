#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
WeGO Vision - Simple Lane Detection (단순 버전)
좌/우 구분 없이 차선만 검출 (ROI 하단 35%)

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


class SimpleLaneDetectionNode:
    def __init__(self):
        rospy.init_node('lane_detection_simple', anonymous=False)
        
        # Parameters
        self.image_topic = rospy.get_param('~image_topic', '/vision/image_rect')
        self.lane_info_topic = rospy.get_param('~lane_info_topic', '/vision/lane_info')
        self.debug_image_topic = rospy.get_param('~debug_image_topic', '/vision/lane_image')
        
        # ROI - 하단 40%만 사용 ⭐
        self.roi_top_ratio = rospy.get_param('~roi_top_ratio', 0.6)  # 60%부터 시작 (하단 40%)
        self.roi_bottom_ratio = rospy.get_param('~roi_bottom_ratio', 1.0)  # 100%까지
        
        # 색상 필터 (흰색 차선)
        self.white_lower = np.array(rospy.get_param('~white_lower', [0, 0, 180]))
        self.white_upper = np.array(rospy.get_param('~white_upper', [180, 30, 255]))
        
        # Edge detection
        self.blur_kernel = rospy.get_param('~blur_kernel', 5)
        self.canny_low = rospy.get_param('~canny_low', 50)
        self.canny_high = rospy.get_param('~canny_high', 150)
        
        # Hough transform
        self.hough_threshold = rospy.get_param('~hough_threshold', 30)
        self.hough_min_line_length = rospy.get_param('~hough_min_line_length', 20)
        self.hough_max_line_gap = rospy.get_param('~hough_max_line_gap', 200)
        
        # Lane parameters
        self.pixels_per_meter = rospy.get_param('~pixels_per_meter', 500)
        
        # Color weight
        self.color_weight = rospy.get_param('~color_weight', 0.3)
        
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
            self.edge_pub = rospy.Publisher('/vision/debug/edges', Image, queue_size=10)
            self.color_pub = rospy.Publisher('/vision/debug/color_mask', Image, queue_size=10)
            self.roi_pub = rospy.Publisher('/vision/debug/roi', Image, queue_size=10)
        
        # Subscriber
        self.image_sub = rospy.Subscriber(self.image_topic, Image, self.image_callback)
        
        rospy.loginfo("[Simple Lane Detection] 초기화 완료")
        rospy.loginfo(f"  📐 ROI: 하단 40% (직사각형)")
        rospy.loginfo(f"  🎯 Edge + 흰색 검출 (가중치 {self.color_weight*100:.0f}%)")
    
    def image_callback(self, msg):
        """카메라 이미지에서 차선 검출"""
        try:
            # ROS Image → OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            if self.image_height is None:
                self.image_height, self.image_width = cv_image.shape[:2]
                roi_height = int(self.image_height * (self.roi_bottom_ratio - self.roi_top_ratio))
                rospy.loginfo(f"[Simple Lane] 이미지: {self.image_width}x{self.image_height}")
                rospy.loginfo(f"[Simple Lane] ROI 높이: {roi_height}px (하단 40%, 직사각형)")
            
            # 차선 검출
            lane_info = self.detect_lanes(cv_image, msg.header)
            
            # LaneInfo 발행
            self.lane_info_pub.publish(lane_info)
            
            # 디버그 이미지
            if self.publish_debug_image:
                debug_image = self.draw_lanes(cv_image, lane_info)
                debug_msg = self.bridge.cv2_to_imgmsg(debug_image, encoding='bgr8')
                debug_msg.header = msg.header
                self.debug_image_pub.publish(debug_msg)
            
        except Exception as e:
            rospy.logerr(f"[Simple Lane] 오류: {e}")
            import traceback
            rospy.logerr(traceback.format_exc())
    
    def detect_lanes(self, image, header):
        """간단한 차선 검출 (좌/우 구분 없음)"""
        lane_info = LaneInfo()
        lane_info.header = header
        
        # 1. ROI 마스크
        roi_mask = self.get_roi_mask(image.shape)
        
        # 디버그: ROI 이미지
        if self.publish_debug_image:
            roi_image = cv2.bitwise_and(image, image, mask=roi_mask)
            roi_msg = self.bridge.cv2_to_imgmsg(roi_image, encoding='bgr8')
            roi_msg.header = header
            self.roi_pub.publish(roi_msg)
        
        # 2. Edge 검출
        edge_mask = self.detect_edges(image, roi_mask)
        
        # 3. 색상 검출
        color_mask = self.detect_white_color(image, roi_mask)
        
        # 디버그: Edge & Color 마스크
        if self.publish_debug_image:
            edge_msg = self.bridge.cv2_to_imgmsg(edge_mask, encoding='mono8')
            edge_msg.header = header
            self.edge_pub.publish(edge_msg)
            
            color_msg = self.bridge.cv2_to_imgmsg(color_mask, encoding='mono8')
            color_msg.header = header
            self.color_pub.publish(color_msg)
        
        # 4. 결합
        combined_mask = cv2.addWeighted(
            edge_mask, 1.0 - self.color_weight,
            color_mask, self.color_weight,
            0
        )
        
        # 5. Morphological operations
        kernel = np.ones((3, 3), np.uint8)
        combined_mask = cv2.morphologyEx(combined_mask, cv2.MORPH_CLOSE, kernel)
        combined_mask = cv2.morphologyEx(combined_mask, cv2.MORPH_OPEN, kernel)
        
        # 6. Hough Line Transform
        lines = cv2.HoughLinesP(
            combined_mask,
            rho=1,
            theta=np.pi/180,
            threshold=self.hough_threshold,
            minLineLength=self.hough_min_line_length,
            maxLineGap=self.hough_max_line_gap
        )
        
        # 7. 차선 검출 여부만 확인 (좌/우 구분 없음!)
        if lines is not None and len(lines) > 0:
            lane_info.left_lane_detected = True  # 단순히 "차선 있음" 표시
            lane_info.confidence = 0.9
            
            # 중심선 계산
            lane_info.center_lane_points = self.get_center_points(lines)
            lane_info.center_offset = self.calculate_simple_offset(lines)
            
            rospy.loginfo_throttle(2.0, f"[Simple Lane] 차선 검출! ({len(lines)}개 선, "
                                        f"Offset: {lane_info.center_offset:.3f}m)")
        else:
            lane_info.confidence = 0.0
            rospy.logwarn_throttle(2.0, "[Simple Lane] 차선 미검출")
        
        return lane_info
    
    def get_roi_mask(self, shape):
        """ROI 마스크 (하단 40% 직사각형)"""
        mask = np.zeros(shape[:2], dtype=np.uint8)
        
        height, width = shape[:2]
        roi_top = int(height * self.roi_top_ratio)      # 60% 지점
        roi_bottom = int(height * self.roi_bottom_ratio) # 100% 지점
        
        # 직사각형 ROI (하단 40% 전체) ⭐
        vertices = np.array([[
            (0, roi_top),           # 좌측 상단
            (width, roi_top),       # 우측 상단
            (width, roi_bottom),    # 우측 하단
            (0, roi_bottom)         # 좌측 하단
        ]], dtype=np.int32)
        
        cv2.fillPoly(mask, vertices, 255)
        return mask
    
    def detect_edges(self, image, roi_mask):
        """Edge 검출"""
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        gray = cv2.bitwise_and(gray, gray, mask=roi_mask)
        blurred = cv2.GaussianBlur(gray, (self.blur_kernel, self.blur_kernel), 0)
        edges = cv2.Canny(blurred, self.canny_low, self.canny_high)
        return edges
    
    def detect_white_color(self, image, roi_mask):
        """흰색 검출"""
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        hsv_roi = cv2.bitwise_and(hsv, hsv, mask=roi_mask)
        white_mask = cv2.inRange(hsv_roi, self.white_lower, self.white_upper)
        white_edges = cv2.Canny(white_mask, 50, 150)
        return white_edges
    
    def get_center_points(self, lines):
        """모든 선의 중심점들"""
        points = []
        for line in lines:
            x1, y1, x2, y2 = line[0]
            cx = (x1 + x2) / 2
            cy = (y1 + y2) / 2
            point = Point()
            point.x = float(cx)
            point.y = float(cy)
            point.z = 0.0
            points.append(point)
        return points
    
    def calculate_simple_offset(self, lines):
        """차선 중심 오프셋 (간단 버전)"""
        if lines is None or len(lines) == 0:
            return 0.0
        
        # 모든 선의 x 좌표 평균
        x_coords = []
        for line in lines:
            x1, y1, x2, y2 = line[0]
            x_coords.extend([x1, x2])
        
        avg_x = np.mean(x_coords)
        image_center = self.image_width / 2
        pixel_offset = image_center - avg_x
        
        # 미터로 변환
        offset_meters = pixel_offset / self.pixels_per_meter
        return float(offset_meters)
    
    def draw_lanes(self, image, lane_info):
        """차선 그리기 - Edge 표시"""
        output = image.copy()
        
        # ROI 영역 표시 (반투명 녹색)
        roi_mask = self.get_roi_mask(image.shape)
        roi_overlay = np.zeros_like(output)
        roi_overlay[roi_mask > 0] = [0, 100, 0]  # 녹색
        cv2.addWeighted(roi_overlay, 0.2, output, 1.0, 0, output)
        
        # ROI 경계선 (노란색 굵은 선)
        height, width = image.shape[:2]
        roi_top = int(height * self.roi_top_ratio)
        cv2.line(output, (0, roi_top), (width, roi_top), (0, 255, 255), 3)  # 노란색 선
        
        # Edge 검출 결과를 원본에 오버레이 ⭐
        if lane_info.left_lane_detected:  # "차선 있음" 의미
            # Edge 재계산
            edge_mask = self.detect_edges(image, roi_mask)
            
            # Edge를 빨간색으로 표시
            edge_overlay = np.zeros_like(output)
            edge_overlay[edge_mask > 0] = [0, 0, 255]  # 빨간색
            cv2.addWeighted(edge_overlay, 0.7, output, 1.0, 0, output)
        
        # 정보 표시
        y_offset = 30
        
        # 배경 박스
        cv2.rectangle(output, (5, 5), (320, 150), (0, 0, 0), -1)
        
        # 텍스트
        status_color = (0, 255, 0) if lane_info.confidence > 0.5 else (0, 0, 255)
        cv2.putText(output, f"Lane: {'DETECTED' if lane_info.left_lane_detected else 'NOT FOUND'}", 
                   (10, y_offset), cv2.FONT_HERSHEY_SIMPLEX, 0.7, status_color, 2)
        cv2.putText(output, f"Offset: {lane_info.center_offset:.3f} m", 
                   (10, y_offset + 35), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        cv2.putText(output, f"Confidence: {lane_info.confidence:.2f}", 
                   (10, y_offset + 70), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        cv2.putText(output, f"ROI: Bottom 40% (Rectangle)", 
                   (10, y_offset + 105), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
        
        return output
    
    def run(self):
        rospy.spin()


if __name__ == '__main__':
    try:
        node = SimpleLaneDetectionNode()
        node.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("[Simple Lane] 종료")

