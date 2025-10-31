#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
WeGO Vision - Hybrid Lane Detection Node
Edge 기반 + 색상 보조 (검은 길 + 흰 차선 최적화)

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


class HybridLaneDetectionNode:
    def __init__(self):
        rospy.init_node('lane_detection_hybrid_node', anonymous=False)
        
        # Parameters
        self.image_topic = rospy.get_param('~image_topic', '/vision/image_rect')
        self.lane_info_topic = rospy.get_param('~lane_info_topic', '/vision/lane_info')
        self.debug_image_topic = rospy.get_param('~debug_image_topic', '/vision/lane_image')
        
        # ROI
        self.roi_top_ratio = rospy.get_param('~roi_top_ratio', 0.5)
        self.roi_bottom_ratio = rospy.get_param('~roi_bottom_ratio', 1.0)
        
        # 색상 필터 (흰색 차선용) - 보조 역할
        self.white_lower = np.array(rospy.get_param('~white_lower', [0, 0, 180]))
        self.white_upper = np.array(rospy.get_param('~white_upper', [180, 30, 255]))
        self.color_weight = rospy.get_param('~color_weight', 0.3)  # 색상 가중치 30%
        
        # Gaussian blur
        self.blur_kernel = rospy.get_param('~blur_kernel', 5)
        
        # Canny edge (메인)
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
            # 추가 디버그 토픽들
            self.edge_pub = rospy.Publisher('/vision/debug/edges', Image, queue_size=10)
            self.color_pub = rospy.Publisher('/vision/debug/color_mask', Image, queue_size=10)
            self.combined_pub = rospy.Publisher('/vision/debug/combined_mask', Image, queue_size=10)
        
        # Subscriber
        self.image_sub = rospy.Subscriber(self.image_topic, Image, self.image_callback)
        
        rospy.loginfo("[Hybrid Lane Detection] 초기화 완료")
        rospy.loginfo("  🎯 Edge 기반 + 흰색 검출 보조")
        rospy.loginfo(f"  - Edge: Canny {self.canny_low}~{self.canny_high}")
        rospy.loginfo(f"  - 색상: 흰색 보조 (가중치 {self.color_weight*100:.0f}%)")
    
    def image_callback(self, msg):
        """카메라 이미지에서 차선 검출"""
        try:
            # ROS Image → OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            if self.image_height is None:
                self.image_height, self.image_width = cv_image.shape[:2]
                rospy.loginfo(f"[Hybrid Lane] 이미지 크기: {self.image_width}x{self.image_height}")
            
            # 차선 검출 (하이브리드)
            lane_info = self.detect_lanes_hybrid(cv_image, msg.header)
            
            # LaneInfo 발행
            self.lane_info_pub.publish(lane_info)
            
            # 디버그 이미지
            if self.publish_debug_image:
                debug_image = self.draw_lanes(cv_image, lane_info)
                debug_msg = self.bridge.cv2_to_imgmsg(debug_image, encoding='bgr8')
                debug_msg.header = msg.header
                self.debug_image_pub.publish(debug_msg)
            
        except Exception as e:
            rospy.logerr(f"[Hybrid Lane] 오류: {e}")
    
    def detect_lanes_hybrid(self, image, header):
        """
        하이브리드 차선 검출
        Edge 기반 (메인) + 색상 필터 (보조)
        """
        lane_info = LaneInfo()
        lane_info.header = header
        
        # 1. ROI 마스크
        roi_mask = self.get_roi_mask(image.shape)
        
        # 2-A. Edge 기반 검출 (메인, 70%)
        edge_mask = self.detect_edges(image, roi_mask)
        
        # 2-B. 색상 기반 검출 (보조, 30%)
        color_mask = self.detect_white_color(image, roi_mask)
        
        # 디버그 이미지 발행
        if self.publish_debug_image:
            self.publish_debug_masks(edge_mask, color_mask, header)
        
        # 3. 하이브리드 결합
        # Edge를 메인으로, 색상을 보조로
        combined_mask = cv2.addWeighted(
            edge_mask, 1.0 - self.color_weight,
            color_mask, self.color_weight,
            0
        )
        
        # 4. Morphological operations (노이즈 제거)
        kernel = np.ones((3, 3), np.uint8)
        combined_mask = cv2.morphologyEx(combined_mask, cv2.MORPH_CLOSE, kernel)
        combined_mask = cv2.morphologyEx(combined_mask, cv2.MORPH_OPEN, kernel)
        
        # 5. Hough Line Transform
        lines = cv2.HoughLinesP(
            combined_mask,
            rho=1,
            theta=np.pi/180,
            threshold=self.hough_threshold,
            minLineLength=self.hough_min_line_length,
            maxLineGap=self.hough_max_line_gap
        )
        
        # 6. 차선 분류 (좌/우)
        left_lines, right_lines = self.classify_lines(lines)
        
        # 7. 차선 피팅
        if left_lines:
            lane_info.left_lane_detected = True
            lane_info.left_a, lane_info.left_b, lane_info.left_c, lane_info.left_lane_points = \
                self.fit_lane(left_lines)
        
        if right_lines:
            lane_info.right_lane_detected = True
            lane_info.right_a, lane_info.right_b, lane_info.right_c, lane_info.right_lane_points = \
                self.fit_lane(right_lines)
        
        # 8. 중심선 계산
        if lane_info.left_lane_detected and lane_info.right_lane_detected:
            lane_info.center_offset, lane_info.heading_error, lane_info.center_lane_points = \
                self.calculate_center_offset(lane_info)
            lane_info.curvature = self.calculate_curvature(lane_info)
            lane_info.confidence = 0.95
        elif lane_info.left_lane_detected or lane_info.right_lane_detected:
            lane_info.confidence = 0.6
        else:
            lane_info.confidence = 0.0
        
        return lane_info
    
    def detect_edges(self, image, roi_mask):
        """Edge 기반 검출 (메인)"""
        # Grayscale
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        gray = cv2.bitwise_and(gray, gray, mask=roi_mask)
        
        # Gaussian Blur
        blurred = cv2.GaussianBlur(gray, (self.blur_kernel, self.blur_kernel), 0)
        
        # Canny Edge
        edges = cv2.Canny(blurred, self.canny_low, self.canny_high)
        
        return edges
    
    def detect_white_color(self, image, roi_mask):
        """색상 기반 검출 (흰색 차선 보조)"""
        # HSV 변환
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        
        # ROI 적용
        hsv_roi = cv2.bitwise_and(hsv, hsv, mask=roi_mask)
        
        # 흰색 범위 검출
        white_mask = cv2.inRange(hsv_roi, self.white_lower, self.white_upper)
        
        # Edge 검출 (색상 마스크에서)
        white_edges = cv2.Canny(white_mask, 50, 150)
        
        return white_edges
    
    def get_roi_mask(self, shape):
        """ROI 마스크 생성"""
        mask = np.zeros(shape[:2], dtype=np.uint8)
        
        height, width = shape[:2]
        roi_top = int(height * self.roi_top_ratio)
        roi_bottom = int(height * self.roi_bottom_ratio)
        
        # 사다리꼴 ROI
        vertices = np.array([[
            (0, roi_bottom),
            (int(width * 0.1), roi_top),      # 좌측 상단
            (int(width * 0.9), roi_top),      # 우측 상단
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
            
            # 너무 수평인 선 제거 (차선은 수직에 가까움)
            if abs(slope) < 0.4:
                continue
            
            # 너무 수직인 선 제거
            if abs(slope) > 3.0:
                continue
            
            # 선의 중심점
            center_x = (x1 + x2) / 2
            
            # 길이 계산 (짧은 선 제거)
            length = np.sqrt((x2-x1)**2 + (y2-y1)**2)
            if length < 15:
                continue
            
            # 좌/우 분류
            if center_x < image_center and slope < 0:
                left_lines.append(line[0])
            elif center_x > image_center and slope > 0:
                right_lines.append(line[0])
        
        return left_lines, right_lines
    
    def fit_lane(self, lines):
        """차선 피팅 (2차 다항식)"""
        x_points = []
        y_points = []
        
        for line in lines:
            x1, y1, x2, y2 = line
            x_points.extend([x1, x2])
            y_points.extend([y1, y2])
        
        x_points = np.array(x_points)
        y_points = np.array(y_points)
        
        if len(x_points) >= 3:
            # 2차 다항식 피팅
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
    
    def publish_debug_masks(self, edge_mask, color_mask, header):
        """디버그 마스크 이미지 발행"""
        try:
            # Edge 마스크
            edge_msg = self.bridge.cv2_to_imgmsg(edge_mask, encoding='mono8')
            edge_msg.header = header
            self.edge_pub.publish(edge_msg)
            
            # Color 마스크
            color_msg = self.bridge.cv2_to_imgmsg(color_mask, encoding='mono8')
            color_msg.header = header
            self.color_pub.publish(color_msg)
        except:
            pass
    
    def draw_lanes(self, image, lane_info):
        """차선 그리기"""
        output = image.copy()
        overlay = np.zeros_like(output)
        
        # 좌측 차선 (녹색, 굵게)
        if lane_info.left_lane_detected and lane_info.left_lane_points:
            pts = [(int(p.x), int(p.y)) for p in lane_info.left_lane_points]
            for i in range(len(pts) - 1):
                cv2.line(overlay, pts[i], pts[i+1], (0, 255, 0), 8)
        
        # 우측 차선 (파란색, 굵게)
        if lane_info.right_lane_detected and lane_info.right_lane_points:
            pts = [(int(p.x), int(p.y)) for p in lane_info.right_lane_points]
            for i in range(len(pts) - 1):
                cv2.line(overlay, pts[i], pts[i+1], (255, 0, 0), 8)
        
        # 중심선 (빨간색)
        if lane_info.center_lane_points:
            pts = [(int(p.x), int(p.y)) for p in lane_info.center_lane_points]
            for i in range(len(pts) - 1):
                cv2.line(overlay, pts[i], pts[i+1], (0, 0, 255), 4)
        
        # 차선 사이 영역 채우기 (반투명 녹색)
        if lane_info.left_lane_detected and lane_info.right_lane_detected:
            left_pts = np.array([(int(p.x), int(p.y)) for p in lane_info.left_lane_points])
            right_pts = np.array([(int(p.x), int(p.y)) for p in lane_info.right_lane_points])
            
            # 폴리곤 생성
            pts = np.vstack([left_pts, right_pts[::-1]])
            cv2.fillPoly(overlay, [pts], (0, 255, 0))
        
        # 오버레이 합성
        cv2.addWeighted(overlay, 0.3, output, 1.0, 0, output)
        
        # 정보 텍스트 (배경 추가)
        y_offset = 30
        info_texts = [
            f"Left: {'YES' if lane_info.left_lane_detected else 'NO'}",
            f"Right: {'YES' if lane_info.right_lane_detected else 'NO'}",
            f"Offset: {lane_info.center_offset:.3f} m",
            f"Heading: {np.degrees(lane_info.heading_error):.1f} deg",
            f"Confidence: {lane_info.confidence:.2f}"
        ]
        
        for i, text in enumerate(info_texts):
            y_pos = y_offset + i * 35
            # 배경 박스
            (text_w, text_h), _ = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 0.7, 2)
            cv2.rectangle(output, (5, y_pos - 25), (15 + text_w, y_pos + 5), (0, 0, 0), -1)
            # 텍스트
            color = (0, 255, 0) if i < 2 and 'YES' in text else (255, 255, 255)
            cv2.putText(output, text, (10, y_pos), cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)
        
        return output
    
    def run(self):
        rospy.spin()


if __name__ == '__main__':
    try:
        node = HybridLaneDetectionNode()
        node.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("[Hybrid Lane] 종료")

