#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
WeGO Vision - Lane Detection Node
차선을 검출하고 Planning 노드에 필요한 정보를 발행합니다.

Topic Subscriptions:
  - /vision/image_rect (sensor_msgs/Image): 보정된 카메라 이미지

Topic Publications:
  - /vision/lane_info (wego_vision/LaneInfo): 차선 정보
  - /vision/lane_image (sensor_msgs/Image): 차선 검출 결과 이미지 (디버깅용)
"""

import rospy
import numpy as np
import cv2
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from std_msgs.msg import Header
from cv_bridge import CvBridge
from wego_vision.msg import LaneInfo


class LaneDetectionNode:
    def __init__(self):
        rospy.init_node('lane_detection_node', anonymous=False)
        
        # Parameters
        self.image_topic = rospy.get_param('~image_topic', '/vision/image_rect')
        self.lane_info_topic = rospy.get_param('~lane_info_topic', '/vision/lane_info')
        self.debug_image_topic = rospy.get_param('~debug_image_topic', '/vision/lane_image')
        
        # Lane detection parameters
        self.roi_top_ratio = rospy.get_param('~roi_top_ratio', 0.6)
        self.roi_bottom_ratio = rospy.get_param('~roi_bottom_ratio', 1.0)
        
        # Color thresholds (HSV)
        self.white_lower = np.array(rospy.get_param('~white_lower', [0, 0, 200]))
        self.white_upper = np.array(rospy.get_param('~white_upper', [180, 30, 255]))
        self.yellow_lower = np.array(rospy.get_param('~yellow_lower', [20, 100, 100]))
        self.yellow_upper = np.array(rospy.get_param('~yellow_upper', [30, 255, 255]))
        
        # Edge detection
        self.canny_low = rospy.get_param('~canny_low', 50)
        self.canny_high = rospy.get_param('~canny_high', 150)
        
        # Hough transform
        self.hough_threshold = rospy.get_param('~hough_threshold', 20)
        self.hough_min_line_length = rospy.get_param('~hough_min_line_length', 20)
        self.hough_max_line_gap = rospy.get_param('~hough_max_line_gap', 300)
        
        # Lane parameters
        self.lane_width_meters = rospy.get_param('~lane_width_meters', 0.3)  # 차선 폭 (미터)
        self.camera_height = rospy.get_param('~camera_height', 0.2)  # 카메라 높이 (미터)
        self.pixels_per_meter = rospy.get_param('~pixels_per_meter', 500)  # 픽셀/미터 변환
        
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
        
        rospy.loginfo("[Lane Detection] 초기화 완료")
        rospy.loginfo(f"  - 입력 이미지: {self.image_topic}")
        rospy.loginfo(f"  - 차선 정보 출력: {self.lane_info_topic}")
        rospy.loginfo(f"  - ROI: {self.roi_top_ratio:.1%} ~ {self.roi_bottom_ratio:.1%}")
    
    def image_callback(self, msg):
        """카메라 이미지에서 차선 검출"""
        try:
            # ROS Image → OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # 이미지 크기 저장
            if self.image_height is None:
                self.image_height, self.image_width = cv_image.shape[:2]
                rospy.loginfo(f"[Lane Detection] 이미지 크기: {self.image_width}x{self.image_height}")
            
            # 차선 검출
            lane_info = self.detect_lanes(cv_image, msg.header)
            
            # LaneInfo 발행
            self.lane_info_pub.publish(lane_info)
            
            # 디버그 이미지 발행
            if self.publish_debug_image:
                debug_image = self.draw_lanes(cv_image, lane_info)
                debug_msg = self.bridge.cv2_to_imgmsg(debug_image, encoding='bgr8')
                debug_msg.header = msg.header
                self.debug_image_pub.publish(debug_msg)
            
        except Exception as e:
            rospy.logerr(f"[Lane Detection] 이미지 처리 오류: {e}")
    
    def detect_lanes(self, image, header):
        """차선 검출 메인 함수"""
        lane_info = LaneInfo()
        lane_info.header = header
        
        # 1. ROI 설정
        roi_image = self.apply_roi(image)
        
        # 2. 색상 기반 차선 검출 (흰색 + 노란색)
        white_mask = self.detect_color(roi_image, self.white_lower, self.white_upper)
        yellow_mask = self.detect_color(roi_image, self.yellow_lower, self.yellow_upper)
        combined_mask = cv2.bitwise_or(white_mask, yellow_mask)
        
        # 3. Edge detection
        edges = cv2.Canny(combined_mask, self.canny_low, self.canny_high)
        
        # 4. Hough Line Transform
        lines = cv2.HoughLinesP(
            edges,
            rho=1,
            theta=np.pi/180,
            threshold=self.hough_threshold,
            minLineLength=self.hough_min_line_length,
            maxLineGap=self.hough_max_line_gap
        )
        
        # 5. 차선 분류 (좌/우)
        left_lines, right_lines = self.classify_lines(lines)
        
        # 6. 차선 피팅 (2차 다항식)
        if left_lines:
            lane_info.left_lane_detected = True
            lane_info.left_a, lane_info.left_b, lane_info.left_c, lane_info.left_lane_points = \
                self.fit_lane(left_lines)
        
        if right_lines:
            lane_info.right_lane_detected = True
            lane_info.right_a, lane_info.right_b, lane_info.right_c, lane_info.right_lane_points = \
                self.fit_lane(right_lines)
        
        # 7. 중심선 및 오프셋 계산
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
    
    def apply_roi(self, image):
        """관심 영역(ROI) 적용"""
        mask = np.zeros_like(image)
        
        height, width = image.shape[:2]
        roi_top = int(height * self.roi_top_ratio)
        roi_bottom = int(height * self.roi_bottom_ratio)
        
        # 사다리꼴 ROI
        vertices = np.array([[
            (0, roi_bottom),
            (0, roi_top),
            (width, roi_top),
            (width, roi_bottom)
        ]], dtype=np.int32)
        
        cv2.fillPoly(mask, vertices, (255, 255, 255))
        masked_image = cv2.bitwise_and(image, mask)
        
        return masked_image
    
    def detect_color(self, image, lower, upper):
        """특정 색상 범위 검출"""
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, lower, upper)
        return mask
    
    def classify_lines(self, lines):
        """라인을 좌/우 차선으로 분류"""
        if lines is None:
            return [], []
        
        left_lines = []
        right_lines = []
        
        image_center = self.image_width / 2
        
        for line in lines:
            x1, y1, x2, y2 = line[0]
            
            # 기울기 계산
            if x2 - x1 == 0:
                continue
            slope = (y2 - y1) / (x2 - x1)
            
            # 너무 수평인 선 제거
            if abs(slope) < 0.3:
                continue
            
            # 라인 중심점
            center_x = (x1 + x2) / 2
            
            # 좌/우 분류
            if center_x < image_center and slope < 0:
                left_lines.append(line[0])
            elif center_x > image_center and slope > 0:
                right_lines.append(line[0])
        
        return left_lines, right_lines
    
    def fit_lane(self, lines):
        """차선에 2차 다항식 피팅"""
        # 모든 점 수집
        x_points = []
        y_points = []
        
        for line in lines:
            x1, y1, x2, y2 = line
            x_points.extend([x1, x2])
            y_points.extend([y1, y2])
        
        x_points = np.array(x_points)
        y_points = np.array(y_points)
        
        # 2차 다항식 피팅: y = ax^2 + bx + c
        if len(x_points) >= 3:
            coeffs = np.polyfit(y_points, x_points, 2)
            a, b, c = coeffs
        else:
            a, b, c = 0.0, 0.0, 0.0
        
        # 차선 포인트 생성 (시각화용)
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
        """차선 중심으로부터의 오프셋 계산"""
        # 이미지 하단에서 좌/우 차선의 x 좌표
        y_eval = self.image_height - 1
        
        left_x = lane_info.left_a * y_eval**2 + lane_info.left_b * y_eval + lane_info.left_c
        right_x = lane_info.right_a * y_eval**2 + lane_info.right_b * y_eval + lane_info.right_c
        
        # 차선 중심
        lane_center = (left_x + right_x) / 2
        
        # 차량 중심 (이미지 중심)
        vehicle_center = self.image_width / 2
        
        # 픽셀 오프셋
        pixel_offset = vehicle_center - lane_center
        
        # 미터로 변환
        center_offset = pixel_offset / self.pixels_per_meter
        
        # Heading error 계산 (차선 방향)
        left_slope = 2 * lane_info.left_a * y_eval + lane_info.left_b
        right_slope = 2 * lane_info.right_a * y_eval + lane_info.right_b
        avg_slope = (left_slope + right_slope) / 2
        heading_error = np.arctan(1.0 / avg_slope) if avg_slope != 0 else 0.0
        
        # 중심선 포인트
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
        """차선 곡률 계산"""
        y_eval = self.image_height - 1
        
        # 좌/우 차선 곡률 평균
        left_curve = abs(2 * lane_info.left_a)
        right_curve = abs(2 * lane_info.right_a)
        
        avg_curve = (left_curve + right_curve) / 2
        
        # 픽셀 → 미터 변환 적용
        curvature = avg_curve / (self.pixels_per_meter ** 2)
        
        return float(curvature)
    
    def draw_lanes(self, image, lane_info):
        """검출된 차선을 이미지에 그리기"""
        output = image.copy()
        
        # 반투명 오버레이
        overlay = np.zeros_like(output)
        
        # 좌측 차선 (녹색)
        if lane_info.left_lane_detected and lane_info.left_lane_points:
            pts = [(int(p.x), int(p.y)) for p in lane_info.left_lane_points]
            for i in range(len(pts) - 1):
                cv2.line(overlay, pts[i], pts[i+1], (0, 255, 0), 3)
        
        # 우측 차선 (파란색)
        if lane_info.right_lane_detected and lane_info.right_lane_points:
            pts = [(int(p.x), int(p.y)) for p in lane_info.right_lane_points]
            for i in range(len(pts) - 1):
                cv2.line(overlay, pts[i], pts[i+1], (255, 0, 0), 3)
        
        # 중심선 (빨간색)
        if lane_info.center_lane_points:
            pts = [(int(p.x), int(p.y)) for p in lane_info.center_lane_points]
            for i in range(len(pts) - 1):
                cv2.line(overlay, pts[i], pts[i+1], (0, 0, 255), 2)
        
        # 오버레이 합성
        cv2.addWeighted(overlay, 0.7, output, 1.0, 0, output)
        
        # 정보 텍스트
        y_offset = 30
        cv2.putText(output, f"Center Offset: {lane_info.center_offset:.3f} m", 
                   (10, y_offset), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        cv2.putText(output, f"Heading Error: {np.degrees(lane_info.heading_error):.2f} deg", 
                   (10, y_offset + 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        cv2.putText(output, f"Curvature: {lane_info.curvature:.4f}", 
                   (10, y_offset + 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        cv2.putText(output, f"Confidence: {lane_info.confidence:.2f}", 
                   (10, y_offset + 90), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        
        return output
    
    def run(self):
        rospy.spin()


if __name__ == '__main__':
    try:
        node = LaneDetectionNode()
        node.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("[Lane Detection] 종료")

