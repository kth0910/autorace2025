#!/usr/bin/env python3
import math, os
import numpy as np
import cv2
import rospy
import tf2_ros
import tf2_geometry_msgs
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

def polyfit_line(points):
    if len(points) < 2:
        return None
    pts = np.array(points)
    # x = a*y + b (이미지 좌표는 y가 아래로 증가)
    a, b = np.polyfit(pts[:,1], pts[:,0], 1)
    return a, b

def intersect_x(line, y):
    a, b = line
    return a*y + b

def pixel_to_ground_scale(px, py, img_w, img_h, params):
    # 기준: 이미지 중심이 x=0, 바닥(맨 아래)이 y=0 [m]
    cx   = params.get("center_col", img_w/2.0)
    sx   = params.get("meters_per_pixel_x", 0.0025) # 약 0.25cm/px (예시)
    sy   = params.get("meters_per_pixel_y", 0.0040) # 약 0.40cm/px (예시)
    # 아래로 갈수록 앞쪽(전방 +y)
    x_m = (px - cx) * sx
    y_m = (img_h - 1 - py) * sy
    return x_m, y_m

def pixel_to_ground_homography(px, py, H):
    v = np.array([px, py, 1.0], dtype=np.float64)
    X = H.dot(v)
    if abs(X[2]) < 1e-6: X[2] = 1e-6
    Xn = X / X[2]
    return float(Xn[0]), float(Xn[1])

class LaneDetectorNode:
    def __init__(self):
        self.bridge = CvBridge()
        # === Params ===
        self.image_topic  = rospy.get_param("~image_topic", "/camera/image_raw")
        self.output_frame = rospy.get_param("~output_frame", "base_link")
        self.path_topic   = rospy.get_param("~path_topic", "/vision/lane_path")
        self.debug_topic  = rospy.get_param("~debug_topic", "/vision/debug_image")
        self.mode         = rospy.get_param("~mode", "scale")  # "scale" | "homography"
        self.scale_params = rospy.get_param("~scale_params", {}) # meters_per_pixel_x/y, center_col
        Hlist             = rospy.get_param("~H", [1,0,0, 0,1,0, 0,0,1])
        self.H            = np.array(Hlist, dtype=np.float64).reshape(3,3)

        # goal 발행 옵션
        self.publish_goal = rospy.get_param("~publish_goal", True)
        self.goal_topic   = rospy.get_param("~goal_topic", "/vision/goal")
        self.lookahead_m  = rospy.get_param("~lookahead_m", 1.5)
        self.goal_rate_hz = float(rospy.get_param("~goal_rate_hz", 0.5))

        self.pub_path  = rospy.Publisher(self.path_topic, Path, queue_size=1, latch=True)
        self.pub_debug = rospy.Publisher(self.debug_topic, Image, queue_size=1)
        self.pub_goal  = rospy.Publisher(self.goal_topic, PoseStamped, queue_size=1) if self.publish_goal else None

        self.tfbuf = tf2_ros.Buffer(cache_time=rospy.Duration(5.0))
        self.tflis = tf2_ros.TransformListener(self.tfbuf)
        self.last_goal_stamp = rospy.Time(0)

        self.sub = rospy.Subscriber(self.image_topic, Image, self.cb_image, queue_size=1, buff_size=2**24)
        rospy.loginfo("lane_detector_node: image=%s, mode=%s, output_frame=%s", self.image_topic, self.mode, self.output_frame)

    def cb_image(self, msg):
        # 1) 이미지 → OpenCV
        try:
            cvimg = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            rospy.logwarn("cv_bridge failed: %s", e); return
        Himg, Wimg = cvimg.shape[:2]

        # 2) 전처리 (ROI: 하단 60%)
        roi = cvimg[int(Himg*0.4):, :]
        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        blur = cv2.GaussianBlur(gray, (5,5), 1.5)
        edges = cv2.Canny(blur, 50, 150)

        # 3) 허프 직선
        lines = cv2.HoughLinesP(edges, 1, np.pi/180, threshold=50, minLineLength=30, maxLineGap=20)
        left_pts, right_pts = [], []
        if lines is not None:
            for l in lines[:,0,:]:
                x1,y1,x2,y2 = map(int, l)
                y1g = y1 + int(Himg*0.4)
                y2g = y2 + int(Himg*0.4)
                dx, dy = x2-x1, y2-y1
                if abs(dx) < 5: continue
                slope = float(dy)/float(dx)
                # 화면 기준 왼쪽/오른쪽 분류
                if slope < 0:
                    left_pts.append((x1, y1g)); left_pts.append((x2, y2g))
                else:
                    right_pts.append((x1, y1g)); right_pts.append((x2, y2g))
                cv2.line(cvimg, (x1, y1g), (x2, y2g), (0,255,255), 2)

        left_line  = polyfit_line(left_pts)  if len(left_pts)  >= 2 else None
        right_line = polyfit_line(right_pts) if len(right_pts) >= 2 else None

        # 4) 중앙 라인 샘플링 (이미지 y값을 아래→위로 일정 간격)
        ys = np.linspace(Himg-1, int(Himg*0.55), 25)  # 하단 45% 구간
        centers_px = []
        if left_line is not None and right_line is not None:
            for y in ys:
                xl = intersect_x(left_line,  y)
                xr = intersect_x(right_line, y)
                xc = 0.5*(xl+xr)
                centers_px.append((xc, y))
        else:
            # 직선 하나만 잡힌 경우: 그 라인에 대해 일정 오프셋으로 중앙을 가정(보수적)
            base = left_line if left_line is not None else right_line
            if base is not None:
                for y in ys:
                    x = intersect_x(base, y)
                    # 트랙 폭 0.5m 가정 → 픽셀 단위로 근사 (scale 모드에서만 의미 있음)
                    offset_px = 0.5 / max(self.scale_params.get("meters_per_pixel_x", 0.0025), 1e-6) * ( -1.0 if base is left_line else +1.0)
                    centers_px.append((x + offset_px, y))

        # 5) Path 생성 (픽셀 → ground 좌표)
        path = Path()
        path.header.stamp = msg.header.stamp
        path.header.frame_id = self.output_frame
        pts3d = []

        for (px, py) in centers_px:
            if self.mode == "homography":
                gx, gy = pixel_to_ground_homography(px, py, self.H)
            else:
                gx, gy = pixel_to_ground_scale(px, py, Wimg, Himg, self.scale_params)
            # output_frame 기준 포즈
            ps = PoseStamped()
            ps.header = path.header
            ps.pose.position.x = gx
            ps.pose.position.y = gy
            ps.pose.orientation.w = 1.0
            path.poses.append(ps)
            pts3d.append((gx, gy))

        if len(path.poses) >= 2:
            self.pub_path.publish(path)

        # 6) goal 발행(주기 제한)
        if self.publish_goal and len(path.poses) >= 2:
            now = rospy.Time.now()
            if (now - self.last_goal_stamp).to_sec() >= 1.0/max(self.goal_rate_hz,1e-6):
                # 룩어헤드 거리 지점 찾기
                acc = 0.0
                prev = path.poses[0].pose.position
                goal_pose = path.poses[-1]  # fallback
                for i in range(1, len(path.poses)):
                    cur = path.poses[i].pose.position
                    seg = math.hypot(cur.x - prev.x, cur.y - prev.y)
                    if acc + seg >= self.lookahead_m:
                        t = (self.lookahead_m - acc)/max(seg,1e-6)
                        gx = prev.x + t*(cur.x - prev.x)
                        gy = prev.y + t*(cur.y - prev.y)
                        goal_pose = PoseStamped()
                        goal_pose.header = path.header
                        goal_pose.pose.position.x = gx
                        goal_pose.pose.position.y = gy
                        goal_pose.pose.orientation.w = 1.0
                        break
                    acc += seg
                    prev = cur
                self.pub_goal.publish(goal_pose)
                self.last_goal_stamp = now

        # 7) 디버그 영상 출력
        try:
            dbg = cvimg.copy()
            for (x,y) in centers_px:
                cv2.circle(dbg, (int(x), int(y)), 3, (0,255,0), -1)
            self.pub_debug.publish(self.bridge.cv2_to_imgmsg(dbg, encoding="bgr8"))
        except Exception:
            pass
