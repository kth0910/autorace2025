#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
WeGO Planning - Send Goal Node
Move Base에 목표 지점을 전송합니다.

Usage:
  rosrun wego_planning send_goal.py _x:=2.0 _y:=1.0
"""

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal


def send_goal(x, y, yaw_w=1.0, frame="map"):
    """
    Move Base에 목표 위치 전송
    
    Args:
        x: 목표 x 좌표 (m)
        y: 목표 y 좌표 (m)
        yaw_w: 목표 방향 (quaternion w)
        frame: 좌표계 프레임
    
    Returns:
        MoveBase 실행 결과
    """
    client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
    rospy.loginfo("[Send Goal] Move Base 서버 대기 중...")
    client.wait_for_server()
    
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = frame
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = x
    goal.target_pose.pose.position.y = y
    goal.target_pose.pose.position.z = 0.0
    goal.target_pose.pose.orientation.x = 0.0
    goal.target_pose.pose.orientation.y = 0.0
    goal.target_pose.pose.orientation.z = 0.0
    goal.target_pose.pose.orientation.w = yaw_w
    
    rospy.loginfo(f"[Send Goal] 목표 전송: ({x:.2f}, {y:.2f}) in {frame}")
    client.send_goal(goal)
    
    rospy.loginfo("[Send Goal] 목표 도달 대기 중...")
    client.wait_for_result()
    
    result = client.get_result()
    state = client.get_state()
    
    if state == actionlib.GoalStatus.SUCCEEDED:
        rospy.loginfo("[Send Goal] 목표 도달 성공!")
    else:
        rospy.logwarn(f"[Send Goal] 목표 도달 실패 (상태: {state})")
    
    return result


if __name__ == "__main__":
    rospy.init_node("send_goal")
    
    # 파라미터 읽기
    x = rospy.get_param("~x", 2.0)
    y = rospy.get_param("~y", 0.0)
    yaw_w = rospy.get_param("~yaw_w", 1.0)
    frame = rospy.get_param("~frame", "map")
    
    try:
        result = send_goal(x, y, yaw_w, frame)
        rospy.loginfo(f"[Send Goal] 결과: {result}")
    except rospy.ROSInterruptException:
        rospy.loginfo("[Send Goal] 종료")
    except Exception as e:
        rospy.logerr(f"[Send Goal] 오류: {e}")

