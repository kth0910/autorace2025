#!/usr/bin/env python3
import rospy, actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

def send(x, y, yaw_w=1.0, frame="map"):
    client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
    client.wait_for_server()
    g = MoveBaseGoal()
    g.target_pose.header.frame_id = frame
    g.target_pose.header.stamp = rospy.Time.now()
    g.target_pose.pose.position.x = x
    g.target_pose.pose.position.y = y
    g.target_pose.pose.orientation.w = yaw_w
    client.send_goal(g)
    client.wait_for_result()
    return client.get_result()

if __name__ == "__main__":
    rospy.init_node("send_goal")
    x = rospy.get_param("~x", 2.0)
    y = rospy.get_param("~y", 0.0)
    res = send(x, y)
    rospy.loginfo("result: %s", str(res))
