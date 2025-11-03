cmake_minimum_required(VERSION 3.0.2)
project(autorace_planning)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  nav_msgs
  geometry_msgs
  sensor_msgs
  tf
  tf2_ros
  tf2_geometry_msgs
  move_base
  move_base_msgs
  cv_bridge
  image_transport
)


catkin_package()

catkin_install_python(PROGRAMS
  scripts/send_goal.py
  scripts/lane_detector_node.py
  scripts/lane_follow_adapter.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

