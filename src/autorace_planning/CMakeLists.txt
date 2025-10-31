cmake_minimum_required(VERSION 3.0.2)
project(autorace_planning)

find_package(catkin REQUIRED COMPONENTS
  roscpp rospy std_msgs nav_msgs geometry_msgs sensor_msgs tf move_base move_base_msgs
)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/send_goal.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
