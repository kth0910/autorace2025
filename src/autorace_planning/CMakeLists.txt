cmake_minimum_required(VERSION 3.0.2)
project(autorace_planning)

find_package(catkin REQUIRED COMPONENTS
  roscpp rospy std_msgs nav_msgs geometry_msgs tf move_base move_base_msgs
)

catkin_package()

# 파이썬 스크립트 설치(선택, 없어도 rosrun은 동작)
catkin_install_python(PROGRAMS
  scripts/send_goal.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
