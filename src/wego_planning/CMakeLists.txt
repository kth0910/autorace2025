cmake_minimum_required(VERSION 3.0.2)
project(wego_planning)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Python setup
catkin_python_setup()

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  std_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
  tf
  tf2_ros
  tf2_geometry_msgs
  move_base
  move_base_msgs
  cv_bridge
  image_transport
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  CATKIN_DEPENDS
    std_msgs
    geometry_msgs
    nav_msgs
    sensor_msgs
    tf
    tf2_ros
    tf2_geometry_msgs
    move_base
    move_base_msgs
)

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(
  ${catkin_INCLUDE_DIRS}
)

#############
## Install ##
#############

# Install Python scripts
catkin_install_python(PROGRAMS
  scripts/path_planner_node.py
  scripts/local_planner_node.py
  scripts/path_planner_with_lane.py
  scripts/send_goal.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Install launch files
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

# Install config files
install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)

