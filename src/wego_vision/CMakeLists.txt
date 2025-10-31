cmake_minimum_required(VERSION 3.0.2)
project(wego_vision)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Python setup
catkin_python_setup()

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  cv_bridge
  image_transport
  message_filters
  message_generation
)

## System dependencies
find_package(OpenCV REQUIRED)

################################################
## Declare ROS messages, services and actions ##
################################################

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  LaneInfo.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  CATKIN_DEPENDS
    std_msgs
    sensor_msgs
    geometry_msgs
    nav_msgs
    cv_bridge
    image_transport
    message_filters
    message_runtime
)

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

#############
## Install ##
#############

# Install Python scripts
catkin_install_python(PROGRAMS
  scripts/camera_node.py
  scripts/detection_node.py
  scripts/fusion_node.py
  scripts/lane_detection_node.py
  scripts/lane_detection_edge_based.py
  scripts/lane_detection_hybrid.py
  scripts/lane_detection_simple.py
  scripts/simple_camera_publisher.py
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

