cmake_minimum_required(VERSION 3.0.2)
project(wego_bringup)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  wego_vision
  wego_planning
  wego_control
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  CATKIN_DEPENDS
    wego_vision
    wego_planning
    wego_control
)

#############
## Install ##
#############

# Install launch files
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

# Install config files
install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)

