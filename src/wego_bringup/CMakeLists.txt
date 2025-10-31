cmake_minimum_required(VERSION 3.0.2)
project(wego_bringup)

## Find catkin macros and libraries
find_package(catkin REQUIRED)

###################################
## catkin specific configuration ##
###################################
catkin_package()

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

