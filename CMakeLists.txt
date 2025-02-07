cmake_minimum_required(VERSION 3.0.2)
project(dataset_builder)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  audio_common_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
)

# include the following for python libaries added to the include directory
#catkin_python_setup()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES speech_recognition
#  CATKIN_DEPENDS audio_common_msgs roscpp rospy std_msgs
#  DEPENDS system_lib
)

catkin_install_python(PROGRAMS
  scripts/config_params.py
  scripts/start_stop.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)