cmake_minimum_required(VERSION 3.0.2)
project(phoenix_msgs)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  actionlib_msgs
  message_generation
  std_msgs
  std_srvs
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  RailGoalPoint.msg
  RailMission.msg
  DimmingIrradianceData.msg
  InternetSpeed.msg
)

## Generate services in the 'srv' folder
add_service_files(
  FILES
  CreateRailMission.srv
  ListOfRailMissions.srv
  SendRailMission.srv
  SendRailGoal.srv
  StartLocalization.srv
  RailMissionName.srv
  SetLampStatus.srv
  LedPattern.srv
  startMission.srv
  setMode.srv
)

## Generate actions in the 'action' folder
add_action_files(
  FILES
  SendRailGoal.action
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  actionlib_msgs
  std_msgs
  std_srvs
)

catkin_package(
 INCLUDE_DIRS include
 CATKIN_DEPENDS actionlib_msgs message_generation std_msgs
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

install (DIRECTORY include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
    FILES_MATCHING PATTERN "*.h"
)