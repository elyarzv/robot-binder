cmake_minimum_required(VERSION 2.8.3)
project(amr_embedded_simulator)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  nav_msgs
  sensor_msgs
  tf
  std_msgs
  message_generation
  genmsg
  std_srvs
  gazebo_msgs
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

find_package(PCL 1.7 REQUIRED)


## Generate services in the 'srv' folder
generate_messages(
  DEPENDENCIES
  geometry_msgs
  std_msgs
  nav_msgs
  std_srvs
)


catkin_package(
  INCLUDE_DIRS include

)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)
link_directories(${PCL_LIBRARY_DIRS})


add_executable(embededsimulator src/embeded_simulator.cpp)


add_dependencies(embededsimulator nav_msgs_generate_messages_cpp)
## Specify libraries to link a library or executable target against


target_link_libraries(embededsimulator
   ${catkin_LIBRARIES}
 )



#############
## Install ##
#############

install(TARGETS embededsimulator
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})        

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination


## Mark executables and/or libraries for installation
install(TARGETS #${PROJECT_NAME} ${PROJECT_NAME}_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

