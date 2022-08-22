cmake_minimum_required(VERSION 3.0.2)
project(encoder_simulator)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  nav_msgs
)

catkin_package(

)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES encoder_simulator
  CATKIN_DEPENDS roscpp std_msgs nav_msgs
#  DEPENDS system_lib
)

include_directories(
 include
  ${catkin_INCLUDE_DIRS}
)


add_executable(
encoder_simulator_node 
src/encoder_simulator_node.cpp
src/encoder_simulator_node_callback.cpp
)


target_link_libraries(encoder_simulator_node ${catkin_LIBRARIES})



add_dependencies(encoder_simulator_node ${catkin_EXPORTED_TARGETS})


install(
  TARGETS encoder_simulator_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


install(
  DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config)

install(
  FILES
  config/stations.yaml
  config/stations_eastlake.yaml
  config/stations_vanbelle.yaml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
) 
