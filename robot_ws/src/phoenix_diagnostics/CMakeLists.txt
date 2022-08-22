cmake_minimum_required(VERSION 3.0.2)
project(phoenix_diagnostics)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES perception_diagnostics
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)


install(DIRECTORY scripts
	  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY params
	  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})


install(DIRECTORY launch
	  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

catkin_install_python(PROGRAMS
	scripts/phoenix_diagnostics.py
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/scripts
	)

catkin_install_python(PROGRAMS
	scripts/phoenix_diagnostics_node.py
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/scripts
	)
