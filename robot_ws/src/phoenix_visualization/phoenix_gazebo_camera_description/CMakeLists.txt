cmake_minimum_required(VERSION 2.8.3)
project(phoenix_gazebo_camera_description)

find_package(catkin REQUIRED)

catkin_package()

install(DIRECTORY realsense_camera
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY urdf
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
