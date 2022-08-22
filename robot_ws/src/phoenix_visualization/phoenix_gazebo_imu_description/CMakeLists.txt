cmake_minimum_required(VERSION 3.5.0)
project(phoenix_gazebo_imu_description)

find_package(catkin REQUIRED)

catkin_package()

install(DIRECTORY urdf/
  DESTINATION share/${PROJECT_NAME}/urdf
)
