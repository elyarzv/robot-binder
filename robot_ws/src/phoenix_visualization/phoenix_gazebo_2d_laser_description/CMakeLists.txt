cmake_minimum_required(VERSION 3.5.0)
project(phoenix_gazebo_2d_laser_description)

find_package(catkin REQUIRED)

catkin_package()

install(DIRECTORY urdf/
  DESTINATION share/${PROJECT_NAME}/urdf
)

install(DIRECTORY meshes/
  DESTINATION share/${PROJECT_NAME}/meshes
)
