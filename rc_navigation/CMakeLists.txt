cmake_minimum_required(VERSION 3.8)
project(rc_navigation)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config launch maps models rviz urdf worlds
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
