cmake_minimum_required(VERSION 3.8)
project(robocon2027_description)

find_package(ament_cmake REQUIRED)
find_package(xacro REQUIRED)
find_package(robot_state_publisher REQUIRED)

install(
  DIRECTORY
    meshes
    urdf
    worlds
    launch
    config
  DESTINATION share/${PROJECT_NAME}
)

# Keep the cloned Gazebo field model unchanged, while making it available from
# the installed ROS package used by both launch files.
set(EXTERNAL_GAZEBO_MODELS "${CMAKE_CURRENT_SOURCE_DIR}/../../gazebo_models")
install(
  DIRECTORY "${EXTERNAL_GAZEBO_MODELS}/robocon_ground"
  DESTINATION share/${PROJECT_NAME}/gazebo_models
)
install(
  FILES "${EXTERNAL_GAZEBO_MODELS}/robocon_track.world"
  DESTINATION share/${PROJECT_NAME}/gazebo_models
)
install(
  DIRECTORY "${EXTERNAL_GAZEBO_MODELS}/robocon_tops"
  DESTINATION share/${PROJECT_NAME}/gazebo_models
)

# Install the TurtleBot3 models beside the field so model:// URIs resolve.
install(
  DIRECTORY
    "${EXTERNAL_GAZEBO_MODELS}/turtlebot3_burger"
    "${EXTERNAL_GAZEBO_MODELS}/turtlebot3_waffle"
    "${EXTERNAL_GAZEBO_MODELS}/turtlebot3_waffle_pi"
    "${EXTERNAL_GAZEBO_MODELS}/turtlebot3_common"
  DESTINATION share/${PROJECT_NAME}/gazebo_models
)

install(
  PROGRAMS
    scripts/mecanum_controller.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME mecanum_controller
)

ament_package()
