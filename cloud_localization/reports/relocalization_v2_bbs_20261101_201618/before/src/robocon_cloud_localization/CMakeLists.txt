cmake_minimum_required(VERSION 3.10)
project(robocon_cloud_localization)
set(CMAKE_CXX_STANDARD 17)

if(POLICY CMP0074)
  cmake_policy(SET CMP0074 NEW)
endif()

find_package(ament_cmake REQUIRED)

ament_environment_hooks(
  "${CMAKE_CURRENT_SOURCE_DIR}/env-hooks/robocon_workspace.sh.in"
)

# Gazebo Classic's config file loads FindPkgConfig while gazebo is the active
# find_package() call. Silence that known nested-name mismatch in CMake 3.22.
set(FPHSA_NAME_MISMATCHED TRUE)
find_package(gazebo REQUIRED)
unset(FPHSA_NAME_MISMATCHED)

find_package(gazebo_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io filters registration)
find_package(GTSAM REQUIRED PATHS "${CMAKE_CURRENT_SOURCE_DIR}/../../vendor/install/lib/cmake/GTSAM" NO_DEFAULT_PATH)
add_library(mid360_sim SHARED src/mid360_sim.cpp)
target_include_directories(mid360_sim PRIVATE ${GAZEBO_INCLUDE_DIRS})
target_link_directories(mid360_sim PRIVATE ${GAZEBO_LIBRARY_DIRS})
target_link_libraries(mid360_sim ${GAZEBO_LIBRARIES})
ament_target_dependencies(mid360_sim gazebo_ros rclcpp sensor_msgs geometry_msgs nav_msgs)
add_executable(loop_closure_pgo src/loop_closure_pgo.cpp src/global_relocalization.cpp)
target_include_directories(loop_closure_pgo PRIVATE ${PCL_INCLUDE_DIRS})
target_link_libraries(loop_closure_pgo ${PCL_LIBRARIES} gtsam)
ament_target_dependencies(loop_closure_pgo rclcpp sensor_msgs nav_msgs geometry_msgs std_srvs tf2_ros pcl_conversions)
install(TARGETS mid360_sim LIBRARY DESTINATION lib)
install(TARGETS loop_closure_pgo DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config worlds models urdf maps DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/drive.py scripts/verify_sim.py DESTINATION lib/${PROJECT_NAME})
ament_package()
