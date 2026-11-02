#!/usr/bin/env bash
# Source only the system ROS installation and THIS workspace's local overlay.
export CLOUD_WS_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH ROS_PACKAGE_PATH
unset GAZEBO_MODEL_PATH GAZEBO_PLUGIN_PATH GAZEBO_RESOURCE_PATH
unset PYTHONPATH LD_LIBRARY_PATH
source /opt/ros/humble/setup.bash
if [[ -f "$CLOUD_WS_ROOT/install/local_setup.bash" ]]; then
  source "$CLOUD_WS_ROOT/install/local_setup.bash"
fi
# The vendored GTSAM build installs its private METIS library alongside libgtsam.
# Put the system libusb ahead of vendor SDK copies installed in ld.so.conf.d.
export LD_LIBRARY_PATH="$CLOUD_WS_ROOT/vendor/install/lib:/lib/x86_64-linux-gnu${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
export GAZEBO_MODEL_DATABASE_URI=""
# Do not force ROS_LOCALHOST_ONLY here. A teleop node launched from a plain
# ROS-sourced terminal must be able to discover the simulator even if that
# terminal does not source this workspace.
unset ROS_LOCALHOST_ONLY
export GAZEBO_IP=127.0.0.1
