#!/usr/bin/env bash
set -euo pipefail

task_workspace="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
cd "$task_workspace"

env -i HOME="$HOME" USER="${USER:-alice}" PATH=/usr/local/bin:/usr/bin:/bin \
  LANG="${LANG:-C.UTF-8}" CMAKE_BUILD_PARALLEL_LEVEL=1 MAKEFLAGS=-j1 \
  bash --noprofile --norc -c '
    set -e
    source /opt/ros/humble/setup.bash
    exec colcon build --symlink-install --executor sequential \
      --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF -DDISTRO_ROS=humble "$@"
  ' bash "$@"
