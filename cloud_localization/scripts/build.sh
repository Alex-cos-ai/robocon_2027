#!/usr/bin/env bash
set -eo pipefail
source "$(dirname -- "${BASH_SOURCE[0]}")/env.sh"
cd "$CLOUD_WS_ROOT"
if [[ ! -f vendor/install/lib/cmake/GTSAM/GTSAMConfig.cmake ]]; then
  cmake -S vendor/gtsam -B vendor/gtsam-build \
    -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX="$CLOUD_WS_ROOT/vendor/install" \
    -DGTSAM_BUILD_TESTS=OFF -DGTSAM_BUILD_EXAMPLES_ALWAYS=OFF \
    -DGTSAM_BUILD_UNSTABLE=OFF -DGTSAM_BUILD_PYTHON=OFF \
    -DGTSAM_WITH_TBB=OFF -DGTSAM_USE_SYSTEM_EIGEN=ON
  cmake --build vendor/gtsam-build --target install --parallel 2
fi
colcon build --symlink-install --executor sequential --parallel-workers 1 \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF
