#!/usr/bin/env bash
set -eo pipefail
source "$(dirname -- "${BASH_SOURCE[0]}")/env.sh"
cd "$CLOUD_WS_ROOT"
exec ros2 launch robocon_cloud_localization simulation.launch.py "$@"
