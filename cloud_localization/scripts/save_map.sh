#!/usr/bin/env bash
set -eo pipefail
source "$(dirname -- "${BASH_SOURCE[0]}")/env.sh"
exec ros2 service call /save_map std_srvs/srv/Trigger '{}'
