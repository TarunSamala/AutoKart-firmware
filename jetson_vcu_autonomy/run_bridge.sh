#!/usr/bin/env bash
set -euo pipefail

[[ $# -eq 1 ]] || {
    echo "Usage: $0 /dev/serial/by-id/ESP32_PORT" >&2
    exit 2
}
ROOT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
PORT=$1
[[ -e "$PORT" ]] || { echo "Port does not exist: $PORT" >&2; exit 1; }
[[ -f "$ROOT/ros2_ws/install/setup.bash" ]] || {
    echo "Build first: bash scripts/build_ros.sh" >&2
    exit 1
}

source /opt/ros/humble/setup.bash
source "$ROOT/ros2_ws/install/setup.bash"
exec ros2 run vcu_bridge vcu_bridge_node --ros-args \
    -p port:="$PORT" -p baud:=115200
