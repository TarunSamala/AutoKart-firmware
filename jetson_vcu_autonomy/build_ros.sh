#!/usr/bin/env bash
set -euo pipefail

ROOT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
[[ -f /opt/ros/humble/setup.bash ]] || {
    echo "Missing /opt/ros/humble/setup.bash" >&2
    exit 1
}

cd "$ROOT/ros2_ws"
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-up-to vcu_bridge \
    --event-handlers console_direct+

source "$ROOT/ros2_ws/install/setup.bash"
ros2 pkg prefix autonomous_odometry
ros2 pkg prefix vcu_bridge
echo "ROS 2 VCU and odometry workspace built."
