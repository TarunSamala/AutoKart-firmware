#!/usr/bin/env bash
set -euo pipefail

ROOT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
source /opt/ros/humble/setup.bash
source "$ROOT/ros2_ws/install/setup.bash"

echo "=== Identity ==="
timeout 5 ros2 topic echo --once /autokart/vcu/identity
echo "=== Link ==="
timeout 5 ros2 topic echo --once /autokart/vcu/link_status
echo "=== Wheel encoder ==="
timeout 5 ros2 topic echo --once /autokart/vcu/wheel_encoder
echo "=== Steering feedback ==="
timeout 5 ros2 topic echo --once /autokart/vcu/steering_feedback
echo "=== VCU state ==="
timeout 5 ros2 topic echo --once /autokart/vcu/state
