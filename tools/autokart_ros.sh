#!/usr/bin/env bash
set -euo pipefail

container="${AUTOKART_DISTROBOX:-kart-ros2-humble}"
ros_root="${AUTOKART_ROS_ROOT:-/home/satvara/Github/AutoKart-ROS/autokart_model}"
venv="${AUTOKART_ROS_VENV:-/home/satvara/.venvs/autokart-ros2-humble}"
workspace="$ros_root/ros2_ws"
perception_src="$ros_root/runtime/phase_01_perception/src"

if ! command -v distrobox >/dev/null 2>&1; then
    echo "distrobox is not installed on the host" >&2
    exit 2
fi

if [[ ! -d "$ros_root" ]]; then
    echo "AutoKart ROS checkout not found: $ros_root" >&2
    exit 2
fi

bootstrap='set -e
source /opt/ros/humble/setup.bash
source "'"$venv"'/bin/activate"
export AUTOKART_ROS_ROOT="'"$ros_root"'"
export PYTHONPATH="'"$perception_src"':${PYTHONPATH:-}"
if [[ -f "'"$workspace"'/install/local_setup.bash" ]]; then
    source "'"$workspace"'/install/local_setup.bash"
fi
cd "'"$ros_root"'"
'

if (( $# == 0 )); then
    exec distrobox enter "$container" -- bash -lc "$bootstrap
echo \"AutoKart ROS 2 Humble environment ready\"
echo \"Workspace: \$AUTOKART_ROS_ROOT\"
exec bash -i"
fi

printf -v command '%q ' "$@"
exec distrobox enter "$container" -- bash -lc "$bootstrap
$command"
