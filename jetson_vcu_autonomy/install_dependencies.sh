#!/usr/bin/env bash
set -euo pipefail

[[ $(uname -m) == aarch64 ]] || {
    echo "WARNING: this deployment profile targets Jetson aarch64." >&2
}
[[ -f /opt/ros/humble/setup.bash ]] || {
    echo "ROS 2 Humble is not installed at /opt/ros/humble." >&2
    echo "Install ROS 2 Humble for Ubuntu 22.04, then rerun this script." >&2
    exit 1
}

sudo apt-get update
sudo apt-get install -y \
    python3-colcon-common-extensions \
    python3-numpy \
    python3-pytest \
    python3-serial \
    python3-smbus2 \
    ros-humble-ament-cmake-python \
    ros-humble-diagnostic-msgs \
    ros-humble-nav-msgs \
    ros-humble-rosidl-default-generators \
    ros-humble-sensor-msgs \
    ros-humble-tf2-ros

echo "Jetson VCU/odometry dependencies installed."
