# AutoKart ROS 2 Humble Distrobox

The Arch host uses the existing `kart-ros2-humble` Ubuntu 22.04 container.
It contains ROS 2 Humble, the C/C++ toolchain, colcon, OpenCV, serial and USB
tools, rosbag2, ONNX, ONNX Runtime, and a dedicated Python environment at:

```text
/home/satvara/.venvs/autokart-ros2-humble
```

## Enter the environment

From the firmware repository:

```bash
./tools/autokart_ros.sh
```

This automatically sources ROS Humble, the Python environment, the built ROS
workspace (when present), and the canonical perception source tree.

Run one command without opening an interactive shell:

```bash
./tools/autokart_ros.sh ros2 doctor --report
./tools/autokart_ros.sh bash scripts/verify_bundle.sh
./tools/autokart_ros.sh bash scripts/build_and_test.sh
```

## Verified on 2026-10-24

- ROS 2 Humble CLI and `rclpy`: pass
- 13-package colcon build: pass
- ROS package tests: 195 total, 193 passed and 2 hardware-dependent skips
- Bundle checksum and YOLO26s model identity: pass
- ONNX Runtime CPU smoke inference: pass
- Native x86-64 HP60C bridge compilation: pass from a clean build directory

Two source-package issues remain outside the container setup:

1. `runtime/phase_01_perception/config/perception.yaml` is a regular text file
   containing a symlink target, rather than an actual symlink. The standalone
   perception suite therefore fails while loading that config.
2. The native HP60C build directory contains a stale CMake cache from
   `/home/mark1/Desktop/autokart_tarun`. Use a clean build directory or remove
   that generated cache before using `scripts/build_hp60c_bridge.sh`.

These are checkout/package defects; adding more container packages will not
correct them.

## Hardware discovery

Distrobox shares the host `/dev`, so USB devices remain host-managed. Discover
ports every time hardware is connected:

```bash
./tools/autokart_ros.sh bash -lc \
  'find /dev -maxdepth 1 \( -name "ttyACM*" -o -name "ttyUSB*" -o -name "video*" \) -print'
./tools/autokart_ros.sh lsusb
./tools/autokart_ros.sh v4l2-ctl --list-devices
```

On Arch, serial devices normally require the `uucp` group. After changing
group membership, log out and back in before testing inside Distrobox.

## Validation boundary

Software builds and simulated tests do not authorize vehicle motion. Camera,
VCU serial, encoder, E-stop, brake response, stopping distance, and end-to-end
latency must still be measured with the real hardware. Keep traction isolated
until the production readiness checks explicitly pass.

The separate `AutoKart-ROS/Brake_Test` checkout currently contains only the
brake-test orchestrator. Its referenced `tools/`, `scripts/`, `kart/`, model,
and perception directories must be restored before stages 0-9 can run.
