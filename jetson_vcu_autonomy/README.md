# AutoKart Jetson VCU and autonomous odometry development bundle

This source bundle moves the validated laptop VCU telemetry work to the
Jetson Orin Nano Super without copying x86 build products. It contains:

- the ESP32-S3 MicroPython telemetry firmware used in the physical tests;
- the ROS 2 `vcu_bridge` that decodes `HELLO`, `MOTION`, and
  `CONTROL_FEEDBACK` frames;
- the ROS 2 `autonomous_odometry` EKF package and its typed messages;
- read-only serial validation tools and the latest laptop evidence;
- the larger development VCU firmware tree as reference only.

The package is intentionally fail-closed. The checked-in odometry
configuration has zero wheelbase/metres-per-count and an unverified
calibration ID, so it will not publish `/odom` until those measurements are
entered. Do not replace those values with guesses.

## Proven before packaging

- AS5047P is the BLDC/motor encoder; forward count direction is positive.
- AS5600 is steering feedback; left is negative and right is positive.
- No brake encoder is used.
- Firmware telemetry rates: MOTION about 50 Hz, CONTROL_FEEDBACK about 20 Hz,
  HELLO 1 Hz.
- Thirty-second serial validation passed with zero CRC errors, zero sequence
  gaps, validity bits `3`, and fault bits `0`.
- Physical ESP32 reset produced a new boot ID and a clean epoch.
- The ROS bridge opened the live serial link and published typed wheel,
  steering, identity, VCU-state, and link-status topics.

These results are bench evidence, not vehicle authorization. Wheelbase,
metres-per-count, IMU mounting/noise, steering calibration provenance,
stopping distance, and ground-motion behaviour remain incomplete.

## Transfer to Jetson

Copy both files from the laptop:

```bash
scp AutoKart-Jetson-VCU-Autonomy.tar.gz \
    AutoKart-Jetson-VCU-Autonomy.tar.gz.sha256 \
    autokart@192.168.55.1:~/
```

On Jetson:

```bash
cd ~
sha256sum -c AutoKart-Jetson-VCU-Autonomy.tar.gz.sha256
tar -xzf AutoKart-Jetson-VCU-Autonomy.tar.gz
cd AutoKart-Jetson-VCU-Autonomy
sha256sum -c MANIFEST.sha256
```

## Jetson prerequisites

Ubuntu 22.04 and ROS 2 Humble are required. Check first:

```bash
uname -m
test -f /opt/ros/humble/setup.bash && echo 'ROS Humble found'
```

Expected architecture is `aarch64`. If ROS Humble is already installed:

```bash
bash scripts/install_dependencies.sh
bash scripts/build_ros.sh
```

No laptop `build/`, `install/`, virtual environment, or TensorRT engine is in
this archive. Everything is rebuilt natively on the Jetson.

## Flash and validate the telemetry firmware

Keep traction isolated. Stop every dashboard/serial monitor first. Discover
the ESP32 port and pass it explicitly:

```bash
find /dev -maxdepth 1 \( -name 'ttyACM*' -o -name 'ttyUSB*' \) -print
ls -l /dev/serial/by-id/ 2>/dev/null

bash scripts/flash_telemetry_firmware.sh /dev/ttyACM0
python3 host_tools/vcu_serial_validation.py --port auto --duration 30
python3 host_tools/vcu_reboot_epoch_test.py --port auto
```

The reboot test asks for one physical ESP32 RST press. None of the bundled
host diagnostics transmits actuator commands.

## Run the ROS bridge

Keep propulsion isolated because the bridge owns the serial port and includes
the VCU command/session path:

```bash
bash scripts/run_bridge.sh /dev/ttyACM0
```

In a second SSH terminal:

```bash
cd ~/AutoKart-Jetson-VCU-Autonomy
bash scripts/inspect_topics.sh
```

Expected topics include:

```text
/autokart/vcu/identity
/autokart/vcu/link_status
/autokart/vcu/wheel_encoder
/autokart/vcu/steering_feedback
/autokart/vcu/state
```

## Continue autonomous odometry development

Do not enable physical motion yet. Continue in this order:

1. Measure wheelbase in metres.
2. Perform a measured-distance wheel test and calculate metres per count.
3. Freeze a real calibration ID and update the ESP32 and odometry config
   together.
4. Run stationary, hand-rotation, straight push, reverse push, and reset-epoch
   tests with traction isolated.
5. Verify `/odom` and `odom -> base_link` have no jumps across wrap or reboot.
6. Wire and characterize the Jetson IMU only after confirming the exact
   carrier-board I2C pins and voltage.
7. Validate steering/yaw and slip detection on restrained, low-speed tests.
8. Only then connect perception, Delaunay planning, and steering commands.

Read `ros2_ws/src/autonomous_odometry/CONTRACT.md`, `JETSON_WIRING.md`, and
`VALIDATION_REPORT.md` before changing the fail-closed configuration.

`reference_vcu_firmware/` is included for continued architecture work, but it
is not the firmware physically validated in this bundle. The validated bench
firmware is `firmware/esp32s3_vcu_odometry_telemetry.py`.
