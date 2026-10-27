#!/usr/bin/env bash
set -euo pipefail

FIRMWARE_ROOT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
ROS_ROOT=${AUTOKART_ROS_ROOT:-/home/satvara/Github/AutoKart-ROS}
NAME=AutoKart-Jetson-VCU-Autonomy
OUT="$FIRMWARE_ROOT/dist/$NAME"
ARCHIVE="$FIRMWARE_ROOT/dist/$NAME.tar.gz"

required=(
    "$FIRMWARE_ROOT/bench/esp32s3_vcu_odometry_telemetry.py"
    "$ROS_ROOT/autonomous_odometry/package.xml"
    "$ROS_ROOT/VCU/vci/vcu_firmware/pi_bridge/vcu_bridge/package.xml"
)
for path in "${required[@]}"; do
    [[ -e "$path" ]] || { echo "Missing required source: $path" >&2; exit 1; }
done

rm -rf -- "$OUT"
mkdir -p "$OUT/firmware" "$OUT/host_tools" "$OUT/evidence" \
    "$OUT/ros2_ws/src" "$OUT/scripts"

cp "$FIRMWARE_ROOT/bench/esp32s3_vcu_odometry_telemetry.py" \
    "$OUT/firmware/esp32s3_vcu_odometry_telemetry.py"

for tool in as5047p_odometry_test.py steering_telemetry_test.py \
    steering_closed_loop_test.py vcu_serial_validation.py \
    vcu_reboot_epoch_test.py; do
    cp "$FIRMWARE_ROOT/tools/$tool" "$OUT/host_tools/$tool"
done

cp "$FIRMWARE_ROOT/log/vcu_serial_validation_latest.json" \
    "$OUT/evidence/" 2>/dev/null || true
cp "$FIRMWARE_ROOT/log/vcu_reboot_epoch_latest.json" \
    "$OUT/evidence/" 2>/dev/null || true

cp -a "$ROS_ROOT/autonomous_odometry" \
    "$OUT/ros2_ws/src/autonomous_odometry"
cp -a "$ROS_ROOT/VCU/vci/vcu_firmware/pi_bridge/vcu_bridge" \
    "$OUT/ros2_ws/src/vcu_bridge"
cp -a "$ROS_ROOT/VCU/vci/vcu_firmware" "$OUT/reference_vcu_firmware"

find "$OUT" -type d \( -name __pycache__ -o -name .pytest_cache \
    -o -name build -o -name install -o -name log -o -name .pio \) \
    -prune -exec rm -rf -- {} +
find "$OUT" -type f \( -name '*.pyc' -o -name '*.pyo' \) -delete

cp "$FIRMWARE_ROOT/jetson_vcu_autonomy/README.md" "$OUT/README.md"
cp "$FIRMWARE_ROOT/jetson_vcu_autonomy/CURRENT_STATUS.md" \
    "$OUT/CURRENT_STATUS.md"
cp "$FIRMWARE_ROOT/docs/JETSON_PRODUCTION_HANDOVER.md" \
    "$OUT/JETSON_PRODUCTION_HANDOVER.md"
for script in install_dependencies.sh build_ros.sh \
    flash_telemetry_firmware.sh run_bridge.sh inspect_topics.sh; do
    cp "$FIRMWARE_ROOT/jetson_vcu_autonomy/$script" "$OUT/scripts/$script"
    chmod +x "$OUT/scripts/$script"
done

(
    cd "$OUT"
    find . -type f ! -name MANIFEST.sha256 -print0 | \
        sort -z | xargs -0 sha256sum > MANIFEST.sha256
)

tar -C "$FIRMWARE_ROOT/dist" -czf "$ARCHIVE" "$NAME"
(
    cd "$FIRMWARE_ROOT/dist"
    sha256sum "$NAME.tar.gz" > "$NAME.tar.gz.sha256"
)

echo "Built: $OUT"
echo "Archive: $ARCHIVE"
echo "Checksum: $ARCHIVE.sha256"
