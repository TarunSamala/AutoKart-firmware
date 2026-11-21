#!/usr/bin/env bash
set -euo pipefail

[[ $# -eq 1 ]] || {
    echo "Usage: $0 /dev/serial/by-id/ESP32_PORT" >&2
    exit 2
}
ROOT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
PORT=$1
[[ -e "$PORT" ]] || { echo "Port does not exist: $PORT" >&2; exit 1; }

if fuser "$PORT" >/dev/null 2>&1; then
    echo "Serial port is owned by another process:" >&2
    fuser -v "$PORT" >&2 || true
    exit 1
fi

python3 -m mpremote connect "$PORT" fs cp \
    "$ROOT/firmware/esp32s3_vcu_odometry_telemetry.py" :main.py
python3 -m mpremote connect "$PORT" soft-reset
echo "VCU odometry telemetry firmware flashed: $PORT"
