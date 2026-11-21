#!/usr/bin/env bash
set -euo pipefail

[[ $# -eq 1 ]] || {
    echo "Usage: $0 /dev/serial/by-id/ESP32_PORT" >&2
    exit 2
}

ROOT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
PORT=$1
PYTHON=${AUTOKART_PYTHON:-python3}

[[ -e "$PORT" ]] || { echo "ESP32 port does not exist: $PORT" >&2; exit 1; }
if fuser "$PORT" >/dev/null 2>&1; then
    echo "ESP32 port is busy; close the dashboard, REPL or serial monitor." >&2
    fuser -v "$PORT" >&2 || true
    exit 1
fi

"$PYTHON" -m mpremote connect "$PORT" fs cp \
    "$ROOT/bench/ra02_lora_chat_common.py" :ra02_lora_chat_common.py
"$PYTHON" -m mpremote connect "$PORT" fs cp \
    "$ROOT/bench/simple_bldc_lora_brake_bench_esp32s3.py" :main.py
"$PYTHON" -m mpremote connect "$PORT" soft-reset

echo "Simple 2.5 V BLDC + LoRa brake bench firmware flashed: $PORT"
