#!/usr/bin/env bash
set -euo pipefail

[[ $# -eq 1 ]] || {
    echo "Usage: $0 /dev/serial/by-id/PICO_PORT" >&2
    exit 2
}

ROOT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
PORT=$1
PYTHON=${AUTOKART_PYTHON:-python3}
[[ -e "$PORT" ]] || { echo "Pico port does not exist: $PORT" >&2; exit 1; }

if fuser "$PORT" >/dev/null 2>&1; then
    echo "Pico port is in use. Close its REPL/serial monitor first:" >&2
    fuser -v "$PORT" >&2 || true
    exit 1
fi

"$PYTHON" -m mpremote connect "$PORT" fs cp \
    "$ROOT/pico/ra02_lora_chat_common.py" :ra02_lora_chat_common.py
"$PYTHON" -m mpremote connect "$PORT" fs cp "$ROOT/pico/main.py" :main.py
"$PYTHON" -m mpremote connect "$PORT" soft-reset

echo "Pico E-stop heartbeat transmitter flashed: $PORT"
