#!/usr/bin/env bash
set -euo pipefail

ROOT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
PYTHON=${AUTOKART_PYTHON:-python3}
exec "$PYTHON" "$ROOT/laptop/estop_serial_validation.py" "$@"
