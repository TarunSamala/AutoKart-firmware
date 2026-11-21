#!/usr/bin/env bash
set -euo pipefail

ROOT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
OUT="$ROOT/dist/AutoKart-Jetson-EStop"
ARCHIVE="$ROOT/dist/AutoKart-Jetson-EStop.tar.gz"

rm -rf -- "$OUT"
mkdir -p "$OUT/esp32" "$OUT/pico" "$OUT/jetson"

cp "$ROOT/bench/full_bldc_lora_estop_esp32s3.py" "$OUT/esp32/main.py"
cp "$ROOT/bench/bldc_lora_estop_required_config.py" \
   "$OUT/esp32/bldc_bench_config.py"
cp "$ROOT/bench/ra02_lora_chat_common.py" \
   "$OUT/esp32/ra02_lora_chat_common.py"
cp "$ROOT/bench/pico2_ra02_estop_transmitter.py" "$OUT/pico/main.py"
cp "$ROOT/bench/ra02_lora_chat_common.py" \
   "$OUT/pico/ra02_lora_chat_common.py"
cp "$ROOT/jetson_estop/estop_validation.py" "$OUT/jetson/estop_validation.py"
cp "$ROOT/jetson_estop/README.md" "$OUT/README.md"
cp "$ROOT/jetson_estop/flash_esp32.sh" "$OUT/flash_esp32.sh"
cp "$ROOT/jetson_estop/flash_pico.sh" "$OUT/flash_pico.sh"
cp "$ROOT/jetson_estop/validate_estop.sh" "$OUT/validate_estop.sh"
chmod +x "$OUT/flash_esp32.sh" "$OUT/flash_pico.sh" \
    "$OUT/validate_estop.sh"

(
    cd "$OUT"
    find . -type f ! -name MANIFEST.sha256 -print0 | \
        sort -z | xargs -0 sha256sum > MANIFEST.sha256
)

tar -C "$ROOT/dist" -czf "$ARCHIVE" AutoKart-Jetson-EStop
(
    cd "$ROOT/dist"
    sha256sum "$(basename "$ARCHIVE")" > "$(basename "$ARCHIVE").sha256"
)

echo "Built: $OUT"
echo "Archive: $ARCHIVE"
echo "Checksum: $ARCHIVE.sha256"
