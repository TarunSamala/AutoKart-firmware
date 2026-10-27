# Simple BLDC + LoRa brake bench test

This is deliberately limited to one run and one latched stop:

- ESP32 boot/reset: release BLD750 BRK, set EN to RUN and command 2.5 V.
- `ESTOP,0`: ignored; the BLDC keeps running.
- First `ESTOP,1`: command 0 V, set EN to STOP, apply BLD750 BRK and push the
  mechanical brake clockwise. STOP remains latched.
- Releasing the Pico button does not restart the BLDC. Reset ESP32 for another
  test run after physically returning the mechanical brake to released/home.

There is no encoder, Wi-Fi, dashboard, radio timeout, host protocol or ARM
state. Repeated STOP packets do not repeat mechanical travel.

Wheels must be raised, restrained and guarded. Keep the independent physical
traction-power cut within reach. Before boot/reset, physically place the
open-loop mechanical brake at its fully released/home position.

```bash
cd /home/satvara/Github/AutoKart-firmware
export AUTOKART_PYTHON=/home/satvara/python/bin/python3
ESP_PORT=/dev/serial/by-id/usb-Espressif_Systems_Espressif_Device_98c377ac9b780000-if00
./simple_bldc_lora_bench/flash_esp32.sh "$ESP_PORT"
```

To watch the state changes:

```bash
/home/satvara/python/bin/python3 -m serial.tools.miniterm \
  "$ESP_PORT" 115200 --raw
```

Use `Ctrl-]` to close the monitor. Do not send `Ctrl-C`, because that interrupts
`main.py`.
