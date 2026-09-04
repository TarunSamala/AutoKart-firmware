# AutoKart Firmware

Firmware and bench-test tools for the autonomous gokart project.

## Repository layout

```text
firmware/   Integrated ESP32-S3 actuator controllers
bench/      IMU, RS485, steering, brake, throttle, and BLDC tests
docs/       Wiring notes and actuator-control report
images/     MicroPython firmware binaries
.vscode/    VS Code/MicroPython workspace settings
```

## Current bench RS485 test

The current two-board test uses:

```text
XIAO ESP32-C3 → MAX3485 → twisted pair → MAX3485 → ESP32-S3
```

The C3 sends `PING` and the S3 responds with `PONG`.

Test files:

- `bench/rs485_c3_test.py` — XIAO ESP32-C3 master
- `bench/rs485_s3_test.py` — ESP32-S3 responder
- `docs/RS485_CONNECTIONS.md` — wiring and direction-control details

For two-laptop text communication, use:

- `bench/rs485_chat_c3.py` — run on the XIAO ESP32-C3
- `bench/rs485_chat_s3.py` — run on the ESP32-S3

Each laptop connects to its local board with USB. Type a line in either
serial terminal and press Enter; it is forwarded to the other board.

To upload a test file with MicroPython, connect one board at a time as
`/dev/ttyACM0`:

```bash
python3 -m mpremote connect /dev/ttyACM0 fs cp bench/rs485_c3_test.py :main.py
python3 -m mpremote connect /dev/ttyACM0 soft-reset
```

Use `bench/rs485_s3_test.py` for the S3. The uploaded `main.py` starts
automatically whenever that board powers on.

## IMU test

`bench/imu_esp32s3_test.py` checks an MPU-9250/6500 directly over I²C using:

```text
GPIO8 → SDA
GPIO9 → SCL
```

This test runs on the ESP32, not on the computer's normal Python interpreter.

## Integrated firmware

The main actuator controller is currently:

```text
firmware/autokart_rc_v5_controlled_stop.py
```

It controls steering and brake through TB6600 step/dir drivers and throttle
through the GP8630N/BLD750 interface. Steering and brake positioning remain
open-loop and require mechanical calibration.

## Safety

Bench-test only until actuator limits, RS485 integrity, throttle scaling,
interlocks, and emergency-stop behavior are validated. Keep wheels off the
ground during bring-up.
