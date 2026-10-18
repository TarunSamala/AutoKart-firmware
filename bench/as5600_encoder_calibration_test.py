"""Generic ESP32-S3 AS5600 angle, magnet, and zero-calibration test.

Isolated wiring:
  AS5600 VCC -> ESP32-S3 3V3
  AS5600 GND -> ESP32-S3 GND
  AS5600 SDA -> ESP32-S3 GPIO8
  AS5600 SCL -> ESP32-S3 GPIO9

Commands:
  r / read  - print one sample
  stream    - continuously print samples; Ctrl-C returns to the prompt
  zero      - assign the current valid position as calibrated 0 degrees
  clear     - remove the saved zero offset
  status    - show wiring, saved zero, and one sample
  help      - show commands
  q         - stop

The zero offset is stored in ``as5600_zero.txt`` on the ESP32 filesystem.
No AS5600 configuration or OTP register is modified.
"""

from machine import I2C, Pin
import time


SDA_GPIO = 8
SCL_GPIO = 9
I2C_FREQUENCY = 100_000
AS5600_ADDRESS = 0x36
ZERO_FILE = "as5600_zero.txt"

REG_STATUS = 0x0B
REG_RAW_ANGLE = 0x0C
REG_ANGLE = 0x0E
REG_AGC = 0x1A
REG_MAGNITUDE = 0x1B

STATUS_MAGNET_TOO_STRONG = 0x08
STATUS_MAGNET_TOO_WEAK = 0x10
STATUS_MAGNET_DETECTED = 0x20

COUNTS_PER_REVOLUTION = 4096
DEGREES_PER_COUNT = 360.0 / COUNTS_PER_REVOLUTION


def read_u16(i2c, register):
    data = i2c.readfrom_mem(AS5600_ADDRESS, register, 2)
    return (data[0] << 8) | data[1]


def magnet_state(status):
    detected = bool(status & STATUS_MAGNET_DETECTED)
    weak = bool(status & STATUS_MAGNET_TOO_WEAK)
    strong = bool(status & STATUS_MAGNET_TOO_STRONG)

    if strong:
        return "MAGNET TOO STRONG"
    if weak:
        return "MAGNET TOO WEAK / NO MAGNET"
    if detected:
        return "MAGNET OK"
    return "NO MAGNET"


def magnet_is_valid(status):
    return (
        bool(status & STATUS_MAGNET_DETECTED)
        and not bool(status & STATUS_MAGNET_TOO_WEAK)
        and not bool(status & STATUS_MAGNET_TOO_STRONG)
    )


def load_zero():
    try:
        with open(ZERO_FILE, "r") as file:
            value = int(file.read().strip())
        if 0 <= value < COUNTS_PER_REVOLUTION:
            return value
    except (OSError, ValueError):
        pass
    return 0


def save_zero(raw_count):
    with open(ZERO_FILE, "w") as file:
        file.write(str(int(raw_count)))


def clear_zero():
    try:
        import os
        os.remove(ZERO_FILE)
    except OSError:
        pass


def signed_calibrated_counts(raw_count, zero_count):
    difference = (raw_count - zero_count) % COUNTS_PER_REVOLUTION
    if difference >= COUNTS_PER_REVOLUTION // 2:
        difference -= COUNTS_PER_REVOLUTION
    return difference


def read_sample(i2c, zero_count):
    status = i2c.readfrom_mem(AS5600_ADDRESS, REG_STATUS, 1)[0]
    raw = read_u16(i2c, REG_RAW_ANGLE) & 0x0FFF
    filtered = read_u16(i2c, REG_ANGLE) & 0x0FFF
    magnitude = read_u16(i2c, REG_MAGNITUDE) & 0x0FFF
    agc = i2c.readfrom_mem(AS5600_ADDRESS, REG_AGC, 1)[0]
    relative_counts = signed_calibrated_counts(filtered, zero_count)
    return {
        "status": status,
        "raw": raw,
        "filtered": filtered,
        "absolute_degrees": filtered * DEGREES_PER_COUNT,
        "calibrated_degrees": relative_counts * DEGREES_PER_COUNT,
        "magnitude": magnitude,
        "agc": agc,
    }


def print_sample(sample):
    print(
        "RAW {:4d}/4095 = {:8.3f} deg | "
        "CALIBRATED {:+8.3f} deg | MAG {:4d} | AGC {:3d} | {}".format(
            sample["raw"],
            sample["absolute_degrees"],
            sample["calibrated_degrees"],
            sample["magnitude"],
            sample["agc"],
            magnet_state(sample["status"]),
        )
    )


def show_help():
    print()
    print("Commands:")
    print("  r / read  - one angle and magnet reading")
    print("  stream    - continuous readings; Ctrl-C returns here")
    print("  zero      - save current valid position as 0 degrees")
    print("  clear     - delete saved zero calibration")
    print("  status    - wiring, zero offset, and one reading")
    print("  help      - show this list")
    print("  q         - stop")


print()
print("========================================")
print(" ESP32-S3 AS5600 ENCODER TEST")
print("========================================")
print("SDA GPIO{} | SCL GPIO{} | address 0x{:02X}".format(
    SDA_GPIO,
    SCL_GPIO,
    AS5600_ADDRESS,
))

i2c = I2C(
    0,
    sda=Pin(SDA_GPIO),
    scl=Pin(SCL_GPIO),
    freq=I2C_FREQUENCY,
)
devices = i2c.scan()
print("I2C devices:", [hex(device) for device in devices])

if AS5600_ADDRESS not in devices:
    raise RuntimeError(
        "AS5600 not detected at 0x36; check 3V3, GND, SDA, SCL, and pull-ups"
    )

zero_offset = load_zero()
print("AS5600 detected")
print("Saved zero raw count:", zero_offset)
print_sample(read_sample(i2c, zero_offset))
show_help()

while True:
    try:
        command = input("\nAS5600 > ").strip().lower()

        if command in ("r", "read"):
            print_sample(read_sample(i2c, zero_offset))

        elif command == "stream":
            print("Streaming every 100 ms. Press Ctrl-C to return.")
            try:
                while True:
                    print_sample(read_sample(i2c, zero_offset))
                    time.sleep_ms(100)
            except KeyboardInterrupt:
                print("Streaming stopped.")

        elif command == "zero":
            sample = read_sample(i2c, zero_offset)
            print_sample(sample)
            if not magnet_is_valid(sample["status"]):
                print("ZERO REJECTED:", magnet_state(sample["status"]))
            else:
                zero_offset = sample["filtered"]
                save_zero(zero_offset)
                print("CALIBRATED: current position = 0.000 deg")
                print("Saved zero raw count:", zero_offset)

        elif command == "clear":
            clear_zero()
            zero_offset = 0
            print("Saved zero cleared; raw encoder zero is active.")

        elif command == "status":
            print("SDA GPIO{} | SCL GPIO{} | I2C 0x{:02X}".format(
                SDA_GPIO,
                SCL_GPIO,
                AS5600_ADDRESS,
            ))
            print("Saved zero raw count:", zero_offset)
            print_sample(read_sample(i2c, zero_offset))

        elif command in ("help", "?"):
            show_help()

        elif command in ("q", "quit", "exit"):
            print("AS5600 test stopped.")
            break

        elif command:
            print("Unknown command. Type help.")

    except KeyboardInterrupt:
        print("\nUse q to stop or help for commands.")
    except Exception as error:
        print("READ ERROR:", error)
