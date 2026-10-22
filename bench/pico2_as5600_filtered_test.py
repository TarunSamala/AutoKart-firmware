"""Raspberry Pi Pico 2 + AS5600 isolated filtered encoder test.

Wiring:
  AS5600 VCC -> Pico 2 3V3(OUT), physical pin 36
  AS5600 GND -> Pico 2 GND, physical pin 38
  AS5600 SDA -> Pico 2 GP0, physical pin 1
  AS5600 SCL -> Pico 2 GP1, physical pin 2

Copy firmware/encoder_filters.py to /encoder_filters.py before installing this
file as /main.py. Commands: read, stream, zero, clear, status, help, q.
No AS5600 configuration or OTP register is modified.
"""

from machine import I2C, Pin
import time

from encoder_filters import AS5600AngleFilter


I2C_ID = 0
SDA_GPIO = 0
SCL_GPIO = 1
# 100 kHz is deliberately conservative for jumper-wire bench wiring.
I2C_FREQUENCY = 100_000
AS5600_ADDRESS = 0x36
ZERO_FILE = "as5600_zero.txt"
I2C_READ_RETRIES = 3

SAMPLE_INTERVAL_MS = 5       # 200 Hz
DISPLAY_INTERVAL_MS = 100    # 10 terminal updates per second

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


def read_register(register, length=1):
    last_error = None
    for attempt in range(I2C_READ_RETRIES):
        try:
            return i2c.readfrom_mem(AS5600_ADDRESS, register, length)
        except OSError as error:
            last_error = error
            time.sleep_ms(10 * (attempt + 1))
    raise OSError(
        "AS5600 register 0x{:02X} read failed after {} attempts: {}".format(
            register,
            I2C_READ_RETRIES,
            last_error,
        )
    )


def read_u16(register):
    data = read_register(register, 2)
    return (data[0] << 8) | data[1]


def magnet_state(status):
    if status & STATUS_MAGNET_TOO_STRONG:
        return "MAGNET TOO STRONG"
    if status & STATUS_MAGNET_TOO_WEAK:
        return "MAGNET TOO WEAK / NO MAGNET"
    if status & STATUS_MAGNET_DETECTED:
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
    return None


def save_zero(raw_count):
    with open(ZERO_FILE, "w") as file:
        file.write(str(int(raw_count) & 0x0FFF))


def clear_saved_zero():
    try:
        import os
        os.remove(ZERO_FILE)
    except OSError:
        pass


i2c = I2C(
    I2C_ID,
    sda=Pin(SDA_GPIO),
    scl=Pin(SCL_GPIO),
    freq=I2C_FREQUENCY,
)
angle_filter = AS5600AngleFilter(
    time_constant_s=0.010,
    maximum_rate_dps=1000.0,
    direction=1,
)

saved_zero = load_zero()
if saved_zero is not None:
    angle_filter.set_zero_raw(saved_zero)

last_sensor = None
last_filtered = None


def acquire_sample():
    global last_sensor, last_filtered

    status = read_register(REG_STATUS)[0]
    raw = read_u16(REG_RAW_ANGLE) & 0x0FFF
    sensor_angle = read_u16(REG_ANGLE) & 0x0FFF
    magnitude = read_u16(REG_MAGNITUDE) & 0x0FFF
    agc = read_register(REG_AGC)[0]

    result = angle_filter.update(
        sensor_angle,
        magnet_ok=magnet_is_valid(status),
    )
    last_sensor = {
        "status": status,
        "raw": raw,
        "sensor_angle": sensor_angle,
        "absolute_degrees": sensor_angle * DEGREES_PER_COUNT,
        "magnitude": magnitude,
        "agc": agc,
    }
    if result is not None:
        last_filtered = result
    return result


def print_latest():
    if last_sensor is None:
        print("No reading available.")
        return

    filtered_text = "FILTER REJECTED"
    if last_filtered is not None:
        filtered_text = "FILTERED {:+9.3f} deg".format(
            last_filtered["angle_deg"]
        )

    print(
        "RAW {:4d}/4095 | ABS {:8.3f} deg | {} | "
        "MAG {:4d} | AGC {:3d} | {} | valid {} rejected {}".format(
            last_sensor["raw"],
            last_sensor["absolute_degrees"],
            filtered_text,
            last_sensor["magnitude"],
            last_sensor["agc"],
            magnet_state(last_sensor["status"]),
            angle_filter.valid_samples,
            angle_filter.rejected_samples,
        )
    )


def collect(duration_ms):
    started = time.ticks_ms()
    while time.ticks_diff(time.ticks_ms(), started) < duration_ms:
        acquire_sample()
        time.sleep_ms(SAMPLE_INTERVAL_MS)


def show_status():
    print("SDA GP0/pin 1 | SCL GP1/pin 2 | 100 kHz | address 0x36")
    print("I2C devices:", [hex(device) for device in i2c.scan()])
    print("Saved zero:", saved_zero if saved_zero is not None else "NONE")
    print("Filter: 200 Hz, median-of-three, 10 ms LPF, 1000 deg/s limit")
    print_latest()


def show_help():
    print("Commands:")
    print("  read / r - collect for 0.5 seconds and print one result")
    print("  stream   - continuous filtered readings; Ctrl-C returns")
    print("  zero     - save current valid position as 0 degrees")
    print("  clear    - remove saved zero calibration")
    print("  status   - show bus, filter, calibration, and reading")
    print("  help     - show commands")
    print("  q        - stop")


print()
print("========================================")
print(" PICO 2 + AS5600 FILTERED TEST")
print("========================================")
print("SDA GP0/pin 1 | SCL GP1/pin 2 | I2C 100 kHz")

devices = i2c.scan()
print("I2C devices:", [hex(device) for device in devices])
if AS5600_ADDRESS not in devices:
    raise RuntimeError(
        "AS5600 not detected at 0x36; check 3V3, GND, GP0/SDA, GP1/SCL"
    )

try:
    collect(250)
    print_latest()
except OSError as error:
    print("INITIAL AS5600 READ ERROR:", error)
    print("The sensor ACKed at 0x36 but a register read failed.")
    print("Check 3V3/GND, shorten SDA/SCL wires, and add pull-ups if needed.")
show_help()

while True:
    try:
        command = input("\nAS5600 > ").strip().lower()

        if command in ("r", "read"):
            collect(500)
            print_latest()

        elif command == "stream":
            print("Sampling at 200 Hz; displaying at 10 Hz. Ctrl-C returns.")
            try:
                while True:
                    collect(DISPLAY_INTERVAL_MS)
                    print_latest()
            except KeyboardInterrupt:
                print("Streaming stopped.")

        elif command == "zero":
            collect(100)
            if not magnet_is_valid(last_sensor["status"]):
                print("ZERO REJECTED:", magnet_state(last_sensor["status"]))
            elif angle_filter.filtered_counts is None:
                print("ZERO REJECTED: no valid filtered reading")
            else:
                raw_zero = last_sensor["sensor_angle"]
                angle_filter.calibrate_zero()
                saved_zero = raw_zero
                save_zero(raw_zero)
                print("CALIBRATED: current position = 0.000 degrees")
                print("Saved zero count:", raw_zero)

        elif command == "clear":
            clear_saved_zero()
            saved_zero = None
            angle_filter.zero_counts = None
            print("Saved zero cleared; absolute encoder zero is active.")

        elif command == "status":
            collect(250)
            show_status()

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
        print("AS5600 READ ERROR:", error)
