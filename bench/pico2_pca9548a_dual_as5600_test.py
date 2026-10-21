"""Pico 2 + PCA9548A + two AS5600 isolated filtered test.

Pico I2C0: GP0 SDA, GP1 SCL
PCA9548A: address 0x70 (A0/A1/A2 grounded)
PCA channel 0: steering AS5600 at 0x36
PCA channel 1: brake AS5600 at 0x36

Install encoder_filters.py beside main.py. No AS5600 configuration or OTP
register is modified, and no actuator output is initialized.
"""

from machine import I2C, Pin
import time

from encoder_filters import AS5600AngleFilter


I2C_SDA = 0
I2C_SCL = 1
I2C_FREQUENCY = 400_000
PCA_ADDRESS = 0x70
AS5600_ADDRESS = 0x36

STEERING_CHANNEL = 0
BRAKE_CHANNEL = 1

STEERING_ZERO_FILE = "as5600_steering_zero.txt"
BRAKE_ZERO_FILE = "as5600_brake_zero.txt"

SAMPLE_INTERVAL_MS = 5
DISPLAY_INTERVAL_MS = 200

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


def select_channel(channel):
    if channel < 0 or channel > 7:
        raise ValueError("PCA9548A channel must be 0-7")
    # Enable exactly one channel because both AS5600 devices use 0x36.
    i2c.writeto(PCA_ADDRESS, bytes((1 << channel,)))


def disable_channels():
    i2c.writeto(PCA_ADDRESS, b"\x00")


def read_u16(register):
    data = i2c.readfrom_mem(AS5600_ADDRESS, register, 2)
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


def load_zero(filename):
    try:
        with open(filename, "r") as file:
            value = int(file.read().strip())
        if 0 <= value < COUNTS_PER_REVOLUTION:
            return value
    except (OSError, ValueError):
        pass
    return None


def save_zero(filename, raw_count):
    with open(filename, "w") as file:
        file.write(str(int(raw_count) & 0x0FFF))


def remove_zero(filename):
    try:
        import os
        os.remove(filename)
    except OSError:
        pass


def read_encoder(channel, angle_filter):
    select_channel(channel)
    status = i2c.readfrom_mem(AS5600_ADDRESS, REG_STATUS, 1)[0]
    raw = read_u16(REG_RAW_ANGLE) & 0x0FFF
    sensor_angle = read_u16(REG_ANGLE) & 0x0FFF
    magnitude = read_u16(REG_MAGNITUDE) & 0x0FFF
    agc = i2c.readfrom_mem(AS5600_ADDRESS, REG_AGC, 1)[0]
    filtered = angle_filter.update(
        sensor_angle,
        magnet_ok=magnet_is_valid(status),
    )
    return {
        "status": status,
        "raw": raw,
        "sensor_angle": sensor_angle,
        "absolute_deg": sensor_angle * DEGREES_PER_COUNT,
        "magnitude": magnitude,
        "agc": agc,
        "filtered": filtered,
    }


def acquire_both():
    global steering_sample, brake_sample
    steering_sample = read_encoder(STEERING_CHANNEL, steering_filter)
    brake_sample = read_encoder(BRAKE_CHANNEL, brake_filter)
    disable_channels()


def print_encoder(name, sample, angle_filter):
    if sample is None:
        print(name + ": no sample")
        return
    result = sample["filtered"]
    filtered_text = "REJECTED"
    if result is not None:
        filtered_text = "{:+9.3f} deg".format(result["angle_deg"])
    print(
        "{}: RAW {:4d}/4095 | ABS {:8.3f} deg | FILTERED {} | "
        "MAG {:4d} | AGC {:3d} | {} | valid {} rejected {}".format(
            name,
            sample["raw"],
            sample["absolute_deg"],
            filtered_text,
            sample["magnitude"],
            sample["agc"],
            magnet_state(sample["status"]),
            angle_filter.valid_samples,
            angle_filter.rejected_samples,
        )
    )


def print_both():
    print_encoder("STEERING CH0", steering_sample, steering_filter)
    print_encoder("BRAKE    CH1", brake_sample, brake_filter)


def collect(duration_ms):
    started = time.ticks_ms()
    while time.ticks_diff(time.ticks_ms(), started) < duration_ms:
        acquire_both()
        time.sleep_ms(SAMPLE_INTERVAL_MS)


def validate_channel(channel, name):
    select_channel(channel)
    devices = i2c.scan()
    print("Channel {} {}: {}".format(
        channel, name, [hex(device) for device in devices]
    ))
    if AS5600_ADDRESS not in devices:
        raise RuntimeError(
            "{} AS5600 not detected on PCA channel {}".format(name, channel)
        )


def calibrate(name):
    global steering_zero, brake_zero
    collect(100)
    if name == "steering":
        sample = steering_sample
        angle_filter = steering_filter
        filename = STEERING_ZERO_FILE
    else:
        sample = brake_sample
        angle_filter = brake_filter
        filename = BRAKE_ZERO_FILE

    if not magnet_is_valid(sample["status"]):
        print("ZERO REJECTED:", magnet_state(sample["status"]))
        return
    if not angle_filter.calibrate_zero():
        print("ZERO REJECTED: no valid filtered sample")
        return

    raw_zero = sample["sensor_angle"]
    save_zero(filename, raw_zero)
    if name == "steering":
        steering_zero = raw_zero
    else:
        brake_zero = raw_zero
    print(name.upper(), "CALIBRATED: current position = 0.000 degrees")
    print("Saved zero count:", raw_zero)


def clear_calibration(name):
    global steering_zero, brake_zero
    if name == "steering":
        remove_zero(STEERING_ZERO_FILE)
        steering_filter.zero_counts = None
        steering_zero = None
    else:
        remove_zero(BRAKE_ZERO_FILE)
        brake_filter.zero_counts = None
        brake_zero = None
    print(name.upper(), "saved zero cleared")


def show_status():
    print("Pico SDA GP0/pin 1 | SCL GP1/pin 2")
    print("PCA9548A address: 0x70 | A0/A1/A2 = GND")
    print("Steering zero:", steering_zero if steering_zero is not None else "NONE")
    print("Brake zero:", brake_zero if brake_zero is not None else "NONE")
    print("Filter: 200 Hz, median-of-three, 10 ms LPF")
    print_both()


def show_help():
    print("Commands:")
    print("  read / r        - sample for 0.5 seconds and show both encoders")
    print("  stream          - continuous readings; Ctrl-C returns")
    print("  zero steering   - set steering position to 0 degrees")
    print("  zero brake      - set brake position to 0 degrees")
    print("  clear steering  - remove steering zero")
    print("  clear brake     - remove brake zero")
    print("  status          - show mux, calibration and both encoders")
    print("  help            - show commands")
    print("  q               - stop")


print()
print("============================================")
print(" PICO 2 + PCA9548A + DUAL AS5600 TEST")
print("============================================")
print("NO ACTUATOR OUTPUTS ARE INITIALIZED")

i2c = I2C(
    0,
    sda=Pin(I2C_SDA),
    scl=Pin(I2C_SCL),
    freq=I2C_FREQUENCY,
)

upstream = i2c.scan()
print("Upstream I2C:", [hex(device) for device in upstream])
if PCA_ADDRESS not in upstream:
    raise RuntimeError("PCA9548A not detected at 0x70")
disable_channels()

validate_channel(STEERING_CHANNEL, "steering")
validate_channel(BRAKE_CHANNEL, "brake")
disable_channels()

steering_filter = AS5600AngleFilter(time_constant_s=0.010, maximum_rate_dps=1000)
brake_filter = AS5600AngleFilter(time_constant_s=0.010, maximum_rate_dps=1000)

steering_zero = load_zero(STEERING_ZERO_FILE)
brake_zero = load_zero(BRAKE_ZERO_FILE)
if steering_zero is not None:
    steering_filter.set_zero_raw(steering_zero)
if brake_zero is not None:
    brake_filter.set_zero_raw(brake_zero)

steering_sample = None
brake_sample = None
collect(250)
print_both()
show_help()

while True:
    try:
        command = input("\nDual AS5600 > ").strip().lower()

        if command in ("r", "read"):
            collect(500)
            print_both()

        elif command == "stream":
            print("Sampling at 200 Hz; displaying at 5 Hz. Ctrl-C returns.")
            try:
                while True:
                    collect(DISPLAY_INTERVAL_MS)
                    print_both()
            except KeyboardInterrupt:
                print("Streaming stopped.")

        elif command == "zero steering":
            calibrate("steering")

        elif command == "zero brake":
            calibrate("brake")

        elif command == "clear steering":
            clear_calibration("steering")

        elif command == "clear brake":
            clear_calibration("brake")

        elif command == "status":
            collect(250)
            show_status()

        elif command in ("help", "?"):
            show_help()

        elif command in ("q", "quit", "exit"):
            disable_channels()
            print("Dual AS5600 test stopped.")
            break

        elif command:
            print("Unknown command. Type help.")

    except KeyboardInterrupt:
        print("\nUse q to stop or help for commands.")
    except Exception as error:
        print("DUAL AS5600 ERROR:", error)
