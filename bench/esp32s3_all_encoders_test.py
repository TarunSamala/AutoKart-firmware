"""Isolated AutoKart encoder test for the YD ESP32-S3.

Reads only:
  - steering AS5600 directly on hardware I2C(0), GPIO1/GPIO2
  - brake AS5600 directly on SoftI2C, GPIO35/GPIO36
  - BLDC AS5047P over SPI mode 1

No actuator, GP8630N, BLD750, motor-driver, LoRa, or Wi-Fi pin is initialized.

Frozen wiring:
  ESP32 GPIO1  -> steering AS5600 SDA
  ESP32 GPIO2  -> steering AS5600 SCL
  ESP32 GPIO35 -> brake AS5600 SDA
  ESP32 GPIO36 -> brake AS5600 SCL
  ESP32 GPIO16 -> AS5047P CS
  ESP32 GPIO18 -> AS5047P CLK
  ESP32 GPIO41 -> AS5047P MOSI
  ESP32 GPIO42 -> AS5047P MISO

All modules use ESP32 3V3 and a common GND. The PCA9548A is not connected.
AS5047P W/PWM and I/PWM remain disconnected.

Commands: read, stream, zero steering, zero brake, zero bldc, clear steering,
clear brake, status, help, q.
"""

from machine import I2C, Pin, SoftI2C, SPI
import math
import os
import time


# Direct, independent AS5600 buses. Both sensors use address 0x36, but they
# cannot conflict because they are on different physical SDA/SCL pairs.
STEERING_SDA_GPIO = 1
STEERING_SCL_GPIO = 2
BRAKE_SDA_GPIO = 35
BRAKE_SCL_GPIO = 36
AS5600_ADDRESS = 0x36

AS5047_CS_GPIO = 16
AS5047_SCK_GPIO = 18
AS5047_MOSI_GPIO = 41
AS5047_MISO_GPIO = 42

I2C_FREQUENCY = 100_000
SPI_FREQUENCY = 1_000_000
SAMPLE_INTERVAL_MS = 10       # combined three-encoder acquisition target: 100 Hz
DISPLAY_INTERVAL_MS = 250     # terminal update rate: 4 Hz

STEERING_ZERO_FILE = "steering_as5600_zero.txt"
BRAKE_ZERO_FILE = "brake_as5600_zero.txt"

AS5600_COUNTS = 4096
AS5047_COUNTS = 16384
AS5600_MAX_RATE_DPS = 1200.0
AS5047_MAX_MOTOR_RPM = 6000.0
# The AS5047P can wander by a few counts while physically stationary. Keep a
# position anchor and require cumulative movement beyond this window before
# adding to turns/distance. At 14 bits, 8 counts is only about 0.176 degrees.
AS5047_POSITION_DEADBAND_COUNTS = 8

# Frozen sign convention: steering right is positive; clockwise brake
# application is positive. Change one sign to -1 only if the corresponding
# mounted encoder is observed counting in the opposite direction.
STEERING_DIRECTION_SIGN = 1
BRAKE_CLOCKWISE_SIGN = 1

# Preliminary drivetrain values. Distance remains nominal until loaded tyre
# circumference is measured on the kart.
DRIVE_SPROCKET_TEETH = 14.0
AXLE_SPROCKET_TEETH = 44.0
MOTOR_REVS_PER_AXLE_REV = AXLE_SPROCKET_TEETH / DRIVE_SPROCKET_TEETH
NOMINAL_TYRE_DIAMETER_M = 11.0 * 0.0254
NOMINAL_TYRE_CIRCUMFERENCE_M = math.pi * NOMINAL_TYRE_DIAMETER_M

# AS5600 registers and status bits.
AS5600_STATUS = 0x0B
AS5600_RAW_ANGLE = 0x0C
AS5600_ANGLE = 0x0E
AS5600_AGC = 0x1A
AS5600_MAGNITUDE = 0x1B
AS5600_MAGNET_DETECTED = 0x20
AS5600_MAGNET_LOW = 0x10
AS5600_MAGNET_HIGH = 0x08

# AS5047P registers and DIAAGC bits.
AS5047_ERRFL = 0x0001
AS5047_DIAAGC = 0x3FFC
AS5047_MAG = 0x3FFD
AS5047_ANGLECOM = 0x3FFF
AS5047_MAGL = 1 << 11
AS5047_MAGH = 1 << 10
AS5047_COF = 1 << 9
AS5047_LF = 1 << 8


def ticks_ms():
    return time.ticks_ms()


def ticks_diff(current, previous):
    return time.ticks_diff(current, previous)


def wrapped_delta(current, previous, counts_per_turn):
    half = counts_per_turn // 2
    return ((int(current) - int(previous) + half) % counts_per_turn) - half


def median(values):
    ordered = sorted(values)
    return ordered[len(ordered) // 2]


def load_zero(filename):
    try:
        with open(filename, "r") as zero_file:
            value = int(zero_file.read().strip())
        if 0 <= value < AS5600_COUNTS:
            return value
    except (OSError, ValueError):
        pass
    return None


def save_zero(filename, raw):
    with open(filename, "w") as zero_file:
        zero_file.write(str(int(raw) & 0x0FFF))


def remove_zero(filename):
    try:
        os.remove(filename)
    except OSError:
        pass


class AS5600Filter:
    """Unwrap, median-of-three, and first-order low-pass filter."""

    def __init__(self, zero_raw=None, direction=1):
        self.zero_counts = zero_raw
        self.direction = direction
        self.last_raw = None
        self.last_ms = None
        self.unwrapped = None
        self.filtered = None
        self.window = []
        self.valid_samples = 0
        self.rejected_samples = 0

    def update(self, raw, valid):
        now = ticks_ms()
        if not valid:
            self.rejected_samples += 1
            return None
        if self.last_raw is None:
            self.last_raw = raw
            self.last_ms = now
            self.unwrapped = float(raw)
            self.filtered = float(raw)
            self.window = [float(raw)]
            self.valid_samples += 1
            return self.angle()

        elapsed_ms = max(1, ticks_diff(now, self.last_ms))
        delta = wrapped_delta(raw, self.last_raw, AS5600_COUNTS)
        rate_dps = abs(delta * 360.0 / AS5600_COUNTS) / (elapsed_ms / 1000.0)
        if rate_dps > AS5600_MAX_RATE_DPS:
            self.rejected_samples += 1
            return None

        self.unwrapped += delta
        self.last_raw = raw
        self.last_ms = now
        self.window.append(self.unwrapped)
        if len(self.window) > 3:
            self.window.pop(0)
        filtered_input = median(self.window)
        alpha = (elapsed_ms / 1000.0) / (0.010 + elapsed_ms / 1000.0)
        self.filtered += alpha * (filtered_input - self.filtered)
        self.valid_samples += 1
        return self.angle()

    def angle(self):
        if self.filtered is None or self.zero_counts is None:
            return None
        # AS5600 steering/brake motion is less than 180 degrees from zero.
        delta = wrapped_delta(round(self.filtered), self.zero_counts, AS5600_COUNTS)
        angle = delta * 360.0 / AS5600_COUNTS * self.direction
        return 0.0 if abs(angle) < 0.10 else angle

    def zero(self, raw):
        self.zero_counts = int(raw) & 0x0FFF

    def clear(self):
        self.zero_counts = None


class BLDCFilter:
    """AS5047P unwrap, RPM LPF, turns, axle speed, and nominal distance."""

    def __init__(self):
        self.reset()

    def reset(self):
        self.last_raw = None
        self.position_anchor_raw = None
        self.last_us = None
        self.total_counts = 0
        self.travel_counts = 0
        self.motor_rpm = 0.0
        self.valid_samples = 0
        self.rejected_samples = 0

    def update(self, raw, valid):
        now = time.ticks_us()
        if not valid:
            self.rejected_samples += 1
            return None
        if self.last_raw is None:
            self.last_raw = raw
            self.position_anchor_raw = raw
            self.last_us = now
            self.valid_samples += 1
            return self.values(0.0)

        elapsed_us = time.ticks_diff(now, self.last_us)
        if elapsed_us <= 0:
            self.rejected_samples += 1
            return None
        delta = wrapped_delta(raw, self.last_raw, AS5047_COUNTS)
        elapsed_s = elapsed_us / 1_000_000.0
        raw_rpm = delta / AS5047_COUNTS / elapsed_s * 60.0
        if abs(raw_rpm) > AS5047_MAX_MOTOR_RPM:
            self.rejected_samples += 1
            self.last_raw = raw
            self.last_us = now
            return None

        # Suppress bounded stationary jitter without losing genuine slow
        # rotation: movement accumulates relative to the last accepted anchor.
        position_delta = wrapped_delta(
            raw, self.position_anchor_raw, AS5047_COUNTS
        )
        accepted_delta = (
            0
            if abs(position_delta) <= AS5047_POSITION_DEADBAND_COUNTS
            else position_delta
        )
        if accepted_delta:
            self.position_anchor_raw = raw
        if accepted_delta == 0:
            raw_rpm = 0.0
        self.last_raw = raw
        self.last_us = now
        self.total_counts += accepted_delta
        self.travel_counts += abs(accepted_delta)
        alpha = elapsed_s / (0.030 + elapsed_s)
        self.motor_rpm += alpha * (raw_rpm - self.motor_rpm)
        if abs(self.motor_rpm) < 0.5:
            self.motor_rpm = 0.0
        self.valid_samples += 1
        return self.values(raw_rpm)

    def values(self, raw_rpm):
        motor_revs = self.total_counts / AS5047_COUNTS
        travel_motor_revs = self.travel_counts / AS5047_COUNTS
        axle_revs = motor_revs / MOTOR_REVS_PER_AXLE_REV
        travel_axle_revs = travel_motor_revs / MOTOR_REVS_PER_AXLE_REV
        axle_rpm = self.motor_rpm / MOTOR_REVS_PER_AXLE_REV
        return {
            "raw_rpm": raw_rpm,
            "motor_rpm": self.motor_rpm,
            "axle_rpm": axle_rpm,
            "motor_revs": motor_revs,
            "full_turns": abs(self.total_counts) // AS5047_COUNTS,
            "distance_m": travel_axle_revs * NOMINAL_TYRE_CIRCUMFERENCE_M,
            "displacement_m": axle_revs * NOMINAL_TYRE_CIRCUMFERENCE_M,
            "speed_kmh": axle_rpm / 60.0 * NOMINAL_TYRE_CIRCUMFERENCE_M * 3.6,
        }


steering_i2c = I2C(
    0,
    sda=Pin(STEERING_SDA_GPIO),
    scl=Pin(STEERING_SCL_GPIO),
    freq=I2C_FREQUENCY,
)
brake_i2c = SoftI2C(
    sda=Pin(BRAKE_SDA_GPIO),
    scl=Pin(BRAKE_SCL_GPIO),
    freq=I2C_FREQUENCY,
)
spi = SPI(
    1, baudrate=SPI_FREQUENCY, polarity=0, phase=1, bits=8,
    firstbit=SPI.MSB, sck=Pin(AS5047_SCK_GPIO),
    mosi=Pin(AS5047_MOSI_GPIO), miso=Pin(AS5047_MISO_GPIO),
)
as5047_cs = Pin(AS5047_CS_GPIO, Pin.OUT, value=1)

steering_filter = AS5600Filter(
    load_zero(STEERING_ZERO_FILE), direction=STEERING_DIRECTION_SIGN
)
brake_filter = AS5600Filter(
    load_zero(BRAKE_ZERO_FILE), direction=BRAKE_CLOCKWISE_SIGN
)
bldc_filter = BLDCFilter()

steering_sample = None
brake_sample = None
bldc_sample = None
bldc_values = None


def i2c_read_retry(bus, register, length, attempts=3):
    last_error = None
    for attempt in range(attempts):
        try:
            return bus.readfrom_mem(AS5600_ADDRESS, register, length)
        except OSError as error:
            last_error = error
            time.sleep_ms(2 * (attempt + 1))
    raise last_error


def i2c_u16(bus, register):
    data = i2c_read_retry(bus, register, 2)
    return (data[0] << 8) | data[1]


def as5600_magnet(status):
    if status & AS5600_MAGNET_HIGH:
        return "MAGNET TOO STRONG"
    if status & AS5600_MAGNET_LOW:
        return "MAGNET TOO WEAK"
    if status & AS5600_MAGNET_DETECTED:
        return "MAGNET OK"
    return "NO MAGNET"


def as5600_valid(status):
    return bool(status & AS5600_MAGNET_DETECTED) and not bool(
        status & (AS5600_MAGNET_LOW | AS5600_MAGNET_HIGH)
    )


def read_as5600(bus, angle_filter):
    status = i2c_read_retry(bus, AS5600_STATUS, 1)[0]
    raw = i2c_u16(bus, AS5600_RAW_ANGLE) & 0x0FFF
    angle_raw = i2c_u16(bus, AS5600_ANGLE) & 0x0FFF
    agc = i2c_read_retry(bus, AS5600_AGC, 1)[0]
    magnitude = i2c_u16(bus, AS5600_MAGNITUDE) & 0x0FFF
    filtered = angle_filter.update(angle_raw, as5600_valid(status))
    return {
        "online": True,
        "status": status,
        "magnet": as5600_magnet(status),
        "raw": raw,
        "absolute_deg": angle_raw * 360.0 / AS5600_COUNTS,
        "filtered_deg": filtered,
        "agc": agc,
        "magnitude": magnitude,
        "error": "",
    }


def parity_is_odd(value):
    return bool(bin(value & 0xFFFF).count("1") & 1)


def as5047_command(register):
    frame = 0x4000 | (register & 0x3FFF)
    if parity_is_odd(frame):
        frame |= 0x8000
    return frame


def as5047_transfer(frame):
    received = bytearray(2)
    transmitted = bytes(((frame >> 8) & 0xFF, frame & 0xFF))
    as5047_cs.value(0)
    spi.write_readinto(transmitted, received)
    as5047_cs.value(1)
    time.sleep_us(2)
    return (received[0] << 8) | received[1]


def as5047_register(register):
    as5047_transfer(as5047_command(register))
    response = as5047_transfer(0x0000)
    return {
        "data": response & 0x3FFF,
        "sensor_error": bool(response & 0x4000),
        "parity_error": parity_is_odd(response),
    }


def as5047_magnet(diaagc):
    if diaagc & AS5047_MAGH:
        return "MAGNET TOO STRONG"
    if diaagc & AS5047_MAGL:
        return "MAGNET TOO WEAK / NO MAGNET"
    if diaagc & AS5047_COF:
        return "MAGNET INVALID"
    if not diaagc & AS5047_LF:
        return "MAGNET NOT READY"
    return "MAGNET OK"


def read_as5047():
    angle = as5047_register(AS5047_ANGLECOM)
    diaagc = as5047_register(AS5047_DIAAGC)
    magnitude = as5047_register(AS5047_MAG)
    errors = as5047_register(AS5047_ERRFL)
    communication_ok = not any(
        item["sensor_error"] or item["parity_error"]
        for item in (angle, diaagc, magnitude, errors)
    )
    magnet = as5047_magnet(diaagc["data"])
    valid = communication_ok and errors["data"] == 0 and magnet == "MAGNET OK"
    values = bldc_filter.update(angle["data"], valid)
    return {
        "online": communication_ok,
        "valid": valid,
        "raw": angle["data"],
        "absolute_deg": angle["data"] * 360.0 / AS5047_COUNTS,
        "diaagc": diaagc["data"],
        "agc": diaagc["data"] & 0xFF,
        "magnitude": magnitude["data"],
        "error_flags": errors["data"],
        "magnet": magnet,
        "values": values,
        "error": "",
    }


def failed_sample(error):
    return {
        "online": False, "valid": False, "magnet": "READ ERROR",
        "raw": -1, "absolute_deg": 0.0, "filtered_deg": None,
        "agc": -1, "magnitude": -1, "error_flags": -1,
        "values": None, "error": repr(error),
    }


def acquire_all():
    global steering_sample, brake_sample, bldc_sample, bldc_values
    try:
        steering_sample = read_as5600(steering_i2c, steering_filter)
    except Exception as error:
        steering_filter.rejected_samples += 1
        steering_sample = failed_sample(error)
    try:
        brake_sample = read_as5600(brake_i2c, brake_filter)
    except Exception as error:
        brake_filter.rejected_samples += 1
        brake_sample = failed_sample(error)
    try:
        bldc_sample = read_as5047()
        if bldc_sample["values"] is not None:
            bldc_values = bldc_sample["values"]
    except Exception as error:
        bldc_filter.rejected_samples += 1
        bldc_sample = failed_sample(error)


def format_angle(value):
    return "NOT ZEROED" if value is None else "{:+8.3f} deg".format(value)


def print_as5600(name, sample, angle_filter):
    if sample is None:
        print(name + ": NO SAMPLE")
        return
    if not sample["online"]:
        print(name + ": OFFLINE | " + sample["error"])
        return
    print(
        "{} | RAW {:4d} | ABS {:8.3f} deg | FILTERED {} | {} | "
        "AGC {:3d} MAG {:4d} | valid {} rejected {}".format(
            name, sample["raw"], sample["absolute_deg"],
            format_angle(sample["filtered_deg"]), sample["magnet"],
            sample["agc"], sample["magnitude"],
            angle_filter.valid_samples, angle_filter.rejected_samples,
        )
    )


def print_all():
    print_as5600("STEERING I2C0 ", steering_sample, steering_filter)
    print_as5600("BRAKE    SOFT ", brake_sample, brake_filter)
    if bldc_sample is None:
        print("BLDC AS5047P: NO SAMPLE")
        return
    if not bldc_sample["online"]:
        print("BLDC AS5047P: SPI ERROR | " + bldc_sample["error"])
        return
    print(
        "BLDC AS5047P | RAW {:5d} | ABS {:8.3f} deg | {} | "
        "AGC 0x{:02X} MAG 0x{:04X} ERR 0x{:04X}".format(
            bldc_sample["raw"], bldc_sample["absolute_deg"],
            bldc_sample["magnet"], bldc_sample["agc"],
            bldc_sample["magnitude"], bldc_sample["error_flags"],
        )
    )
    if bldc_values is None:
        print("  FILTER: waiting for a valid AS5047P sample")
    else:
        print(
            "  RPM motor {:+8.1f} axle {:+8.1f} | speed {:+7.2f} km/h | "
            "turns {:+9.4f} full {} | distance {:.4f} m displacement {:+.4f} m | "
            "valid {} rejected {}".format(
                bldc_values["motor_rpm"], bldc_values["axle_rpm"],
                bldc_values["speed_kmh"], bldc_values["motor_revs"],
                bldc_values["full_turns"], bldc_values["distance_m"],
                bldc_values["displacement_m"], bldc_filter.valid_samples,
                bldc_filter.rejected_samples,
            )
        )


def collect(duration_ms, display=False):
    started = ticks_ms()
    last_display = started
    while ticks_diff(ticks_ms(), started) < duration_ms:
        cycle_started = ticks_ms()
        acquire_all()
        now = ticks_ms()
        if display and ticks_diff(now, last_display) >= DISPLAY_INTERVAL_MS:
            print_all()
            print()
            last_display = now
        remaining = SAMPLE_INTERVAL_MS - ticks_diff(ticks_ms(), cycle_started)
        if remaining > 0:
            time.sleep_ms(remaining)


def validate_direct_bus(bus, name):
    devices = bus.scan()
    found = AS5600_ADDRESS in devices
    print("{} bus: {} {}".format(
        name, [hex(device) for device in devices],
        "FOUND" if found else "MISSING",
    ))
    return found


def zero_as5600(name):
    collect(200)
    if name == "steering":
        sample, angle_filter, filename = (
            steering_sample, steering_filter, STEERING_ZERO_FILE
        )
    else:
        sample, angle_filter, filename = brake_sample, brake_filter, BRAKE_ZERO_FILE
    if sample is None or not sample["online"] or sample["magnet"] != "MAGNET OK":
        print("ZERO REJECTED: {} encoder/magnet is not healthy".format(name))
        return
    angle_filter.zero(sample["raw"])
    save_zero(filename, sample["raw"])
    collect(100)
    print("{} ZERO SAVED: raw {} = 0.000 deg".format(name.upper(), sample["raw"]))


def clear_as5600(name):
    if name == "steering":
        angle_filter, filename = steering_filter, STEERING_ZERO_FILE
    else:
        angle_filter, filename = brake_filter, BRAKE_ZERO_FILE
    angle_filter.clear()
    remove_zero(filename)
    print(name.upper() + " zero cleared")


def show_status():
    print("Steering I2C0: SDA GPIO1, SCL GPIO2, 100 kHz, address 0x36")
    print("Brake SoftI2C: SDA GPIO35, SCL GPIO36, 100 kHz, address 0x36")
    print("PCA9548A: NOT USED")
    print("AS5047P SPI1: CS16 SCK18 MOSI41 MISO42, mode 1, 1 MHz")
    print("AS5600 filter: median 3 + 10 ms LPF | combined sample target 100 Hz")
    print("AS5047P filter: 30 ms RPM LPF | stationary position window +/-8 counts")
    print("Brake expected after zero: RELEASED 0 deg, APPLIED clockwise +7 deg")
    print("Distance uses nominal 11-inch tyre and 14T:44T ratio; not calibrated")
    print_all()


def show_help():
    print("Commands:")
    print("  read / r        sample for 1 second and show all encoders")
    print("  stream          live filtered output; Ctrl-C returns to prompt")
    print("  zero steering   save current steering position as 0 degrees")
    print("  zero brake      save current released-brake position as 0 degrees")
    print("  zero bldc       reset BLDC turns, RPM and distance counters")
    print("  clear steering  remove saved steering zero")
    print("  clear brake     remove saved brake zero")
    print("  status          show wiring, filters and latest readings")
    print("  help            show commands")
    print("  q               stop")


print()
print("================================================")
print(" AUTOKART ESP32-S3 ALL ENCODERS ISOLATED TEST")
print("================================================")
print("ENCODER ONLY: NO ACTUATOR OUTPUTS ARE INITIALIZED")
validate_direct_bus(steering_i2c, "Steering AS5600 GPIO1/GPIO2")
validate_direct_bus(brake_i2c, "Brake AS5600 GPIO35/GPIO36")

collect(300)
show_status()
show_help()

while True:
    try:
        command = input("\nAll encoders > ").strip().lower()
        if command in ("r", "read"):
            collect(1000)
            print_all()
        elif command == "stream":
            print("Streaming at approximately 100 Hz; displaying at 4 Hz.")
            print("Press Ctrl-C to return to the command prompt.")
            try:
                while True:
                    collect(DISPLAY_INTERVAL_MS, display=True)
            except KeyboardInterrupt:
                print("Streaming stopped.")
        elif command == "zero steering":
            zero_as5600("steering")
        elif command == "zero brake":
            zero_as5600("brake")
        elif command in ("zero bldc", "reset bldc"):
            bldc_filter.reset()
            bldc_values = None
            collect(100)
            print("BLDC turns, RPM and distance reset")
        elif command == "clear steering":
            clear_as5600("steering")
        elif command == "clear brake":
            clear_as5600("brake")
        elif command == "status":
            collect(300)
            show_status()
        elif command in ("help", "?"):
            show_help()
        elif command in ("q", "quit", "exit"):
            print("All-encoder test stopped.")
            break
        elif command:
            print("Unknown command. Type help.")
    except KeyboardInterrupt:
        print("\nUse q to stop or help for commands.")
    except Exception as error:
        print("ENCODER TEST ERROR:", repr(error))
