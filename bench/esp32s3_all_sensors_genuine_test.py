"""AutoKart ESP32-S3 genuine sensor and DAC validation.

Devices:
  - steering AS5600 directly on I2C(0), GPIO1/GPIO2
  - brake AS5600 directly on SoftI2C, GPIO35/GPIO36
  - Meukron AS5047P BLDC sprocket encoder over SPI
  - DFRobot GP8630N DAC (0x58)

This program boots the BLD-750 control outputs into STOP/BRAKE and does not
start either motor. GP8630N voltage validation is only allowed after the DAC
OUT wire is disconnected from BLD-750 SV and uses a multimeter for genuine
analog-output verification.
"""

from machine import I2C, Pin, SoftI2C, SPI
import math
import time


# Direct encoder buses plus the separate DAC bus.
STEERING_I2C_SDA = 1
STEERING_I2C_SCL = 2
BRAKE_I2C_SDA = 35
BRAKE_I2C_SCL = 36
GP_I2C_SDA = 7
GP_I2C_SCL = 21

AS5047_CS = 16
AS5047_SCK = 18
AS5047_MOSI = 41
AS5047_MISO = 42

BLD_BRK = 13
BLD_EN = 14
BLD_FR = 15

AS5600_ADDRESS = 0x36
GP8630N_ADDRESS = 0x58


# AS5600 registers and status bits.
AS5600_STATUS = 0x0B
AS5600_RAW_ANGLE = 0x0C
AS5600_AGC = 0x1A
AS5600_MAGNITUDE = 0x1B
AS5600_MD = 0x20
AS5600_ML = 0x10
AS5600_MH = 0x08


# AS5047P registers and diagnostic bits.
AS5047_ERRFL = 0x0001
AS5047_DIAAGC = 0x3FFC
AS5047_MAG = 0x3FFD
AS5047_ANGLECOM = 0x3FFF
AS5047_MAGL = 1 << 11
AS5047_MAGH = 1 << 10
AS5047_COF = 1 << 9
AS5047_LF = 1 << 8


# GP8630N official 0-10 V I2C-mode configuration.
GP8630_MODE_REGISTER = 0x01
GP8630_DATA_REGISTER = 0x02
GP8630_MODE_0_TO_10V = 0x1C


# Recorded AutoKart drivetrain values.
ENCODER_SPROCKET_TEETH = 14.0
AXLE_SPROCKET_TEETH = 44.0
MOTOR_REVS_PER_AXLE_REV = AXLE_SPROCKET_TEETH / ENCODER_SPROCKET_TEETH
NOMINAL_TYRE_DIAMETER_M = 11.0 * 0.0254
NOMINAL_TYRE_CIRCUMFERENCE_M = math.pi * NOMINAL_TYRE_DIAMETER_M
AS5047_COUNTS_PER_REV = 16384
AS5600_COUNTS_PER_REV = 4096

SAMPLE_INTERVAL_MS = 10
DISPLAY_INTERVAL_MS = 200
MAX_MOTOR_RPM = 6000.0
STEERING_TARGET_DEG = 35.0
STEERING_TARGET_TOLERANCE_DEG = 3.0
STEERING_CENTER_TOLERANCE_DEG = 2.0


# Safe BLD-750 state through the tested PN2222 interface.
bld_en = Pin(BLD_EN, Pin.OUT, value=0)     # EN open/high -> STOP
bld_brk = Pin(BLD_BRK, Pin.OUT, value=0)  # BRK open/high -> APPLIED
bld_fr = Pin(BLD_FR, Pin.OUT, value=0)     # direction only; motor stopped

steering_i2c = I2C(
    0,
    sda=Pin(STEERING_I2C_SDA),
    scl=Pin(STEERING_I2C_SCL),
    freq=100_000,
)
brake_i2c = SoftI2C(
    sda=Pin(BRAKE_I2C_SDA),
    scl=Pin(BRAKE_I2C_SCL),
    freq=100_000,
)
gp_i2c = I2C(
    1, sda=Pin(GP_I2C_SDA), scl=Pin(GP_I2C_SCL), freq=100_000
)

spi = SPI(
    1,
    baudrate=1_000_000,
    polarity=0,
    phase=1,
    bits=8,
    firstbit=SPI.MSB,
    sck=Pin(AS5047_SCK),
    mosi=Pin(AS5047_MOSI),
    miso=Pin(AS5047_MISO),
)
as5047_cs = Pin(AS5047_CS, Pin.OUT, value=1)


def wrapped_delta(current, previous, counts_per_rev):
    half = counts_per_rev // 2
    return ((int(current) - int(previous) + half) % counts_per_rev) - half


def median(values):
    ordered = sorted(values)
    return ordered[len(ordered) // 2]


def i2c_addresses(bus):
    return [hex(address) for address in bus.scan()]


def i2c_read_retry(bus, address, register, length, attempts=3):
    last_error = None
    for attempt in range(attempts):
        try:
            return bus.readfrom_mem(address, register, length)
        except OSError as error:
            last_error = error
            time.sleep_ms(5 * (attempt + 1))
    raise OSError(
        "address 0x{:02X} register 0x{:02X}: {}".format(
            address, register, last_error
        )
    )


def i2c_u16(bus, address, register):
    data = i2c_read_retry(bus, address, register, 2)
    return (data[0] << 8) | data[1]


def as5600_magnet_state(status):
    if status & AS5600_MH:
        return "MAGNET TOO STRONG"
    if status & AS5600_ML:
        return "MAGNET TOO WEAK"
    if status & AS5600_MD:
        return "MAGNET OK"
    return "NO MAGNET"


def as5600_read(bus):
    status = i2c_read_retry(bus, AS5600_ADDRESS, AS5600_STATUS, 1)[0]
    raw = i2c_u16(bus, AS5600_ADDRESS, AS5600_RAW_ANGLE) & 0x0FFF
    agc = i2c_read_retry(bus, AS5600_ADDRESS, AS5600_AGC, 1)[0]
    magnitude = i2c_u16(
        bus, AS5600_ADDRESS, AS5600_MAGNITUDE
    ) & 0x0FFF
    state = as5600_magnet_state(status)
    return {
        "raw": raw,
        "absolute_deg": raw * 360.0 / AS5600_COUNTS_PER_REV,
        "status": status,
        "magnet": state,
        "magnet_ok": state == "MAGNET OK",
        "agc": agc,
        "magnitude": magnitude,
    }


def odd_parity(value):
    return bool(bin(value & 0xFFFF).count("1") & 1)


def as5047_command(register):
    frame = 0x4000 | (register & 0x3FFF)
    if odd_parity(frame):
        frame |= 0x8000
    return frame


def as5047_transfer(frame):
    receive = bytearray(2)
    transmit = bytes(((frame >> 8) & 0xFF, frame & 0xFF))
    as5047_cs.value(0)
    spi.write_readinto(transmit, receive)
    as5047_cs.value(1)
    time.sleep_us(2)
    return (receive[0] << 8) | receive[1]


def as5047_register(register):
    # AS5047P replies one SPI frame after the command.
    as5047_transfer(as5047_command(register))
    response = as5047_transfer(0x0000)
    return {
        "data": response & 0x3FFF,
        "sensor_error": bool(response & 0x4000),
        "parity_error": odd_parity(response),
        "frame": response,
    }


def as5047_magnet_state(diaagc):
    if diaagc & AS5047_MAGH:
        return "MAGNET TOO STRONG"
    if diaagc & AS5047_MAGL:
        return "MAGNET TOO WEAK / NO MAGNET"
    if diaagc & AS5047_COF:
        return "MAGNET INVALID / CORDIC OVERFLOW"
    if not diaagc & AS5047_LF:
        return "MAGNET NOT READY"
    return "MAGNET OK"


def as5047_read():
    angle = as5047_register(AS5047_ANGLECOM)
    diaagc = as5047_register(AS5047_DIAAGC)
    magnitude = as5047_register(AS5047_MAG)
    errors = as5047_register(AS5047_ERRFL)
    communication_ok = not any(
        item["sensor_error"] or item["parity_error"]
        for item in (angle, diaagc, magnitude, errors)
    )
    state = as5047_magnet_state(diaagc["data"])
    error_flags_ok = errors["data"] == 0
    return {
        "raw": angle["data"],
        "absolute_deg": angle["data"] * 360.0 / AS5047_COUNTS_PER_REV,
        "diaagc": diaagc["data"],
        "agc": diaagc["data"] & 0xFF,
        "magnitude": magnitude["data"],
        "error_flags": errors["data"],
        "communication_ok": communication_ok,
        "magnet": state,
        "valid": communication_ok and error_flags_ok and state == "MAGNET OK",
    }


class SteeringFilter:
    def __init__(self):
        self.direction = 1
        self.zero_counts = None
        self.last_raw = None
        self.unwrapped = None
        self.filtered = None
        self.window = []

    def reset(self):
        self.last_raw = None
        self.unwrapped = None
        self.filtered = None
        self.window = []

    def update(self, raw):
        if self.last_raw is None:
            self.last_raw = raw
            self.unwrapped = float(raw)
            self.filtered = float(raw)
            self.window = [float(raw)]
        else:
            self.unwrapped += wrapped_delta(raw, self.last_raw, AS5600_COUNTS_PER_REV)
            self.last_raw = raw
            self.window.append(self.unwrapped)
            if len(self.window) > 3:
                self.window.pop(0)
            self.filtered += 0.30 * (median(self.window) - self.filtered)
        if self.zero_counts is None:
            return None
        return (
            (self.filtered - self.zero_counts)
            * 360.0 / AS5600_COUNTS_PER_REV
            * self.direction
        )

    def zero(self):
        if self.filtered is None:
            return False
        self.zero_counts = self.filtered
        return True


class DrivetrainFilter:
    def __init__(self):
        self.reset()

    def reset(self):
        self.last_raw = None
        self.last_us = None
        self.total_counts = 0
        self.filtered_motor_rpm = 0.0
        self.valid_samples = 0
        self.rejected_samples = 0

    def update(self, raw, valid):
        now = time.ticks_us()
        if not valid:
            self.rejected_samples += 1
            return None
        if self.last_raw is None:
            self.last_raw = raw
            self.last_us = now
            self.valid_samples += 1
            return self.values(0.0)
        elapsed_us = time.ticks_diff(now, self.last_us)
        if elapsed_us <= 0:
            self.rejected_samples += 1
            return None
        delta = wrapped_delta(raw, self.last_raw, AS5047_COUNTS_PER_REV)
        elapsed_s = elapsed_us / 1_000_000.0
        raw_rpm = delta / AS5047_COUNTS_PER_REV / elapsed_s * 60.0
        if abs(raw_rpm) > MAX_MOTOR_RPM:
            self.rejected_samples += 1
            return None
        self.last_raw = raw
        self.last_us = now
        self.total_counts += delta
        alpha = elapsed_s / (0.030 + elapsed_s)
        self.filtered_motor_rpm += alpha * (raw_rpm - self.filtered_motor_rpm)
        self.valid_samples += 1
        return self.values(raw_rpm)

    def values(self, raw_rpm):
        motor_revs = self.total_counts / AS5047_COUNTS_PER_REV
        axle_revs = motor_revs / MOTOR_REVS_PER_AXLE_REV
        axle_rpm = self.filtered_motor_rpm / MOTOR_REVS_PER_AXLE_REV
        speed_mps = axle_rpm / 60.0 * NOMINAL_TYRE_CIRCUMFERENCE_M
        return {
            "raw_motor_rpm": raw_rpm,
            "motor_rpm": self.filtered_motor_rpm,
            "axle_rpm": axle_rpm,
            "speed_kmh": speed_mps * 3.6,
            "motor_revs": motor_revs,
            "axle_revs": axle_revs,
            "distance_m": axle_revs * NOMINAL_TYRE_CIRCUMFERENCE_M,
        }


steering_filter = SteeringFilter()
drivetrain_filter = DrivetrainFilter()


def gp8630_set_voltage(voltage):
    voltage = max(0.0, min(10.0, float(voltage)))
    gp_i2c.writeto_mem(
        GP8630N_ADDRESS,
        GP8630_MODE_REGISTER,
        bytes((GP8630_MODE_0_TO_10V,)),
    )
    value = int(round(voltage / 10.0 * 65535.0))
    gp_i2c.writeto_mem(
        GP8630N_ADDRESS,
        GP8630_DATA_REGISTER,
        bytes((value & 0xFF, (value >> 8) & 0xFF)),
    )
    return value


def safe_dac_zero():
    try:
        return gp8630_set_voltage(0.0)
    except OSError:
        return None


def bus_check():
    print("\nI2C BUS CHECK")
    try:
        steering_bus = i2c_addresses(steering_i2c)
        brake_bus = i2c_addresses(brake_i2c)
        gp_bus = i2c_addresses(gp_i2c)
        print("  Steering I2C(0) SDA1/SCL2:", steering_bus)
        print("  Brake SoftI2C SDA35/SCL36:", brake_bus)
        print("  GP I2C(1) SDA7/SCL21:", gp_bus)
        steering_ok = hex(AS5600_ADDRESS) in steering_bus
        brake_ok = hex(AS5600_ADDRESS) in brake_bus
        gp_ok = hex(GP8630N_ADDRESS) in gp_bus
        print("  GP8630N:", "PASS" if gp_ok else "FAIL")
        print("  Steering AS5600:", "PASS" if steering_ok else "FAIL")
        print("  Brake AS5600:", "PASS" if brake_ok else "FAIL")
        return gp_ok and steering_ok and brake_ok
    except Exception as error:
        print("  I2C CHECK FAIL:", error)
        return False


def snapshot():
    print("\nLIVE SENSOR SNAPSHOT")
    try:
        steering = as5600_read(steering_i2c)
        angle = (
            steering_filter.update(steering["raw"])
            if steering["magnet_ok"] else None
        )
        print(
            "  AS5600 raw {:4d} abs {:8.3f} deg filtered {} | {} | "
            "MAG {} AGC {}".format(
                steering["raw"], steering["absolute_deg"],
                "not zeroed" if angle is None else "{:+.3f} deg".format(angle),
                steering["magnet"], steering["magnitude"], steering["agc"]
            )
        )
    except Exception as error:
        print("  STEERING AS5600 READ FAIL:", error)

    try:
        brake = as5600_read(brake_i2c)
        print(
            "  Brake AS5600 raw {:4d} abs {:8.3f} deg | {} | "
            "MAG {} AGC {}".format(
                brake["raw"], brake["absolute_deg"], brake["magnet"],
                brake["magnitude"], brake["agc"]
            )
        )
    except Exception as error:
        print("  BRAKE AS5600 READ FAIL:", error)

    try:
        encoder = as5047_read()
        motion = drivetrain_filter.update(encoder["raw"], encoder["valid"])
        print(
            "  AS5047P raw {:5d} abs {:8.3f} deg | {} | SPI {} | "
            "MAG 0x{:04X} AGC 0x{:02X} ERR 0x{:04X}".format(
                encoder["raw"], encoder["absolute_deg"], encoder["magnet"],
                "OK" if encoder["communication_ok"] else "ERROR",
                encoder["magnitude"], encoder["agc"], encoder["error_flags"]
            )
        )
        if motion:
            print(
                "  BLDC motor {:+8.1f} RPM | axle {:+7.1f} RPM | "
                "nominal {:+6.2f} km/h | distance {:+.3f} m".format(
                    motion["motor_rpm"], motion["axle_rpm"],
                    motion["speed_kmh"], motion["distance_m"]
                )
            )
    except Exception as error:
        print("  AS5047P READ FAIL:", error)


def steering_validation():
    steering_filter.reset()
    valid = 0
    print("\nCenter steering and keep it still for automatic zeroing...")
    started = time.ticks_ms()
    while time.ticks_diff(time.ticks_ms(), started) < 600:
        try:
            sample = as5600_read(steering_i2c)
            if sample["magnet_ok"]:
                steering_filter.update(sample["raw"])
                valid += 1
        except OSError:
            pass
        time.sleep_ms(SAMPLE_INTERVAL_MS)
    if valid < 20 or not steering_filter.zero():
        print("STEERING TEST FAIL: insufficient valid samples for zero")
        return

    seen_center = False
    seen_left = False
    seen_right = False
    minimum = 999.0
    maximum = -999.0
    errors = 0
    print("Zero = 0 deg. Manually or by RC move LEFT 35, CENTER, RIGHT 35.")
    print("Ctrl-C when complete.")
    last_display = time.ticks_ms()
    try:
        while True:
            try:
                sample = as5600_read(steering_i2c)
                if not sample["magnet_ok"]:
                    errors += 1
                    angle = None
                else:
                    angle = steering_filter.update(sample["raw"])
                if angle is not None and sample["magnet_ok"]:
                    minimum = min(minimum, angle)
                    maximum = max(maximum, angle)
                    seen_center |= abs(angle) <= STEERING_CENTER_TOLERANCE_DEG
                    seen_left |= abs(angle + STEERING_TARGET_DEG) <= STEERING_TARGET_TOLERANCE_DEG
                    seen_right |= abs(angle - STEERING_TARGET_DEG) <= STEERING_TARGET_TOLERANCE_DEG
                if time.ticks_diff(time.ticks_ms(), last_display) >= DISPLAY_INTERVAL_MS:
                    print(
                        "STEERING {:+7.2f} deg | raw {:4d} | {} | MAG {} AGC {}".format(
                            angle or 0.0, sample["raw"], sample["magnet"],
                            sample["magnitude"], sample["agc"]
                        )
                    )
                    last_display = time.ticks_ms()
            except OSError as error:
                errors += 1
                print("AS5600 I2C ERROR:", error)
            time.sleep_ms(SAMPLE_INTERVAL_MS)
    except KeyboardInterrupt:
        print("\nSTEERING VALIDATION")
        print("  Center 0 deg:", "PASS" if seen_center else "NOT REACHED")
        print("  Left -35 deg:", "PASS" if seen_left else "NOT REACHED")
        print("  Right +35 deg:", "PASS" if seen_right else "NOT REACHED")
        if minimum < 900:
            print("  Observed range: {:+.2f} to {:+.2f} deg".format(minimum, maximum))
        print("  Read/magnet errors:", errors)
        passed = seen_center and seen_left and seen_right and errors == 0
        print("  RESULT:", "PASS" if passed else "INCOMPLETE / FAIL")
        if seen_right and minimum > -10:
            print("  Direction may be reversed; use the invert command and retest.")


def bldc_encoder_validation():
    drivetrain_filter.reset()
    communication_errors = 0
    magnet_errors = 0
    print("\nBLDC POWER OFF. Rotate the marked 14T sprocket by hand.")
    print("For scale validation, turn it exactly one revolution. Ctrl-C ends.")
    last_display = time.ticks_ms()
    latest_motion = None
    try:
        while True:
            sample = as5047_read()
            if not sample["communication_ok"]:
                communication_errors += 1
            if sample["magnet"] != "MAGNET OK":
                magnet_errors += 1
            motion = drivetrain_filter.update(sample["raw"], sample["valid"])
            if motion is not None:
                latest_motion = motion
            if time.ticks_diff(time.ticks_ms(), last_display) >= DISPLAY_INTERVAL_MS:
                if latest_motion:
                    print(
                        "RAW {:5d} | motor {:+8.1f} rpm {:+7.3f} rev | "
                        "axle {:+7.1f} rpm | nominal {:+6.2f} km/h {:+.3f} m | {}".format(
                            sample["raw"], latest_motion["motor_rpm"],
                            latest_motion["motor_revs"], latest_motion["axle_rpm"],
                            latest_motion["speed_kmh"], latest_motion["distance_m"],
                            sample["magnet"]
                        )
                    )
                last_display = time.ticks_ms()
            time.sleep_ms(SAMPLE_INTERVAL_MS)
    except KeyboardInterrupt:
        turns = 0.0 if latest_motion is None else latest_motion["motor_revs"]
        moved = abs(drivetrain_filter.total_counts) >= 100
        one_turn = abs(abs(turns) - 1.0) <= 0.05
        print("\nBLDC ENCODER VALIDATION")
        print("  Genuine angle movement:", "PASS" if moved else "NOT DETECTED")
        print("  Motor sprocket turns: {:+.4f}".format(turns))
        print("  One marked revolution:", "PASS" if one_turn else "NOT CONFIRMED")
        print("  SPI/parity errors:", communication_errors)
        print("  Magnet errors:", magnet_errors)
        passed = moved and one_turn and communication_errors == 0 and magnet_errors == 0
        print("  RESULT:", "PASS" if passed else "INCOMPLETE / FAIL")
        print("  14T/44T conversions are nominal until loaded tyre circumference is measured.")


def linear_fit(points):
    count = len(points)
    sum_x = sum(point[0] for point in points)
    sum_y = sum(point[1] for point in points)
    sum_xx = sum(point[0] * point[0] for point in points)
    sum_xy = sum(point[0] * point[1] for point in points)
    denominator = count * sum_xx - sum_x * sum_x
    if count < 2 or abs(denominator) < 0.000001:
        return None
    slope = (count * sum_xy - sum_x * sum_y) / denominator
    intercept = (sum_y - slope * sum_x) / count
    return slope, intercept


def gp8630_voltage_validation():
    print("\nGP8630N GENUINE OUTPUT TEST")
    print("Disconnect GP8630N OUT from BLD-750 SV.")
    print("Connect multimeter: red=OUT, black=GP8630N output GND, DC volts.")
    confirmation = input("Type DISCONNECTED to continue: ").strip().upper()
    if confirmation != "DISCONNECTED":
        print("DAC test cancelled; output remains 0 V")
        safe_dac_zero()
        return

    points = []
    failures = 0
    try:
        for requested in (0.0, 0.5, 1.0, 2.0, 3.0, 4.0, 5.0):
            raw = gp8630_set_voltage(requested)
            time.sleep_ms(750)
            answer = input(
                "Requested {:.1f} V (DAC {}). Enter measured V or s: ".format(
                    requested, raw
                )
            ).strip().lower()
            if answer in ("s", "skip", ""):
                print("  SKIPPED")
                continue
            measured = float(answer)
            error = measured - requested
            tolerance = max(0.10, requested * 0.05)
            passed = abs(error) <= tolerance
            failures += 0 if passed else 1
            points.append((requested, measured))
            print(
                "  {} | error {:+.3f} V ({:+.1f}%) | tolerance +/-{:.3f} V".format(
                    "PASS" if passed else "FAIL", error,
                    0.0 if requested == 0 else error / requested * 100.0,
                    tolerance
                )
            )
    except (ValueError, KeyboardInterrupt) as error:
        print("DAC test interrupted:", error)
    finally:
        safe_dac_zero()
        print("GP8630N COMMAND RETURNED TO 0.0 V")

    fit = linear_fit(points)
    if fit:
        print("Measured transfer: measured_V = {:.5f} * command_V {:+.5f}".format(
            fit[0], fit[1]
        ))
    print("Measured points:", len(points), "| failed points:", failures)
    print("RESULT:", "PASS" if len(points) == 7 and failures == 0 else "INCOMPLETE / FAIL")


def invert_steering():
    steering_filter.direction *= -1
    print("Steering encoder direction:", steering_filter.direction)
    print("Required convention: LEFT negative, RIGHT positive")


def run_all_tests():
    print("\nGUIDED ALL-SENSOR CHECK")
    print("Motors remain stopped. You will move both encoder magnets manually.")
    bus_check()
    snapshot()
    input("\nCenter steering, then press Enter for the AS5600 movement test: ")
    steering_validation()
    input("\nSwitch BLDC power OFF, then press Enter for the AS5047P test: ")
    bldc_encoder_validation()
    answer = input(
        "\nRun GP8630N multimeter test too? Type yes or no: "
    ).strip().lower()
    if answer in ("y", "yes"):
        gp8630_voltage_validation()
    else:
        safe_dac_zero()
        print("GP8630N analog-output test skipped; DAC command = 0 V")


def show_status():
    print("\nFINAL PINOUT")
    print("  Steering AS5600 I2C(0): SDA GPIO1, SCL GPIO2")
    print("  Brake AS5600 SoftI2C: SDA GPIO35, SCL GPIO36")
    print("  GP8630N I2C(1): SDA GPIO7, SCL GPIO21")
    print("  PCA9548A: not used")
    print("  GP8630N 0x58, OUT disconnected from BLD during voltage test")
    print("  AS5047P: CS16 SCK18 MOSI41 MISO42, SPI mode 1")
    print("  BLD safety: EN14 STOP, BRK13 APPLIED; F/R15 defaults to CW")
    print("  Drivetrain: 14T encoder sprocket -> 44T axle sprocket")
    print("  Nominal tyre: 11 inch, circumference {:.6f} m".format(
        NOMINAL_TYRE_CIRCUMFERENCE_M
    ))
    print("  Steering direction:", steering_filter.direction)


def show_help():
    print("Commands:")
    print("  all       - guided genuine validation of every listed device")
    print("  bus       - verify both AS5600 buses and GP8630N routing")
    print("  read      - live readings from both encoders")
    print("  steering  - genuine 0/-35/+35 steering movement validation")
    print("  bldc      - genuine AS5047P movement, one-turn and drivetrain test")
    print("  dac       - guided GP8630N multimeter accuracy test")
    print("  invert    - reverse steering encoder sign")
    print("  zero-bldc - reset RPM/distance/count history")
    print("  status    - show pins and mechanical constants")
    print("  help      - show commands")
    print("  q         - stop")


print()
print("====================================================")
print(" AUTOKART ESP32-S3 GENUINE SENSOR / DAC VALIDATION")
print("====================================================")
print("SAFE BOOT: BLD-750 STOP + BRAKE APPLIED")
print("This program never starts the BLDC or steering motor.")

safe_dac_zero()
bus_check()
snapshot()
show_help()

while True:
    try:
        command = input("\nAll sensors > ").strip().lower()
        if command == "all":
            run_all_tests()
        elif command == "bus":
            bus_check()
        elif command in ("read", "r"):
            snapshot()
        elif command in ("steering", "steer"):
            steering_validation()
        elif command in ("bldc", "encoder"):
            bldc_encoder_validation()
        elif command in ("dac", "gp8630"):
            gp8630_voltage_validation()
        elif command == "invert":
            invert_steering()
        elif command == "zero-bldc":
            drivetrain_filter.reset()
            print("BLDC count, distance and RPM history reset")
        elif command == "status":
            show_status()
            snapshot()
        elif command in ("help", "?"):
            show_help()
        elif command in ("q", "quit", "exit"):
            safe_dac_zero()
            print("All-sensor test stopped; GP8630N command = 0 V")
            break
        elif command:
            print("Unknown command. Type help.")
    except KeyboardInterrupt:
        safe_dac_zero()
        print("\nInterrupted; GP8630N command = 0 V")
    except Exception as error:
        safe_dac_zero()
        print("TEST ERROR:", error)
