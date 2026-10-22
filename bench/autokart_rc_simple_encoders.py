"""AutoKart full Wi-Fi RC bench controller for ESP32-S3 MicroPython.

Functions:
  - game-style steering: hold left/right, release to hold position, center button
  - traction: forward=CCW, reverse=CW, controlled ramp and five voltage gears
  - momentary brake: press applies, release disengages while armed
  - arm releases stepper and BLD750 brakes; disarm applies both brakes
  - steering AS5600 through PCA9548A channel 0
  - brake AS5600 through PCA9548A channel 1
  - AS5047P BLDC encoder through SPI with RPM, turns and distance telemetry

BLDC distance is nominal and cannot detect chain or tyre slip. This is a
wheels-off-ground bench controller, not production vehicle firmware.
"""

import network
import socket
import time
from machine import I2C, Pin, SPI


SSID = "AutoKart-RC"
PASSWORD = "12345678"
WATCHDOG_MS = 1800

# Steering R86mini: tested scale is 2000 pulses for 45 degrees. RC travel is
# limited to +/-35 degrees, therefore each side uses about 1556 pulses.
STEER_STEP_GPIO = 4
STEER_DIR_GPIO = 5
STEER_RIGHT_DIR = 1
STEER_LIMIT_DEG = 35.0
STEER_PULSES_AT_45 = 2000
STEER_PULSES_PER_DEG = STEER_PULSES_AT_45 / 45.0
STEER_START_PPS = 180
STEER_MAX_PPS = 700
STEER_HOLD_TIMEOUT_MS = 450
STEER_ENCODER_RIGHT_SIGN = 1
BRAKE_ENCODER_APPLY_SIGN = 1

# Brake TB6600 with physical AS5600 position feedback on PCA channel 1.
BRAKE_STEP_GPIO = 38
BRAKE_DIR_GPIO = 39
BRAKE_EN_GPIO = 40
BRAKE_APPLY_DIR = 1
BRAKE_ENABLE_LEVEL = 0
BRAKE_DISABLE_LEVEL = 1
BRAKE_PULSES_PER_REV = 3200
BRAKE_PULSES_PER_DEG = BRAKE_PULSES_PER_REV / 360.0
BRAKE_APPLY_DEG = 45.0

# Frozen separate I2C buses: PCA/encoders on I2C(0), DAC on I2C(1).
PCA_SDA_GPIO = 1
PCA_SCL_GPIO = 2
GP_SDA_GPIO = 7
GP_SCL_GPIO = 21
PCA_ADDR = 0x70
AS5600_ADDR = 0x36
GP_ADDR = 0x58
GP_MODE_REG = 0x01
GP_DAC_REG = 0x02
GP_MODE_0_10V = 0x1C
THROTTLE_MAX_V = 5.0
THROTTLE_RAMP_STEP_V = 0.08
THROTTLE_RAMP_INTERVAL_MS = 35
DIRECTION_SETTLE_MS = 350

# BLD750 transistor stages.
BLD_BRK_GPIO = 13
BLD_EN_GPIO = 14
BLD_FR_GPIO = 15

# AS5047P SPI.
ENC_CS_GPIO = 16
ENC_SCK_GPIO = 18
ENC_MOSI_GPIO = 41
ENC_MISO_GPIO = 42
AS5047_COUNTS_PER_REV = 16384
BLDC_ENCODER_INTERVAL_MS = 5
STEERING_ENCODER_INTERVAL_MS = 20
BLDC_MAX_VALID_RPM = 6000.0
BLDC_RPM_FILTER_ALPHA = 0.22
BLDC_NOISE_COUNTS = 2

# BLDC encoder is on the 14T drive sprocket. It drives the 44T axle sprocket.
BLDC_SPROCKET_TEETH = 14.0
AXLE_SPROCKET_TEETH = 44.0
MOTOR_REVS_PER_AXLE_REV = AXLE_SPROCKET_TEETH / BLDC_SPROCKET_TEETH
REAR_TYRE_DIAMETER_M = 11.0 * 0.0254
REAR_TYRE_CIRCUMFERENCE_M = 3.141592653589793 * REAR_TYRE_DIAMETER_M
TRAVEL_PER_MOTOR_REV_M = REAR_TYRE_CIRCUMFERENCE_M / MOTOR_REVS_PER_AXLE_REV

STEP_HIGH_US = 20


def clamp(value, low, high):
    return max(low, min(high, value))


def wrapped_delta(current, previous, counts_per_turn):
    half = counts_per_turn // 2
    return ((current - previous + half) % counts_per_turn) - half


class StepperAxis:
    def __init__(self, step_gpio, dir_gpio, pulses_per_degree,
                 positive_direction, start_pps, max_pps):
        self.step = Pin(step_gpio, Pin.OUT, value=0)
        self.direction = Pin(dir_gpio, Pin.OUT, value=0)
        self.pulses_per_degree = float(pulses_per_degree)
        self.positive_direction = positive_direction
        self.start_period_us = 1_000_000 // start_pps
        self.fast_period_us = 1_000_000 // max_pps
        self.period_us = self.start_period_us
        self.current_steps = 0
        self.target_steps = 0
        self.last_step_us = time.ticks_us()
        self.last_direction = 0

    def degrees(self):
        return self.current_steps / self.pulses_per_degree

    def target_degrees(self):
        return self.target_steps / self.pulses_per_degree

    def set_target(self, degrees):
        self.target_steps = round(float(degrees) * self.pulses_per_degree)

    def hold(self):
        self.target_steps = self.current_steps

    def at_target(self):
        return self.current_steps == self.target_steps

    def update(self):
        if self.at_target():
            self.period_us = self.start_period_us
            self.last_direction = 0
            return
        move = 1 if self.target_steps > self.current_steps else -1
        if move != self.last_direction:
            self.last_direction = move
            self.period_us = self.start_period_us
            self.direction.value(
                self.positive_direction if move > 0
                else 1 - self.positive_direction
            )
        now = time.ticks_us()
        if time.ticks_diff(now, self.last_step_us) < self.period_us:
            return
        self.step.value(1)
        time.sleep_us(STEP_HIGH_US)
        self.step.value(0)
        self.current_steps += move
        self.last_step_us = now
        self.period_us = max(self.fast_period_us, self.period_us - 10)


def parity_is_odd(value):
    return bool(bin(value & 0xFFFF).count("1") & 1)


def as5047_command(register):
    frame = 0x4000 | (register & 0x3FFF)
    if parity_is_odd(frame):
        frame |= 0x8000
    return frame


def as5047_bytes(frame):
    return bytes(((frame >> 8) & 0xFF, frame & 0xFF))


# Outputs enter their safe state before Wi-Fi or buses initialize.
bld_en = Pin(BLD_EN_GPIO, Pin.OUT, value=0)
bld_brk = Pin(BLD_BRK_GPIO, Pin.OUT, value=0)
bld_fr = Pin(BLD_FR_GPIO, Pin.OUT, value=1)  # selected forward/CCW
brake_enable = Pin(BRAKE_EN_GPIO, Pin.OUT, value=BRAKE_ENABLE_LEVEL)

steering = StepperAxis(
    STEER_STEP_GPIO, STEER_DIR_GPIO, STEER_PULSES_PER_DEG,
    STEER_RIGHT_DIR, STEER_START_PPS, STEER_MAX_PPS,
)
brake = StepperAxis(
    BRAKE_STEP_GPIO, BRAKE_DIR_GPIO, BRAKE_PULSES_PER_DEG,
    BRAKE_APPLY_DIR, 250, 900,
)

pca_i2c = I2C(
    0, sda=Pin(PCA_SDA_GPIO), scl=Pin(PCA_SCL_GPIO), freq=100_000
)
gp_i2c = I2C(
    1, sda=Pin(GP_SDA_GPIO), scl=Pin(GP_SCL_GPIO), freq=100_000
)
pca_i2c_devices = pca_i2c.scan()
gp_i2c_devices = gp_i2c.scan()
pca_present = PCA_ADDR in pca_i2c_devices
gp_present = GP_ADDR in gp_i2c_devices
if gp_present:
    gp_i2c.writeto_mem(GP_ADDR, GP_MODE_REG, bytes((GP_MODE_0_10V,)))

spi = SPI(
    1, baudrate=1_000_000, polarity=0, phase=1, bits=8,
    firstbit=SPI.MSB, sck=Pin(ENC_SCK_GPIO),
    mosi=Pin(ENC_MOSI_GPIO), miso=Pin(ENC_MISO_GPIO),
)
encoder_cs = Pin(ENC_CS_GPIO, Pin.OUT, value=1)

armed = False
direction = "FORWARD"
requested_direction = "FORWARD"
selected_gear = 1
gear = 0
throttle_command = 0.0
target_voltage = 0.0
drive_state = "STOPPED"
reversal_pending = False
direction_settle_started_ms = None
last_throttle_ramp_ms = time.ticks_ms()
brake_state = "APPLYING"
brake_transition_started_ms = time.ticks_ms()
brake_transition_ms = 0
brake_validation = "MOVING"
brake_validation_error_deg = 0.0
brake_transition_count = 0
steer_hold_direction = "STOP"
last_steer_hold_ms = 0
last_contact_ms = time.ticks_ms()
last_steering_encoder_ms = 0
last_bldc_encoder_ms = 0

steer_encoder = {
    "online": False, "magnet": "NO SENSOR", "raw": -1,
    "angle": 0.0, "errors": 0, "initialized": False,
    "zero_raw": -1, "direction": "CENTER",
}
brake_encoder = {
    "online": False, "magnet": "NO SENSOR", "raw": -1,
    "angle": 0.0, "errors": 0, "initialized": False,
    "zero_raw": -1,
}
bldc_encoder = {
    "online": False, "magnet": "NO SENSOR", "raw": -1,
    "angle": 0.0, "errors": 0, "initialized": False,
    "zero_raw": -1, "last_raw": -1, "last_us": 0,
    "signed_counts": 0, "travel_counts": 0,
    "rpm": 0.0, "axle_rpm": 0.0, "motor_revs": 0.0,
    "full_turns": 0, "distance_m": 0.0, "displacement_m": 0.0,
    "speed_kmh": 0.0,
}


def pca_select(channel):
    if not pca_present:
        return False
    pca_i2c.writeto(PCA_ADDR, bytes((1 << channel,)))
    return True


def as5600_read_channel(channel, state, direction_sign, allow_initialize=True):
    try:
        if not pca_select(channel):
            state["online"] = False
            state["magnet"] = "PCA9548A MISSING"
            return
        status = pca_i2c.readfrom_mem(AS5600_ADDR, 0x0B, 1)[0]
        data = pca_i2c.readfrom_mem(AS5600_ADDR, 0x0E, 2)
        raw = ((data[0] << 8) | data[1]) & 0x0FFF
        if status & 0x08:
            magnet = "TOO STRONG"
        elif status & 0x10:
            magnet = "TOO WEAK"
        elif status & 0x20:
            magnet = "OK"
        else:
            magnet = "NO MAGNET"
        state.update(online=True, magnet=magnet, raw=raw)
        if magnet == "OK":
            if not state["initialized"]:
                if not allow_initialize:
                    return
                state["zero_raw"] = raw
                state["initialized"] = True
            delta = wrapped_delta(raw, state["zero_raw"], 4096)
            angle = delta * 360.0 / 4096.0 * direction_sign
            if abs(angle) < 0.15:
                angle = 0.0
            state["angle"] = angle
    except Exception:
        state["online"] = False
        state["magnet"] = "READ ERROR"
        state["errors"] += 1


def as5600_update():
    as5600_read_channel(0, steer_encoder, STEER_ENCODER_RIGHT_SIGN)
    steer_encoder["direction"] = (
        "RIGHT" if steer_encoder["angle"] > 0.15
        else "LEFT" if steer_encoder["angle"] < -0.15
        else "CENTER"
    )
    brake_is_at_released_home = (
        brake.at_target() and abs(brake.degrees()) < 1.0
    )
    as5600_read_channel(
        1, brake_encoder, BRAKE_ENCODER_APPLY_SIGN,
        allow_initialize=brake_is_at_released_home,
    )


def as5047_transfer(frame):
    received = bytearray(2)
    encoder_cs.value(0)
    spi.write_readinto(as5047_bytes(frame), received)
    encoder_cs.value(1)
    time.sleep_us(2)
    return (received[0] << 8) | received[1]


def as5047_read(register):
    as5047_transfer(as5047_command(register))
    response = as5047_transfer(0)
    return response & 0x3FFF, bool(response & 0x4000), parity_is_odd(response)


def as5047_update():
    try:
        raw, sensor_error, parity_error = as5047_read(0x3FFF)
        diag, diag_error, diag_parity = as5047_read(0x3FFC)
        bad = sensor_error or parity_error or diag_error or diag_parity
        if diag & (1 << 10):
            magnet = "TOO STRONG"
        elif diag & (1 << 11):
            magnet = "TOO WEAK"
        elif diag & (1 << 9):
            magnet = "INVALID"
        elif not diag & (1 << 8):
            magnet = "NOT READY"
        else:
            magnet = "OK"
        bldc_encoder.update(online=not bad, magnet=magnet, raw=raw)
        if bad:
            bldc_encoder["errors"] += 1
            return
        if magnet != "OK":
            return

        now_us = time.ticks_us()
        if not bldc_encoder["initialized"]:
            bldc_encoder.update(
                initialized=True, zero_raw=raw, last_raw=raw,
                last_us=now_us, angle=0.0,
            )
            return

        dt_us = time.ticks_diff(now_us, bldc_encoder["last_us"])
        delta = wrapped_delta(raw, bldc_encoder["last_raw"],
                              AS5047_COUNTS_PER_REV)
        bldc_encoder["last_raw"] = raw
        bldc_encoder["last_us"] = now_us
        bldc_encoder["angle"] = (
            wrapped_delta(raw, bldc_encoder["zero_raw"],
                          AS5047_COUNTS_PER_REV)
            * 360.0 / AS5047_COUNTS_PER_REV
        )
        if dt_us <= 0:
            return
        raw_rpm = (
            delta * 60_000_000.0
            / (AS5047_COUNTS_PER_REV * dt_us)
        )
        if abs(raw_rpm) > BLDC_MAX_VALID_RPM:
            bldc_encoder["errors"] += 1
            return
        if abs(delta) <= BLDC_NOISE_COUNTS:
            delta = 0
            raw_rpm = 0.0
        bldc_encoder["signed_counts"] += delta
        bldc_encoder["travel_counts"] += abs(delta)
        bldc_encoder["rpm"] += BLDC_RPM_FILTER_ALPHA * (
            raw_rpm - bldc_encoder["rpm"]
        )
        if abs(bldc_encoder["rpm"]) < 1.0 and delta == 0:
            bldc_encoder["rpm"] = 0.0
        bldc_encoder["axle_rpm"] = (
            bldc_encoder["rpm"] / MOTOR_REVS_PER_AXLE_REV
        )
        bldc_encoder["motor_revs"] = (
            bldc_encoder["signed_counts"] / AS5047_COUNTS_PER_REV
        )
        travelled_revs = (
            bldc_encoder["travel_counts"] / AS5047_COUNTS_PER_REV
        )
        bldc_encoder["full_turns"] = int(travelled_revs)
        bldc_encoder["distance_m"] = (
            travelled_revs * TRAVEL_PER_MOTOR_REV_M
        )
        bldc_encoder["displacement_m"] = (
            bldc_encoder["motor_revs"] * TRAVEL_PER_MOTOR_REV_M
        )
        bldc_encoder["speed_kmh"] = abs(
            bldc_encoder["axle_rpm"] * REAR_TYRE_CIRCUMFERENCE_M
            * 60.0 / 1000.0
        )
    except Exception:
        bldc_encoder["online"] = False
        bldc_encoder["magnet"] = "READ ERROR"
        bldc_encoder["errors"] += 1


def update_encoders():
    global last_steering_encoder_ms, last_bldc_encoder_ms
    now = time.ticks_ms()
    if time.ticks_diff(now, last_bldc_encoder_ms) >= BLDC_ENCODER_INTERVAL_MS:
        last_bldc_encoder_ms = now
        as5047_update()
    if time.ticks_diff(now, last_steering_encoder_ms) >= STEERING_ENCODER_INTERVAL_MS:
        last_steering_encoder_ms = now
        as5600_update()


def zero_encoders():
    """Zero steering and BLDC while preserving brake released calibration."""
    if steer_encoder["online"] and steer_encoder["magnet"] == "OK":
        steer_encoder.update(
            initialized=True, zero_raw=steer_encoder["raw"],
            angle=0.0, direction="CENTER",
        )
        steering.current_steps = 0
        steering.target_steps = 0
    if bldc_encoder["online"] and bldc_encoder["magnet"] == "OK":
        now_us = time.ticks_us()
        raw = bldc_encoder["raw"]
        bldc_encoder.update(
            initialized=True, zero_raw=raw, last_raw=raw, last_us=now_us,
            angle=0.0, signed_counts=0, travel_counts=0, rpm=0.0,
            axle_rpm=0.0, motor_revs=0.0, full_turns=0,
            distance_m=0.0, displacement_m=0.0, speed_kmh=0.0,
        )
    # Brake zero is captured at boot in the released position, before apply.
    return (
        steer_encoder["initialized"] and brake_encoder["initialized"]
        and bldc_encoder["initialized"]
    )


def write_voltage(voltage):
    global throttle_command, gp_present
    voltage = clamp(float(voltage), 0.0, THROTTLE_MAX_V)
    if gp_present:
        try:
            raw = int(voltage / 10.0 * 65535)
            gp_i2c.writeto_mem(
                GP_ADDR, GP_DAC_REG, bytes((raw & 0xFF, raw >> 8))
            )
        except Exception:
            gp_present = False
            throttle_command = 0.0
            raise
    elif voltage > 0.0:
        raise OSError("GP8630N offline")
    throttle_command = voltage


def traction_stop():
    global gear, target_voltage, drive_state, reversal_pending
    global direction_settle_started_ms
    gear = 0
    target_voltage = 0.0
    drive_state = "STOPPED"
    reversal_pending = False
    direction_settle_started_ms = None
    bld_en.value(0)
    try:
        write_voltage(0.0)
    except Exception as error:
        throttle_command = 0.0
        print("DAC STOP WRITE ERROR:", error)


def request_soft_stop():
    global target_voltage, drive_state, reversal_pending
    target_voltage = 0.0
    reversal_pending = False
    drive_state = "SLOWING" if throttle_command > 0.01 else "STOPPED"


def request_drive(new_direction):
    global requested_direction, target_voltage, drive_state
    global reversal_pending, direction_settle_started_ms
    requested_direction = new_direction
    if new_direction != direction:
        target_voltage = 0.0
        reversal_pending = True
        direction_settle_started_ms = None
        drive_state = "SLOWING FOR " + new_direction
    else:
        reversal_pending = False
        target_voltage = float(selected_gear)
        bld_brk.value(1)
        bld_en.value(1)
        drive_state = "RAMPING " + direction
        return


def update_traction():
    global throttle_command, target_voltage, direction, gear, drive_state
    global reversal_pending, direction_settle_started_ms
    global last_throttle_ramp_ms

    now = time.ticks_ms()
    if time.ticks_diff(now, last_throttle_ramp_ms) < THROTTLE_RAMP_INTERVAL_MS:
        return
    last_throttle_ramp_ms = now

    if not armed or brake_state != "RELEASED":
        if throttle_command > 0.0:
            traction_stop()
        return

    if reversal_pending:
        if throttle_command > 0.0:
            write_voltage(max(0.0, throttle_command - THROTTLE_RAMP_STEP_V))
            drive_state = "SLOWING FOR " + requested_direction
            return
        bld_en.value(0)
        gear = 0
        if direction_settle_started_ms is None:
            direction_settle_started_ms = now
            drive_state = "DIRECTION PAUSE"
            return
        if time.ticks_diff(now, direction_settle_started_ms) < DIRECTION_SETTLE_MS:
            return
        direction = requested_direction
        bld_fr.value(1 if direction == "FORWARD" else 0)
        reversal_pending = False
        direction_settle_started_ms = None
        target_voltage = float(selected_gear)
        bld_brk.value(1)
        bld_en.value(1)
        drive_state = "RAMPING " + direction
        # Expose one complete zero-voltage scheduler interval after changing
        # F/R before beginning the ramp in the new direction.
        return

    if throttle_command < target_voltage:
        write_voltage(min(target_voltage, throttle_command + THROTTLE_RAMP_STEP_V))
        gear = selected_gear
        drive_state = "RAMPING " + direction
    elif throttle_command > target_voltage:
        write_voltage(max(target_voltage, throttle_command - THROTTLE_RAMP_STEP_V))
        drive_state = "SLOWING"
    elif target_voltage > 0.0:
        gear = selected_gear
        drive_state = "RUNNING " + direction
    else:
        gear = 0
        bld_en.value(0)
        drive_state = "STOPPED"


def brake_encoder_reading():
    return {
        "online": brake_encoder["online"],
        "magnet": brake_encoder["magnet"],
        "angle": brake_encoder["angle"],
        "raw": brake_encoder["raw"],
        "initialized": brake_encoder["initialized"],
        "errors": brake_encoder["errors"],
        "target": brake.target_degrees(),
    }


def apply_brakes():
    global brake_state, brake_transition_started_ms, brake_validation
    traction_stop()
    bld_brk.value(0)
    brake_enable.value(BRAKE_ENABLE_LEVEL)
    brake.set_target(BRAKE_APPLY_DEG)
    brake_state = "APPLYING"
    brake_validation = "MOVING"
    brake_transition_started_ms = time.ticks_ms()


def release_brakes():
    global brake_state, brake_transition_started_ms, brake_validation
    traction_stop()
    bld_brk.value(0)
    brake_enable.value(BRAKE_ENABLE_LEVEL)
    brake.set_target(0.0)
    brake_state = "RELEASING"
    brake_validation = "MOVING"
    brake_transition_started_ms = time.ticks_ms()


def update_brake_state():
    global brake_state, brake_transition_ms, brake_validation
    global brake_validation_error_deg, brake_transition_count
    if not brake.at_target():
        return
    encoder = brake_encoder_reading()
    if brake_state == "APPLYING":
        brake_state = "APPLIED"
    elif brake_state == "RELEASING":
        brake_state = "RELEASED"
        if armed:
            bld_brk.value(1)
    else:
        return
    brake_transition_ms = max(
        0, time.ticks_diff(time.ticks_ms(), brake_transition_started_ms)
    )
    brake_validation_error_deg = abs(encoder["angle"] - encoder["target"])
    brake_validation = (
        "PASS" if encoder["online"] and encoder["magnet"] == "OK"
        and brake_validation_error_deg <= 3.0 else "FAIL"
    )
    brake_transition_count += 1


def steer_hold(command):
    global steer_hold_direction, last_steer_hold_ms
    steer_hold_direction = command
    last_steer_hold_ms = time.ticks_ms()
    if command == "LEFT":
        steering.set_target(-STEER_LIMIT_DEG)
    elif command == "RIGHT":
        steering.set_target(STEER_LIMIT_DEG)
    else:
        steering.hold()
        steer_hold_direction = "STOP"


def update_steer_hold():
    global steer_hold_direction
    if steer_hold_direction == "STOP":
        return
    if time.ticks_diff(time.ticks_ms(), last_steer_hold_ms) > STEER_HOLD_TIMEOUT_MS:
        steering.hold()
        steer_hold_direction = "STOP"


def arm_controller():
    global armed
    if armed:
        return
    armed = True
    release_brakes()


def disarm_controller(reason):
    global armed, steer_hold_direction
    armed = False
    steering.hold()
    steer_hold_direction = "STOP"
    apply_brakes()
    print("DISARM:", reason)


# Capture steering, released brake and BLDC positions as 0 degrees before any
# actuator movement. Failed/missing sensors remain available for later retry.
as5600_update()
as5047_update()
apply_brakes()


HTML = r'''<!doctype html>
<html><head><meta name="viewport" content="width=device-width,initial-scale=1,user-scalable=no">
<title>AutoKart RC</title><style>
:root{--bg:#070a10;--panel:#111a26;--panel2:#0b121c;--line:#26364c;--text:#f6f9ff;--muted:#8e9db2;--blue:#397dff;--cyan:#55d9ff;--green:#19b978;--red:#ed4054;--amber:#ffbd4a}*{box-sizing:border-box}body{margin:0;background:radial-gradient(circle at 50% -15%,#1b2d48 0,var(--bg) 43%);color:var(--text);font-family:system-ui,sans-serif;touch-action:manipulation;overscroll-behavior:none;-webkit-user-select:none;user-select:none}.app{max-width:560px;margin:auto;padding:12px 12px calc(18px + env(safe-area-inset-bottom))}.top{display:flex;align-items:center;justify-content:space-between;gap:12px}.brand h1{font-size:25px;line-height:1;margin:8px 0 5px;letter-spacing:-.045em}.brand small{color:var(--muted)}.state{padding:9px 13px;border:1px solid #74323b;border-radius:999px;background:#39161c;color:#ff99a4;font-size:11px;font-weight:900;letter-spacing:.09em;white-space:nowrap}.state.on{border-color:#267052;background:#103a2a;color:#75e8b2}.state.busy{border-color:#8a6724;background:#3e3014;color:#ffd477}.card{background:linear-gradient(145deg,#151f2d,var(--panel));border:1px solid var(--line);border-radius:19px;padding:14px;margin-top:11px;box-shadow:0 12px 30px #0005}.label{color:var(--muted);font-size:10px;font-weight:900;letter-spacing:.16em;margin-bottom:10px}.row{display:flex;gap:8px}.row>*{flex:1;min-width:0}button{position:relative;border:1px solid #ffffff14;border-radius:14px;min-height:54px;background:#2b394e;color:white;font-size:15px;font-weight:850;cursor:pointer;box-shadow:inset 0 1px #ffffff1a,0 5px 12px #0003;transition:transform .09s,filter .09s,box-shadow .16s;-webkit-tap-highlight-color:transparent}button:active,button.pressed{transform:scale(.95);filter:brightness(.82);box-shadow:inset 0 2px 8px #0005}button:focus-visible{outline:3px solid white;outline-offset:3px}button:disabled{opacity:.34;filter:saturate(.3);cursor:not-allowed;transform:none}.green{background:linear-gradient(145deg,#20c987,#10945f)}.blue{background:linear-gradient(145deg,#438dff,#2867dc)}.red{background:linear-gradient(145deg,#ff4b5f,#cc2c41)}.pad{display:grid;grid-template-columns:repeat(3,82px);grid-template-rows:repeat(3,72px);justify-content:center;gap:9px}.pad button{min-height:72px;border-radius:23px;font-size:29px}.pad .up{grid-column:2;grid-row:1}.pad .left{grid-column:1;grid-row:2}.pad .center{grid-column:2;grid-row:2;border-radius:50%;color:var(--cyan)}.pad .right{grid-column:3;grid-row:2}.pad .down{grid-column:2;grid-row:3}.pad .active{outline:3px solid var(--cyan);outline-offset:2px;box-shadow:0 0 24px #55d9ff55}.gears button{font-size:18px;border-radius:15px}.gears button.active{outline:3px solid var(--cyan);outline-offset:2px}.off,.brake,.stop{width:100%;margin-top:9px}.off{background:#202c3c}.brake{min-height:76px;font-size:20px}.stop{position:sticky;bottom:calc(8px + env(safe-area-inset-bottom));z-index:5;min-height:60px;font-size:17px;box-shadow:0 10px 30px #ed405455}.telemetry{display:grid;grid-template-columns:1fr 1fr;gap:8px}.tile{background:var(--panel2);border:1px solid #213047;padding:10px;border-radius:12px}.tile span{display:block;color:var(--muted);font-size:9px;font-weight:750;letter-spacing:.09em;margin-bottom:4px}.tile b{font-size:13px}.wide{grid-column:1/-1}.pass{color:#73e6ae}.warn{color:#ffd477}.bad{color:#ff929d}.sensors{color:var(--muted);font-size:10px;line-height:1.8;margin-top:10px}.message{position:sticky;bottom:76px;z-index:4;text-align:center;background:#101925eF;border:1px solid var(--line);border-radius:11px;color:#ffd477;font-size:11px;padding:9px;min-height:35px;margin-top:10px;backdrop-filter:blur(8px)}.message.ok{color:#77e8b3}.message.bad{color:#ff929d}.hint{text-align:center;color:var(--muted);font-size:10px;margin-top:8px}@media(max-width:390px){.app{padding:8px 8px calc(14px + env(safe-area-inset-bottom))}.card{padding:12px}.pad{grid-template-columns:repeat(3,72px);grid-template-rows:repeat(3,65px)}.pad button{min-height:65px}.row{gap:5px}.gears button{font-size:16px}.brand h1{font-size:23px}}
</style></head><body><main class="app">
<div class="top"><div class="brand"><h1>AutoKart RC</h1><small>Bench control · wheels raised</small></div><div id="armed" class="state">DISARMED</div></div>
<section class="card"><div class="row"><button id="armBtn" class="green">ARM</button><button id="disarmBtn" class="red">DISARM</button></div><button id="zeroEncodersBtn" class="off">ZERO ENCODERS</button><div class="hint">Keep steering centered and BLDC stopped before zeroing</div></section>
<section class="card"><div class="label">DRIVE + STEERING · HOLD ◀ / ▶ TO MOVE</div><div class="pad"><button id="forward" class="up green" aria-label="Drive forward" title="Drive forward">▲</button><button id="leftBtn" class="left blue" aria-label="Hold to steer left" title="Hold to steer left">◀</button><button id="centerBtn" class="center" aria-label="Center steering" title="Center steering">●</button><button id="rightBtn" class="right blue" aria-label="Hold to steer right" title="Hold to steer right">▶</button><button id="reverse" class="down" aria-label="Drive backward" title="Drive backward">▼</button></div><div class="hint">Drive arrows latch · steering arrows move only while held · center returns to 0°</div></section>
<section class="card"><div class="label">SPEED GEAR · SELECTED <b id="selectedGear">1</b></div><div class="row gears"><button data-gear="1">1</button><button data-gear="2">2</button><button data-gear="3">3</button><button data-gear="4">4</button><button data-gear="5">5</button></div><button id="offBtn" class="off">THROTTLE OFF</button></section>
<section class="card"><button id="brakeBtn" class="red brake">HOLD TO BRAKE</button><div class="hint">Brake press stops traction immediately; release returns brake to 0° while armed</div></section>
<section class="card"><div class="label">LIVE FEEDBACK</div><div class="telemetry"><div class="tile"><span>DRIVE</span><b id="driveState">STOPPED</b></div><div class="tile"><span>OUTPUT / TARGET</span><b><i id="voltage">0.00 V</i> / <i id="targetVoltage">0.00 V</i></b></div><div class="tile"><span>STEERING COMMAND</span><b><i id="steerNow">0.0°</i> → <i id="steerTarget">0.0°</i></b></div><div class="tile"><span>STEERING ENCODER</span><b><i id="steerDirection">CENTER</i> · <i id="steerPosition">0.0°</i></b></div><div class="tile"><span>BLDC / AXLE RPM</span><b><i id="bldcRpm">0</i> / <i id="axleRpm">0</i></b></div><div class="tile"><span>SPEED</span><b id="speed">0.00 km/h</b></div><div class="tile"><span>FULL MOTOR TURNS</span><b id="fullTurns">0</b></div><div class="tile"><span>MOTOR REVOLUTIONS</span><b id="motorRevs">0.00</b></div><div class="tile"><span>DISTANCE TRAVELLED</span><b id="distance">0.000 m</b></div><div class="tile"><span>SIGNED DISPLACEMENT</span><b id="displacement">0.000 m</b></div><div class="tile wide"><span>BRAKE ENCODER</span><b><i id="brakeState">APPLIED</i> · <i id="brakeAngle">45.0°</i> → <i id="brakeTarget">45.0°</i></b><div class="sensors">Magnet <b id="brakeMagnet" class="pass">OK</b> · validation <b id="brakeValidation">MOVING</b> · error <b id="brakeError">0.00°</b> · transition <b id="brakeTime">0 ms</b></div></div></div><div class="sensors">Encoder zero: <b id="encoderZeroState">INITIALIZING</b><br>Steering AS5600: <b id="steerDetail">checking</b> · errors <b id="steerErrors">0</b><br>BLDC AS5047P: <b id="bldcEnc">checking</b> · raw/errors <b id="bldcDetail">--</b><br>GP8630N: <b id="gpState">checking</b></div></section>
<div id="message" class="message" role="status" aria-live="polite">Connecting…</div><button id="emergencyBtn" class="red stop">EMERGENCY STOP</button>
</main><script>
const $=id=>document.getElementById(id);let latest=null,brakeHeld=false,steerHeld=null,steerTimer=null;
function feedback(el){if(el){el.classList.add('pressed');setTimeout(()=>el.classList.remove('pressed'),120)}if(navigator.vibrate)navigator.vibrate(10)}
function note(text,kind=''){const m=$('message');m.textContent=text;m.className='message '+kind}
async function send(path,quiet=false){if(!quiet)note('COMMANDING…');try{const r=await fetch(path,{cache:'no-store'}),t=await r.text();if(!quiet)note(t,r.ok?'ok':'bad');return r.ok}catch(e){note('CONTROLLER OFFLINE','bad');return false}}
function command(path,el){feedback(el);return send(path)}
function drive(dir,el){command('/drive?d='+dir,el)}
function gear(n,el){command('/gear?n='+n,el)}
function steerStart(e,dir){e.preventDefault();const el=e.currentTarget;if(el.disabled||steerHeld)return;if(e.pointerId!==undefined&&el.setPointerCapture)try{el.setPointerCapture(e.pointerId)}catch(_){}steerHeld=dir;el.classList.add('pressed');feedback();send('/steer_hold?d='+dir);steerTimer=setInterval(()=>send('/steer_hold?d='+dir,true),180)}
function steerStop(e){if(!steerHeld)return;if(e)e.preventDefault();clearInterval(steerTimer);steerTimer=null;const id=steerHeld==='LEFT'?'leftBtn':'rightBtn';$(id).classList.remove('pressed');steerHeld=null;send('/steer_hold?d=STOP')}
function brakeDown(e){e.preventDefault();const b=$('brakeBtn');if(b.disabled||brakeHeld)return;if(e.pointerId!==undefined&&b.setPointerCapture)try{b.setPointerCapture(e.pointerId)}catch(_){}brakeHeld=true;b.classList.add('pressed');feedback();send('/brake?s=apply')}
function brakeUp(e){if(!brakeHeld)return;if(e)e.preventDefault();brakeHeld=false;$('brakeBtn').classList.remove('pressed');send('/brake?s=release')}
$('armBtn').onclick=e=>command('/arm',e.currentTarget);$('disarmBtn').onclick=e=>command('/disarm',e.currentTarget);$('zeroEncodersBtn').onclick=e=>command('/zero_encoders',e.currentTarget);$('emergencyBtn').onclick=e=>command('/disarm',e.currentTarget);$('forward').onclick=e=>drive('FORWARD',e.currentTarget);$('reverse').onclick=e=>drive('REVERSE',e.currentTarget);$('centerBtn').onclick=e=>command('/steer?v=0',e.currentTarget);$('offBtn').onclick=e=>gear(0,e.currentTarget);document.querySelectorAll('[data-gear]').forEach(b=>b.onclick=()=>gear(Number(b.dataset.gear),b));['leftBtn','rightBtn'].forEach(id=>{const b=$(id),dir=id==='leftBtn'?'LEFT':'RIGHT';b.addEventListener('pointerdown',e=>steerStart(e,dir));['pointerup','pointercancel','lostpointercapture'].forEach(x=>b.addEventListener(x,steerStop))});$('brakeBtn').addEventListener('pointerdown',brakeDown);['pointerup','pointercancel','lostpointercapture'].forEach(x=>$('brakeBtn').addEventListener(x,brakeUp));addEventListener('blur',()=>{steerStop();brakeUp()});addEventListener('pagehide',()=>navigator.sendBeacon('/disarm'));
addEventListener('keydown',e=>{if(e.repeat)return;if(e.key==='ArrowLeft')steerStart({preventDefault(){},currentTarget:$('leftBtn'),pointerId:0},'LEFT');else if(e.key==='ArrowRight')steerStart({preventDefault(){},currentTarget:$('rightBtn'),pointerId:0},'RIGHT');else if(e.key==='ArrowUp')drive('FORWARD',$('forward'));else if(e.key==='ArrowDown')drive('REVERSE',$('reverse'));else if(e.code==='Space')brakeDown({preventDefault(){},pointerId:0})});addEventListener('keyup',e=>{if(e.key==='ArrowLeft'||e.key==='ArrowRight')steerStop();if(e.code==='Space')brakeUp()});
async function update(){try{const r=await fetch('/status',{cache:'no-store'});if(!r.ok)throw Error();const s=await r.json();latest=s;const busy=s.brake_state==='APPLYING'||s.brake_state==='RELEASING';$('armed').textContent=s.armed?(busy?'ARMING':'ARMED'):'DISARMED';$('armed').classList.toggle('on',s.armed&&!busy);$('armed').classList.toggle('busy',busy);$('armBtn').disabled=s.armed;$('disarmBtn').disabled=!s.armed&&!busy;$('zeroEncodersBtn').disabled=s.armed||s.voltage>.01;const driveDisabled=!s.armed||s.brake_state!=='RELEASED'||!s.gp_online;$('forward').disabled=driveDisabled;$('reverse').disabled=driveDisabled;$('leftBtn').disabled=!s.armed;$('centerBtn').disabled=!s.armed;$('rightBtn').disabled=!s.armed;$('brakeBtn').disabled=!s.armed;$('brakeBtn').textContent=s.armed?'HOLD TO BRAKE':'BRAKE APPLIED';$('forward').classList.toggle('active',s.direction==='FORWARD'&&s.drive_state!=='STOPPED');$('reverse').classList.toggle('active',s.direction==='REVERSE'&&s.drive_state!=='STOPPED');$('leftBtn').classList.toggle('active',s.steer_hold==='LEFT');$('rightBtn').classList.toggle('active',s.steer_hold==='RIGHT');$('centerBtn').classList.toggle('active',Math.abs(s.steer_target)<=.5&&s.steer_hold==='STOP');document.querySelectorAll('[data-gear]').forEach(b=>b.classList.toggle('active',s.selected_gear===Number(b.dataset.gear)));$('selectedGear').textContent=s.selected_gear;$('voltage').textContent=s.voltage.toFixed(2)+' V';$('targetVoltage').textContent=s.target_voltage.toFixed(2)+' V';$('driveState').textContent=s.drive_state;$('steerNow').textContent=s.steer_command.toFixed(1)+'°';$('steerTarget').textContent=s.steer_target.toFixed(1)+'°';$('steerDirection').textContent=s.steer_direction;$('steerPosition').textContent=(s.steer_angle>0?'+':'')+s.steer_angle.toFixed(1)+'°';$('brakeState').textContent=s.brake_state;$('brakeAngle').textContent=s.brake_encoder_angle.toFixed(1)+'°';$('brakeTarget').textContent=s.brake_encoder_target.toFixed(1)+'°';$('brakeMagnet').textContent=s.brake_encoder_magnet;$('brakeValidation').textContent=s.brake_validation;$('brakeValidation').className=s.brake_validation==='PASS'?'pass':s.brake_validation==='FAIL'?'bad':'warn';$('brakeError').textContent=s.brake_error.toFixed(2)+'°';$('brakeTime').textContent=s.brake_transition_ms+' ms';$('bldcRpm').textContent=Math.abs(s.bldc_rpm).toFixed(0);$('axleRpm').textContent=Math.abs(s.axle_rpm).toFixed(0);$('speed').textContent=s.speed_kmh.toFixed(2)+' km/h';$('fullTurns').textContent=s.full_turns;$('motorRevs').textContent=s.motor_revolutions.toFixed(2);$('distance').textContent=s.distance_m.toFixed(3)+' m';$('displacement').textContent=s.displacement_m.toFixed(3)+' m';$('encoderZeroState').textContent=s.encoders_initialized?'READY':'WAITING FOR VALID MAGNETS';$('encoderZeroState').className=s.encoders_initialized?'pass':'warn';$('steerDetail').textContent=s.steer_online?s.steer_angle.toFixed(1)+'° '+s.steer_direction+' / '+s.steer_magnet:s.steer_magnet;$('steerErrors').textContent=s.steer_errors;$('bldcEnc').textContent=s.bldc_online?s.bldc_angle.toFixed(1)+'° / '+s.bldc_magnet:s.bldc_magnet;$('bldcDetail').textContent=s.bldc_raw+'/'+s.bldc_errors;$('gpState').textContent=s.gp_online?'ONLINE':'OFFLINE'}catch(e){note('CONTROLLER OFFLINE','bad')}}setInterval(update,200);update();
</script></body></html>'''


def query(path, key, default):
    try:
        return path.split(key + "=", 1)[1].split("&", 1)[0]
    except Exception:
        return default


def send_all(client, data):
    view = memoryview(data)
    while view:
        sent = client.send(view)
        if not sent:
            return
        view = view[sent:]


def reply(client, body, status="200 OK", content_type="text/plain"):
    if isinstance(body, str):
        body = body.encode()
    header = (
        "HTTP/1.1 " + status + "\r\nContent-Type: " + content_type
        + "\r\nContent-Length: " + str(len(body))
        + "\r\nCache-Control: no-store\r\nConnection: close\r\n\r\n"
    ).encode()
    send_all(client, header)
    send_all(client, body)


def json_status():
    brake_encoder = brake_encoder_reading()
    encoders_initialized = (
        steer_encoder["initialized"] and brake_encoder["initialized"]
        and bldc_encoder["initialized"]
    )
    return (
        '{{"armed":{},"direction":"{}","requested_direction":"{}",'
        '"drive_state":"{}","gear":{},"selected_gear":{},'
        '"voltage":{:.2f},"target_voltage":{:.2f},'
        '"steer_command":{:.2f},"steer_target":{:.2f},"steer_hold":"{}",'
        '"steer_angle":{:.2f},"steer_raw":{},'
        '"steer_direction":"{}","steer_initialized":{},'
        '"steer_errors":{},"steer_online":{},"steer_magnet":"{}",'
        '"brake_state":"{}","brake_angle":{:.2f},'
        '"brake_encoder_online":{},"brake_encoder_magnet":"{}",'
        '"brake_encoder_initialized":{},"brake_encoder_errors":{},'
        '"brake_encoder_angle":{:.2f},"brake_encoder_raw":{},'
        '"brake_encoder_target":{:.2f},"brake_validation":"{}",'
        '"brake_error":{:.3f},"brake_transition_ms":{},'
        '"brake_transition_count":{},'
        '"bldc_angle":{:.2f},"bldc_raw":{},"bldc_errors":{},'
        '"bldc_online":{},"bldc_magnet":"{}","bldc_initialized":{},'
        '"bldc_rpm":{:.2f},"axle_rpm":{:.2f},'
        '"motor_revolutions":{:.4f},"full_turns":{},'
        '"distance_m":{:.4f},"displacement_m":{:.4f},'
        '"speed_kmh":{:.3f},"encoders_initialized":{},"gp_online":{}}}'
    ).format(
        str(armed).lower(), direction, requested_direction, drive_state,
        gear, selected_gear, throttle_command, target_voltage,
        steering.degrees(), steering.target_degrees(), steer_hold_direction,
        steer_encoder["angle"],
        steer_encoder["raw"], steer_encoder["direction"],
        str(steer_encoder["initialized"]).lower(), steer_encoder["errors"],
        str(steer_encoder["online"]).lower(), steer_encoder["magnet"],
        brake_state, brake.degrees(),
        str(brake_encoder["online"]).lower(), brake_encoder["magnet"],
        str(brake_encoder["initialized"]).lower(), brake_encoder["errors"],
        brake_encoder["angle"], brake_encoder["raw"],
        brake_encoder["target"], brake_validation,
        brake_validation_error_deg, brake_transition_ms,
        brake_transition_count, bldc_encoder["angle"],
        bldc_encoder["raw"], bldc_encoder["errors"],
        str(bldc_encoder["online"]).lower(), bldc_encoder["magnet"],
        str(bldc_encoder["initialized"]).lower(), bldc_encoder["rpm"],
        bldc_encoder["axle_rpm"], bldc_encoder["motor_revs"],
        bldc_encoder["full_turns"], bldc_encoder["distance_m"],
        bldc_encoder["displacement_m"], bldc_encoder["speed_kmh"],
        str(encoders_initialized).lower(),
        str(gp_present).lower(),
    )


ap = network.WLAN(network.AP_IF)
ap.active(True)
ap.config(essid=SSID, password=PASSWORD)
while not ap.active():
    time.sleep_ms(100)

server = socket.socket()
server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
server.bind(("0.0.0.0", 80))
server.listen(3)
server.settimeout(0)

print("AUTOKART FULL RC READY")
print("Wi-Fi:", SSID, "Password:", PASSWORD)
print("Open: http://" + ap.ifconfig()[0])
print("PCA I2C(0) SDA1/SCL2:", [hex(item) for item in pca_i2c_devices])
print("GP I2C(1) SDA7/SCL21:", [hex(item) for item in gp_i2c_devices])
print("Encoder zeroing: automatic on first valid readings")

try:
    while True:
        steering.update()
        update_steer_hold()
        brake.update()
        update_brake_state()
        update_traction()
        update_encoders()

        if armed and time.ticks_diff(time.ticks_ms(), last_contact_ms) > WATCHDOG_MS:
            disarm_controller("WEB WATCHDOG")

        try:
            client, _ = server.accept()
        except OSError:
            time.sleep_us(100)
            continue

        try:
            request = client.recv(1200).decode("utf-8", "ignore")
            first = request.split("\r\n", 1)[0].split(" ")
            path = first[1] if len(first) > 1 else "/bad"

            if path == "/":
                reply(client, HTML, content_type="text/html")
            elif path.startswith("/status"):
                if armed:
                    last_contact_ms = time.ticks_ms()
                reply(client, json_status(), content_type="application/json")
            elif path.startswith("/arm"):
                arm_controller()
                last_contact_ms = time.ticks_ms()
                reply(client, "ARMED; BRAKES RELEASING")
            elif path.startswith("/disarm"):
                disarm_controller("REMOTE")
                reply(client, "DISARMED; BRAKES APPLYING")
            elif path.startswith("/zero_encoders"):
                if armed or throttle_command > 0.01:
                    reply(client, "DISARM AND STOP BEFORE ZEROING", "409 Conflict")
                elif zero_encoders():
                    reply(client, "ENCODERS ZEROED: STEERING 0 DEG / BLDC 0 DEG")
                else:
                    reply(client, "WAITING FOR VALID ENCODER MAGNETS", "409 Conflict")
            elif path.startswith("/steer_hold"):
                requested = query(path, "d", "STOP")
                if not armed:
                    reply(client, "ARM FIRST", "409 Conflict")
                elif requested not in ("LEFT", "RIGHT", "STOP"):
                    reply(client, "BAD STEERING HOLD", "400 Bad Request")
                else:
                    steer_hold(requested)
                    last_contact_ms = time.ticks_ms()
                    reply(client, "STEERING " + requested)
            elif path.startswith("/steer"):
                if not armed:
                    reply(client, "ARM FIRST", "409 Conflict")
                else:
                    requested = clamp(
                        float(query(path, "v", "0")),
                        -STEER_LIMIT_DEG, STEER_LIMIT_DEG,
                    )
                    steer_hold_direction = "STOP"
                    steering.set_target(requested)
                    last_contact_ms = time.ticks_ms()
                    reply(client, "STEERING CENTER" if requested == 0 else
                          "STEERING {:.0f} DEG".format(requested))
            elif path.startswith("/drive") or path.startswith("/direction"):
                requested = query(path, "d", "FORWARD")
                if not armed:
                    reply(client, "ARM FIRST", "409 Conflict")
                elif brake_state != "RELEASED":
                    reply(client, "WAIT FOR BRAKE RELEASE", "409 Conflict")
                elif not gp_present:
                    reply(client, "GP8630N OFFLINE", "409 Conflict")
                elif requested not in ("FORWARD", "REVERSE"):
                    reply(client, "BAD DIRECTION", "400 Bad Request")
                else:
                    request_drive(requested)
                    last_contact_ms = time.ticks_ms()
                    if requested == direction:
                        reply(client, requested + " RAMPING")
                    else:
                        reply(client, "SLOWING THEN " + requested)
            elif path.startswith("/gear"):
                requested = int(query(path, "n", "0"))
                if requested < 0 or requested > 5:
                    reply(client, "GEAR MUST BE 0..5", "400 Bad Request")
                elif requested == 0:
                    request_soft_stop()
                    last_contact_ms = time.ticks_ms()
                    reply(client, "THROTTLE RAMPING TO ZERO")
                else:
                    selected_gear = requested
                    if target_voltage > 0.0 and not reversal_pending:
                        target_voltage = float(selected_gear)
                    last_contact_ms = time.ticks_ms()
                    reply(client, "GEAR {} SELECTED / {:.1f} V".format(
                        requested, float(requested)
                    ))
            elif path.startswith("/brake"):
                requested = query(path, "s", "apply")
                if requested == "release":
                    if armed:
                        release_brakes()
                        last_contact_ms = time.ticks_ms()
                        reply(client, "BRAKES RELEASING TO 0 DEG")
                    else:
                        reply(client, "DISARMED: BRAKE REMAINS APPLIED", "409 Conflict")
                else:
                    apply_brakes()
                    if armed:
                        last_contact_ms = time.ticks_ms()
                    reply(client, "BRAKES APPLYING TO 45 DEG")
            else:
                reply(client, "NOT FOUND", "404 Not Found")
        except Exception as error:
            print("HTTP ERROR:", error)
            try:
                reply(client, "CONTROLLER ERROR", "500 Internal Server Error")
            except Exception:
                pass
        finally:
            try:
                client.close()
            except Exception:
                pass
finally:
    disarm_controller("CONTROLLER EXIT")
    brake_enable.value(BRAKE_DISABLE_LEVEL)
    server.close()
