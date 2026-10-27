"""Isolated ESP32-S3 DM860 + AS5600 closed-loop steering test.

Only these GPIOs are initialized:
  AS5600 SDA GPIO1, SCL GPIO2
  DM860 STEP GPIO4, DIR GPIO5

GPIO6/ENA is deliberately not initialized. Leave DM860 ENA disconnected for
this test unless its interface polarity has been validated separately.

Serial commands at 115200:
  STATUS
  ZERO
  ARM
  TARGET -25 | TARGET 0 | TARGET 25
  DISARM

This is wheels-off-ground commissioning firmware, not production firmware.
"""

from machine import I2C, Pin
import select
import sys
import time


SDA_GPIO = 1
SCL_GPIO = 2
STEP_GPIO = 4
DIR_GPIO = 5

AS5600_ADDRESS = 0x36
AS5600_STATUS = 0x0B
AS5600_ANGLE = 0x0E
AS5600_COUNTS = 4096

# Measured vehicle convention: AS5600 raw increases when moving physically
# left, while road-wheel right is positive.
ENCODER_DIRECTION_SIGN = -1
# Existing tested DM860/NPN steering mapping: GPIO LOW moves physical right.
RIGHT_DIR_GPIO_LEVEL = 0

TARGET_LIMIT_DEG = 25.0
TARGET_TOLERANCE_DEG = 0.40
HOLD_REENGAGE_DEG = 0.90
HOLD_REENGAGE_DELAY_MS = 180
START_PPS = 900
MAX_PPS = 3500
NEAR_PPS = 450
MID_PPS = 850
APPROACH_PPS = 1400
STEP_HIGH_US = 20
SENSOR_INTERVAL_MS = 8
STATUS_INTERVAL_MS = 200
MOVE_TIMEOUT_MS = 6000
NO_PROGRESS_TIMEOUT_MS = 700
WRONG_DIRECTION_MARGIN_DEG = 2.0
EXTRA_TRAVEL_LIMIT_DEG = 8.0


def clamp(value, low, high):
    return max(low, min(high, value))


def wrapped_delta(current, previous, counts):
    half = counts // 2
    return ((int(current) - int(previous) + half) % counts) - half


step = Pin(STEP_GPIO, Pin.OUT, value=0)
direction = Pin(DIR_GPIO, Pin.OUT, value=RIGHT_DIR_GPIO_LEVEL)
i2c = I2C(0, sda=Pin(SDA_GPIO), scl=Pin(SCL_GPIO), freq=100_000)
poll = select.poll()
poll.register(sys.stdin, select.POLLIN)

armed = False
zero_raw = None
angle_deg = 0.0
raw_angle = -1
magnet = "UNKNOWN"
sensor_valid = False
target_deg = 0.0
moving = False
fault = ""
move_started_ms = 0
move_start_angle = 0.0
move_start_error = 0.0
last_progress_ms = 0
best_error = 999.0
last_sensor_ms = 0
last_status_ms = 0
last_step_us = time.ticks_us()
step_period_us = 1_000_000 // START_PPS
input_buffer = ""
hold_error_started_ms = None


def read_sensor():
    global raw_angle, angle_deg, magnet, sensor_valid
    try:
        status = i2c.readfrom_mem(AS5600_ADDRESS, AS5600_STATUS, 1)[0]
        data = i2c.readfrom_mem(AS5600_ADDRESS, AS5600_ANGLE, 2)
        raw_angle = ((data[0] << 8) | data[1]) & 0x0FFF
        if status & 0x08:
            magnet = "TOO_STRONG"
        elif status & 0x10:
            magnet = "TOO_WEAK"
        elif status & 0x20:
            magnet = "OK"
        else:
            magnet = "NO_MAGNET"
        sensor_valid = magnet == "OK"
        if zero_raw is not None:
            angle_deg = (
                wrapped_delta(raw_angle, zero_raw, AS5600_COUNTS)
                * 360.0 / AS5600_COUNTS * ENCODER_DIRECTION_SIGN
            )
            if abs(angle_deg) < 0.05:
                angle_deg = 0.0
    except Exception as error:
        sensor_valid = False
        magnet = "READ_ERROR"
        return repr(error)
    return ""


def stop_motion(reason=""):
    global moving, fault, step_period_us
    moving = False
    step.value(0)
    step_period_us = 1_000_000 // START_PPS
    if reason:
        fault = reason


def update_hold():
    global hold_error_started_ms
    if moving or not armed or zero_raw is None or not sensor_valid or fault:
        hold_error_started_ms = None
        return
    if abs(target_deg - angle_deg) <= HOLD_REENGAGE_DEG:
        hold_error_started_ms = None
        return
    now = time.ticks_ms()
    if hold_error_started_ms is None:
        hold_error_started_ms = now
    elif time.ticks_diff(now, hold_error_started_ms) >= HOLD_REENGAGE_DELAY_MS:
        hold_error_started_ms = None
        command_target(target_deg, automatic=True)


def emit_status(prefix="STATUS"):
    print(
        "@STEER,{},{},{:.2f},{:.2f},{},{},{},{}".format(
            prefix,
            1 if armed else 0,
            angle_deg,
            target_deg,
            raw_angle,
            magnet,
            "MOVING" if moving else "STOPPED",
            fault or "NONE",
        )
    )


def command_target(requested, automatic=False):
    global target_deg, moving, fault, move_started_ms, move_start_angle
    global move_start_error, last_progress_ms, best_error, step_period_us
    if not armed:
        print("@STEER,NAK,NOT_ARMED")
        return
    if zero_raw is None:
        print("@STEER,NAK,NOT_ZEROED")
        return
    if not sensor_valid:
        print("@STEER,NAK,SENSOR_INVALID")
        return
    requested = clamp(float(requested), -TARGET_LIMIT_DEG, TARGET_LIMIT_DEG)
    target_deg = requested
    fault = ""
    error = target_deg - angle_deg
    if abs(error) <= TARGET_TOLERANCE_DEG:
        stop_motion()
        emit_status("TARGET_REACHED")
        return
    now = time.ticks_ms()
    moving = True
    move_started_ms = now
    last_progress_ms = now
    move_start_angle = angle_deg
    move_start_error = abs(error)
    best_error = abs(error)
    step_period_us = 1_000_000 // START_PPS
    direction.value(RIGHT_DIR_GPIO_LEVEL if error > 0 else 1 - RIGHT_DIR_GPIO_LEVEL)
    print(
        "@STEER,ACK,{},TARGET,{:.2f}".format(
            "HOLD_CORRECTION" if automatic else "COMMAND", target_deg
        )
    )


def handle_command(line):
    global armed, zero_raw, angle_deg, target_deg, fault
    parts = line.strip().upper().split()
    if not parts:
        return
    command = parts[0]
    if command == "STATUS":
        emit_status()
    elif command == "ZERO":
        stop_motion()
        read_sensor()
        if not sensor_valid:
            print("@STEER,NAK,ZERO,SENSOR_{}".format(magnet))
            return
        zero_raw = raw_angle
        angle_deg = 0.0
        target_deg = 0.0
        fault = ""
        print("@STEER,ACK,ZERO,{}".format(zero_raw))
    elif command == "ARM":
        if zero_raw is None or not sensor_valid:
            print("@STEER,NAK,ARM,ZERO_OR_SENSOR")
            return
        armed = True
        fault = ""
        print("@STEER,ACK,ARM")
    elif command == "TARGET" and len(parts) == 2:
        try:
            command_target(float(parts[1]))
        except ValueError:
            print("@STEER,NAK,BAD_TARGET")
    elif command in ("STOP", "DISARM"):
        stop_motion("OPERATOR_STOP" if command == "STOP" else "")
        armed = False
        print("@STEER,ACK,{}".format(command))
    else:
        print("@STEER,NAK,UNKNOWN_COMMAND")


def drain_input():
    global input_buffer
    while poll.poll(0):
        character = sys.stdin.read(1)
        if not character:
            break
        if character in "\r\n":
            if input_buffer:
                handle_command(input_buffer)
                input_buffer = ""
        elif len(input_buffer) < 80:
            input_buffer += character


def update_motion():
    global best_error, last_progress_ms, last_step_us, step_period_us
    if not moving:
        return
    if not armed:
        stop_motion("DISARMED")
        return
    if not sensor_valid:
        stop_motion("SENSOR_INVALID")
        return

    now_ms = time.ticks_ms()
    error = target_deg - angle_deg
    absolute_error = abs(error)
    if absolute_error <= TARGET_TOLERANCE_DEG:
        stop_motion()
        emit_status("TARGET_REACHED")
        return
    if time.ticks_diff(now_ms, move_started_ms) > MOVE_TIMEOUT_MS:
        stop_motion("MOVE_TIMEOUT")
        emit_status("FAULT")
        return
    expected_motion = target_deg - move_start_angle
    measured_motion = angle_deg - move_start_angle
    if (
        abs(measured_motion) > 1.0
        and measured_motion * expected_motion < 0.0
    ):
        stop_motion("WRONG_DIRECTION")
        emit_status("FAULT")
        return
    if abs(angle_deg - move_start_angle) > abs(target_deg - move_start_angle) + EXTRA_TRAVEL_LIMIT_DEG:
        stop_motion("EXCESS_TRAVEL")
        emit_status("FAULT")
        return
    if absolute_error + 0.15 < best_error:
        best_error = absolute_error
        last_progress_ms = now_ms
    elif time.ticks_diff(now_ms, last_progress_ms) > NO_PROGRESS_TIMEOUT_MS:
        reason = "WRONG_DIRECTION" if absolute_error > move_start_error + WRONG_DIRECTION_MARGIN_DEG else "NO_PROGRESS"
        stop_motion(reason)
        emit_status("FAULT")
        return

    desired_direction = RIGHT_DIR_GPIO_LEVEL if error > 0 else 1 - RIGHT_DIR_GPIO_LEVEL
    direction.value(desired_direction)
    now_us = time.ticks_us()
    if time.ticks_diff(now_us, last_step_us) >= step_period_us:
        step.value(1)
        time.sleep_us(STEP_HIGH_US)
        step.value(0)
        last_step_us = now_us
        if absolute_error <= 2.0:
            demanded_period = 1_000_000 // NEAR_PPS
        elif absolute_error <= 5.0:
            demanded_period = 1_000_000 // MID_PPS
        elif absolute_error <= 10.0:
            demanded_period = 1_000_000 // APPROACH_PPS
        else:
            demanded_period = max(
                1_000_000 // MAX_PPS, step_period_us - 4
            )
        step_period_us = demanded_period


print("@STEER,BOOT,ISOLATED_CLOSED_LOOP_V1")
print("@STEER,PINS,SDA1,SCL2,STEP4,DIR5,ENA_UNUSED")
print("@STEER,SAFE,DISARMED")

while True:
    drain_input()
    now = time.ticks_ms()
    if time.ticks_diff(now, last_sensor_ms) >= SENSOR_INTERVAL_MS:
        last_sensor_ms = now
        read_sensor()
    update_motion()
    update_hold()
    # Emit periodic telemetry only during motion. When the operator is paused
    # at a host prompt, continuous stopped-state output can fill the USB serial
    # buffer and delay reception of the next safety/control command.
    if moving and time.ticks_diff(now, last_status_ms) >= STATUS_INTERVAL_MS:
        last_status_ms = now
        emit_status()
    time.sleep_us(100)
