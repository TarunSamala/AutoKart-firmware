"""ESP32-S3 read-only VCU odometry telemetry for laptop commissioning.

Sensors:
  steering AS5600: SDA GPIO1, SCL GPIO2, I2C address 0x36
  BLDC AS5047P: CS16, SCK18, MOSI41, MISO42, SPI mode 1

The program deliberately does not initialize GP8630N, BLD750, steering,
brake, LoRa, or any actuator output. Incoming USB data is drained and ignored.
It emits CRC16-framed HELLO, CONTROL_FEEDBACK, and MOTION messages for the ROS
vcu_bridge. Steering zero is captured from the first healthy AS5600 sample on
each boot, so mechanically centre steering before reset.

This is commissioning firmware, not production vehicle firmware. The
calibration identity remains explicitly UNVERIFIED and must not be used to
authorize autonomous motion.
"""

from machine import I2C, Pin, SPI, unique_id
import select
import sys
import time


# Frozen encoder pinout.
STEERING_SDA_GPIO = 1
STEERING_SCL_GPIO = 2
AS5047_CS_GPIO = 16
AS5047_SCK_GPIO = 18
AS5047_MOSI_GPIO = 41
AS5047_MISO_GPIO = 42

AS5600_ADDRESS = 0x36
AS5600_STATUS = 0x0B
AS5600_ANGLE = 0x0E
AS5600_MAGNET_DETECTED = 0x20
AS5600_MAGNET_LOW = 0x10
AS5600_MAGNET_HIGH = 0x08

AS5047_ERRFL = 0x0001
AS5047_DIAAGC = 0x3FFC
AS5047_ANGLECOM = 0x3FFF
AS5047_MAGL = 1 << 11
AS5047_MAGH = 1 << 10
AS5047_COF = 1 << 9
AS5047_LF = 1 << 8

AS5600_COUNTS = 4096
AS5047_COUNTS = 16384
# Vehicle convention used by ROS: right road-wheel angle is positive. The
# mounted encoder test measured left as increasing raw angle, hence -1.
STEERING_DIRECTION_SIGN = -1
# Measured stationary raw variation in the first laptop capture was 12 counts.
# A 24-count anchor window is about 0.53 motor degrees / 0.4 mm nominal tyre
# travel after the 14T:44T reduction, so it rejects observed jitter without
# materially hiding kart movement.
POSITION_DEADBAND_COUNTS = 24
MAXIMUM_MOTOR_RPM = 6000.0
# Physical calibration on 26 October 2026: one forward (CCW when viewed from
# the encoder/magnet side) 14T-sprocket revolution produced -16187 raw counts;
# reverse produced +16420. Vehicle odometry convention is forward-positive.
ODOMETRY_DIRECTION_SIGN = -1
STEERING_ZERO_SAMPLES = 50
STEERING_ZERO_MAX_SPREAD_COUNTS = 24

VALID_ENCODER = 1 << 0
VALID_STEERING = 1 << 1
FAULT_STEER_SENSOR = 1 << 4
FAULT_ENCODER_SENSOR = 1 << 11

STATE_SAFE = 2
FIRMWARE_ID = "esp32s3-odom-telemetry-0.1.0"
PROTOCOL_VERSION = 1
CAPABILITY_BITS = VALID_ENCODER | VALID_STEERING
CALIBRATION_ID = "UNVERIFIED-LAPTOP-V1"

SAMPLE_INTERVAL_MS = 10       # sensor acquisition: 100 Hz
MOTION_INTERVAL_MS = 20       # MOTION telemetry: 50 Hz
CONTROL_INTERVAL_MS = 50      # CONTROL_FEEDBACK telemetry: 20 Hz
HELLO_INTERVAL_MS = 1000


def crc16_ccitt_false(data):
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc


def send_frame(frame_type, fields):
    body = frame_type
    if fields:
        body += "," + ",".join(str(field) for field in fields)
    checksum = crc16_ccitt_false(body.encode())
    sys.stdout.write("${}*{:04X}\n".format(body, checksum))


def wrapped_delta(current, previous, counts_per_turn):
    half = counts_per_turn // 2
    return ((int(current) - int(previous) + half) % counts_per_turn) - half


def parity_is_odd(value):
    return bool(bin(value & 0xFFFF).count("1") & 1)


def as5047_command(register):
    frame = 0x4000 | (register & 0x3FFF)
    if parity_is_odd(frame):
        frame |= 0x8000
    return frame


steering_i2c = I2C(
    0,
    sda=Pin(STEERING_SDA_GPIO),
    scl=Pin(STEERING_SCL_GPIO),
    freq=100_000,
)
spi = SPI(
    1,
    baudrate=1_000_000,
    polarity=0,
    phase=1,
    bits=8,
    firstbit=SPI.MSB,
    sck=Pin(AS5047_SCK_GPIO),
    mosi=Pin(AS5047_MOSI_GPIO),
    miso=Pin(AS5047_MISO_GPIO),
)
as5047_cs = Pin(AS5047_CS_GPIO, Pin.OUT, value=1)


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
    return (
        response & 0x3FFF,
        bool(response & 0x4000),
        parity_is_odd(response),
    )


def read_as5047():
    angle, angle_error, angle_parity = as5047_register(AS5047_ANGLECOM)
    diaagc, diag_error, diag_parity = as5047_register(AS5047_DIAAGC)
    errors, err_error, err_parity = as5047_register(AS5047_ERRFL)
    communication_ok = not (
        angle_error or angle_parity or diag_error or diag_parity
        or err_error or err_parity
    )
    magnet_ok = bool(diaagc & AS5047_LF) and not bool(
        diaagc & (AS5047_MAGL | AS5047_MAGH | AS5047_COF)
    )
    return angle, communication_ok and magnet_ok and errors == 0


def read_steering():
    status = steering_i2c.readfrom_mem(AS5600_ADDRESS, AS5600_STATUS, 1)[0]
    data = steering_i2c.readfrom_mem(AS5600_ADDRESS, AS5600_ANGLE, 2)
    raw = ((data[0] << 8) | data[1]) & 0x0FFF
    valid = bool(status & AS5600_MAGNET_DETECTED) and not bool(
        status & (AS5600_MAGNET_LOW | AS5600_MAGNET_HIGH)
    )
    return raw, valid


uid = unique_id()
uid_part = 0
for byte in uid[:6]:
    uid_part = (uid_part << 8) | byte


def next_boot_counter():
    """Return a persistent monotonic boot counter."""
    try:
        import esp32
        nvs = esp32.NVS("autokart")
        try:
            counter = nvs.get_i32("boot_count")
        except OSError:
            counter = 0
        counter = (counter + 1) & 0x7FFFFFFF
        if counter == 0:
            counter = 1
        nvs.set_i32("boot_count", counter)
        nvs.commit()
        return counter
    except Exception:
        # Compatibility fallback for a MicroPython build without esp32.NVS.
        path = ".vcu_boot_count"
        try:
            with open(path, "r") as source:
                counter = int(source.read().strip())
        except Exception:
            counter = 0
        counter = (counter + 1) & 0x7FFFFFFF
        if counter == 0:
            counter = 1
        with open(path, "w") as destination:
            destination.write(str(counter))
        return counter


boot_counter = next_boot_counter()
# Device-specific high bits plus a persistent 31-bit epoch counter. Startup
# timing alone is insufficient because it can repeat after a physical reset.
boot_id = (((uid_part & 0xFFFFFFFF) << 31) | boot_counter) & 0x7FFFFFFFFFFFFFFF

sequence = 0
last_tick_us = time.ticks_us()
monotonic_time_us = 0

last_encoder_raw = None
encoder_anchor_raw = None
encoder_count = 0
encoder_delta = 0
encoder_valid = False
last_encoder_sample_us = None

steering_zero_raw = None
steering_zero_candidates = []
steering_cdeg = 0
steering_valid = False

last_motion_ms = time.ticks_ms()
last_control_ms = last_motion_ms
last_hello_ms = time.ticks_add(last_motion_ms, -HELLO_INTERVAL_MS)

stdin_poll = select.poll()
stdin_poll.register(sys.stdin, select.POLLIN)


def update_monotonic_time():
    global last_tick_us, monotonic_time_us
    current = time.ticks_us()
    elapsed = time.ticks_diff(current, last_tick_us)
    if elapsed > 0:
        monotonic_time_us += elapsed
    last_tick_us = current


def drain_host_input():
    # The commissioning firmware never interprets host commands. Draining them
    # prevents the USB receive queue from filling when the normal bridge sends
    # its safe heartbeat.
    while stdin_poll.poll(0):
        if not sys.stdin.read(1):
            break


def sample_sensors():
    global last_encoder_raw, encoder_anchor_raw, encoder_count
    global encoder_delta, encoder_valid, last_encoder_sample_us
    global steering_zero_raw, steering_zero_candidates
    global steering_cdeg, steering_valid

    update_monotonic_time()

    try:
        raw, valid = read_as5047()
        encoder_valid = valid
        if valid:
            if last_encoder_raw is None:
                last_encoder_raw = raw
                encoder_anchor_raw = raw
                last_encoder_sample_us = monotonic_time_us
            else:
                elapsed_us = monotonic_time_us - last_encoder_sample_us
                instantaneous_delta = wrapped_delta(raw, last_encoder_raw, AS5047_COUNTS)
                if elapsed_us > 0:
                    rpm = instantaneous_delta / AS5047_COUNTS
                    rpm *= 60_000_000.0 / elapsed_us
                    if abs(rpm) > MAXIMUM_MOTOR_RPM:
                        encoder_valid = False
                if encoder_valid:
                    movement = wrapped_delta(raw, encoder_anchor_raw, AS5047_COUNTS)
                    if abs(movement) > POSITION_DEADBAND_COUNTS:
                        signed_movement = movement * ODOMETRY_DIRECTION_SIGN
                        encoder_delta += signed_movement
                        encoder_count += signed_movement
                        encoder_anchor_raw = raw
                last_encoder_raw = raw
                last_encoder_sample_us = monotonic_time_us
        else:
            encoder_delta = 0
            last_encoder_raw = None
            encoder_anchor_raw = None
            last_encoder_sample_us = None
        encoder_raw = raw
    except Exception:
        encoder_raw = -1
        encoder_delta = 0
        encoder_valid = False
        last_encoder_raw = None
        encoder_anchor_raw = None
        last_encoder_sample_us = None

    try:
        raw, sensor_valid = read_steering()
        if sensor_valid and steering_zero_raw is None:
            reference = (
                steering_zero_candidates[0]
                if steering_zero_candidates else raw
            )
            offset = wrapped_delta(raw, reference, AS5600_COUNTS)
            steering_zero_candidates.append(offset)
            if len(steering_zero_candidates) >= STEERING_ZERO_SAMPLES:
                ordered = sorted(steering_zero_candidates)
                # Ignore the outer 10% so one sensor-startup outlier cannot
                # invalidate an otherwise stationary zero window. Genuine
                # steering motion still produces a broad central spread.
                trim = max(1, len(ordered) // 10)
                central = ordered[trim:-trim]
                spread = central[-1] - central[0]
                if spread <= STEERING_ZERO_MAX_SPREAD_COUNTS:
                    median_offset = central[len(central) // 2]
                    steering_zero_raw = (
                        reference + median_offset
                    ) % AS5600_COUNTS
                    steering_zero_candidates = []
                else:
                    # Require a fresh stable window if steering moved during
                    # startup instead of accepting an arbitrary first sample.
                    steering_zero_candidates = []
        if not sensor_valid:
            steering_zero_candidates = []
        steering_valid = sensor_valid and steering_zero_raw is not None
        if steering_valid:
            delta = wrapped_delta(raw, steering_zero_raw, AS5600_COUNTS)
            steering_cdeg = int(round(
                delta * 36000.0 / AS5600_COUNTS * STEERING_DIRECTION_SIGN
            ))
    except Exception:
        steering_valid = False

    return encoder_raw


def validity_bits():
    result = 0
    if encoder_valid:
        result |= VALID_ENCODER
    if steering_valid:
        result |= VALID_STEERING
    return result


def fault_bits():
    result = 0
    if not encoder_valid:
        result |= FAULT_ENCODER_SENSOR
    if not steering_valid:
        result |= FAULT_STEER_SENSOR
    return result


def emit_hello():
    send_frame("HELLO", [
        boot_id,
        FIRMWARE_ID,
        PROTOCOL_VERSION,
        CAPABILITY_BITS,
        CALIBRATION_ID,
    ])


def emit_motion(raw):
    global sequence, encoder_delta
    sequence += 1
    delta_to_send = encoder_delta
    encoder_delta = 0
    send_frame("MOTION", [
        boot_id,
        sequence,
        monotonic_time_us,
        encoder_count,
        delta_to_send,
        raw,
        validity_bits(),
        CALIBRATION_ID,
    ])


def emit_control_feedback():
    global sequence
    sequence += 1
    send_frame("CONTROL_FEEDBACK", [
        boot_id,
        sequence,
        monotonic_time_us,
        0,                  # acknowledged command sequence
        STATE_SAFE,
        0, 0, 0,           # requested/limited/measured speed mm/s
        0, 0, steering_cdeg,
        -steering_cdeg,     # steering error relative to SAFE centre request
        1000, 1000, 1000,  # brake remains represented as applied in SAFE
        0,
        0,                  # DAC mV; no DAC initialized
        0,                  # stepper flags; no stepper initialized
        0,                  # limit bits
        validity_bits(),
        fault_bits(),
    ])


while True:
    cycle_started = time.ticks_ms()
    drain_host_input()
    encoder_raw = sample_sensors()
    now = time.ticks_ms()

    if time.ticks_diff(now, last_hello_ms) >= HELLO_INTERVAL_MS:
        emit_hello()
        # Advance from the prior deadline instead of resetting to `now`.
        # Otherwise a ~13 ms sensor loop quantizes a requested 20 ms stream
        # to every second loop (~38 Hz) rather than averaging 50 Hz.
        last_hello_ms = time.ticks_add(last_hello_ms, HELLO_INTERVAL_MS)
        if time.ticks_diff(now, last_hello_ms) >= HELLO_INTERVAL_MS:
            last_hello_ms = now
    if time.ticks_diff(now, last_motion_ms) >= MOTION_INTERVAL_MS:
        emit_motion(encoder_raw)
        last_motion_ms = time.ticks_add(last_motion_ms, MOTION_INTERVAL_MS)
        if time.ticks_diff(now, last_motion_ms) >= MOTION_INTERVAL_MS:
            last_motion_ms = now
    if time.ticks_diff(now, last_control_ms) >= CONTROL_INTERVAL_MS:
        emit_control_feedback()
        last_control_ms = time.ticks_add(last_control_ms, CONTROL_INTERVAL_MS)
        if time.ticks_diff(now, last_control_ms) >= CONTROL_INTERVAL_MS:
            last_control_ms = now

    remaining = SAMPLE_INTERVAL_MS - time.ticks_diff(time.ticks_ms(), cycle_started)
    if remaining > 0:
        time.sleep_ms(remaining)
