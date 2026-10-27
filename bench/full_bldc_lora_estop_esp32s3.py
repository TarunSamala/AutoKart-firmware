"""ESP32-S3 full BLD750 + GP8630N + AS5047P + RA-02 E-stop test.

The UI uses the AutoKart RC controls: ARM/DISARM, Forward/Reverse, gears 1-5,
Throttle Off, Wheel Lock and Emergency Stop. Normal starts, gear changes,
direction changes and link-loss stops use a voltage ramp. Direction reversal
reaches 0 V, turns EN off, waits, changes F/R, then enables and ramps in the
new direction.

Wheel Lock, Emergency Stop, explicit Disarm, an actuator interlock and program
failure are deliberately immediate: DAC=0 V, EN=STOP and BRK=APPLIED. The lock
remains asserted until ARM is explicitly requested. AS5047P faults remain
visible in telemetry but do not randomly stop this isolated BLDC bench test.

WHEELS-OFF-GROUND TESTING ONLY. The BLD750 control inputs are driven through
the existing PN2222 open-collector stages; never wire them directly to ESP32.

ESP32-S3 pins:
  Steering AS5600 SDA GPIO1, SCL GPIO2, I2C(0), address 0x36
  Steering NEMA34 STEP GPIO4, DIR GPIO5 (ENA GPIO6 remains untouched)
  GP8630N SDA GPIO7, SCL GPIO21, I2C(1), address 0x58
  BLD750 BRK GPIO13, EN GPIO14, F/R GPIO15 (through PN2222 stages)
  AS5047P CS GPIO16, SCK GPIO18, MOSI GPIO41, MISO GPIO42 (SPI mode 1)
  RA-02 CS GPIO9, SCK GPIO12, MOSI GPIO11, MISO GPIO10,
        RESET GPIO8, DIO0 GPIO17 (SPI mode 0)

Primary host control is framed USB serial. Wi-Fi remains available as a
standalone fallback at BLD750-Encoder-Test / 12345678, http://192.168.4.1.
"""

import sys
import network
import socket
import time
from machine import I2C, Pin, SPI
from ra02_lora_chat_common import SX1278

try:
    import uselect as select
except ImportError:
    import select


SSID = "BLD750-Encoder-Test"
PASSWORD = "12345678"

# GP8630N
SDA_GPIO = 7
SCL_GPIO = 21
GP_ADDR = 0x58
GP_MODE_REG = 0x01
GP_DAC_REG = 0x02
GP_MODE_0_10V = 0x1C

# BLD750 PN2222 control stages. GPIO HIGH pulls the BLD input LOW.
BLD_BRK_GPIO = 13
BLD_EN_GPIO = 14
BLD_FR_GPIO = 15

# Open-loop NEMA23/TB6600 mechanical brake.
BRAKE_STEP_GPIO = 38
BRAKE_DIR_GPIO = 39
BRAKE_EN_GPIO = 40
BRAKE_STEP_HIGH_US = 20
BRAKE_STEP_PERIOD_US = 900

# AS5047P SPI
ENC_CS_GPIO = 16
ENC_SCK_GPIO = 18
ENC_MOSI_GPIO = 41
ENC_MISO_GPIO = 42
REG_ERRFL = 0x0001
REG_DIAAGC = 0x3FFC
REG_MAG = 0x3FFD
REG_ANGLE = 0x3FFF
COUNTS_PER_REV = 16384

# RA-02 / SX1278. The AS5047P remains on SPI(1); LoRa uses SPI(2).
RA02_RESET_GPIO = 8
RA02_CS_GPIO = 9
RA02_MISO_GPIO = 10
RA02_MOSI_GPIO = 11
RA02_SCK_GPIO = 12
RA02_DIO0_GPIO = 17
RA02_FREQUENCY_HZ = 433_000_000

# Steering AS5600 + NEMA34/R86mini.  ENA is intentionally not driven: its
# electrical polarity has not been physically qualified on this vehicle.
STEERING_SDA_GPIO = 1
STEERING_SCL_GPIO = 2
STEERING_STEP_GPIO = 4
STEERING_DIR_GPIO = 5
STEERING_AS5600_ADDR = 0x36
STEERING_AS5600_STATUS = 0x0B
STEERING_AS5600_ANGLE = 0x0E
STEERING_COUNTS_PER_REV = 4096
STEERING_RIGHT_DIR_LEVEL = 0
STEERING_SENSOR_INTERVAL_MS = 10
STEERING_STEP_HIGH_US = 20
STEERING_NEAR_PPS = 350
STEERING_FAR_PPS = 700

# Bench limits and timing.
MAX_VOLTAGE = 5.0
BLUE_CONE_TEST_VOLTAGE = 0.4
BLUE_CONE_TEST_ALLOWED_VOLTAGES = (0.5, 1.0, 1.25, 1.5, 2.0, 2.5, 3.0)
RAMP_STEP_V = 0.08
RAMP_INTERVAL_MS = 35
ENCODER_INTERVAL_MS = 5
DIAGNOSTIC_INTERVAL_MS = 100
SERIAL_CHAR_BUDGET = 256
HTTP_CLIENT_TIMEOUT_S = 0.05
# The normal profile is fail-closed: loss of HTTP traffic locks the wheel in
# under one second. A dedicated WHEELS-OFF blue-cone visualization test may
# override this through :bldc_bench_config.py without weakening the default.
try:
    import bldc_bench_config
    CONTROL_LINK_TIMEOUT_MS = int(getattr(
        bldc_bench_config, "CONTROL_LINK_TIMEOUT_MS", 750
    ))
    BLUE_CONE_TEST_VOLTAGE = float(getattr(
        bldc_bench_config, "BLUE_CONE_TEST_VOLTAGE",
        BLUE_CONE_TEST_VOLTAGE,
    ))
    REQUIRE_ENCODER_FOR_START = bool(getattr(
        bldc_bench_config, "REQUIRE_ENCODER_FOR_START", True
    ))
    REQUIRE_LORA_FOR_START = bool(getattr(
        bldc_bench_config, "REQUIRE_LORA_FOR_START", True
    ))
    LORA_PACKET_TIMEOUT_MS = int(getattr(
        bldc_bench_config, "LORA_PACKET_TIMEOUT_MS", 1500
    ))
    LORA_CLEAR_CONFIRMATIONS = int(getattr(
        bldc_bench_config, "LORA_CLEAR_CONFIRMATIONS", 3
    ))
    MECHANICAL_BRAKE_ENABLED = bool(getattr(
        bldc_bench_config, "MECHANICAL_BRAKE_ENABLED", False
    ))
    MECHANICAL_BRAKE_APPLY_DEG = float(getattr(
        bldc_bench_config, "MECHANICAL_BRAKE_APPLY_DEG", 30.0
    ))
    MECHANICAL_BRAKE_PULSES_PER_REV = int(getattr(
        bldc_bench_config, "MECHANICAL_BRAKE_PULSES_PER_REV", 3200
    ))
    MECHANICAL_BRAKE_APPLY_DIR_LEVEL = int(getattr(
        bldc_bench_config, "MECHANICAL_BRAKE_APPLY_DIR_LEVEL", 1
    ))
    MECHANICAL_BRAKE_ENABLE_LEVEL = int(getattr(
        bldc_bench_config, "MECHANICAL_BRAKE_ENABLE_LEVEL", 0
    ))
    STEERING_CENTER_HOLD_ENABLED = bool(getattr(
        bldc_bench_config, "STEERING_CENTER_HOLD_ENABLED", False
    ))
    STEERING_CENTER_RAW = int(getattr(
        bldc_bench_config, "STEERING_CENTER_RAW", -1
    ))
    STEERING_CENTER_DEADBAND_COUNTS = int(getattr(
        bldc_bench_config, "STEERING_CENTER_DEADBAND_COUNTS", 2
    ))
    STEERING_CENTER_REENGAGE_COUNTS = int(getattr(
        bldc_bench_config, "STEERING_CENTER_REENGAGE_COUNTS", 4
    ))
    STEERING_CENTER_MAX_ERROR_COUNTS = int(getattr(
        bldc_bench_config, "STEERING_CENTER_MAX_ERROR_COUNTS", 80
    ))
    STEERING_CENTER_REENGAGE_MS = int(getattr(
        bldc_bench_config, "STEERING_CENTER_REENGAGE_MS", 120
    ))
    STEERING_CENTER_MOVE_TIMEOUT_MS = int(getattr(
        bldc_bench_config, "STEERING_CENTER_MOVE_TIMEOUT_MS", 2500
    ))
    STEERING_CENTER_NO_PROGRESS_MS = int(getattr(
        bldc_bench_config, "STEERING_CENTER_NO_PROGRESS_MS", 600
    ))
except ImportError:
    CONTROL_LINK_TIMEOUT_MS = 750
    REQUIRE_ENCODER_FOR_START = True
    REQUIRE_LORA_FOR_START = True
    LORA_PACKET_TIMEOUT_MS = 1500
    LORA_CLEAR_CONFIRMATIONS = 3
    MECHANICAL_BRAKE_ENABLED = False
    MECHANICAL_BRAKE_APPLY_DEG = 30.0
    MECHANICAL_BRAKE_PULSES_PER_REV = 3200
    MECHANICAL_BRAKE_APPLY_DIR_LEVEL = 1
    MECHANICAL_BRAKE_ENABLE_LEVEL = 0
    STEERING_CENTER_HOLD_ENABLED = False
    STEERING_CENTER_RAW = -1
    STEERING_CENTER_DEADBAND_COUNTS = 2
    STEERING_CENTER_REENGAGE_COUNTS = 4
    STEERING_CENTER_MAX_ERROR_COUNTS = 80
    STEERING_CENTER_REENGAGE_MS = 120
    STEERING_CENTER_MOVE_TIMEOUT_MS = 2500
    STEERING_CENTER_NO_PROGRESS_MS = 600
if BLUE_CONE_TEST_VOLTAGE < 0.0 or BLUE_CONE_TEST_VOLTAGE > MAX_VOLTAGE:
    raise ValueError("BLUE_CONE_TEST_VOLTAGE OUT OF RANGE")
if STEERING_CENTER_HOLD_ENABLED and not 0 <= STEERING_CENTER_RAW < 4096:
    raise ValueError("STEERING_CENTER_RAW MUST BE 0..4095")
ENCODER_STALE_MS = 500
MAX_VALID_RPM = 6000.0
RPM_TIME_CONSTANT_S = 0.030
NOISE_COUNTS = 8

# Encoder magnet is on 14T sprocket; 44T sprocket is fixed to rear axle.
MOTOR_REVS_PER_AXLE_REV = 44.0 / 14.0
TYRE_CIRCUMFERENCE_M = 3.141592653589793 * 11.0 * 0.0254
TRAVEL_PER_MOTOR_REV_M = TYRE_CIRCUMFERENCE_M / MOTOR_REVS_PER_AXLE_REV


def clamp(value, low, high):
    return max(low, min(high, value))


def wrapped_delta(current, previous, counts):
    half = counts // 2
    return ((current - previous + half) % counts) - half


def odd_parity(value):
    return bool(bin(value & 0xFFFF).count("1") & 1)


def command_frame(register):
    frame = 0x4000 | (register & 0x3FFF)
    if odd_parity(frame):
        frame |= 0x8000
    return frame


def frame_bytes(frame):
    return bytes(((frame >> 8) & 0xFF, frame & 0xFF))


# Safe output state is established before buses and Wi-Fi.
en_pin = Pin(BLD_EN_GPIO, Pin.OUT, value=0)       # transistor OFF -> STOP
brk_pin = Pin(BLD_BRK_GPIO, Pin.OUT, value=0)    # transistor OFF -> BRAKE
fr_pin = Pin(BLD_FR_GPIO, Pin.OUT, value=1)      # transistor ON -> CCW
brake_step_pin = None
brake_dir_pin = None
brake_enable_pin = None
mechanical_brake_ready = not MECHANICAL_BRAKE_ENABLED
mechanical_brake_error = "NONE"
if MECHANICAL_BRAKE_ENABLED:
    try:
        brake_step_pin = Pin(BRAKE_STEP_GPIO, Pin.OUT, value=0)
        brake_dir_pin = Pin(BRAKE_DIR_GPIO, Pin.OUT, value=0)
        brake_enable_pin = Pin(
            BRAKE_EN_GPIO,
            Pin.OUT,
            value=1 - MECHANICAL_BRAKE_ENABLE_LEVEL,
        )
        mechanical_brake_ready = True
    except Exception as error:
        # The direct BLD750 stop outputs above are already safe. Keep USB
        # diagnostics alive if this board reserves one of GPIO38/39/40.
        mechanical_brake_ready = False
        mechanical_brake_error = str(error)

armed = False
running = False
brake_released = False
wheel_locked = True
direction = "FORWARD"
requested_direction = "FORWARD"
selected_gear = 1
gear = 0
current_voltage = 0.0
target_voltage = 0.0
drive_state = "STOPPED"
last_lock_reason = "STARTUP"
last_command_ms = time.ticks_ms()
last_ramp_ms = time.ticks_ms()
reversal_pending = False
direction_settle_started_ms = None
link_timeout_active = False
DIRECTION_SETTLE_MS = 350

# Fail-closed LoRa state. A valid CLEAR packet is required before ARM. STOP is
# latched into the normal wheel-lock state; CLEAR never restarts traction.
lora_ready = False
remote_estop = True
remote_clear_seen = False
lora_stop_latched = False
lora_last_packet = "NONE"
lora_last_sequence = -1
lora_packets = 0
lora_invalid_packets = 0
lora_last_rx_ms = -1
lora_link_timeout = False
lora_clear_confirmations = 0
lora_radio_errors = 0

# Open-loop brake position is valid only if the mechanism was physically at
# released/home when this firmware booted. Repeated STOP packets do not add
# travel; manual ARM retracts to zero before releasing the BLD750 brake.
mechanical_brake_steps = 0
mechanical_brake_state = (
    "RELEASED" if MECHANICAL_BRAKE_ENABLED and mechanical_brake_ready
    else "PIN ERROR" if MECHANICAL_BRAKE_ENABLED else "DISABLED"
)

# Steering centre hold starts disarmed.  It is enabled only by ARM/TEST_START,
# after the saved centre and a valid AS5600 magnet state have been verified.
steering_step_pin = Pin(STEERING_STEP_GPIO, Pin.OUT, value=0)
steering_dir_pin = Pin(
    STEERING_DIR_GPIO, Pin.OUT, value=STEERING_RIGHT_DIR_LEVEL
)
steering_i2c = None
steering_bus_error = "NONE"
try:
    steering_i2c = I2C(
        0,
        sda=Pin(STEERING_SDA_GPIO),
        scl=Pin(STEERING_SCL_GPIO),
        freq=100_000,
    )
    if STEERING_AS5600_ADDR not in steering_i2c.scan():
        raise RuntimeError("AS5600 NOT FOUND AT 0x36")
except Exception as error:
    steering_i2c = None
    steering_bus_error = str(error)

steering = {
    "enabled": STEERING_CENTER_HOLD_ENABLED,
    "armed": False,
    "sensor_valid": False,
    "magnet": "NOT CHECKED",
    "raw": -1,
    "center_raw": STEERING_CENTER_RAW,
    "delta_counts": 0,
    "angle_deg": 0.0,
    "state": "DISABLED" if not STEERING_CENTER_HOLD_ENABLED else "DISARMED",
    "fault": "NONE",
    "moving": False,
    "last_valid_ms": -1,
    "last_sensor_ms": 0,
    "last_step_us": time.ticks_us(),
    "error_started_ms": None,
    "move_started_ms": 0,
    "last_progress_ms": 0,
    "best_error": 4096,
    "move_start_delta": 0,
}


def steering_stop(state=None, fault=None):
    steering_step_pin.value(0)
    steering["moving"] = False
    steering["error_started_ms"] = None
    if state is not None:
        steering["state"] = state
    if fault is not None:
        steering["fault"] = fault


def steering_read_sensor():
    if steering_i2c is None:
        steering["sensor_valid"] = False
        steering["magnet"] = "BUS ERROR"
        steering["fault"] = steering_bus_error
        return
    try:
        status = steering_i2c.readfrom_mem(
            STEERING_AS5600_ADDR, STEERING_AS5600_STATUS, 1
        )[0]
        data = steering_i2c.readfrom_mem(
            STEERING_AS5600_ADDR, STEERING_AS5600_ANGLE, 2
        )
        raw = ((data[0] << 8) | data[1]) & 0x0FFF
        if status & 0x08:
            magnet = "TOO STRONG"
        elif status & 0x10:
            magnet = "TOO WEAK"
        elif status & 0x20:
            magnet = "OK"
        else:
            magnet = "NO MAGNET"
        valid = magnet == "OK"
        delta = wrapped_delta(
            raw, steering["center_raw"], STEERING_COUNTS_PER_REV
        )
        steering.update(
            sensor_valid=valid,
            magnet=magnet,
            raw=raw,
            delta_counts=delta,
            # Frozen vehicle convention: raw decreases to physical right.
            angle_deg=-delta * 360.0 / STEERING_COUNTS_PER_REV,
            last_valid_ms=time.ticks_ms() if valid else steering["last_valid_ms"],
        )
        # A manual LOCK/START cycle may recover from a transient I2C error, but
        # never clear a fault while steering is actively armed.
        if valid and not steering["armed"]:
            steering["fault"] = "NONE"
        if not valid and steering["armed"]:
            steering_stop("FAULT", "SENSOR " + magnet)
    except Exception as error:
        steering["sensor_valid"] = False
        steering["magnet"] = "READ ERROR"
        if steering["armed"]:
            steering_stop("FAULT", "I2C " + str(error))


def steering_ready():
    return bool(
        steering["enabled"]
        and steering["sensor_valid"]
        and steering["center_raw"] >= 0
        and steering["last_valid_ms"] >= 0
        and time.ticks_diff(time.ticks_ms(), steering["last_valid_ms"]) <= 250
        and abs(steering["delta_counts"]) <= STEERING_CENTER_MAX_ERROR_COUNTS
        and steering["fault"] == "NONE"
    )


def steering_arm_hold():
    steering_read_sensor()
    if not steering_ready():
        raise RuntimeError(
            "STEERING CENTER HOLD NOT READY: {} raw={} delta={} fault={}".format(
                steering["magnet"], steering["raw"],
                steering["delta_counts"], steering["fault"],
            )
        )
    steering["armed"] = True
    steering["state"] = "CENTRED" if abs(steering["delta_counts"]) <= STEERING_CENTER_DEADBAND_COUNTS else "HOLDING"


def steering_disarm(reason="DISARMED"):
    steering["armed"] = False
    steering_stop(reason)


def service_steering_hold():
    now_ms = time.ticks_ms()
    if time.ticks_diff(now_ms, steering["last_sensor_ms"]) >= STEERING_SENSOR_INTERVAL_MS:
        steering["last_sensor_ms"] = now_ms
        steering_read_sensor()
    if not steering["enabled"]:
        return
    if not steering["armed"]:
        steering_stop("DISARMED")
        return
    if not steering["sensor_valid"]:
        steering_stop("FAULT", "SENSOR INVALID")
        return

    delta = steering["delta_counts"]
    absolute_error = abs(delta)
    if absolute_error > STEERING_CENTER_MAX_ERROR_COUNTS:
        steering_stop("FAULT", "CENTER ERROR LIMIT")
        return
    if absolute_error <= STEERING_CENTER_DEADBAND_COUNTS:
        steering_stop("CENTRED")
        steering["fault"] = "NONE"
        return

    if not steering["moving"]:
        if absolute_error < STEERING_CENTER_REENGAGE_COUNTS:
            steering["state"] = "HOLDING"
            return
        if steering["error_started_ms"] is None:
            steering["error_started_ms"] = now_ms
            steering["state"] = "HOLDING"
            return
        if time.ticks_diff(now_ms, steering["error_started_ms"]) < STEERING_CENTER_REENGAGE_MS:
            return
        steering.update(
            moving=True,
            state="CORRECTING",
            fault="NONE",
            move_started_ms=now_ms,
            last_progress_ms=now_ms,
            best_error=absolute_error,
            move_start_delta=delta,
            error_started_ms=None,
        )

    if time.ticks_diff(now_ms, steering["move_started_ms"]) > STEERING_CENTER_MOVE_TIMEOUT_MS:
        steering_stop("FAULT", "MOVE TIMEOUT")
        return
    if absolute_error < steering["best_error"]:
        steering["best_error"] = absolute_error
        steering["last_progress_ms"] = now_ms
    elif time.ticks_diff(now_ms, steering["last_progress_ms"]) > STEERING_CENTER_NO_PROGRESS_MS:
        steering_stop("FAULT", "NO PROGRESS")
        return
    if (
        abs(delta - steering["move_start_delta"]) >= 3
        and abs(delta) > abs(steering["move_start_delta"])
    ):
        steering_stop("FAULT", "WRONG DIRECTION")
        return

    # delta < 0 means right of centre, so GPIO5 HIGH corrects left.
    steering_dir_pin.value(
        1 - STEERING_RIGHT_DIR_LEVEL if delta < 0 else STEERING_RIGHT_DIR_LEVEL
    )
    pps = STEERING_NEAR_PPS if absolute_error <= 10 else STEERING_FAR_PPS
    period_us = 1_000_000 // pps
    now_us = time.ticks_us()
    if time.ticks_diff(now_us, steering["last_step_us"]) >= period_us:
        steering_step_pin.value(1)
        time.sleep_us(STEERING_STEP_HIGH_US)
        steering_step_pin.value(0)
        steering["last_step_us"] = now_us


def mechanical_brake_target_steps():
    return int(round(
        MECHANICAL_BRAKE_APPLY_DEG
        * MECHANICAL_BRAKE_PULSES_PER_REV
        / 360.0
    ))


def move_mechanical_brake(target_steps, final_state):
    global mechanical_brake_steps, mechanical_brake_state
    if not MECHANICAL_BRAKE_ENABLED:
        mechanical_brake_state = "DISABLED"
        return False
    if not mechanical_brake_ready:
        mechanical_brake_state = "PIN ERROR"
        print("MECHANICAL BRAKE UNAVAILABLE:", mechanical_brake_error)
        return False
    target_steps = int(target_steps)
    delta = target_steps - mechanical_brake_steps
    if delta == 0:
        mechanical_brake_state = final_state
        return True

    mechanical_brake_state = (
        "APPLYING" if delta > 0 else "RELEASING"
    )
    brake_enable_pin.value(MECHANICAL_BRAKE_ENABLE_LEVEL)
    brake_dir_pin.value(
        MECHANICAL_BRAKE_APPLY_DIR_LEVEL
        if delta > 0
        else 1 - MECHANICAL_BRAKE_APPLY_DIR_LEVEL
    )
    time.sleep_ms(2)
    for _ in range(abs(delta)):
        brake_step_pin.value(1)
        time.sleep_us(BRAKE_STEP_HIGH_US)
        brake_step_pin.value(0)
        time.sleep_us(BRAKE_STEP_PERIOD_US - BRAKE_STEP_HIGH_US)
    mechanical_brake_steps = target_steps
    mechanical_brake_state = final_state
    print(
        "MECHANICAL BRAKE -> {} | {} steps | {:.1f} deg".format(
            final_state,
            mechanical_brake_steps,
            mechanical_brake_steps * 360.0 / MECHANICAL_BRAKE_PULSES_PER_REV,
        )
    )
    return True


def apply_mechanical_brake():
    move_mechanical_brake(mechanical_brake_target_steps(), "APPLIED")


def release_mechanical_brake():
    move_mechanical_brake(0, "RELEASED")


def enable_run():
    global running
    en_pin.value(1)  # transistor ON -> BLD EN LOW -> RUN
    running = True


def enable_stop():
    global running
    en_pin.value(0)
    running = False


def release_brake():
    global brake_released
    brk_pin.value(1)  # transistor ON -> BLD BRK LOW -> RELEASED
    brake_released = True


def apply_brake():
    global brake_released
    brk_pin.value(0)
    brake_released = False


def set_direction(value):
    global direction
    direction = value
    fr_pin.value(1 if value == "FORWARD" else 0)


i2c = None
gp_online = False
gp_error = "NOT INITIALISED"
try:
    i2c = I2C(1, sda=Pin(SDA_GPIO), scl=Pin(SCL_GPIO), freq=100_000)
    gp_online = GP_ADDR in i2c.scan()
    if not gp_online:
        gp_error = "NOT FOUND AT 0x58"
    else:
        i2c.writeto_mem(GP_ADDR, GP_MODE_REG, bytes((GP_MODE_0_10V,)))
        gp_error = "NONE"
except Exception as error:
    gp_online = False
    gp_error = str(error)


def write_voltage(voltage):
    global current_voltage, gp_online
    voltage = clamp(float(voltage), 0.0, MAX_VOLTAGE)
    raw = int(voltage / 10.0 * 65535)
    try:
        if not gp_online or i2c is None:
            raise RuntimeError("GP8630N OFFLINE: " + gp_error)
        i2c.writeto_mem(GP_ADDR, GP_DAC_REG, bytes((raw & 0xFF, raw >> 8)))
        current_voltage = voltage
    except Exception:
        gp_online = False
        current_voltage = 0.0
        raise


spi = None
cs = None
encoder_bus_error = "NONE"
try:
    spi = SPI(
        1, baudrate=1_000_000, polarity=0, phase=1, bits=8,
        firstbit=SPI.MSB, sck=Pin(ENC_SCK_GPIO),
        mosi=Pin(ENC_MOSI_GPIO), miso=Pin(ENC_MISO_GPIO),
    )
    cs = Pin(ENC_CS_GPIO, Pin.OUT, value=1)
except Exception as error:
    encoder_bus_error = str(error)

radio_spi = None
radio_cs = None
radio_reset = None
radio_dio0 = None
radio = None
radio_init_error = "NONE"
try:
    radio_spi = SPI(
        2, baudrate=2_000_000, polarity=0, phase=0, bits=8,
        firstbit=SPI.MSB, sck=Pin(RA02_SCK_GPIO),
        mosi=Pin(RA02_MOSI_GPIO), miso=Pin(RA02_MISO_GPIO),
    )
    radio_cs = Pin(RA02_CS_GPIO, Pin.OUT, value=1)
    radio_reset = Pin(RA02_RESET_GPIO, Pin.OUT, value=1)
    radio_dio0 = Pin(RA02_DIO0_GPIO, Pin.IN)
    radio = SX1278(
        radio_spi, radio_cs, radio_reset, RA02_FREQUENCY_HZ
    )
except Exception as error:
    radio_init_error = str(error)


def transfer(frame):
    if spi is None or cs is None:
        raise RuntimeError("AS5047P BUS INIT FAILED: " + encoder_bus_error)
    received = bytearray(2)
    cs.value(0)
    spi.write_readinto(frame_bytes(frame), received)
    cs.value(1)
    time.sleep_us(2)
    return (received[0] << 8) | received[1]


def read_register(register):
    transfer(command_frame(register))
    response = transfer(0x0000)
    return response & 0x3FFF, bool(response & 0x4000), odd_parity(response)


encoder = {
    "online": False, "magnet": "NOT CHECKED", "raw": -1,
    "absolute_deg": 0.0, "relative_deg": 0.0, "zero_raw": -1,
    "initialized": False, "last_raw": -1, "last_us": 0,
    "rpm": 0.0, "axle_rpm": 0.0, "signed_counts": 0,
    "travel_counts": 0, "motor_revs": 0.0, "full_turns": 0,
    "distance_m": 0.0, "displacement_m": 0.0, "speed_kmh": 0.0,
    "magnitude": 0, "agc": 0, "error_flags": 0, "errors": 0,
    "last_valid_ms": 0,
}
last_encoder_ms = 0
last_diagnostic_ms = 0


def magnet_state(diaagc):
    if diaagc & (1 << 10):
        return "TOO STRONG"
    if diaagc & (1 << 11):
        return "TOO WEAK / NO MAGNET"
    if diaagc & (1 << 9):
        return "INVALID"
    if not diaagc & (1 << 8):
        return "NOT READY"
    return "OK"


def update_diagnostics():
    try:
        diag, diag_error, diag_parity = read_register(REG_DIAAGC)
        magnitude, mag_error, mag_parity = read_register(REG_MAG)
        flags, _, _ = read_register(REG_ERRFL)
        bad = diag_error or diag_parity or mag_error or mag_parity
        encoder.update(
            online=not bad, magnet=magnet_state(diag),
            magnitude=magnitude, agc=diag & 0xFF, error_flags=flags,
        )
        if bad:
            encoder["errors"] += 1
    except Exception:
        encoder["online"] = False
        encoder["magnet"] = "READ ERROR"
        encoder["errors"] += 1


def update_encoder():
    try:
        raw, sensor_error, parity_error = read_register(REG_ANGLE)
        if sensor_error or parity_error or encoder["magnet"] != "OK":
            encoder["online"] = False if sensor_error or parity_error else True
            encoder["errors"] += 1
            return

        now_us = time.ticks_us()
        now_ms = time.ticks_ms()
        encoder.update(
            online=True, raw=raw, absolute_deg=raw * 360.0 / COUNTS_PER_REV,
            last_valid_ms=now_ms,
        )
        if not encoder["initialized"]:
            encoder.update(
                initialized=True, zero_raw=raw, last_raw=raw,
                last_us=now_us, relative_deg=0.0,
            )
            return

        dt_us = time.ticks_diff(now_us, encoder["last_us"])
        delta = wrapped_delta(raw, encoder["last_raw"], COUNTS_PER_REV)
        encoder["last_raw"] = raw
        encoder["last_us"] = now_us
        encoder["relative_deg"] = (
            wrapped_delta(raw, encoder["zero_raw"], COUNTS_PER_REV)
            * 360.0 / COUNTS_PER_REV
        )
        if dt_us <= 0:
            return
        raw_rpm = delta * 60_000_000.0 / (COUNTS_PER_REV * dt_us)
        if abs(raw_rpm) > MAX_VALID_RPM:
            encoder["errors"] += 1
            return
        if abs(delta) <= NOISE_COUNTS:
            delta = 0
            raw_rpm = 0.0
        dt_s = dt_us / 1_000_000.0
        alpha = dt_s / (RPM_TIME_CONSTANT_S + dt_s)
        encoder["rpm"] += alpha * (raw_rpm - encoder["rpm"])
        if delta == 0 and abs(encoder["rpm"]) < 1.0:
            encoder["rpm"] = 0.0
        encoder["signed_counts"] += delta
        encoder["travel_counts"] += abs(delta)
        encoder["motor_revs"] = encoder["signed_counts"] / COUNTS_PER_REV
        travelled_revs = encoder["travel_counts"] / COUNTS_PER_REV
        encoder["full_turns"] = int(travelled_revs)
        encoder["axle_rpm"] = encoder["rpm"] / MOTOR_REVS_PER_AXLE_REV
        encoder["distance_m"] = travelled_revs * TRAVEL_PER_MOTOR_REV_M
        encoder["displacement_m"] = (
            encoder["motor_revs"] * TRAVEL_PER_MOTOR_REV_M
        )
        encoder["speed_kmh"] = abs(
            encoder["axle_rpm"] * TYRE_CIRCUMFERENCE_M * 60.0 / 1000.0
        )
    except Exception:
        encoder["online"] = False
        encoder["magnet"] = "READ ERROR"
        encoder["errors"] += 1


def service_encoder():
    global last_encoder_ms, last_diagnostic_ms
    now = time.ticks_ms()
    if time.ticks_diff(now, last_diagnostic_ms) >= DIAGNOSTIC_INTERVAL_MS:
        last_diagnostic_ms = now
        update_diagnostics()
    if time.ticks_diff(now, last_encoder_ms) >= ENCODER_INTERVAL_MS:
        last_encoder_ms = now
        update_encoder()


def encoder_ready():
    return (
        encoder["online"] and encoder["initialized"]
        and encoder["magnet"] == "OK"
        and time.ticks_diff(time.ticks_ms(), encoder["last_valid_ms"])
        <= ENCODER_STALE_MS
    )


def zero_encoder():
    raw = encoder["raw"]
    now_us = time.ticks_us()
    encoder.update(
        initialized=True, zero_raw=raw, last_raw=raw, last_us=now_us,
        relative_deg=0.0, rpm=0.0, axle_rpm=0.0, signed_counts=0,
        travel_counts=0, motor_revs=0.0, full_turns=0,
        distance_m=0.0, displacement_m=0.0, speed_kmh=0.0,
        errors=0,
    )


def lock_wheel(reason="WHEEL LOCK"):
    global armed, target_voltage, gear, reversal_pending, drive_state
    global direction_settle_started_ms, wheel_locked, last_lock_reason
    armed = False
    wheel_locked = True
    target_voltage = 0.0
    gear = 0
    reversal_pending = False
    direction_settle_started_ms = None
    drive_state = "WHEEL LOCKED"
    last_lock_reason = reason
    steering_disarm("LOCKED")
    # Assert the two direct hardware stop outputs before touching I2C.  BRK and
    # EN must not be delayed if the GP8630N bus is stalled or disconnected.
    apply_brake()
    enable_stop()
    apply_mechanical_brake()
    try:
        write_voltage(0.0)
    except Exception:
        pass
    print("WHEEL LOCKED:", reason, "| SV=0 V | EN=STOP | BRK=APPLIED")


def safe_stop(reason):
    lock_wheel(reason)
    print("SAFE STOP:", reason)


def parse_estop_packet(packet):
    try:
        text = packet.decode().strip()
        fields = text.split(",")
        if len(fields) != 3 or fields[0] != "ESTOP":
            return None
        state = int(fields[1])
        sequence = int(fields[2])
        if state not in (0, 1) or sequence < 0 or sequence > 0xFFFF:
            return None
        return text, state, sequence
    except Exception:
        return None


def service_lora_estop():
    global remote_estop, remote_clear_seen, lora_stop_latched, lora_last_packet
    global lora_last_sequence, lora_packets, lora_invalid_packets
    global lora_last_rx_ms, lora_link_timeout, lora_clear_confirmations
    global lora_radio_errors

    if not lora_ready:
        return
    try:
        packet = radio.receive()
    except Exception as error:
        lora_radio_errors += 1
        if lora_radio_errors == 1 or lora_radio_errors % 20 == 0:
            print("LORA RX ERROR:", error)
        return
    if packet is None:
        return

    parsed = parse_estop_packet(packet)
    if parsed is None:
        lora_invalid_packets += 1
        print("LORA INVALID PACKET")
        return

    text, state, sequence = parsed
    lora_packets += 1
    lora_last_packet = text
    lora_last_rx_ms = time.ticks_ms()
    lora_last_sequence = sequence
    lora_link_timeout = False

    # Direct E-stop rule: every received state=1 packet forces the safe outputs.
    # The separate watchdog also locks traction if this heartbeat disappears.
    if state == 1:
        first_stop = not remote_estop
        outputs_unsafe = (
            armed or running or brake_released or not wheel_locked
            or current_voltage > 0.01 or target_voltage > 0.01
        )
        remote_estop = True
        lora_stop_latched = True
        remote_clear_seen = False
        lora_clear_confirmations = 0
        if first_stop or outputs_unsafe:
            lock_wheel("REMOTE LORA E-STOP")
            print("LORA RX:", text, "-> BRK APPLIED")
        else:
            # Reassert direct hardware outputs for every STOP heartbeat.  This
            # is intentionally idempotent and does not wait for I2C.
            apply_brake()
            enable_stop()
            apply_mechanical_brake()
        return

    lora_clear_confirmations = min(
        LORA_CLEAR_CONFIRMATIONS, lora_clear_confirmations + 1
    )
    if lora_clear_confirmations < LORA_CLEAR_CONFIRMATIONS:
        return
    was_stopped = remote_estop or not remote_clear_seen or lora_stop_latched
    remote_estop = False
    remote_clear_seen = True
    lora_stop_latched = False
    if was_stopped:
        print(
            "LORA RX:", text, "-> CLEAR CONFIRMED; MANUAL ARM REQUIRED"
        )


def service_lora_watchdog():
    """Convert missing radio heartbeats into a latched stop, never an exception."""
    global remote_estop, remote_clear_seen, lora_stop_latched
    global lora_link_timeout, lora_clear_confirmations

    if not REQUIRE_LORA_FOR_START or LORA_PACKET_TIMEOUT_MS <= 0:
        return
    now = time.ticks_ms()
    if lora_last_rx_ms < 0:
        # Boot is already electrically locked. With no brake position sensor,
        # do not invent another 30-degree move merely because the transmitter
        # has not appeared yet. An actual STOP packet still applies it, and
        # loss after any established heartbeat follows the full lock path.
        if not lora_link_timeout:
            lora_link_timeout = True
            remote_estop = True
            remote_clear_seen = False
            lora_stop_latched = True
            lora_clear_confirmations = 0
            print("WAITING FOR FIRST LORA HEARTBEAT; BLD750 LOCKED")
        return
    expired = time.ticks_diff(now, lora_last_rx_ms) > LORA_PACKET_TIMEOUT_MS
    if not expired or lora_link_timeout:
        return

    lora_link_timeout = True
    remote_estop = True
    remote_clear_seen = False
    lora_stop_latched = True
    lora_clear_confirmations = 0
    lock_wheel("LORA HEARTBEAT LOST")
    print("LORA LINK LOST -> BRK APPLIED; CLEAR + MANUAL ARM REQUIRED")


def request_soft_stop(reason="SOFT STOP"):
    global target_voltage, reversal_pending, direction_settle_started_ms
    global drive_state
    target_voltage = 0.0
    reversal_pending = False
    direction_settle_started_ms = None
    if wheel_locked:
        drive_state = "WHEEL LOCKED"
        return
    drive_state = "SLOWING" if current_voltage > 0.01 else "STOPPED"
    print(reason + ": ramping SV to zero")


def request_drive(new_direction):
    global requested_direction, target_voltage, reversal_pending
    global direction_settle_started_ms, drive_state
    if MECHANICAL_BRAKE_ENABLED and mechanical_brake_state != "RELEASED":
        raise RuntimeError("MECHANICAL BRAKE NOT RELEASED")
    requested_direction = new_direction
    if new_direction != direction:
        target_voltage = 0.0
        reversal_pending = True
        direction_settle_started_ms = None
        drive_state = "SLOWING FOR " + new_direction
    else:
        reversal_pending = False
        target_voltage = float(selected_gear)
        release_brake()
        enable_run()
        drive_state = "RAMPING " + direction


def arm_controller():
    global armed, drive_state, wheel_locked, last_lock_reason
    if armed:
        return
    if MECHANICAL_BRAKE_ENABLED and not mechanical_brake_ready:
        raise RuntimeError(
            "MECHANICAL BRAKE PIN INIT FAILED: " + mechanical_brake_error
        )
    if REQUIRE_LORA_FOR_START and not lora_ready:
        raise RuntimeError("RA-02 NOT READY")
    if ((REQUIRE_LORA_FOR_START and (remote_estop or not remote_clear_seen))
            or lora_stop_latched):
        raise RuntimeError("REMOTE E-STOP NOT CLEAR")
    if STEERING_CENTER_HOLD_ENABLED:
        steering_read_sensor()
        if not steering_ready():
            raise RuntimeError(
                "STEERING CENTER HOLD NOT READY: {} raw={} delta={} fault={}".format(
                    steering["magnet"], steering["raw"],
                    steering["delta_counts"], steering["fault"],
                )
            )
    # CLEAR only permits a manual ARM; it never retracts the physical brake.
    # Retraction happens here while BLD750 EN remains STOP and BRK remains
    # APPLIED. Traction is released only after the open-loop return completes.
    apply_brake()
    enable_stop()
    release_mechanical_brake()
    write_voltage(0.0)
    enable_stop()
    release_brake()
    wheel_locked = False
    armed = True
    if STEERING_CENTER_HOLD_ENABLED:
        steering_arm_hold()
    last_lock_reason = "NONE"
    drive_state = "ARMED AT 0 V"
    print("ARMED AT 0 V")


def start_blue_cone_test(test_voltage):
    """Atomically zero, arm and start forward at the fixed blue-cone test level."""
    global selected_gear
    if not gp_online:
        raise RuntimeError("GP8630N OFFLINE")
    encoder_is_ready = encoder_ready()
    if REQUIRE_ENCODER_FOR_START and not encoder_is_ready:
        raise RuntimeError("AS5047P NOT READY")
    if not armed:
        if encoder_is_ready:
            zero_encoder()
        arm_controller()
    selected_gear = test_voltage
    request_drive("FORWARD")


def service_traction():
    global last_ramp_ms, direction, gear, target_voltage, drive_state
    global reversal_pending, direction_settle_started_ms
    now = time.ticks_ms()
    if time.ticks_diff(now, last_ramp_ms) < RAMP_INTERVAL_MS:
        return
    last_ramp_ms = now

    if not armed or not brake_released:
        if current_voltage > 0.0:
            safe_stop("INTERLOCK")
        return

    if reversal_pending:
        if current_voltage > 0.0:
            write_voltage(max(0.0, current_voltage - RAMP_STEP_V))
            drive_state = "SLOWING FOR " + requested_direction
            return
        enable_stop()
        gear = 0
        if direction_settle_started_ms is None:
            direction_settle_started_ms = now
            drive_state = "DIRECTION PAUSE"
            return
        if time.ticks_diff(now, direction_settle_started_ms) < DIRECTION_SETTLE_MS:
            return
        set_direction(requested_direction)
        reversal_pending = False
        direction_settle_started_ms = None
        target_voltage = float(selected_gear)
        release_brake()
        enable_run()
        drive_state = "RAMPING " + direction
        return

    if current_voltage < target_voltage:
        write_voltage(min(target_voltage, current_voltage + RAMP_STEP_V))
        gear = selected_gear
        drive_state = "RAMPING " + direction
    elif current_voltage > target_voltage:
        write_voltage(max(target_voltage, current_voltage - RAMP_STEP_V))
        drive_state = "SLOWING"
    elif target_voltage > 0.0:
        gear = selected_gear
        drive_state = "RUNNING " + direction
    else:
        gear = 0
        enable_stop()
        drive_state = "STOPPED"


try:
    write_voltage(0.0)
except Exception as error:
    # Communication and diagnostics must still start in a fail-closed state.
    # START remains rejected while gp_online is false.
    current_voltage = 0.0
    gp_online = False
    gp_error = str(error)
update_diagnostics()
update_encoder()
steering_read_sensor()
try:
    if radio is None:
        raise RuntimeError("RA-02 BUS INIT FAILED: " + radio_init_error)
    radio.begin()
    lora_ready = True
except Exception as error:
    # A missing/failed radio must not kill USB/Wi-Fi diagnostics.  The remote
    # E-stop remains asserted and ARM is rejected until lora_ready is true.
    lora_ready = False
    remote_estop = True
    print("RA-02 OFFLINE:", error)


HTML = r'''<!doctype html><html><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1,user-scalable=no"><title>BLD750 Encoder Test</title><style>
:root{--bg:#080c12;--card:#131c28;--line:#29384d;--text:#f5f8ff;--muted:#93a2b7;--green:#18b978;--red:#e94255;--blue:#397dff;--orange:#ef8d31}*{box-sizing:border-box}body{margin:0;background:radial-gradient(circle at 50% -10%,#203451,var(--bg) 42%);color:var(--text);font-family:system-ui,sans-serif;touch-action:manipulation}.app{max-width:620px;margin:auto;padding:14px}.top{display:flex;justify-content:space-between;align-items:center;gap:10px}.top h1{font-size:24px;margin:8px 0}.badge{padding:8px 12px;border-radius:999px;background:#441821;color:#ff9aa5;font-size:11px;font-weight:900}.badge.on{background:#123d2c;color:#78e9b5}.card{background:linear-gradient(145deg,#172231,var(--card));border:1px solid var(--line);border-radius:18px;padding:14px;margin-top:11px}.label{color:var(--muted);font-size:10px;font-weight:900;letter-spacing:.14em;margin-bottom:9px}.voltage{text-align:center;font-size:44px;font-weight:900}.voltage small{font-size:16px;color:var(--muted)}.row{display:flex;gap:8px}.row>*{flex:1}.gears{display:grid;grid-template-columns:repeat(5,1fr);gap:6px}button{min-height:52px;border:0;border-radius:13px;color:white;background:#2b394d;font-size:15px;font-weight:850;box-shadow:inset 0 1px #ffffff22;transition:transform .09s,filter .09s}button:active,button.active{transform:scale(.96);filter:brightness(.85)}button.active{outline:3px solid #55d9ff;outline-offset:2px}button:disabled{opacity:.35}.green{background:var(--green)}.red{background:var(--red)}.blue{background:var(--blue)}.orange{background:var(--orange)}.wide{width:100%;margin-top:9px}.stop{width:100%;min-height:66px;font-size:19px;margin-top:10px}.grid{display:grid;grid-template-columns:1fr 1fr;gap:8px}.tile{background:#0c131d;border:1px solid #253449;border-radius:12px;padding:10px}.tile span{display:block;color:var(--muted);font-size:9px;font-weight:800;letter-spacing:.08em}.tile b{font-size:16px}.wideTile{grid-column:1/-1}.ok{color:#73e6ae}.bad{color:#ff929d}.msg{text-align:center;color:#ffd078;min-height:25px;margin-top:10px;font-size:12px}.warning{text-align:center;color:#ff929d;font-size:11px;font-weight:800;margin:14px}@media(max-width:390px){.app{padding:8px}.card{padding:11px}.top h1{font-size:21px}}
</style></head><body><main class="app"><div class="top"><h1>BLD750 Smooth Test</h1><div id="runState" class="badge">DISARMED</div></div>
<section class="card"><div class="row"><button id="armBtn" class="green">ARM + UNLOCK</button><button id="disarmBtn" class="red">DISARM</button></div><button id="lockBtn" class="red wide">🔒 LOCK WHEEL</button><div style="color:var(--muted);font-size:10px;text-align:center;margin-top:8px">Lock forces SV to 0 V, EN to STOP and BLD750 BRK to APPLIED. ARM releases the lock.</div></section>
<section class="card"><div class="label">DIRECTION</div><div class="row"><button id="forwardBtn" class="blue">▲ FORWARD</button><button id="reverseBtn" class="orange">▼ REVERSE</button></div><div style="color:var(--muted);font-size:10px;text-align:center;margin-top:8px">Direction changes ramp to 0 V, pause, switch F/R, then ramp up.</div></section>
<section class="card"><div class="label">SPEED GEAR</div><div class="gears"><button data-gear="1">1</button><button data-gear="2">2</button><button data-gear="3">3</button><button data-gear="4">4</button><button data-gear="5">5</button></div><button id="offBtn" class="wide">THROTTLE OFF · SMOOTH STOP</button><div class="voltage"><span id="targetV">0.00</span> <small>V target</small></div></section>
<section class="card"><button id="zeroBtn" class="wide">ZERO ENCODER + DISTANCE</button></section>
<section class="card"><div class="label">LIVE AS5047P + LORA FEEDBACK</div><div class="grid"><div class="tile"><span>DRIVE</span><b id="driveState">WHEEL LOCKED</b></div><div class="tile"><span>BLDC BRAKE LOCK</span><b id="lockState" class="bad">LOCKED</b></div><div class="tile"><span>LORA E-STOP</span><b id="loraState" class="bad">WAITING FOR CLEAR</b></div><div class="tile"><span>LORA PACKET</span><b id="loraPacket">NONE</b></div><div class="tile"><span>OUTPUT / TARGET</span><b><i id="actualV">0.00 V</i> / <i id="commandV">0.00 V</i></b></div><div class="tile"><span>MOTOR RPM</span><b id="motorRpm">0</b></div><div class="tile"><span>AXLE RPM</span><b id="axleRpm">0</b></div><div class="tile"><span>FULL MOTOR TURNS</span><b id="turns">0</b></div><div class="tile"><span>MOTOR REVOLUTIONS</span><b id="revs">0.000</b></div><div class="tile"><span>NOMINAL SPEED</span><b id="speed">0.00 km/h</b></div><div class="tile"><span>DISTANCE TRAVELLED</span><b id="distance">0.000 m</b></div><div class="tile"><span>SIGNED DISPLACEMENT</span><b id="displacement">0.000 m</b></div><div class="tile"><span>ANGLE FROM ZERO</span><b id="angle">0.0°</b></div><div class="tile wideTile"><span>ENCODER HEALTH</span><b id="health">CHECKING</b><div id="detail" style="color:var(--muted);font-size:10px;margin-top:5px">--</div></div></div></section>
<button id="safeBtn" class="red stop">EMERGENCY STOP + BRAKE</button><div id="message" class="msg">Connecting…</div><div class="warning">BENCH TEST ONLY · DRIVE WHEELS MUST BE RAISED AND GUARDED</div></main><script>
const $=x=>document.getElementById(x);function msg(t,b=false){$('message').textContent=t;$('message').className='msg '+(b?'bad':'')}async function cmd(p,quiet=false){try{const r=await fetch(p,{cache:'no-store'}),t=await r.text();if(!quiet)msg(t,!r.ok);return r.ok}catch(e){if(!quiet)msg('CONTROLLER OFFLINE',true);return false}}$('armBtn').onclick=()=>cmd('/arm');$('disarmBtn').onclick=()=>cmd('/disarm');$('lockBtn').onclick=()=>cmd('/lock');$('forwardBtn').onclick=()=>cmd('/drive?d=FORWARD');$('reverseBtn').onclick=()=>cmd('/drive?d=REVERSE');$('offBtn').onclick=()=>cmd('/gear?n=0');$('safeBtn').onclick=()=>cmd('/lock');$('zeroBtn').onclick=()=>cmd('/zero');document.querySelectorAll('[data-gear]').forEach(b=>b.onclick=()=>cmd('/gear?n='+b.dataset.gear));
async function status(){try{const r=await fetch('/status',{cache:'no-store'}),s=await r.json();$('runState').textContent=s.wheel_locked?'WHEEL LOCKED':s.armed?'ARMED':'DISARMED';$('runState').classList.toggle('on',s.armed&&!s.wheel_locked);$('targetV').textContent=s.target_voltage.toFixed(2);$('actualV').textContent=s.voltage.toFixed(2)+' V';$('commandV').textContent=s.target_voltage.toFixed(2)+' V';$('driveState').textContent=s.drive_state;$('lockState').textContent=s.wheel_locked?'LOCKED':'RELEASED';$('lockState').className=s.wheel_locked?'bad':'ok';$('loraState').textContent=!s.lora_ready?'RADIO OFFLINE':s.remote_estop?'STOP ACTIVE':s.remote_clear_seen?'CLEAR':'WAITING FOR CLEAR';$('loraState').className=s.lora_ready&&!s.remote_estop&&s.remote_clear_seen?'ok':'bad';$('loraPacket').textContent=s.lora_last_packet;$('motorRpm').textContent=Math.abs(s.rpm).toFixed(0);$('axleRpm').textContent=Math.abs(s.axle_rpm).toFixed(0);$('turns').textContent=s.full_turns;$('revs').textContent=s.motor_revs.toFixed(3);$('speed').textContent=s.speed_kmh.toFixed(2)+' km/h';$('distance').textContent=s.distance_m.toFixed(3)+' m';$('displacement').textContent=s.displacement_m.toFixed(3)+' m';$('angle').textContent=s.relative_deg.toFixed(1)+'°';$('health').textContent=s.encoder_ready?'MAGNET OK':'CHECK: '+s.magnet;$('health').className=s.encoder_ready?'ok':'bad';$('detail').textContent='raw '+s.raw+' · errors '+s.errors+' · AGC '+s.agc+' · MAG '+s.magnitude;$('armBtn').disabled=s.remote_estop||!s.remote_clear_seen||(s.armed&&!s.wheel_locked);$('disarmBtn').disabled=s.wheel_locked;$('lockBtn').disabled=s.wheel_locked;$('forwardBtn').disabled=!s.armed||s.wheel_locked;$('reverseBtn').disabled=!s.armed||s.wheel_locked;$('zeroBtn').disabled=s.armed||s.voltage>.01;$('forwardBtn').classList.toggle('active',s.direction==='FORWARD'&&s.target_voltage>0);$('reverseBtn').classList.toggle('active',s.direction==='REVERSE'&&s.target_voltage>0);document.querySelectorAll('[data-gear]').forEach(b=>b.classList.toggle('active',Number(b.dataset.gear)===s.selected_gear))}catch(e){msg('CONTROLLER OFFLINE',true)}}setInterval(status,250);addEventListener('pagehide',()=>navigator.sendBeacon('/soft_stop'));status();
</script></body></html>'''


def send_all(client, data):
    view = memoryview(data)
    while view:
        sent = client.send(view)
        if not sent:
            return
        view = view[sent:]


def reply(client, body, status="200 OK", content_type="text/plain; charset=utf-8"):
    if isinstance(body, str):
        body = body.encode()
    header = (
        "HTTP/1.1 " + status + "\r\nContent-Type: " + content_type
        + "\r\nContent-Length: " + str(len(body))
        + "\r\nCache-Control: no-store\r\nConnection: close\r\n\r\n"
    ).encode()
    send_all(client, header)
    send_all(client, body)


def query(path, key, default):
    try:
        return path.split(key + "=", 1)[1].split("&", 1)[0]
    except Exception:
        return default


def json_status():
    return (
        '{{"armed":{},"running":{},"brake_released":{},"wheel_locked":{},'
        '"direction":"{}","requested_direction":"{}",'
        '"drive_state":"{}","last_lock_reason":"{}",'
        '"gear":{},"selected_gear":{},'
        '"voltage":{:.2f},"target_voltage":{:.2f},"gp_online":{},'
        '"link_timeout":{},"control_link_timeout_ms":{},'
        '"encoder_ready":{},"encoder_required":{},"magnet":"{}","raw":{},'
        '"absolute_deg":{:.3f},"relative_deg":{:.3f},'
        '"rpm":{:.2f},"axle_rpm":{:.2f},"motor_revs":{:.5f},'
        '"full_turns":{},"speed_kmh":{:.3f},"distance_m":{:.4f},'
        '"displacement_m":{:.4f},"agc":{},"magnitude":{},"errors":{},'
        '"lora_ready":{},"lora_required":{},"lora_interlock_active":{},'
        '"remote_estop":{},"remote_clear_seen":{},'
        '"lora_last_packet":"{}","lora_last_sequence":{},'
        '"lora_packets":{},"lora_invalid_packets":{},"lora_age_ms":{},'
        '"lora_link_timeout":{},"lora_timeout_ms":{},'
        '"lora_clear_confirmations":{},"lora_clear_required":{},'
        '"lora_radio_errors":{},"mechanical_brake_enabled":{},'
        '"mechanical_brake_ready":{},"mechanical_brake_error":"{}",'
        '"mechanical_brake_state":"{}","mechanical_brake_steps":{},'
        '"mechanical_brake_degrees":{:.2f},'
        '"mechanical_brake_apply_degrees":{:.2f},'
        '"steering_hold_enabled":{},"steering_hold_armed":{},'
        '"steering_ready":{},"steering_sensor_valid":{},'
        '"steering_magnet":"{}","steering_raw":{},'
        '"steering_center_raw":{},"steering_delta_counts":{},'
        '"steering_angle_deg":{:.3f},"steering_state":"{}",'
        '"steering_fault":"{}"}}'
    ).format(
        str(armed).lower(), str(running).lower(),
        str(brake_released).lower(), str(wheel_locked).lower(),
        direction, requested_direction,
        drive_state, last_lock_reason.replace('"', "'").replace("\\", "/"),
        gear, selected_gear,
        current_voltage, target_voltage, str(gp_online).lower(),
        str(link_timeout_active).lower(), CONTROL_LINK_TIMEOUT_MS,
        str(encoder_ready()).lower(), str(REQUIRE_ENCODER_FOR_START).lower(),
        encoder["magnet"], encoder["raw"],
        encoder["absolute_deg"], encoder["relative_deg"], encoder["rpm"],
        encoder["axle_rpm"], encoder["motor_revs"], encoder["full_turns"],
        encoder["speed_kmh"], encoder["distance_m"],
        encoder["displacement_m"], encoder["agc"], encoder["magnitude"],
        encoder["errors"], str(lora_ready).lower(),
        str(REQUIRE_LORA_FOR_START).lower(),
        str((REQUIRE_LORA_FOR_START and (
            not lora_ready or remote_estop or not remote_clear_seen
        )) or lora_stop_latched).lower(),
        str(remote_estop).lower(), str(remote_clear_seen).lower(),
        lora_last_packet, lora_last_sequence, lora_packets,
        lora_invalid_packets,
        -1 if lora_last_rx_ms < 0 else time.ticks_diff(
            time.ticks_ms(), lora_last_rx_ms
        ),
        str(lora_link_timeout).lower(), LORA_PACKET_TIMEOUT_MS,
        lora_clear_confirmations, LORA_CLEAR_CONFIRMATIONS,
        lora_radio_errors,
        str(MECHANICAL_BRAKE_ENABLED).lower(),
        str(mechanical_brake_ready).lower(),
        mechanical_brake_error.replace('"', "'").replace("\\", "/"),
        mechanical_brake_state,
        mechanical_brake_steps,
        mechanical_brake_steps * 360.0 / MECHANICAL_BRAKE_PULSES_PER_REV,
        MECHANICAL_BRAKE_APPLY_DEG,
        str(steering["enabled"]).lower(),
        str(steering["armed"]).lower(),
        str(steering_ready()).lower(),
        str(steering["sensor_valid"]).lower(),
        steering["magnet"], steering["raw"], steering["center_raw"],
        steering["delta_counts"], steering["angle_deg"],
        steering["state"],
        steering["fault"].replace('"', "'").replace("\\", "/"),
    )


def execute_control(command, argument=""):
    """Execute one transport-independent control command.

    Returns the plain-text response used by both USB serial and HTTP.  Command
    failures deliberately raise RuntimeError so each transport can report a
    rejected action without hiding it as a successful request.
    """
    global selected_gear, target_voltage, last_command_ms, link_timeout_active

    command = command.upper()
    argument = argument.upper()
    last_command_ms = time.ticks_ms()
    link_timeout_active = False

    if command == "STATUS":
        return json_status()
    if command == "ARM":
        if not gp_online:
            raise RuntimeError("GP8630N OFFLINE")
        arm_controller()
        return "ARMED AT 0 V"
    if command == "TEST_START":
        try:
            requested_voltage = (
                BLUE_CONE_TEST_VOLTAGE if not argument else float(argument)
            )
        except Exception:
            raise RuntimeError("INVALID TEST VOLTAGE")
        matched_voltage = None
        for allowed_voltage in BLUE_CONE_TEST_ALLOWED_VOLTAGES:
            if abs(requested_voltage - allowed_voltage) < 0.001:
                matched_voltage = allowed_voltage
                break
        if matched_voltage is None:
            raise RuntimeError(
                "TEST VOLTAGE MUST BE 0.50, 1.00, 1.25, 1.50, 2.00, 2.50 OR 3.00 V"
            )
        if armed and target_voltage > 0.0:
            return "BLUE-CONE TEST ALREADY RUNNING AT {:.2f} V".format(
                target_voltage
            )
        start_blue_cone_test(matched_voltage)
        return "BLUE-CONE TEST STARTED: FORWARD {:.2f} V".format(
            matched_voltage
        )
    if command == "SOFT_STOP":
        request_soft_stop()
        return "THROTTLE RAMPING TO ZERO"
    if command == "LOCK":
        lock_wheel("REMOTE WHEEL LOCK")
        return "WHEEL LOCKED: SV=0 V, EN=STOP, BRK=APPLIED"
    if command == "DISARM":
        safe_stop("REMOTE DISARM")
        return "DISARMED + WHEEL LOCKED"
    if command == "DRIVE":
        if not armed:
            raise RuntimeError("ARM FIRST")
        if ((REQUIRE_LORA_FOR_START and (
                not lora_ready or remote_estop or not remote_clear_seen
            )) or lora_stop_latched):
            lock_wheel("REMOTE E-STOP INTERLOCK")
            raise RuntimeError("REMOTE E-STOP NOT CLEAR")
        if not gp_online:
            raise RuntimeError("GP8630N OFFLINE")
        if argument not in ("FORWARD", "REVERSE"):
            raise RuntimeError("BAD DIRECTION")
        request_drive(argument)
        return argument + " REQUESTED"
    if command == "GEAR":
        try:
            requested = int(argument)
        except Exception:
            raise RuntimeError("GEAR MUST BE 0..5")
        if requested < 0 or requested > 5:
            raise RuntimeError("GEAR MUST BE 0..5")
        if requested == 0:
            request_soft_stop("THROTTLE OFF")
            return "THROTTLE RAMPING TO ZERO"
        selected_gear = requested
        if target_voltage > 0.0 and not reversal_pending:
            target_voltage = float(selected_gear)
        return "GEAR {} SELECTED / {:.1f} V".format(
            selected_gear, float(selected_gear)
        )
    if command == "ZERO":
        if armed or current_voltage > 0.01:
            raise RuntimeError("DISARM AND STOP BEFORE ZERO")
        if not encoder_ready():
            raise RuntimeError("AS5047P NOT READY")
        zero_encoder()
        return "ENCODER, TURNS AND DISTANCE ZEROED"
    raise RuntimeError("UNKNOWN COMMAND")


serial_poll = select.poll()
serial_poll.register(sys.stdin, select.POLLIN)
serial_buffer = ""


def serial_reply(request_id, outcome, payload):
    payload = str(payload).replace("\r", " ").replace("\n", " ")
    sys.stdout.write("@AKR {} {} {}\n".format(request_id, outcome, payload))


def service_serial():
    """Service newline-framed USB commands without blocking motor control."""
    global serial_buffer
    processed = 0
    while processed < SERIAL_CHAR_BUDGET and serial_poll.poll(0):
        character = sys.stdin.read(1)
        processed += 1
        if not character:
            break
        # Let mpremote interrupt a running controller.  Without this, the
        # application-level serial parser consumes Ctrl-C before MicroPython
        # can stop main.py, making later firmware updates depend on racing the
        # physical reset button.  The outer finally block still applies the
        # full safe stop before returning to the REPL.
        if character == "\x03":
            raise KeyboardInterrupt
        if character in "\r\n":
            line = serial_buffer.strip()
            serial_buffer = ""
            if not line.startswith("@AKCMD "):
                continue
            parts = line.split(" ", 3)
            request_id = parts[1] if len(parts) > 1 else "0"
            command = parts[2] if len(parts) > 2 else ""
            argument = parts[3].strip() if len(parts) > 3 else ""
            try:
                serial_reply(request_id, "OK", execute_control(command, argument))
            except Exception as error:
                serial_reply(request_id, "ERR", error)
        elif len(serial_buffer) < 512:
            serial_buffer += character
        else:
            serial_buffer = ""


ap = None
server = None
wifi_error = "NONE"
try:
    ap = network.WLAN(network.AP_IF)
    ap.active(True)
    ap.config(essid=SSID, password=PASSWORD)
    wifi_deadline = time.ticks_add(time.ticks_ms(), 2000)
    while (
        not ap.active()
        and time.ticks_diff(wifi_deadline, time.ticks_ms()) > 0
    ):
        time.sleep_ms(50)
    if not ap.active():
        raise RuntimeError("AP START TIMEOUT")
    server = socket.socket()
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.bind(("0.0.0.0", 80))
    server.listen(3)
    server.settimeout(0)
except Exception as error:
    wifi_error = str(error)
    try:
        if server is not None:
            server.close()
    except Exception:
        pass
    server = None

print("BLD750 + AS5047P + RA-02 E-STOP FULL BENCH TEST READY")
print("USB serial control: @AKCMD <id> <command> [argument]")
if server is not None:
    print("Wi-Fi:", SSID, "Password:", PASSWORD)
    print("Open: http://" + ap.ifconfig()[0])
else:
    print("Wi-Fi fallback OFFLINE:", wifi_error)
print("GP8630N I2C(1): SDA7 SCL21 | BLD: BRK13 EN14 F/R15")
print(
    "Steering hold: AS5600 SDA1 SCL2 | STEP4 DIR5 | center {} | {}".format(
        STEERING_CENTER_RAW,
        "ENABLED" if STEERING_CENTER_HOLD_ENABLED else "DISABLED",
    )
)
print("AS5047P: CS16 SCK18 MOSI41 MISO42")
print("RA-02: CS9 SCK12 MOSI11 MISO10 RESET8 DIO0=17")
print(
    "Mechanical brake: STEP38 DIR39 ENA40 | apply {:.1f} deg | {}".format(
        MECHANICAL_BRAKE_APPLY_DEG,
        "ENABLED" if MECHANICAL_BRAKE_ENABLED else "DISABLED",
    )
)
print("E-STOP: WAITING FOR ESTOP,0 CLEAR; STARTUP IS LOCKED")
print("Encoder:", encoder["magnet"], "raw", encoder["raw"])
if encoder_bus_error != "NONE":
    print("AS5047P BUS OFFLINE:", encoder_bus_error)
if not mechanical_brake_ready:
    print("MECHANICAL BRAKE PIN ERROR:", mechanical_brake_error)

try:
    while True:
        # Radio is serviced before traction and host commands so a received
        # STOP reaches the immediate lock path first.
        service_lora_estop()
        service_lora_watchdog()
        service_encoder()
        service_steering_hold()
        if (
            armed and STEERING_CENTER_HOLD_ENABLED
            and steering["state"] == "FAULT"
        ):
            lock_wheel("STEERING HOLD FAULT: " + steering["fault"])
        service_traction()
        service_serial()

        if (
            CONTROL_LINK_TIMEOUT_MS > 0
            and armed and not link_timeout_active
            and time.ticks_diff(time.ticks_ms(), last_command_ms)
            > CONTROL_LINK_TIMEOUT_MS
        ):
            lock_wheel("CONTROL LINK TIMEOUT")
            link_timeout_active = True

        if server is None:
            time.sleep_us(100)
            continue

        try:
            client, _ = server.accept()
        except OSError:
            time.sleep_us(100)
            continue

        try:
            client.settimeout(HTTP_CLIENT_TIMEOUT_S)
            request = client.recv(1200).decode("utf-8", "ignore")
            first = request.split("\r\n", 1)[0].split(" ")
            path = first[1] if len(first) > 1 else "/bad"

            if path == "/":
                reply(client, HTML, content_type="text/html; charset=utf-8")
            elif path.startswith("/status"):
                reply(client, execute_control("STATUS"),
                      content_type="application/json; charset=utf-8")
            elif path.startswith("/arm"):
                reply(client, execute_control("ARM"))
            elif path.startswith("/test_start"):
                reply(client, execute_control(
                    "TEST_START", query(path, "v", "")
                ))
            elif path.startswith("/soft_stop"):
                reply(client, execute_control("SOFT_STOP"))
            elif path.startswith("/lock"):
                reply(client, execute_control("LOCK"))
            elif path.startswith("/disarm"):
                reply(client, execute_control("DISARM"))
            elif path.startswith("/drive"):
                requested = query(path, "d", "FORWARD")
                reply(client, execute_control("DRIVE", requested))
            elif path.startswith("/gear"):
                reply(client, execute_control("GEAR", query(path, "n", "0")))
            elif path.startswith("/zero"):
                reply(client, execute_control("ZERO"))
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
    safe_stop("PROGRAM EXIT")
    if server is not None:
        server.close()
