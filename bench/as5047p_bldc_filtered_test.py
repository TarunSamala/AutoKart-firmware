"""Isolated ESP32-S3 + AS5047P BLDC encoder test with standard filters.

This test does not initialize or command the BLD750, GP8630N, brake, steering,
or LoRa hardware. It only reads the AS5047P over SPI.

Wiring:
  AS5047P 3.3V -> ESP32-S3 3V3
  AS5047P GND  -> ESP32-S3 GND
  AS5047P CS   -> ESP32-S3 GPIO16
  AS5047P CLK  -> ESP32-S3 GPIO18
  AS5047P MOSI -> ESP32-S3 GPIO41
  AS5047P MISO -> ESP32-S3 GPIO42
  W/PWM and I/PWM are not connected.

Copy firmware/encoder_filters.py to /encoder_filters.py on the ESP32-S3 before
copying this file to /main.py.

Commands: read, stream, zero, clear, status, help, q.
No AS5047P configuration or OTP register is written.
"""

from machine import Pin, SPI
import time

from encoder_filters import AS5047KinematicsFilter, monotonic_us


SPI_SCK = 18
SPI_MOSI = 41
SPI_MISO = 42
SPI_CS = 16
SPI_BAUDRATE = 1_000_000

SAMPLE_INTERVAL_MS = 5       # 200 Hz acquisition
DISPLAY_INTERVAL_MS = 250    # 4 terminal updates per second
MAXIMUM_MOTOR_RPM = 6000.0

# These values intentionally produce motor-side measurements only. Set the
# real reduction and wheel circumference later before trusting axle speed,
# vehicle speed, or distance.
TEST_REDUCTION = 1.0
TEST_CIRCUMFERENCE_M = 1.0

REG_ERRFL = 0x0001
REG_DIAAGC = 0x3FFC
REG_MAG = 0x3FFD
REG_ANGLECOM = 0x3FFF

DIAAGC_MAGL = 1 << 11
DIAAGC_MAGH = 1 << 10
DIAAGC_COF = 1 << 9
DIAAGC_LF = 1 << 8

COUNTS_PER_REVOLUTION = 16384
DEGREES_PER_COUNT = 360.0 / COUNTS_PER_REVOLUTION


def parity_is_odd(value):
    return bool(bin(value & 0xFFFF).count("1") & 1)


def command_frame(register):
    frame = 0x4000 | (register & 0x3FFF)
    if parity_is_odd(frame):
        frame |= 0x8000
    return frame


def frame_bytes(frame):
    return bytes(((frame >> 8) & 0xFF, frame & 0xFF))


spi = SPI(
    1,
    baudrate=SPI_BAUDRATE,
    polarity=0,
    phase=1,
    bits=8,
    firstbit=SPI.MSB,
    sck=Pin(SPI_SCK),
    mosi=Pin(SPI_MOSI),
    miso=Pin(SPI_MISO),
)
cs = Pin(SPI_CS, Pin.OUT, value=1)


def transfer(frame):
    received = bytearray(2)
    cs.value(0)
    spi.write_readinto(frame_bytes(frame), received)
    cs.value(1)
    time.sleep_us(2)
    return (received[0] << 8) | received[1]


def read_register(register):
    # AS5047P responses are pipelined: command first, response on the NOP.
    transfer(command_frame(register))
    response = transfer(0x0000)
    return {
        "data": response & 0x3FFF,
        "sensor_error": bool(response & 0x4000),
        "parity_error": parity_is_odd(response),
    }


def magnet_state(diaagc):
    if diaagc & DIAAGC_MAGH:
        return "MAGNET TOO STRONG"
    if diaagc & DIAAGC_MAGL:
        return "MAGNET TOO WEAK / NO MAGNET"
    if diaagc & DIAAGC_COF:
        return "MAGNET INVALID / CORDIC OVERFLOW"
    if not diaagc & DIAAGC_LF:
        return "MAGNET NOT READY"
    return "MAGNET OK"


def ticks_ms_difference(current, previous):
    if hasattr(time, "ticks_diff"):
        return time.ticks_diff(current, previous)
    return current - previous


def now_ms():
    if hasattr(time, "ticks_ms"):
        return time.ticks_ms()
    return int(time.monotonic() * 1000)


kinematics = AS5047KinematicsFilter(
    motor_revolutions_per_axle_revolution=TEST_REDUCTION,
    wheel_circumference_m=TEST_CIRCUMFERENCE_M,
    time_constant_s=0.030,
    maximum_motor_rpm=MAXIMUM_MOTOR_RPM,
)

last_diagnostics = None
last_filtered = None


def acquire_sample():
    global last_diagnostics, last_filtered

    angle = read_register(REG_ANGLECOM)
    diaagc = read_register(REG_DIAAGC)
    magnitude = read_register(REG_MAG)
    errors = read_register(REG_ERRFL)

    state = magnet_state(diaagc["data"])
    communication_ok = not any(
        item["sensor_error"] or item["parity_error"]
        for item in (angle, diaagc, magnitude)
    )
    sample_ok = communication_ok and state == "MAGNET OK"

    filtered = kinematics.update(
        angle["data"],
        timestamp_us=monotonic_us(),
        sample_ok=sample_ok,
    )
    last_diagnostics = {
        "raw": angle["data"],
        "absolute_angle_deg": angle["data"] * DEGREES_PER_COUNT,
        "diaagc": diaagc["data"],
        "agc": diaagc["data"] & 0xFF,
        "magnitude": magnitude["data"],
        "error_flags": errors["data"],
        "communication_ok": communication_ok,
        "magnet_state": state,
        "sample_ok": sample_ok,
    }
    if filtered is not None:
        last_filtered = filtered
    return filtered


def print_latest():
    diagnostic = last_diagnostics
    filtered = last_filtered
    if diagnostic is None:
        print("No sample available.")
        return

    link = "SPI OK" if diagnostic["communication_ok"] else "SPI ERROR"
    print(
        "RAW {:5d} | ANGLE {:8.3f} deg | MAG 0x{:04X} | AGC 0x{:02X} | "
        "ERR 0x{:04X} | {} | {}".format(
            diagnostic["raw"],
            diagnostic["absolute_angle_deg"],
            diagnostic["magnitude"],
            diagnostic["agc"],
            diagnostic["error_flags"],
            link,
            diagnostic["magnet_state"],
        )
    )
    if filtered is None:
        print(
            "FILTER: waiting/rejected | valid {} | rejected {}".format(
                kinematics.valid_samples,
                kinematics.rejected_samples,
            )
        )
        return

    motor_revolutions = (
        filtered["total_motor_counts"] / COUNTS_PER_REVOLUTION
    )
    print(
        "FILTERED RPM {:+9.2f} | RAW RPM {:+9.2f} | "
        "MOTOR TURNS {:+10.4f} | COUNTS {:+d} | valid {} | rejected {}".format(
            filtered["motor_rpm"],
            filtered["raw_motor_rpm"],
            motor_revolutions,
            filtered["total_motor_counts"],
            filtered["valid_samples"],
            filtered["rejected_samples"],
        )
    )


def collect(duration_ms, display=False):
    started = now_ms()
    last_display = started
    while ticks_ms_difference(now_ms(), started) < duration_ms:
        acquire_sample()
        current = now_ms()
        if display and ticks_ms_difference(current, last_display) >= DISPLAY_INTERVAL_MS:
            print_latest()
            last_display = current
        time.sleep_ms(SAMPLE_INTERVAL_MS)


def show_help():
    print("Commands:")
    print("  read / r - sample for 1 second, then show filtered result")
    print("  stream   - continuous 200 Hz sampling; Ctrl-C returns")
    print("  zero     - reset cumulative turns/distance at current position")
    print("  clear    - same as zero; starts a fresh filter history")
    print("  status   - show wiring, filter settings, and latest result")
    print("  help     - show commands")
    print("  q        - stop")


def show_status():
    print(
        "SCK GPIO{} | MOSI GPIO{} | MISO GPIO{} | CS GPIO{}".format(
            SPI_SCK, SPI_MOSI, SPI_MISO, SPI_CS
        )
    )
    print(
        "Acquisition {} Hz | RPM LPF 30 ms | maximum accepted RPM {:.0f}".format(
            1000 // SAMPLE_INTERVAL_MS,
            MAXIMUM_MOTOR_RPM,
        )
    )
    print("Vehicle speed/distance: NOT CALIBRATED (reduction and tyre TBD)")
    print_latest()


print()
print("============================================")
print(" AUTOKART AS5047P BLDC FILTERED ENCODER TEST")
print("============================================")
print("ENCODER ONLY: BLDC OUTPUTS ARE NOT INITIALIZED")
collect(250)
show_status()
show_help()

while True:
    try:
        command = input("\nBLDC encoder > ").strip().lower()

        if command in ("r", "read"):
            collect(1000)
            print_latest()

        elif command == "stream":
            print("Streaming at 200 Hz; displaying at 4 Hz. Press Ctrl-C to return.")
            try:
                while True:
                    collect(DISPLAY_INTERVAL_MS)
                    print_latest()
            except KeyboardInterrupt:
                print("Streaming stopped.")

        elif command in ("zero", "clear"):
            kinematics.reset()
            last_filtered = None
            collect(100)
            print("ZERO COMPLETE: cumulative motor turns = 0 at current position")
            print_latest()

        elif command == "status":
            collect(250)
            show_status()

        elif command in ("help", "?"):
            show_help()

        elif command in ("q", "quit", "exit"):
            print("AS5047P BLDC encoder test stopped.")
            break

        elif command:
            print("Unknown command. Type help.")

    except KeyboardInterrupt:
        print("\nUse q to stop or help for commands.")
    except Exception as error:
        print("ENCODER TEST ERROR:", error)
