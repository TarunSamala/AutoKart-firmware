"""Pico 2 + Meukron AS5047P isolated SPI filtered test.

Wiring:
  AS5047P 3.3V -> Pico 2 3V3(OUT), physical pin 36
  AS5047P GND  -> Pico 2 GND, physical pin 38
  AS5047P MISO -> Pico 2 GP16, physical pin 21
  AS5047P CS   -> Pico 2 GP17, physical pin 22
  AS5047P CLK  -> Pico 2 GP18, physical pin 24
  AS5047P MOSI -> Pico 2 GP19, physical pin 25
  W/PWM and I/PWM are not connected.

Disconnect the RA-02 because this uses the same Pico SPI pins. Copy
firmware/encoder_filters.py to /encoder_filters.py before installing this file
as /main.py. No AS5047P configuration or OTP register is written.
"""

from machine import Pin, SPI
import time

from encoder_filters import AS5047KinematicsFilter, monotonic_us


SPI_ID = 0
SPI_MISO = 16
SPI_CS = 17
SPI_SCK = 18
SPI_MOSI = 19
SPI_BAUDRATE = 1_000_000

SAMPLE_INTERVAL_MS = 5
DISPLAY_INTERVAL_MS = 250
MAXIMUM_MOTOR_RPM = 6000.0

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
    SPI_ID,
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
    # AS5047P responses are pipelined: response arrives during the next frame.
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


kinematics = AS5047KinematicsFilter(
    motor_revolutions_per_axle_revolution=1.0,
    wheel_circumference_m=1.0,
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
        "angle_deg": angle["data"] * DEGREES_PER_COUNT,
        "agc": diaagc["data"] & 0xFF,
        "magnitude": magnitude["data"],
        "errors": errors["data"],
        "communication_ok": communication_ok,
        "magnet_state": state,
    }
    if filtered is not None:
        last_filtered = filtered


def print_latest():
    if last_diagnostics is None:
        print("No sample available.")
        return

    diagnostic = last_diagnostics
    link = "SPI OK" if diagnostic["communication_ok"] else "SPI ERROR"
    print(
        "RAW {:5d}/16383 | ANGLE {:8.3f} deg | MAG 0x{:04X} | "
        "AGC 0x{:02X} | ERR 0x{:04X} | {} | {}".format(
            diagnostic["raw"],
            diagnostic["angle_deg"],
            diagnostic["magnitude"],
            diagnostic["agc"],
            diagnostic["errors"],
            link,
            diagnostic["magnet_state"],
        )
    )

    if last_filtered is None:
        print(
            "FILTER: waiting/rejected | valid {} | rejected {}".format(
                kinematics.valid_samples, kinematics.rejected_samples
            )
        )
        return

    turns = last_filtered["total_motor_counts"] / COUNTS_PER_REVOLUTION
    print(
        "FILTERED RPM {:+9.2f} | RAW RPM {:+9.2f} | "
        "MOTOR TURNS {:+10.4f} | COUNTS {:+d} | valid {} rejected {}".format(
            last_filtered["motor_rpm"],
            last_filtered["raw_motor_rpm"],
            turns,
            last_filtered["total_motor_counts"],
            last_filtered["valid_samples"],
            last_filtered["rejected_samples"],
        )
    )


def collect(duration_ms):
    started = time.ticks_ms()
    while time.ticks_diff(time.ticks_ms(), started) < duration_ms:
        acquire_sample()
        time.sleep_ms(SAMPLE_INTERVAL_MS)


def show_status():
    print("MISO GP16 | CS GP17 | SCK GP18 | MOSI GP19")
    print("SPI mode 1 at {} Hz".format(SPI_BAUDRATE))
    print("Filter: 200 Hz, 30 ms RPM LPF, maximum {:.0f} RPM".format(
        MAXIMUM_MOTOR_RPM
    ))
    print("Vehicle speed/distance: NOT CALIBRATED")
    print_latest()


def show_help():
    print("Commands:")
    print("  read / r - sample for 1 second and show filtered result")
    print("  stream   - continuous sampling; Ctrl-C returns")
    print("  zero     - reset cumulative turns at current position")
    print("  status   - show SPI/filter status and latest result")
    print("  help     - show commands")
    print("  q        - stop")


print()
print("========================================")
print(" PICO 2 + AS5047P FILTERED SPI TEST")
print("========================================")
print("ENCODER ONLY: NO MOTOR OUTPUTS ARE INITIALIZED")
collect(250)
show_status()
show_help()

while True:
    try:
        command = input("\nAS5047P > ").strip().lower()

        if command in ("r", "read"):
            collect(1000)
            print_latest()

        elif command == "stream":
            print("Sampling at 200 Hz; displaying at 4 Hz. Ctrl-C returns.")
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
            print("ZERO COMPLETE: cumulative motor turns = 0")
            print_latest()

        elif command == "status":
            collect(250)
            show_status()

        elif command in ("help", "?"):
            show_help()

        elif command in ("q", "quit", "exit"):
            print("AS5047P test stopped.")
            break

        elif command:
            print("Unknown command. Type help.")

    except KeyboardInterrupt:
        print("\nUse q to stop or help for commands.")
    except Exception as error:
        print("AS5047P READ ERROR:", error)
