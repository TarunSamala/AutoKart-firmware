"""ESP32-S3 + Meukron AS5047P isolated SPI-only test.

Wiring:
  AS5047P 3.3V -> ESP32-S3 3V3
  AS5047P GND  -> ESP32-S3 GND
  AS5047P CS   -> ESP32-S3 GPIO16
  AS5047P CLK  -> ESP32-S3 GPIO18
  AS5047P MOSI -> ESP32-S3 GPIO41
  AS5047P MISO -> ESP32-S3 GPIO42
  W/PWM and I/PWM are not connected.

Commands: r, stream, zero, clear, status, help, q.
No configuration or OTP register is written.
"""

from machine import Pin, SPI
import time


SPI_SCK = 18
SPI_MOSI = 41
SPI_MISO = 42
SPI_CS = 16
SPI_BAUDRATE = 1_000_000

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


def odd_parity(value):
    return bool(bin(value & 0xFFFF).count("1") & 1)


def command_frame(register):
    frame = 0x4000 | (register & 0x3FFF)
    if odd_parity(frame):
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
    receive = bytearray(2)
    cs.value(0)
    spi.write_readinto(frame_bytes(frame), receive)
    cs.value(1)
    time.sleep_us(2)
    return (receive[0] << 8) | receive[1]


def read_register(register):
    # AS5047P responses are pipelined: issue the read command, then one NOP.
    transfer(command_frame(register))
    response = transfer(0x0000)
    return {
        "data": response & 0x3FFF,
        "error": bool(response & 0x4000),
        "parity_error": odd_parity(response),
        "frame": response,
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


def signed_counts(raw_count, zero_count):
    difference = (raw_count - zero_count) % COUNTS_PER_REVOLUTION
    if difference >= COUNTS_PER_REVOLUTION // 2:
        difference -= COUNTS_PER_REVOLUTION
    return difference


zero_count = 0


def read_sample():
    angle = read_register(REG_ANGLECOM)
    diaagc = read_register(REG_DIAAGC)
    magnitude = read_register(REG_MAG)
    errors = read_register(REG_ERRFL)
    raw = angle["data"]
    communication_error = any(
        result["error"] or result["parity_error"]
        for result in (angle, diaagc, magnitude)
    )
    return {
        "raw": raw,
        "absolute_degrees": raw * DEGREES_PER_COUNT,
        "calibrated_degrees": signed_counts(raw, zero_count) * DEGREES_PER_COUNT,
        "diaagc": diaagc["data"],
        "agc": diaagc["data"] & 0xFF,
        "magnitude": magnitude["data"],
        "errors": errors["data"],
        "communication_error": communication_error,
    }


def print_sample(sample):
    link = "SPI ERROR" if sample["communication_error"] else "SPI OK"
    print(
        "RAW {:5d}/16383 = {:8.3f} deg | "
        "CAL {:+8.3f} deg | MAG 0x{:04X} | AGC 0x{:02X} | "
        "ERR 0x{:04X} | {} | {}".format(
            sample["raw"],
            sample["absolute_degrees"],
            sample["calibrated_degrees"],
            sample["magnitude"],
            sample["agc"],
            sample["errors"],
            link,
            magnet_state(sample["diaagc"]),
        )
    )


def show_help():
    print("Commands:")
    print("  r / read  - one SPI reading")
    print("  stream    - continuous readings; Ctrl-C returns")
    print("  zero      - current position becomes calibrated 0 degrees")
    print("  clear     - clear runtime zero")
    print("  status    - wiring and one reading")
    print("  help      - show commands")
    print("  q         - stop")


print()
print("========================================")
print(" ESP32-S3 AS5047P SPI-ONLY TEST")
print("========================================")
print(
    "SCK GPIO{} | MOSI GPIO{} | MISO GPIO{} | CS GPIO{}".format(
        SPI_SCK,
        SPI_MOSI,
        SPI_MISO,
        SPI_CS,
    )
)
print("SPI mode 1 at", SPI_BAUDRATE, "Hz")
print("W/PWM and I/PWM are not used.")
print_sample(read_sample())
show_help()

while True:
    try:
        command = input("\nAS5047P SPI > ").strip().lower()

        if command in ("r", "read"):
            print_sample(read_sample())

        elif command == "stream":
            print("Streaming every 100 ms. Press Ctrl-C to return.")
            try:
                while True:
                    print_sample(read_sample())
                    time.sleep_ms(100)
            except KeyboardInterrupt:
                print("Streaming stopped.")

        elif command == "zero":
            sample = read_sample()
            if sample["communication_error"]:
                print("ZERO REJECTED: SPI communication error")
            elif magnet_state(sample["diaagc"]) != "MAGNET OK":
                print("ZERO REJECTED:", magnet_state(sample["diaagc"]))
            else:
                zero_count = sample["raw"]
                print("CALIBRATED: current position = 0.000 deg")

        elif command == "clear":
            zero_count = 0
            print("Runtime zero cleared.")

        elif command == "status":
            print(
                "SCK GPIO{} | MOSI GPIO{} | MISO GPIO{} | CS GPIO{}".format(
                    SPI_SCK,
                    SPI_MOSI,
                    SPI_MISO,
                    SPI_CS,
                )
            )
            print_sample(read_sample())

        elif command in ("help", "?"):
            show_help()

        elif command in ("q", "quit", "exit"):
            print("AS5047P SPI test stopped.")
            break

        elif command:
            print("Unknown command. Type help.")

    except KeyboardInterrupt:
        print("\nUse q to stop or help for commands.")
    except Exception as error:
        print("SPI READ ERROR:", error)
