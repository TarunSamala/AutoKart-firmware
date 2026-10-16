"""AS5047P temporary W/PWM enable + isolated PWM readout.

This remains independent of braking, steering, TB6600 drivers, and motors.
Unlike as5047p_pwm_isolated_test.py, this file uses SPI once at startup to
enable the AS5047P PWM output in volatile configuration, then reads W/PWM.
The setting is lost after a sensor reset/power cycle; no OTP is programmed.

Meukron/adapter-board wiring:
  5V or 3V3      -> board supply selected for the breakout
  GND            -> ESP32-S3 GND
  CSn            -> ESP32-S3 GPIO16
  CLK            -> ESP32-S3 GPIO18
  MOSI           -> ESP32-S3 GPIO41
  MISO           -> ESP32-S3 GPIO42
  W/PWM          -> ESP32-S3 GPIO7

Use a 3.3 V logic-level PWM output. Do not connect a 5 V signal to GPIO7.
The existing SETTINGS1 value is preserved, PWMon is enabled, and UVW_ABI is
cleared to route PWM to W/PWM. No OTP is programmed.
"""

from machine import Pin, SPI, time_pulse_us
import time


SPI_SCK = 18
SPI_MOSI = 41
SPI_MISO = 42
SPI_CS = 16
PWM_PIN = 7

SPI_BAUDRATE = 1000000
SETTINGS1 = 0x0018
PW_MON_MASK = 0x0080
UVW_ABI_MASK = 0x0008
TIMEOUT_US = 100000
PWM_CLOCK_US = 0.444
# A normal PWM high interval contains 12 INIT clocks and four healthy
# error-status clocks before the angle-data clocks.
PWM_HIGH_OVERHEAD_CLOCKS = 16
DATA_CLOCKS = 4095
MIN_VALID_HIGH_US = 6
MAX_VALID_HIGH_US = 1900
PWM_SAMPLES = 7
REG_ANGLECOM = 0x3FFF
REG_DIAAGC = 0x3FFC
REG_MAG = 0x3FFD

DIAAGC_MAGL = 1 << 11
DIAAGC_MAGH = 1 << 10
DIAAGC_COF = 1 << 9
DIAAGC_LF = 1 << 8


def even_parity(frame):
    """Set bit 15 so the complete 16-bit frame has even parity."""
    frame &= 0x7FFF
    if bin(frame).count("1") & 1:
        frame |= 0x8000
    return frame


def spi_frame(frame):
    return bytes(((frame >> 8) & 0xFF, frame & 0xFF))


cs = Pin(SPI_CS, Pin.OUT, value=1)
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


def transfer(frame):
    tx = spi_frame(frame)
    rx = bytearray(2)
    cs.value(0)
    spi.write_readinto(tx, rx)
    cs.value(1)
    time.sleep_us(2)
    return (rx[0] << 8) | rx[1]


def read_register(register):
    command = even_parity(0x4000 | (register & 0x3FFF))
    transfer(command)
    return transfer(0x0000) & 0x3FFF


def write_register(register, value):
    command = even_parity(register & 0x3FFF)       # RW=0: write
    data = even_parity(value & 0x3FFF)

    cs.value(0)
    spi.write(spi_frame(command))
    cs.value(1)
    time.sleep_us(2)

    cs.value(0)
    spi.write(spi_frame(data))
    cs.value(1)
    time.sleep_us(2)


def capture_pwm():
    pin = Pin(PWM_PIN, Pin.IN)
    valid_highs = []
    rejected = 0

    # time_pulse_us() calls made separately for HIGH and LOW can lock onto
    # different frames when Python resumes between calls. Angle information is
    # entirely in the HIGH width, so collect and median-filter HIGH pulses only.
    for _ in range(PWM_SAMPLES * 4):
        high_us = time_pulse_us(pin, 1, TIMEOUT_US)
        if MIN_VALID_HIGH_US <= high_us <= MAX_VALID_HIGH_US:
            valid_highs.append(high_us)
            if len(valid_highs) >= PWM_SAMPLES:
                break
        else:
            rejected += 1

    if not valid_highs:
        raise RuntimeError("no valid W/PWM high pulse detected")

    valid_highs.sort()
    high_us = valid_highs[len(valid_highs) // 2]
    raw = (
        int(round(high_us / PWM_CLOCK_US))
        - PWM_HIGH_OVERHEAD_CLOCKS
    )
    raw = max(0, min(DATA_CLOCKS, raw))
    angle = raw * 360.0 / DATA_CLOCKS
    return high_us, raw, angle, len(valid_highs), rejected


def read_magnet_diagnostics():
    diaagc = read_register(REG_DIAAGC)
    magnitude = read_register(REG_MAG)
    agc = diaagc & 0xFF
    mag_low = bool(diaagc & DIAAGC_MAGL)
    mag_high = bool(diaagc & DIAAGC_MAGH)
    cordic_overflow = bool(diaagc & DIAAGC_COF)
    loops_finished = bool(diaagc & DIAAGC_LF)

    if mag_high:
        status = "MAGNET TOO STRONG"
    elif mag_low:
        # AS5047P has one field-too-low flag. It cannot reliably distinguish
        # a distant/weak magnet from no magnet using a separate status bit.
        status = "MAGNET TOO WEAK / NO MAGNET"
    elif cordic_overflow:
        status = "MAGNET INVALID / CORDIC OVERFLOW"
    elif not loops_finished:
        status = "MAGNET NOT READY"
    else:
        status = "MAGNET OK"

    return status, agc, magnitude, diaagc


def print_pwm():
    high_us, raw, angle, accepted, rejected = capture_pwm()
    spi_raw = read_register(REG_ANGLECOM) & 0x3FFF
    spi_angle = spi_raw * 360.0 / 16384.0
    difference = (angle - spi_angle + 180.0) % 360.0 - 180.0
    magnet_status, agc, magnitude, diaagc = read_magnet_diagnostics()
    print(
        "PWM HIGH {:4d} us | RAW {:4d}/4095 | ANGLE {:8.3f} deg | "
        "samples {}/{}".format(
            high_us, raw, angle, accepted, accepted + rejected
        )
    )
    print(
        "SPI RAW {:5d}/16383 | ANGLE {:8.3f} deg | "
        "PWM-SPI DELTA {:+7.3f} deg".format(
            spi_raw, spi_angle, difference
        )
    )
    print(
        "{} | AGC 0x{:02X} | MAG 0x{:04X} | DIAAGC 0x{:04X}".format(
            magnet_status, agc, magnitude, diaagc
        )
    )


print("\nESP32-S3 AS5047P W/PWM ENABLE + READ TEST")
print("No braking, steering, TB6600, or motor control is used.")
print(
    "SPI: SCK GPIO{} MOSI GPIO{} MISO GPIO{} CS GPIO{}".format(
        SPI_SCK, SPI_MOSI, SPI_MISO, SPI_CS
    )
)

settings_before = read_register(SETTINGS1)
settings_target = (settings_before | PW_MON_MASK) & ~UVW_ABI_MASK
print("SETTINGS1 before: 0x{:04X}".format(settings_before))
print("Writing SETTINGS1: 0x{:04X}".format(settings_target))
write_register(SETTINGS1, settings_target)
settings_after = read_register(SETTINGS1)
print("SETTINGS1 after : 0x{:04X}".format(settings_after))

if (settings_after & PW_MON_MASK) == 0:
    raise RuntimeError("PWMon did not enable; check SPI wiring")

if settings_after & UVW_ABI_MASK:
    raise RuntimeError("PWM is routed to I/PWM, not W/PWM")

print("PWM enabled on W/PWM; output pin is ESP32-S3 GPIO{}".format(PWM_PIN))
print("Type r for one reading, stream for continuous data, or q to stop.")

while True:
    try:
        command = input("\nAS5047P > ").strip().lower()
        if command == "r":
            print_pwm()
        elif command == "stream":
            try:
                while True:
                    print_pwm()
                    time.sleep_ms(100)
            except KeyboardInterrupt:
                print("\nStream stopped.")
        elif command == "q":
            print("AS5047P test stopped.")
            break
        elif command:
            print("Use r, stream, or q.")
    except KeyboardInterrupt:
        print("\nUse q to stop.")
    except Exception as error:
        print("PWM read error:", error)
