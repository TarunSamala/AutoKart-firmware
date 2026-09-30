"""AS5047P temporary W/PWM enable + isolated PWM readout.

This remains independent of braking, steering, TB6600 drivers, and motors.
Unlike as5047p_pwm_isolated_test.py, this file uses SPI once at startup to
enable the AS5047P PWM output in volatile configuration, then reads W/PWM.
The setting is lost after a sensor reset/power cycle; no OTP is programmed.

Meukron/adapter-board wiring:
  5V or 3V3      -> board supply selected for the breakout
  GND            -> ESP32-S3 GND
  CSn            -> ESP32-S3 GPIO16
  CLK            -> ESP32-S3 GPIO13
  MOSI           -> ESP32-S3 GPIO14
  MISO           -> ESP32-S3 GPIO15
  W/PWM          -> ESP32-S3 GPIO4

Use a 3.3 V logic-level PWM output. Do not connect a 5 V signal to GPIO4.
SETTINGS1 = 0x0081 means factory bit set, W/PWM selected, and PWMon enabled.
"""

from machine import Pin, SPI, time_pulse_us
import time


SPI_SCK = 13
SPI_MOSI = 14
SPI_MISO = 15
SPI_CS = 16
PWM_PIN = 4

SPI_BAUDRATE = 1000000
SETTINGS1 = 0x0018
SETTINGS1_W_PWM_ON = 0x0081
TIMEOUT_US = 100000
PWM_CLOCK_US = 0.444
INIT_CLOCKS = 12
DATA_CLOCKS = 4095


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
    high_us = time_pulse_us(pin, 1, TIMEOUT_US)
    low_us = time_pulse_us(pin, 0, TIMEOUT_US)
    if high_us < 0 or low_us < 0:
        raise RuntimeError(
            "PWM edge timeout (high={}, low={})".format(high_us, low_us)
        )

    period_us = high_us + low_us
    frequency_hz = 1000000.0 / period_us if period_us else 0.0
    duty_percent = 100.0 * high_us / period_us if period_us else 0.0
    raw = int(round(high_us / PWM_CLOCK_US)) - INIT_CLOCKS
    raw = max(0, min(DATA_CLOCKS, raw))
    angle = raw * 360.0 / DATA_CLOCKS
    return high_us, low_us, period_us, frequency_hz, duty_percent, raw, angle


def print_pwm():
    high_us, low_us, period_us, frequency_hz, duty, raw, angle = capture_pwm()
    print(
        "HIGH {:6d} us | LOW {:6d} us | PERIOD {:7d} us | "
        "FREQ {:7.1f} Hz | DUTY {:6.2f}%".format(
            high_us, low_us, period_us, frequency_hz, duty
        )
    )
    print("RAW PWM: {:4d}/4095 | ANGLE: {:8.3f} deg".format(raw, angle))


print("\nESP32-S3 AS5047P W/PWM ENABLE + READ TEST")
print("No braking, steering, TB6600, or motor control is used.")
print("Configuring SETTINGS1 = 0x{:04X}".format(SETTINGS1_W_PWM_ON))
write_register(SETTINGS1, SETTINGS1_W_PWM_ON)
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
