"""ESP32-S3 direct AS5047P SPI encoder test.

Temporary test wiring:
  AS5047P VCC -> ESP32-S3 3V3 (or the module's regulated 3V3 input)
  AS5047P GND -> ESP32-S3 GND
  AS5047P CLK -> ESP32-S3 GPIO13
  AS5047P MOSI -> ESP32-S3 GPIO14
  AS5047P MISO -> ESP32-S3 GPIO15
  AS5047P CSn -> ESP32-S3 GPIO16

SPI mode 1 is used. The sensor magnet must be centered over the IC.
"""

from machine import Pin, SPI
import time


SCK_PIN = 13
MOSI_PIN = 14
MISO_PIN = 15
CS_PIN = 16
SPI_FREQ = 1000000

REG_NOP = 0x0000
REG_ANGLECOM = 0x3FFF
REG_MAG = 0x3FFD
REG_ERRFL = 0x0001


def has_odd_parity(value):
    ones = 0
    for bit in range(16):
        ones += (value >> bit) & 1
    return (ones & 1) == 1


def make_read_command(register):
    command = 0x4000 | (register & 0x3FFF)
    if has_odd_parity(command):
        command |= 0x8000
    return command


spi = SPI(
    1,
    baudrate=SPI_FREQ,
    polarity=0,
    phase=1,
    bits=8,
    firstbit=SPI.MSB,
    sck=Pin(SCK_PIN),
    mosi=Pin(MOSI_PIN),
    miso=Pin(MISO_PIN),
)
cs = Pin(CS_PIN, Pin.OUT, value=1)


def transfer(word):
    tx = bytearray([(word >> 8) & 0xFF, word & 0xFF])
    rx = bytearray(2)
    cs.value(0)
    spi.write_readinto(tx, rx)
    cs.value(1)
    return (rx[0] << 8) | rx[1]


def read_register(register):
    transfer(make_read_command(register))
    return transfer(REG_NOP)


print("\nESP32-S3 AS5047P DIRECT SPI TEST")
print("SCK", SCK_PIN, "MOSI", MOSI_PIN,
      "MISO", MISO_PIN, "CS", CS_PIN)
print("SPI mode 1 at", SPI_FREQ, "Hz")
print("Rotate the magnet; ANGLE should change from 0 to 360 degrees.\n")

while True:
    angle_word = read_register(REG_ANGLECOM)
    angle = angle_word & 0x3FFF
    angle_deg = angle * 360.0 / 16384.0

    mag_word = read_register(REG_MAG)
    err_word = read_register(REG_ERRFL)

    print(
        "ANGLE: {:5d} ({:7.2f} deg) | "
        "MAG: 0x{:04X} | ERR: 0x{:04X}".format(
            angle, angle_deg, mag_word, err_word
        )
    )
    time.sleep_ms(100)

