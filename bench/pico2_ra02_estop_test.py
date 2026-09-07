"""Pico 2 + RA-02 + LAY37 emergency-stop bench test.

Wiring is documented in docs/PICO2_RA02_ESTOP_CONNECTIONS.md.
This test does not transmit radio packets. It only verifies the RA-02 SPI
identity and reports the fail-stop input continuously.
"""

from machine import Pin, SPI
import time


# RA-02 (SX1278) SPI pins
SPI_SCK = 18
SPI_MOSI = 19
SPI_MISO = 16
RA02_CS = 17
RA02_RESET = 20
RA02_DIO0 = 21

# LAY37 NC contact: closed/LOW is the healthy state; open/HIGH is STOP.
ESTOP_PIN = 22


spi = SPI(
    0,
    baudrate=1_000_000,
    polarity=0,
    phase=0,
    bits=8,
    firstbit=SPI.MSB,
    sck=Pin(SPI_SCK),
    mosi=Pin(SPI_MOSI),
    miso=Pin(SPI_MISO),
)
cs = Pin(RA02_CS, Pin.OUT, value=1)
reset = Pin(RA02_RESET, Pin.OUT, value=1)
dio0 = Pin(RA02_DIO0, Pin.IN)
estop = Pin(ESTOP_PIN, Pin.IN, Pin.PULL_UP)


def read_register(address):
    tx = bytearray((address & 0x7F, 0x00))
    rx = bytearray(2)
    cs.value(0)
    spi.write_readinto(tx, rx)
    cs.value(1)
    return rx[1]


def reset_ra02():
    reset.value(0)
    time.sleep_ms(10)
    reset.value(1)
    time.sleep_ms(10)


print("PICO 2 RA-02 + LAY37 FAIL-STOP TEST")
print("SPI SCK/MOSI/MISO = GP18/GP19/GP16")
print("RA-02 CS/RESET/DIO0 = GP17/GP20/GP21")
print("LAY37 NC input = GP22 (LOW=OK, HIGH=STOP/FAULT)")

reset_ra02()
version = read_register(0x42)  # SX127x RegVersion; RA-02/SX1278 normally 0x12
print("RA-02 RegVersion: 0x{:02X}".format(version))
if version == 0x12:
    print("RA-02 SPI: FOUND")
else:
    print("RA-02 SPI: NOT CONFIRMED; check 3V3, GND, CS, and SPI wiring")

last_state = None
while True:
    stopped = estop.value() == 1
    state = "STOP/FAULT" if stopped else "OK / NC CLOSED"
    if state != last_state:
        print("ESTOP: {} | GPIO22={}".format(state, estop.value()))
        last_state = state
    time.sleep_ms(100)
