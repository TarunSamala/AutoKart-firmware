"""Isolated XIAO ESP32-C3 + RA-02 + LAY37 hardware diagnostic.

This program does not transmit LoRa packets and controls no actuator.

Expected wiring:
  RA-02 SCK   -> XIAO D8  / GPIO8
  RA-02 MISO  -> XIAO D9  / GPIO9
  RA-02 MOSI  -> XIAO D10 / GPIO10
  RA-02 NSS   -> XIAO D3  / GPIO5
  RA-02 RESET -> XIAO D2  / GPIO4
  RA-02 DIO0  -> XIAO D1  / GPIO3
  LAY37 NC    -> XIAO D4  / GPIO6
  LAY37 COM   -> XIAO GND

LAY37 logic:
  LOW  = released / NC contact closed
  HIGH = pressed or NC wire open / STOP
"""

from machine import Pin, SPI
import time


SPI_SCK = 8
SPI_MISO = 9
SPI_MOSI = 10
RA02_CS = 5
RA02_RESET = 4
RA02_DIO0 = 3
ESTOP_PIN = 6

SX127X_REG_VERSION = 0x42
SX127X_EXPECTED_VERSION = 0x12


spi = SPI(
    1,
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


def reset_ra02():
    reset.value(0)
    time.sleep_ms(10)
    reset.value(1)
    time.sleep_ms(20)


def read_register(address):
    tx = bytearray((address & 0x7F, 0x00))
    rx = bytearray(2)

    cs.value(0)
    spi.write_readinto(tx, rx)
    cs.value(1)

    return rx[1]


print()
print("========================================")
print(" XIAO ESP32-C3 + RA-02 + LAY37 TEST")
print("========================================")
print("SPI: SCK=D8/GPIO8 MISO=D9/GPIO9 MOSI=D10/GPIO10")
print("RA-02: CS=D3/GPIO5 RESET=D2/GPIO4 DIO0=D1/GPIO3")
print("LAY37: NC=D4/GPIO6 COM=GND")

reset_ra02()
version = read_register(SX127X_REG_VERSION)

print("RA-02 RegVersion: 0x{:02X}".format(version))

if version == SX127X_EXPECTED_VERSION:
    print("RA-02 SPI: FOUND (SX1278)")
else:
    print("RA-02 SPI: NOT DETECTED")
    print("Expected 0x12; check 3V3, GND, SCK, MISO, MOSI, CS and RESET")

print()
print("LAY37 monitoring started")
print("Expected: released=LOW, pressed/open-wire=HIGH")
print("Press Ctrl-C to stop.")

last_state = None

try:
    while True:
        pin_value = estop.value()
        state = "STOP / PRESSED / WIRE OPEN" if pin_value else "RELEASED / NC CLOSED"

        if state != last_state:
            print("LAY37: {} | GPIO6={}".format(state, pin_value))
            last_state = state

        time.sleep_ms(50)

except KeyboardInterrupt:
    print("\nHardware test stopped")
