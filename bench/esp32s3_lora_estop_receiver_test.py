"""ESP32-S3 RA-02 receive-only E-stop link test.

This intentionally does not import network and does not control the BLDC.
It only verifies that packets from the Pico 2 arrive over LoRa.
"""

from machine import Pin, SPI
import time
from ra02_lora_chat_common import SX1278


# Alternate ESP32-S3 pins: BLDC GPIO13/14/15 remain untouched.
spi = SPI(
    2,
    baudrate=2000000,
    polarity=0,
    phase=0,
    sck=Pin(4),
    mosi=Pin(5),
    miso=Pin(6),
)
cs = Pin(7, Pin.OUT, value=1)
reset = Pin(10, Pin.OUT, value=1)
dio0 = Pin(18, Pin.IN)
radio = SX1278(spi, cs, reset, 433000000)


radio.begin()
print("ESP32-S3 LORA E-STOP RECEIVER TEST READY")
print("Waiting for ESTOP,0 and ESTOP,1 packets...")

while True:
    packet = radio.receive()
    if packet:
        try:
            print("RX:", packet.decode().strip())
        except Exception:
            print("RX BYTES:", packet)
    time.sleep_ms(5)
