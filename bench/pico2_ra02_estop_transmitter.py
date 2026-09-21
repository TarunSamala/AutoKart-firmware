"""Pico 2 RA-02 fail-stop heartbeat transmitter.

GP22 LOW  = LAY37 NC contact closed, healthy
GP22 HIGH = switch pressed or wire open, STOP
"""

from machine import Pin, SPI
import time
from ra02_lora_chat_common import SX1278


spi = SPI(
    0,
    baudrate=2000000,
    polarity=0,
    phase=0,
    sck=Pin(18),
    mosi=Pin(19),
    miso=Pin(16),
)

cs = Pin(17, Pin.OUT, value=1)
reset = Pin(20, Pin.OUT, value=1)
estop = Pin(22, Pin.IN, Pin.PULL_UP)

radio = SX1278(spi, cs, reset, 433000000)
radio.begin()

print("PICO 2 LORA E-STOP TRANSMITTER READY")
print("GP22 LOW=HEALTHY, HIGH=STOP/FAULT")

sequence = 0
while True:
    sequence = (sequence + 1) & 0xFFFF
    state = 1 if estop.value() else 0
    packet = "ESTOP,{},{}".format(state, sequence)
    try:
        radio.send(packet)
        print("TX:", packet)
    except Exception as error:
        print("RADIO TX ERROR:", error)
    time.sleep_ms(200)
