"""Pico 2 one-shot/edge RA-02 E-stop link test.

Startup: sends ESTOP,0 once when the LAY37 is released.
Button press: sends ESTOP,1 once.
Button release: sends ESTOP,0 once.
No heartbeat is transmitted.
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


def send_state(state):
    global sequence
    sequence = (sequence + 1) & 0xFFFF
    packet = "ESTOP,{},{}".format(state, sequence)
    radio.send(packet)
    print("TX:", packet)


radio.begin()
sequence = 0
last_state = estop.value()

print("PICO 2 ONE-SHOT E-STOP TEST READY")
print("GP22=0 released/clear, GP22=1 pressed/stop")

# Startup packet: clear only when the switch is actually released.
send_state(1 if last_state else 0)

while True:
    state = estop.value()
    if state != last_state:
        last_state = state
        try:
            send_state(1 if state else 0)
        except Exception as error:
            print("TX ERROR:", error)
    time.sleep_ms(25)
