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
print("GP22 LOW=RELEASED, HIGH=PRESSED/STOP")

sequence = 0
last_state = None
clear_repeats = 0
while True:
    state = 1 if estop.value() else 0
    # Healthy state is not a heartbeat. Send ESTOP,0 only on startup or after
    # release, and repeat that clear a few times so one lost packet cannot
    # leave the red UI latched. Repeat ESTOP,1 while the switch is pressed.
    if state != last_state:
        last_state = state
        clear_repeats = 5 if state == 0 else 0

    if state == 1 or clear_repeats > 0:
        sequence = (sequence + 1) & 0xFFFF
        packet = "ESTOP,{},{}".format(state, sequence)
        try:
            radio.send(packet)
            print("TX:", packet)
            if state == 0:
                clear_repeats -= 1
        except Exception as error:
            print("RADIO TX ERROR:", error)
        time.sleep_ms(200 if state == 1 else 50)
    else:
        time.sleep_ms(50)
