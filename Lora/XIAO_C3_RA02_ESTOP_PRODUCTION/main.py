from machine import Pin
import time
from lora_driver import LoRa
from packet_crc import build_packet
from watchdog import Watchdog

ESTOP_PIN = Pin(6, Pin.IN)
LED = Pin(21, Pin.OUT)   # XIAO onboard LED (verify on board revision)

TX_INTERVAL_MS = 50

radio = LoRa()
wd = Watchdog(8000)

sequence = 0
last_state = None

def read_estop():
    # NC switch:
    # LOW = healthy/released
    # HIGH = pressed or broken wire
    return 1 if ESTOP_PIN.value() else 0

radio.begin()

while True:
    wd.feed()

    state = read_estop()

    packet = build_packet(
        device="XIAO01",
        state=state,
        sequence=sequence,
        uptime_ms=time.ticks_ms()
    )

    radio.send(packet)

    if state:
        LED.value(0)
    else:
        LED.value(1)

    sequence = (sequence + 1) & 0xFFFF
    time.sleep_ms(TX_INTERVAL_MS)
