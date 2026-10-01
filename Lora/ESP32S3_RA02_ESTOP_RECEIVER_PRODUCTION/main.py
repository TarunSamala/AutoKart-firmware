# ESP32-S3 + RA-02 Production E-STOP Receiver
# Vehicle side receiver
#
# RA-02:
# SCK  GPIO4
# MOSI GPIO5
# MISO GPIO6
# CS   GPIO7
# RESET GPIO10
# DIO0 GPIO18

from machine import Pin
import time
from packet_crc import verify_packet
from watchdog import Watchdog
from lora_driver import LoRa

ESTOP_OUTPUT = Pin(15, Pin.OUT)   # connect to safety interface logic
STATUS_LED = Pin(48, Pin.OUT)     # verify board LED

TIMEOUT_MS = 300

radio = LoRa()
wd = Watchdog(5000)

state = "UNKNOWN"
last_valid_packet = 0
last_sequence = -1

def emergency_stop():
    ESTOP_OUTPUT.value(1)
    STATUS_LED.value(0)

def clear_stop():
    ESTOP_OUTPUT.value(0)
    STATUS_LED.value(1)

radio.begin()

while True:

    wd.feed()

    packet = radio.receive()

    if packet:

        if verify_packet(packet):

            fields = packet.split(",")

            rx_state = int(fields[1])
            seq = int(fields[2])

            last_valid_packet = time.ticks_ms()

            if seq != last_sequence:
                last_sequence = seq

            if rx_state == 1:
                state = "ACTIVE"
                emergency_stop()

            elif rx_state == 0:
                state = "CLEAR"
                clear_stop()


    if time.ticks_diff(
        time.ticks_ms(),
        last_valid_packet
    ) > TIMEOUT_MS:

        # No communication = SAFE STOP
        state = "TIMEOUT"
        emergency_stop()


    time.sleep_ms(20)
