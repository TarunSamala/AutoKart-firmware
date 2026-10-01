# Receiver safety logic
# Runs on ESP32-S3 or RP2350
#
# Principle:
# No valid packet = STOP

import time

TIMEOUT_MS = 300

last_sequence = None
last_packet_time = 0

def process_packet(packet):

    global last_sequence,last_packet_time

    # Verify CRC before accepting.
    # Parse device/state/sequence.

    last_packet_time = time.ticks_ms()


def safety_check():

    age = time.ticks_diff(
        time.ticks_ms(),
        last_packet_time
    )

    if age > TIMEOUT_MS:
        emergency_stop()

def emergency_stop():

    # Remove BLDC enable
    # Apply brake
    # Latch fault
    print("ESTOP ACTIVE")


while True:
    safety_check()
    time.sleep_ms(20)
