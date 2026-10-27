"""Pico 2 RA-02 LAY37 E-stop transmitter.

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

CLEAR_DEBOUNCE_MS = 150
STOP_REPEAT_MS = 75
CLEAR_REPEAT_MS = 250

sequence = 0
candidate_state = 1 if estop.value() else 0
candidate_since_ms = time.ticks_ms()
stable_state = None
last_tx_ms = time.ticks_add(time.ticks_ms(), -STOP_REPEAT_MS)
last_tx_attempt_ms = last_tx_ms
repeat_count = 0
radio_errors = 0


def transmit(state, announce=True):
    global sequence, last_tx_ms, last_tx_attempt_ms, radio_errors
    sequence = (sequence + 1) & 0xFFFF
    packet = "ESTOP,{},{}".format(state, sequence)
    last_tx_attempt_ms = time.ticks_ms()
    radio.send(packet)
    last_tx_ms = time.ticks_ms()
    radio_errors = 0
    if announce:
        print("TX:", packet)


def recover_radio(error):
    global radio_errors
    radio_errors += 1
    print("RADIO TX ERROR {}: {}".format(radio_errors, error))
    # Reinitialize the SX1278 after a bounded TX failure. The input loop then
    # keeps retrying the current physical state; it never changes STOP to CLEAR.
    try:
        radio.begin()
        print("RADIO RECOVERED; RETRYING CURRENT STATE")
    except Exception as recovery_error:
        print("RADIO RECOVERY ERROR:", recovery_error)


while True:
    now = time.ticks_ms()
    raw_state = 1 if estop.value() else 0
    state_changed = False

    # STOP is asymmetric by design: the first observed HIGH is transmitted
    # immediately. Only release/CLEAR is debounced, so contact bounce cannot
    # clear an active stop.
    if raw_state == 1 and stable_state != 1:
        stable_state = 1
        candidate_state = 1
        candidate_since_ms = now
        repeat_count = 0
        state_changed = True
        try:
            transmit(1)
        except Exception as error:
            recover_radio(error)

    elif raw_state == 0 and stable_state != 0:
        if candidate_state != 0:
            candidate_state = 0
            candidate_since_ms = now
        if time.ticks_diff(now, candidate_since_ms) >= CLEAR_DEBOUNCE_MS:
            stable_state = 0
            repeat_count = 0
            state_changed = True
            try:
                transmit(0)
            except Exception as error:
                recover_radio(error)

    elif raw_state == 0:
        candidate_state = 0

    if not state_changed and stable_state is not None and time.ticks_diff(
        now, last_tx_attempt_ms
    ) >= (STOP_REPEAT_MS if stable_state else CLEAR_REPEAT_MS):
        try:
            repeat_count += 1
            # Both states are continuous heartbeats. Keep the REPL readable:
            # state changes are always announced and steady-state packets are
            # sampled. STOP is sent faster for low emergency-stop latency.
            transmit(stable_state, announce=(repeat_count % 10 == 0))
        except Exception as error:
            recover_radio(error)

    time.sleep_ms(10)
