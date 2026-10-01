"""XIAO ESP32-C3 + RA-02 + LAY37 LoRa E-stop transmitter.

Packet protocol remains compatible with the existing ESP32-S3 receiver:
  ESTOP,0,sequence = switch released / clear
  ESTOP,1,sequence = switch pressed or NC wire open / stop

There is no healthy-state heartbeat. A clear packet is sent at startup and
after release; STOP packets repeat while the switch remains pressed/open.
"""

from machine import Pin, SPI
import time
from ra02_lora_chat_common import SX1278


# XIAO ESP32-C3 hardware SPI pins.
SPI_SCK = 8       # D8
SPI_MISO = 9      # D9
SPI_MOSI = 10     # D10

RA02_CS = 5       # D3 / NSS
RA02_RESET = 4    # D2
RA02_DIO0 = 3     # D1 (connected for future interrupt use)

# LAY37 COM -> GND, NC -> D4/GPIO6. LOW=healthy, HIGH=STOP/fault.
ESTOP_PIN = 6     # D4

FREQUENCY_HZ = 433_000_000
CLEAR_REPEAT_COUNT = 5
CLEAR_REPEAT_DELAY_MS = 50
STOP_REPEAT_DELAY_MS = 200
POLL_DELAY_MS = 25


spi = SPI(
    1,
    baudrate=2_000_000,
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

radio = SX1278(spi, cs, reset, FREQUENCY_HZ)
sequence = 0


def read_estop_state():
    """Return 1 for pressed/open-wire STOP, or 0 for released/NC closed."""
    return 1 if estop.value() else 0


def send_state(state):
    global sequence

    sequence = (sequence + 1) & 0xFFFF
    packet = "ESTOP,{},{}".format(state, sequence)
    radio.send(packet)
    print("TX:", packet)


radio.begin()

print()
print("XIAO ESP32-C3 LORA E-STOP TRANSMITTER READY")
print("RA-02: SCK=D8 MISO=D9 MOSI=D10 CS=D3 RST=D2 DIO0=D1")
print("LAY37: D4/GPIO6 LOW=RELEASED, HIGH=PRESSED/WIRE-OPEN")
print("No healthy heartbeat; STOP repeats only while active.")

last_state = read_estop_state()

# Announce the real startup state several times. Starting while pressed or
# with an open wire therefore sends STOP, never a false clear.
startup_repeats = CLEAR_REPEAT_COUNT
for _ in range(startup_repeats):
    try:
        send_state(last_state)
    except Exception as error:
        print("STARTUP TX ERROR:", error)
    time.sleep_ms(CLEAR_REPEAT_DELAY_MS)

while True:
    state = read_estop_state()

    if state != last_state:
        # Small debounce, then require the new electrical state to persist.
        time.sleep_ms(POLL_DELAY_MS)
        state = read_estop_state()

        if state != last_state:
            last_state = state
            repeats = CLEAR_REPEAT_COUNT

            for _ in range(repeats):
                try:
                    send_state(state)
                except Exception as error:
                    print("EDGE TX ERROR:", error)
                time.sleep_ms(CLEAR_REPEAT_DELAY_MS)

    elif state == 1:
        # Repeat STOP while held/open. This is not a healthy-state heartbeat.
        try:
            send_state(1)
        except Exception as error:
            print("STOP TX ERROR:", error)
        time.sleep_ms(STOP_REPEAT_DELAY_MS)

    else:
        time.sleep_ms(POLL_DELAY_MS)
