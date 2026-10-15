"""Pico 2 + RA-02 + LAY37 production E-stop transmitter.

Install this file as ``main.py`` and install ``ra02_lora_chat_common.py`` on
the Pico filesystem. The program then starts automatically whenever powered.

Wiring:
  RA-02 VCC   -> Pico 3V3(OUT)
  RA-02 GND   -> Pico GND
  RA-02 MISO  -> GP16
  RA-02 NSS   -> GP17
  RA-02 SCK   -> GP18
  RA-02 MOSI  -> GP19
  RA-02 RESET -> GP20
  RA-02 DIO0  -> GP21
  LAY37 NC    -> GP22
  LAY37 COM   -> GND
  4.7 kOhm pull-up between GP22 and 3V3

Protocol:
  ESTOP,0,sequence = released / NC contact closed
  ESTOP,1,sequence = pressed or NC wire open

There is no healthy-state heartbeat. The actual state is sent at startup and
on every transition. STOP is repeated while the switch remains active.
"""

from machine import Pin, SPI
import time

from ra02_lora_chat_common import SX1278


SPI_SCK = 18
SPI_MOSI = 19
SPI_MISO = 16
RA02_CS = 17
RA02_RESET = 20
RA02_DIO0 = 21
ESTOP_PIN = 22

FREQUENCY_HZ = 433_000_000
TRANSITION_REPEATS = 5
TRANSITION_DELAY_MS = 50
STOP_REPEAT_MS = 200
POLL_MS = 25


spi = SPI(
    0,
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


def read_estop():
    """Return 0 for healthy NC-closed and 1 for STOP/open-wire."""
    return 1 if estop.value() else 0


def send_state(state):
    global sequence

    sequence = (sequence + 1) & 0xFFFF
    packet = "ESTOP,{},{}".format(state, sequence)
    radio.send(packet)
    print("TX:", packet)


def repeat_state(state):
    for _ in range(TRANSITION_REPEATS):
        try:
            send_state(state)
        except Exception as error:
            print("TX ERROR:", error)
        time.sleep_ms(TRANSITION_DELAY_MS)


print()
print("========================================")
print(" PICO 2 + RA-02 LORA E-STOP TRANSMITTER")
print("========================================")
print("SPI: SCK18 MOSI19 MISO16 CS17 RST20 DIO0=21")
print("LAY37: NC=GP22 COM=GND; LOW=CLEAR HIGH=STOP")

radio.begin()

last_state = read_estop()
print(
    "STARTUP LAY37:",
    "STOP/PRESSED/WIRE-OPEN" if last_state else "RELEASED/NC-CLOSED",
)
repeat_state(last_state)

while True:
    state = read_estop()

    if state != last_state:
        time.sleep_ms(POLL_MS)
        confirmed_state = read_estop()

        if confirmed_state != last_state:
            last_state = confirmed_state
            print(
                "LAY37 CHANGED:",
                "STOP/PRESSED/WIRE-OPEN"
                if last_state
                else "RELEASED/NC-CLOSED",
            )
            repeat_state(last_state)

    elif state == 1:
        try:
            send_state(1)
        except Exception as error:
            print("STOP TX ERROR:", error)
        time.sleep_ms(STOP_REPEAT_MS)

    else:
        time.sleep_ms(POLL_MS)
