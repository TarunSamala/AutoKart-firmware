"""Standalone XIAO ESP32-C3 + RA-02 + LAY37 E-stop transmitter.

Copy this single file to the C3 as ``main.py``. No helper modules required.

Boot-safe wiring:
  RA-02 VCC   -> XIAO 3V3
  RA-02 GND   -> XIAO GND
  RA-02 SCK   -> XIAO D5  / GPIO7
  RA-02 MISO  -> XIAO D7  / GPIO20
  RA-02 MOSI  -> XIAO D10 / GPIO10
  RA-02 NSS   -> XIAO D3  / GPIO5
  RA-02 RESET -> XIAO D2  / GPIO4
  RA-02 DIO0  -> XIAO D1  / GPIO3
  LAY37 NC    -> XIAO D4  / GPIO6
  LAY37 COM   -> XIAO GND

Packet protocol:
  ESTOP,0,sequence = released / clear
  ESTOP,1,sequence = pressed or NC wire open / stop

This design has no healthy-state heartbeat. It sends the actual state at
startup and on each state change, and repeats STOP while STOP remains active.
"""

from machine import Pin, SPI
import time


# -------------------- XIAO ESP32-C3 pins --------------------
SPI_SCK = 7       # D5
SPI_MISO = 20     # D7
SPI_MOSI = 10     # D10
RA02_CS = 5       # D3
RA02_RESET = 4    # D2
RA02_DIO0 = 3     # D1; connected but polling is used
ESTOP_PIN = 6     # D4

FREQUENCY_HZ = 433_000_000

# Repeat transitions to reduce the chance of losing a single state packet.
TRANSITION_REPEATS = 5
TRANSITION_DELAY_MS = 50
STOP_REPEAT_MS = 200
POLL_MS = 25


class SX1278:
    FIFO = 0x00
    OP_MODE = 0x01
    FRF_MSB = 0x06
    FRF_MID = 0x07
    FRF_LSB = 0x08
    PA_CONFIG = 0x09
    OCP = 0x0B
    LNA = 0x0C
    FIFO_ADDR_PTR = 0x0D
    FIFO_TX_BASE = 0x0E
    FIFO_RX_BASE = 0x0F
    IRQ_FLAGS = 0x12
    MODEM_CONFIG_1 = 0x1D
    MODEM_CONFIG_2 = 0x1E
    PREAMBLE_MSB = 0x20
    PREAMBLE_LSB = 0x21
    PAYLOAD_LENGTH = 0x22
    MODEM_CONFIG_3 = 0x26
    SYNC_WORD = 0x39
    VERSION = 0x42

    LONG_RANGE = 0x80
    SLEEP = 0x00
    STANDBY = 0x01
    TX = 0x03
    TX_DONE = 0x08

    def __init__(self, spi, cs, reset, frequency):
        self.spi = spi
        self.cs = cs
        self.reset = reset
        self.frequency = frequency

    def write_register(self, address, value):
        self.cs.value(0)
        self.spi.write(bytes((address | 0x80, value & 0xFF)))
        self.cs.value(1)

    def read_register(self, address):
        self.cs.value(0)
        self.spi.write(bytes((address & 0x7F,)))
        value = self.spi.read(1, 0)[0]
        self.cs.value(1)
        return value

    def write_fifo(self, payload):
        self.cs.value(0)
        self.spi.write(bytes((self.FIFO | 0x80,)))
        self.spi.write(payload)
        self.cs.value(1)

    def hardware_reset(self):
        self.reset.value(0)
        time.sleep_ms(10)
        self.reset.value(1)
        time.sleep_ms(20)

    def begin(self):
        self.cs.value(1)
        self.hardware_reset()

        version = self.read_register(self.VERSION)
        print("RA-02 RegVersion: 0x{:02X}".format(version))

        if version != 0x12:
            raise OSError("RA-02 not found; expected SX1278 version 0x12")

        self.write_register(self.OP_MODE, self.LONG_RANGE | self.SLEEP)
        time.sleep_ms(10)

        frf = int((self.frequency * 524288) / 32_000_000)
        self.write_register(self.FRF_MSB, frf >> 16)
        self.write_register(self.FRF_MID, frf >> 8)
        self.write_register(self.FRF_LSB, frf)

        self.write_register(self.FIFO_TX_BASE, 0)
        self.write_register(self.FIFO_RX_BASE, 0)
        self.write_register(self.LNA, self.read_register(self.LNA) | 0x03)

        # BW 125 kHz, coding rate 4/5, explicit header, SF7, CRC enabled.
        self.write_register(self.MODEM_CONFIG_1, 0x72)
        self.write_register(self.MODEM_CONFIG_2, 0x74)
        self.write_register(self.MODEM_CONFIG_3, 0x04)
        self.write_register(self.PREAMBLE_MSB, 0)
        self.write_register(self.PREAMBLE_LSB, 8)
        self.write_register(self.SYNC_WORD, 0x12)

        # PA_BOOST, maximum configured module output; OCP about 100 mA.
        self.write_register(self.PA_CONFIG, 0x8F)
        self.write_register(self.OCP, 0x2B)
        self.write_register(self.IRQ_FLAGS, 0xFF)
        self.standby()

        print("LoRa ready: 433 MHz, SF7, BW125, hardware CRC ON")

    def standby(self):
        self.write_register(self.OP_MODE, self.LONG_RANGE | self.STANDBY)

    def send(self, payload):
        if isinstance(payload, str):
            payload = payload.encode()

        if not payload or len(payload) > 240:
            raise ValueError("LoRa payload must contain 1 to 240 bytes")

        self.standby()
        self.write_register(self.IRQ_FLAGS, 0xFF)
        self.write_register(self.FIFO_ADDR_PTR, 0)
        self.write_fifo(payload)
        self.write_register(self.PAYLOAD_LENGTH, len(payload))
        self.write_register(self.OP_MODE, self.LONG_RANGE | self.TX)

        started = time.ticks_ms()

        while not (self.read_register(self.IRQ_FLAGS) & self.TX_DONE):
            if time.ticks_diff(time.ticks_ms(), started) > 3000:
                self.standby()
                raise OSError("LoRa transmit timeout")
            time.sleep_ms(2)

        self.write_register(self.IRQ_FLAGS, self.TX_DONE)
        self.standby()


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


def read_estop():
    # Normally-closed contact: LOW=healthy, HIGH=pressed/open-wire STOP.
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
print(" XIAO ESP32-C3 LORA E-STOP TRANSMITTER")
print("========================================")
print("SPI: SCK=D5 MISO=D7 MOSI=D10 CS=D3 RST=D2 DIO0=D1")
print("LAY37: NC=D4/GPIO6 COM=GND")

radio.begin()

last_state = read_estop()
print(
    "STARTUP LAY37:",
    "STOP/PRESSED/WIRE-OPEN" if last_state else "RELEASED/NC-CLOSED",
)

# Send the actual startup state. A pressed switch or broken NC wire therefore
# sends STOP and can never generate a startup CLEAR.
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
                "STOP/PRESSED/WIRE-OPEN" if last_state else "RELEASED/NC-CLOSED",
            )
            repeat_state(last_state)

    elif state == 1:
        # STOP repeats while active. CLEAR is not sent as a heartbeat.
        try:
            send_state(1)
        except Exception as error:
            print("STOP TX ERROR:", error)
        time.sleep_ms(STOP_REPEAT_MS)

    else:
        time.sleep_ms(POLL_MS)
