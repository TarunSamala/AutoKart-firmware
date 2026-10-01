"""Standalone ESP32-S3 + RA-02 receive-only E-stop monitor.

Copy this single file to the ESP32-S3 as ``main.py``. No helper modules are
required. This isolated receiver prints state only; it controls no actuator.

Wiring:
  RA-02 VCC   -> ESP32-S3 3V3
  RA-02 GND   -> ESP32-S3 GND
  RA-02 SCK   -> GPIO4
  RA-02 MOSI  -> GPIO5
  RA-02 MISO  -> GPIO6
  RA-02 NSS   -> GPIO7
  RA-02 RESET -> GPIO10
  RA-02 DIO0  -> GPIO18

Accepted packets:
  ESTOP,0,sequence = released / clear
  ESTOP,1,sequence = pressed or NC wire open / stop

There is no heartbeat timeout in this isolated test.
"""

from machine import Pin, SPI
import time


SPI_SCK = 4
SPI_MOSI = 5
SPI_MISO = 6
RA02_CS = 7
RA02_RESET = 10
RA02_DIO0 = 18
FREQUENCY_HZ = 433_000_000


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
    FIFO_RX_CURRENT = 0x10
    IRQ_FLAGS = 0x12
    RX_NB_BYTES = 0x13
    MODEM_CONFIG_1 = 0x1D
    MODEM_CONFIG_2 = 0x1E
    PREAMBLE_MSB = 0x20
    PREAMBLE_LSB = 0x21
    MODEM_CONFIG_3 = 0x26
    SYNC_WORD = 0x39
    VERSION = 0x42

    LONG_RANGE = 0x80
    SLEEP = 0x00
    STANDBY = 0x01
    RX_CONTINUOUS = 0x05

    RX_DONE = 0x40
    PAYLOAD_CRC_ERROR = 0x20

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

    def read_fifo(self, count):
        self.cs.value(0)
        self.spi.write(bytes((self.FIFO & 0x7F,)))
        payload = self.spi.read(count, 0)
        self.cs.value(1)
        return payload

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

        # Must exactly match the C3 transmitter.
        self.write_register(self.MODEM_CONFIG_1, 0x72)
        self.write_register(self.MODEM_CONFIG_2, 0x74)
        self.write_register(self.MODEM_CONFIG_3, 0x04)
        self.write_register(self.PREAMBLE_MSB, 0)
        self.write_register(self.PREAMBLE_LSB, 8)
        self.write_register(self.SYNC_WORD, 0x12)
        self.write_register(self.PA_CONFIG, 0x8F)
        self.write_register(self.OCP, 0x2B)

        self.receive_mode()
        print("LoRa ready: 433 MHz, SF7, BW125, hardware CRC ON")

    def receive_mode(self):
        self.write_register(self.IRQ_FLAGS, 0xFF)
        self.write_register(self.FIFO_ADDR_PTR, 0)
        self.write_register(
            self.OP_MODE,
            self.LONG_RANGE | self.RX_CONTINUOUS,
        )

    def receive(self):
        flags = self.read_register(self.IRQ_FLAGS)

        if not (flags & self.RX_DONE):
            return None

        self.write_register(
            self.IRQ_FLAGS,
            self.RX_DONE | self.PAYLOAD_CRC_ERROR,
        )

        if flags & self.PAYLOAD_CRC_ERROR:
            print("RX DROPPED: HARDWARE CRC ERROR")
            return None

        current_address = self.read_register(self.FIFO_RX_CURRENT)
        byte_count = self.read_register(self.RX_NB_BYTES)
        self.write_register(self.FIFO_ADDR_PTR, current_address)
        return self.read_fifo(byte_count)


def parse_estop_packet(packet):
    try:
        text = packet.decode().strip()
        fields = text.split(",")

        if len(fields) != 3 or fields[0] != "ESTOP":
            return None

        state = int(fields[1])
        sequence = int(fields[2])

        if state not in (0, 1):
            return None

        if sequence < 0 or sequence > 0xFFFF:
            return None

        return text, state, sequence

    except Exception:
        return None


spi = SPI(
    2,
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
radio = SX1278(spi, cs, reset, FREQUENCY_HZ)


print()
print("========================================")
print(" ESP32-S3 LORA E-STOP RECEIVER MONITOR")
print("========================================")
print("RA-02: SCK4 MOSI5 MISO6 CS7 RESET10 DIO0=18")

radio.begin()

estop_active = None
last_sequence = None

print("Receiver state: UNKNOWN")
print("Waiting for C3 ESTOP packets...")

while True:
    packet = radio.receive()

    if packet:
        parsed = parse_estop_packet(packet)

        if parsed is None:
            try:
                print("IGNORED INVALID PACKET:", packet.decode().strip())
            except Exception:
                print("IGNORED INVALID BYTES:", packet)
        else:
            text, state, sequence = parsed
            print("RX:", text)

            if estop_active is None or state != estop_active:
                estop_active = state

                if estop_active:
                    print("*** E-STOP ACTIVE: STOP / BRAKE REQUIRED ***")
                else:
                    print("E-STOP CLEAR: SWITCH RELEASED")

            if last_sequence is not None:
                expected = (last_sequence + 1) & 0xFFFF
                if sequence != expected:
                    print(
                        "SEQUENCE NOTICE: expected {}, received {}".format(
                            expected,
                            sequence,
                        )
                    )

            last_sequence = sequence

    time.sleep_ms(5)
