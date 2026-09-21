"""Small MicroPython SX1278 LoRa text-chat driver for RA-02."""

import sys
import time
import uselect


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
    PAYLOAD_LENGTH = 0x22
    MODEM_CONFIG_3 = 0x26
    SYNC_WORD = 0x39
    VERSION = 0x42

    LONG_RANGE = 0x80
    SLEEP = 0x00
    STANDBY = 0x01
    TX = 0x03
    RX_CONTINUOUS = 0x05
    RX_DONE = 0x40
    PAYLOAD_CRC_ERROR = 0x20
    TX_DONE = 0x08

    def __init__(self, spi, cs, reset, frequency=433000000):
        self.spi = spi
        self.cs = cs
        self.reset = reset
        self.frequency = frequency

    def write_reg(self, address, value):
        self.cs.value(0)
        self.spi.write(bytes((address | 0x80, value & 0xFF)))
        self.cs.value(1)

    def read_reg(self, address):
        self.cs.value(0)
        self.spi.write(bytes((address & 0x7F,)))
        value = self.spi.read(1, 0)[0]
        self.cs.value(1)
        return value

    def read_fifo(self, count):
        self.cs.value(0)
        self.spi.write(bytes((self.FIFO & 0x7F,)))
        data = self.spi.read(count, 0)
        self.cs.value(1)
        return data

    def write_fifo(self, data):
        self.cs.value(0)
        self.spi.write(bytes((self.FIFO | 0x80,)))
        self.spi.write(data)
        self.cs.value(1)

    def reset_radio(self):
        self.reset.value(0)
        time.sleep_ms(10)
        self.reset.value(1)
        time.sleep_ms(20)

    def begin(self):
        self.cs.value(1)
        self.reset_radio()
        version = self.read_reg(self.VERSION)
        print("RA-02 SX1278 version: 0x{:02X}".format(version))
        if version != 0x12:
            raise OSError("RA-02 not found; expected 0x12")

        self.write_reg(self.OP_MODE, self.LONG_RANGE | self.SLEEP)
        time.sleep_ms(10)
        frf = int((self.frequency * 524288) / 32000000)
        self.write_reg(self.FRF_MSB, frf >> 16)
        self.write_reg(self.FRF_MID, frf >> 8)
        self.write_reg(self.FRF_LSB, frf)
        self.write_reg(self.FIFO_TX_BASE, 0)
        self.write_reg(self.FIFO_RX_BASE, 0)
        self.write_reg(self.LNA, self.read_reg(self.LNA) | 0x03)
        # BW 125 kHz, CR 4/5, explicit header; SF7 and CRC enabled.
        self.write_reg(self.MODEM_CONFIG_1, 0x72)
        self.write_reg(self.MODEM_CONFIG_2, 0x74)
        self.write_reg(self.MODEM_CONFIG_3, 0x04)
        self.write_reg(self.PREAMBLE_MSB, 0)
        self.write_reg(self.PREAMBLE_LSB, 8)
        self.write_reg(self.SYNC_WORD, 0x12)
        self.write_reg(self.PA_CONFIG, 0x8F)
        self.write_reg(self.OCP, 0x2B)
        self.write_reg(self.IRQ_FLAGS, 0xFF)
        self.receive_mode()
        print("LoRa ready: 433 MHz, SF7, BW125, CRC ON")

    def standby(self):
        self.write_reg(self.OP_MODE, self.LONG_RANGE | self.STANDBY)

    def receive_mode(self):
        self.write_reg(self.IRQ_FLAGS, 0xFF)
        self.write_reg(self.FIFO_ADDR_PTR, 0)
        self.write_reg(self.OP_MODE, self.LONG_RANGE | self.RX_CONTINUOUS)

    def send(self, payload):
        if isinstance(payload, str):
            payload = payload.encode()
        if not payload or len(payload) > 240:
            raise ValueError("message must contain 1-240 bytes")
        self.standby()
        self.write_reg(self.IRQ_FLAGS, 0xFF)
        self.write_reg(self.FIFO_ADDR_PTR, 0)
        self.write_fifo(payload)
        self.write_reg(self.PAYLOAD_LENGTH, len(payload))
        self.write_reg(self.OP_MODE, self.LONG_RANGE | self.TX)
        started = time.ticks_ms()
        while not (self.read_reg(self.IRQ_FLAGS) & self.TX_DONE):
            if time.ticks_diff(time.ticks_ms(), started) > 3000:
                self.receive_mode()
                raise OSError("LoRa transmit timeout")
            time.sleep_ms(2)
        self.write_reg(self.IRQ_FLAGS, self.TX_DONE)
        self.receive_mode()

    def receive(self):
        flags = self.read_reg(self.IRQ_FLAGS)
        if not (flags & self.RX_DONE):
            return None
        self.write_reg(self.IRQ_FLAGS, self.RX_DONE | self.PAYLOAD_CRC_ERROR)
        if flags & self.PAYLOAD_CRC_ERROR:
            return None
        current = self.read_reg(self.FIFO_RX_CURRENT)
        count = self.read_reg(self.RX_NB_BYTES)
        self.write_reg(self.FIFO_ADDR_PTR, current)
        return self.read_fifo(count)


def chat_loop(radio, board_name):
    radio.begin()
    print(board_name + " TEXT CHAT READY")
    print("Type a message and press Enter. Ctrl-C stops the test.")
    print("> ", end="")
    poller = uselect.poll()
    poller.register(sys.stdin, uselect.POLLIN)
    line = ""
    while True:
        packet = radio.receive()
        if packet:
            try:
                message = packet.decode().strip()
            except Exception:
                message = "<binary: {} bytes>".format(len(packet))
            print("\nREMOTE: " + message)
            print("> " + line, end="")

        if poller.poll(0):
            char = sys.stdin.read(1)
            if char in ("\r", "\n"):
                if line.strip():
                    radio.send(line.strip())
                    print("\nSENT: " + line.strip())
                line = ""
                print("> ", end="")
            elif char in ("\x08", "\x7f"):
                line = line[:-1]
            else:
                line += char
        time.sleep_ms(5)
