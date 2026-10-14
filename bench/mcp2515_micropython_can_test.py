"""MicroPython MCP2515 CAN 2.0 bench test.

This is for ESP32-S3 and XIAO ESP32-C3 boards running standard MicroPython.
It uses the MCP2515 controller over SPI and a CAN transceiver module.

IMPORTANT POWER/LOGIC NOTE
--------------------------
Many MCP2515 + TJA1050 modules are 5 V logic modules. Do not connect their
MISO, INT, or other 5 V outputs directly to ESP32 GPIOs. Use a confirmed
3.3 V-compatible module or suitable level shifting. The SN65HVD230 is a
3.3 V transceiver, but it still needs an MCP2515 controller in this
MicroPython setup.

Select the board below before flashing:
  BOARD = "S3" -> ESP32-S3 node 1
  BOARD = "C3" -> XIAO ESP32-C3 node 2

MCP2515 clock:
  Most blue MCP2515 modules use 8 MHz. Set MCP_CLOCK_MHZ = 16 for a module
  with a confirmed 16 MHz crystal.

The driver uses polling, so MCP2515 INT is optional for this test.
"""

import sys
import time
import uselect
from machine import Pin, SoftSPI


# Change this line to "C3" before flashing the XIAO ESP32-C3.
BOARD = "S3"
NODE_ID = 1 if BOARD == "S3" else 2

CAN_BITRATE = 500000
MCP_CLOCK_MHZ = 8


if BOARD == "S3":
    # ESP32-S3: isolated CAN test pins.
    SPI_SCK = 16
    SPI_MOSI = 17
    SPI_MISO = 18
    MCP_CS = 19
    MCP_INT = 20
else:
    # XIAO ESP32-C3: D3/GPIO5, D10/GPIO10, D7/GPIO20, D2/GPIO4.
    # GPIO8/GPIO9 are deliberately avoided because of boot strapping.
    SPI_SCK = 5
    SPI_MOSI = 10
    SPI_MISO = 20
    MCP_CS = 4
    MCP_INT = 3


# MCP2515 commands.
CMD_RESET = 0xC0
CMD_READ = 0x03
CMD_WRITE = 0x02
CMD_BIT_MODIFY = 0x05
CMD_RTS_TXB0 = 0x81

# MCP2515 registers.
CANSTAT = 0x0E
CANCTRL = 0x0F
CNF3 = 0x28
CNF2 = 0x29
CNF1 = 0x2A
CANINTE = 0x2B
CANINTF = 0x2C
EFLG = 0x2D

TXB0CTRL = 0x30
TXB0SIDH = 0x31

RXB0CTRL = 0x60
RXB0SIDH = 0x61
RXB1CTRL = 0x70
RXB1SIDH = 0x71

MODE_CONFIG = 0x80
MODE_NORMAL = 0x00


class MCP2515:
    def __init__(self):
        self.cs = Pin(MCP_CS, Pin.OUT, value=1)
        self.spi = SoftSPI(
            baudrate=1000000,
            polarity=0,
            phase=0,
            sck=Pin(SPI_SCK),
            mosi=Pin(SPI_MOSI),
            miso=Pin(SPI_MISO),
        )
        self.reset()
        self.configure()

    def select(self):
        self.cs.value(0)

    def deselect(self):
        self.cs.value(1)

    def command(self, value):
        self.select()
        self.spi.write(bytes([value]))
        self.deselect()

    def reset(self):
        self.command(CMD_RESET)
        time.sleep_ms(10)

    def read_registers(self, address, count):
        self.select()
        self.spi.write(bytes([CMD_READ, address]))
        data = self.spi.read(count, 0)
        self.deselect()
        return data

    def read_register(self, address):
        return self.read_registers(address, 1)[0]

    def write_registers(self, address, data):
        self.select()
        self.spi.write(bytes([CMD_WRITE, address]) + bytes(data))
        self.deselect()

    def write_register(self, address, value):
        self.write_registers(address, bytes([value & 0xFF]))

    def bit_modify(self, address, mask, value):
        self.select()
        self.spi.write(bytes([
            CMD_BIT_MODIFY,
            address,
            mask,
            value,
        ]))
        self.deselect()

    def configure(self):
        self.write_register(CANCTRL, MODE_CONFIG)

        if MCP_CLOCK_MHZ == 8 and CAN_BITRATE == 500000:
            # 8 MHz crystal, 500 kbit/s, approximately 75% sample point.
            cnf1, cnf2, cnf3 = 0x00, 0x98, 0x01
        elif MCP_CLOCK_MHZ == 16 and CAN_BITRATE == 500000:
            # 16 MHz crystal, 500 kbit/s, approximately 75% sample point.
            cnf1, cnf2, cnf3 = 0x00, 0xB9, 0x04
        else:
            raise ValueError("Supported timing: 8/16 MHz MCP2515 at 500 kbps")

        self.write_registers(CNF1, bytes([cnf1, cnf2, cnf3]))

        # Receive all standard and extended frames into both RX buffers.
        self.write_register(RXB0CTRL, 0x60)
        self.write_register(RXB1CTRL, 0x60)

        # Enable RX buffer interrupts; this test still polls CANINTF.
        self.write_register(CANINTE, 0x03)
        self.write_register(CANCTRL, MODE_NORMAL)
        time.sleep_ms(10)

        mode = self.read_register(CANSTAT) & 0xE0
        if mode != MODE_NORMAL:
            raise RuntimeError("MCP2515 did not enter normal mode: 0x%02X" % mode)

    def send(self, can_id, payload):
        can_id = int(can_id)
        payload = bytes(payload)

        if can_id < 0 or can_id > 0x7FF or len(payload) > 8:
            raise ValueError("Use an 11-bit standard ID and 0-8 data bytes")

        deadline = time.ticks_add(time.ticks_ms(), 100)
        while self.read_register(TXB0CTRL) & 0x08:
            if time.ticks_diff(deadline, time.ticks_ms()) <= 0:
                raise OSError("MCP2515 TX buffer busy")
            time.sleep_ms(1)

        sidh = (can_id >> 3) & 0xFF
        sidl = (can_id << 5) & 0xE0
        frame = bytes([sidh, sidl, 0x00, 0x00, len(payload)]) + payload
        self.write_registers(TXB0SIDH, frame)
        self.command(CMD_RTS_TXB0)

    def receive_buffer(self, base, flag):
        frame = self.read_registers(base, 13)
        can_id = (frame[0] << 3) | (frame[1] >> 5)
        length = min(frame[4] & 0x0F, 8)
        payload = bytes(frame[5:5 + length])
        self.bit_modify(CANINTF, flag, 0)
        return can_id, payload

    def receive_all(self):
        messages = []
        flags = self.read_register(CANINTF)

        if flags & 0x01:
            messages.append(self.receive_buffer(RXB0SIDH, 0x01))
        if flags & 0x02:
            messages.append(self.receive_buffer(RXB1SIDH, 0x02))

        return messages

    def status(self):
        return {
            "canstat": self.read_register(CANSTAT),
            "canintf": self.read_register(CANINTF),
            "eflg": self.read_register(EFLG),
            "txb0ctrl": self.read_register(TXB0CTRL),
        }


def parse_payload(tokens):
    payload = bytearray()
    for token in tokens:
        if len(payload) >= 8:
            raise ValueError("Maximum payload is 8 bytes")
        payload.append(int(token, 16))
    return payload


def send_ping(can):
    can.send(0x100 + NODE_ID, bytes([NODE_ID, 0xAA]))
    print("TX PING ID=0x%03X NODE=%d" % (0x100 + NODE_ID, NODE_ID))


tx_message_id = [0]
rx_message_id = [None]
rx_message_total = [0]
rx_message_parts = [[]]


def send_message(can, message):
    payload = message.encode("utf-8")
    if not payload:
        return
    if len(payload) > 1020:
        raise ValueError("Message is too long; maximum is 1020 UTF-8 bytes")

    message_id = tx_message_id[0]
    tx_message_id[0] = (message_id + 1) & 0xFF
    total = (len(payload) + 3) // 4

    for part in range(total):
        chunk = payload[part * 4:(part + 1) * 4]
        can.send(0x120, bytes([0xA5, message_id, part, total]) + chunk)
    print("TX MESSAGE: %s" % message)


def receive_message_frame(payload):
    if len(payload) < 4 or payload[0] != 0xA5:
        return False

    message_id = payload[1]
    part = payload[2]
    total = payload[3]
    if total == 0 or part >= total:
        return True

    if message_id != rx_message_id[0] or part == 0:
        rx_message_id[0] = message_id
        rx_message_total[0] = total
        rx_message_parts[0] = []

    if part == len(rx_message_parts[0]):
        rx_message_parts[0].append(bytes(payload[4:]))

    if len(rx_message_parts[0]) == rx_message_total[0]:
        data = b"".join(rx_message_parts[0])
        try:
            message = data.decode("utf-8")
        except UnicodeError:
            message = "<invalid UTF-8>"
        print("RX MESSAGE: %s" % message)
        rx_message_id[0] = None
        rx_message_parts[0] = []
    return True


def print_help():
    print("Commands:")
    print("  Type any message and press Enter")
    print("  send 0x120 01 02 AA   (raw CAN frame)")
    print("  ping")
    print("  auto on|off")
    print("  status")
    print("  help")


def handle_command(can, line):
    tokens = line.strip().split()
    if not tokens:
        return

    command = tokens[0].lower()

    try:
        if command == "send" and len(tokens) >= 2:
            can.send(int(tokens[1], 0), parse_payload(tokens[2:]))
            print("TX OK ID=%s" % tokens[1])
        elif command == "ping":
            send_ping(can)
        elif command == "auto" and len(tokens) == 2:
            auto_state[0] = tokens[1].lower() == "on"
            print("AUTO TX", "ON" if auto_state[0] else "OFF")
        elif command == "status":
            print("NODE=%d BOARD=%s SPI=%d/%d/%d CS=%d" % (
                NODE_ID, BOARD, SPI_SCK, SPI_MOSI, SPI_MISO, MCP_CS))
            print(can.status())
        elif command == "help":
            print_help()
        else:
            send_message(can, line.strip())
    except Exception as error:
        print("COMMAND ERROR:", error)


auto_state = [True]
counter = [0]
can = MCP2515()
reader = uselect.poll()
reader.register(sys.stdin, uselect.POLLIN)
next_auto_ms = time.ticks_ms()

print("AUTOKART MICROPYTHON MCP2515 CAN TEST")
print("BOARD=%s NODE=%d BITRATE=%d CLOCK=%d MHz" % (
    BOARD, NODE_ID, CAN_BITRATE, MCP_CLOCK_MHZ))
print("SPI SCK=%d MOSI=%d MISO=%d CS=%d INT=%d" % (
    SPI_SCK, SPI_MOSI, SPI_MISO, MCP_CS, MCP_INT))
print("MCP2515 ready. Use 'help'.")
print_help()

while True:
    for can_id, payload in can.receive_all():
        if receive_message_frame(payload):
            continue
        print("RX ID=0x%03X DLC=%d DATA=%s" % (
            can_id,
            len(payload),
            " ".join("%02X" % byte for byte in payload),
        ))
        print("RX TEXT: %s" % message)

    now = time.ticks_ms()
    if auto_state[0] and time.ticks_diff(now, next_auto_ms) >= 0:
        can.send(0x100 + NODE_ID, bytes([NODE_ID, counter[0]]))
        print("TX AUTO ID=0x%03X COUNT=%d" % (
            0x100 + NODE_ID, counter[0]))
        counter[0] = (counter[0] + 1) & 0xFF
        next_auto_ms = time.ticks_add(now, 1000)

    events = reader.poll(10)
    if events:
        line = sys.stdin.readline()
        handle_command(can, line)
