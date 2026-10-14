# MicroPython CAN Connection Guide

This setup uses:

```text
ESP32-S3 or XIAO ESP32-C3
        |
        | SPI
        v
MCP2515 CAN controller
        |
        | TXCAN / RXCAN
        v
SN65HVD230 3.3 V CAN transceiver
        |
        | CANH / CANL twisted pair
        v
Second CAN node
```

The SN65HVD230 is only the CAN transceiver. The MCP2515 is the CAN controller
used by the MicroPython test firmware.

Do not connect an SN65HVD230 directly to SPI pins. Also, do not connect an
MCP2515/TJA1050 5 V module directly to ESP32 GPIOs unless its logic levels are
confirmed to be 3.3 V safe or level shifting is installed.

## 1. ESP32-S3 to MCP2515

| MCP2515 pin | ESP32-S3 pin |
|---|---:|
| SCK | GPIO16 |
| SI / MOSI | GPIO17 |
| SO / MISO | GPIO18 |
| CS | GPIO19 |
| INT | GPIO20, optional in polling mode |
| VCC | 3.3 V-compatible supply |
| GND | GND |

## 2. XIAO ESP32-C3 to MCP2515

| MCP2515 pin | XIAO ESP32-C3 pin |
|---|---:|
| SCK | D3 / GPIO5 |
| SI / MOSI | D10 / GPIO10 |
| SO / MISO | D7 / GPIO20 |
| CS | D2 / GPIO4 |
| INT | D1 / GPIO3, optional in polling mode |
| VCC | 3.3 V-compatible supply |
| GND | GND |

The test firmware uses polling, so the INT wire is not required for the first
communication test.

## 3. MCP2515 to SN65HVD230

| MCP2515 | SN65HVD230 |
|---|---|
| TXCAN | D / TXD |
| RXCAN | R / RXD |
| VCC | VCC, 3.3 V |
| GND | GND |

Then connect the CAN bus:

| CAN bus | Connection |
|---|---|
| CANH | CANH on the other node |
| CANL | CANL on the other node |
| GND | Common ground between nodes |

Use one twisted pair for CANH/CANL. Add 120 ohm termination at the two physical
ends of the bus only. Do not add termination to every node.

If using an all-in-one MCP2515 + TJA1050 board, it already contains a CAN
transceiver. Do not connect a second SN65HVD230 to that board. The SN65HVD230
is used when the MCP2515 controller and CAN transceiver are separate.

## 4. Firmware

The test file is:

```text
bench/mcp2515_micropython_can_test.py
```

It assumes an 8 MHz MCP2515 crystal and 500 kbit/s CAN. Confirm the crystal
on the controller board before testing. Change `MCP_CLOCK_MHZ` to `16` only
for a confirmed 16 MHz MCP2515 crystal.

For the ESP32-S3, leave this setting unchanged:

```python
BOARD = "S3"
```

For the XIAO ESP32-C3, change it to:

```python
BOARD = "C3"
```

Only one board configuration should be flashed to each board.

## 5. Flash the ESP32-S3

First identify the port:

```bash
ls /dev/ttyACM* /dev/ttyUSB* 2>/dev/null
```

Then flash the file as `main.py`:

```bash
cd /home/satvara/Github/AutoKart-firmware
source ~/python/bin/activate

PORT=/dev/ttyACM0

python3 -m mpremote connect "$PORT" fs cp \
  bench/mcp2515_micropython_can_test.py :main.py

python3 -m mpremote connect "$PORT" soft-reset
python3 -m mpremote connect "$PORT" repl
```

## 6. Flash the XIAO ESP32-C3

Change `BOARD = "S3"` to `BOARD = "C3"` in the test file, then identify the
C3 port:

```bash
ls /dev/ttyACM* /dev/ttyUSB* 2>/dev/null
```

Flash it using the detected port, for example:

```bash
cd /home/satvara/Github/AutoKart-firmware
source ~/python/bin/activate

PORT=/dev/ttyACM1

python3 -m mpremote connect "$PORT" fs cp \
  bench/mcp2515_micropython_can_test.py :main.py

python3 -m mpremote connect "$PORT" soft-reset
python3 -m mpremote connect "$PORT" repl
```

`main.py` runs automatically after reset, so the boards can communicate
without a laptop after they are powered.

## 7. Send and receive messages

At the MicroPython prompt, use:

```text
help
send 0x120 01 02 AA
ping
auto off
auto on
status
```

Example:

```text
CAN> send 0x120 01 02 AA
TX OK ID=0x120
```

The other board should print:

```text
RX ID=0x120 DLC=3 DATA=01 02 AA
```

To send a message, simply type the message and press Enter on either board:

```text
Hello how are you
```

The other board will print:

```text
RX MESSAGE: Hello how are you
```

The firmware automatically splits longer messages into multiple CAN frames and
reassembles them at the receiver. Messages can be up to 1020 UTF-8 bytes.
Commands such as `ping`, `status`, `auto on`, `auto off`, and `help` retain
their special functions.

`ping` sends a short test frame. With the default firmware, node 1 uses ID
`0x101` and node 2 uses ID `0x102`. `auto on` sends one automatic frame every
second; use `auto off` for manual testing.

## 8. Troubleshooting order

1. Confirm both boards have the same CAN bitrate: 500 kbit/s.
2. Confirm the MCP2515 crystal frequency in the firmware.
3. Confirm CANH is connected to CANH and CANL to CANL.
4. Confirm both nodes share GND.
5. Confirm exactly two 120 ohm terminators are installed.
6. Run `status` and check that the MCP2515 is in normal mode.
7. Test `send 0x120 01 02 AA` from one board and watch the other board's log.

This firmware is a communication bring-up test only. It does not control the
BLDC, steering motor, or emergency-stop chain.
