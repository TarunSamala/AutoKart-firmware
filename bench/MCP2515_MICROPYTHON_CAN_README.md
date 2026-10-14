# MicroPython MCP2515 CAN test

Use [mcp2515_micropython_can_test.py](mcp2515_micropython_can_test.py) as
`main.py` on both boards. It uses the MCP2515 over SPI, so it works with
standard ESP32 MicroPython builds without requiring a custom TWAI module.

## Board configuration

The file defaults to the ESP32-S3 node:

```python
BOARD = "S3"
```

Before flashing the XIAO ESP32-C3, change that one line to:

```python
BOARD = "C3"
```

The S3 is node 1 and the C3 is node 2.

## SPI connections

ESP32-S3:

| MCP2515 | ESP32-S3 |
|---|---:|
| SCK | GPIO16 |
| SI / MOSI | GPIO17 |
| SO / MISO | GPIO18 |
| CS | GPIO19 |
| INT | GPIO20, optional for this polling test |
| VCC | Confirm module logic voltage first |
| GND | GND |

XIAO ESP32-C3:

| MCP2515 | XIAO pin |
|---|---:|
| SCK | D3 / GPIO5 |
| SI / MOSI | D10 / GPIO10 |
| SO / MISO | D7 / GPIO20 |
| CS | D2 / GPIO4 |
| INT | D1 / GPIO3, optional |
| VCC | Confirm module logic voltage first |
| GND | GND |

Connect CANH to CANH and CANL to CANL using a twisted pair. Install 120 ohms
at each physical end of the bus only.

## Important voltage warning

The MCP2515/TJA1050 module shown in the component list is commonly a 5 V
module. Do not connect a 5 V MISO or INT output directly to an ESP32. Use a
confirmed 3.3 V-compatible MCP2515 board or level shifting on the SPI/control
signals. Also check the module crystal: the code assumes an 8 MHz MCP2515
crystal. Change `MCP_CLOCK_MHZ = 16` only after confirming a 16 MHz crystal.

## Flash

S3:

```bash
python3 -m mpremote connect /dev/ttyACM0 fs cp \
  bench/mcp2515_micropython_can_test.py :main.py
python3 -m mpremote connect /dev/ttyACM0 soft-reset
python3 -m mpremote connect /dev/ttyACM0 repl
```

For the C3, change `BOARD = "S3"` to `BOARD = "C3"`, then flash the edited
file to its detected port.

## Commands

```text
send 0x120 01 02 AA
ping
auto off
auto on
status
help
```

Each node sends an automatic frame every second when auto mode is enabled:

```text
Node 1: ID 0x101, [node_id, counter]
Node 2: ID 0x102, [node_id, counter]
```

This is communication bring-up only. It does not control actuators or the
emergency-stop chain.
