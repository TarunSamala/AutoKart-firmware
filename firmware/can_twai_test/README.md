# AutoKart CAN/TWAI bench test

Two-node communication test:

- ESP32-S3 + SN65HVD230: node 1
- Seeed XIAO ESP32-C3 + SN65HVD230: node 2

It uses ESP-IDF's current on-chip TWAI driver, CAN 2.0 standard frames, and
500 kbit/s. It is communication bring-up only; it does not control steering,
braking, traction, or the emergency-stop chain.

## Transceiver wiring

| SN65HVD230 | ESP32-S3 node 1 | XIAO ESP32-C3 node 2 |
|---|---:|---:|
| VCC | 3V3 | 3V3 |
| GND | GND | GND |
| TXD / D | GPIO16 | D1 / GPIO3 |
| RXD / R | GPIO17 | D2 / GPIO4 |
| CANH | CAN bus H | CAN bus H |
| CANL | CAN bus L | CAN bus L |
| RS, if present | GND | GND |

Use one twisted pair for CANH/CANL. Install exactly one 120-ohm termination
resistor at each physical end of the bus. Do not terminate intermediate nodes.
Connect the node grounds together and power the SN65HVD230 logic side from
3.3 V.

GPIO16/17 leave the existing ESP32-S3 BLDC controls on GPIO13/14/15 alone.
The C3 GPIO3/4 assignment is for this isolated CAN test; it conflicts with
the earlier C3 RA-02 mapping if that radio remains connected.

## Build and flash

Load the ESP-IDF environment first. Use separate build directories:

```bash
cd firmware/can_twai_test

idf.py -B build-s3 set-target esp32s3
idf.py -B build-s3 build
idf.py -B build-s3 -p /dev/ttyACM0 flash monitor
```

For the XIAO ESP32-C3:

```bash
idf.py -B build-c3 set-target esp32c3
idf.py -B build-c3 build
idf.py -B build-c3 -p /dev/ttyACM1 flash monitor
```

Replace the serial ports after identifying them with `find /dev` or `lsusb`.

## Serial commands

Both nodes automatically send one frame per second:

```text
ID 0x101: node 1, counter
ID 0x102: node 2, counter
```

Commands from either serial monitor:

```text
send 0x120 01 02 AA
ping
auto off
auto on
status
help
reset
```

`send` accepts hexadecimal or decimal standard IDs and up to eight hexadecimal
payload bytes. Received frames print their ID, type, DLC, and payload.

## Acceptance test

1. Power both nodes and verify `CAN/TWAI test ready` on both monitors.
2. Confirm both transceivers have 3.3 V and common ground.
3. Confirm CANH-to-CANH and CANL-to-CANL with 120 ohms at both bus ends.
4. Type `ping` on node 1 and verify an RX line on node 2.
5. Type `send 0x120 01 02 AA` on node 2 and verify it on node 1.
6. Type `auto off` on both nodes and test manual messages.
7. Disconnect one node and record the resulting transmit/error behavior.
