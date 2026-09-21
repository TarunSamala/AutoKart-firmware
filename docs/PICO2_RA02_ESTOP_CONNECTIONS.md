# Pico 2 + RA-02 + LAY37 emergency-stop test

## Wiring

| RA-02 | Raspberry Pi Pico 2 |
|---|---|
| 3.3V | 3V3(OUT) |
| GND | GND |
| MISO | GP16 |
| MOSI | GP19 |
| SCK | GP18 |
| NSS / CS | GP17 |
| RESET | GP20 |
| DIO0 | GP21 |

LAY37 wiring:

```text
Pico GP22 ---- LAY37 NC contact ---- GND
Pico GP22 ---- 4.7 kΩ resistor ---- 3.3 V
```

With the normally-closed contact healthy, GP22 is LOW. When the emergency
stop is pressed, the NC contact opens and GP22 becomes HIGH. A disconnected
wire also becomes HIGH, so the input fails safe.

Use 3.3 V only. Do not connect the RA-02 to 5 V. Keep the RA-02 supply short
and add local 100 nF and 10–100 µF decoupling if the radio resets during TX.

## Test file

`bench/pico2_ra02_estop_test.py` reads the RA-02 SX1278 register `0x42` and
prints the LAY37 state. For the live fail-stop heartbeat, use
`bench/pico2_ra02_estop_transmitter.py`. It sends `ESTOP,0,sequence` while
the NC contact is healthy and `ESTOP,1,sequence` when pressed or open.

Expected output includes:

```text
RA-02 RegVersion: 0x12
RA-02 SPI: FOUND
ESTOP: OK / NC CLOSED | GPIO22=0
```

Pressing the emergency stop should produce:

```text
ESTOP: STOP/FAULT | GPIO22=1
```

If the state is reversed on the physical LAY37 module, stop and verify the
actual NC/COM contact with a continuity meter before changing firmware.
