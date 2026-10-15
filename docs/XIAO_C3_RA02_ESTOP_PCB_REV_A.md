# XIAO ESP32-C3 + RA-02 E-Stop PCB — Rev A

This is the frozen electrical layout for the C3 LoRa E-Stop transmitter.
The RA-02 header order is preserved exactly as requested.

## RA-02 header orientation

View the RA-02 from the component/antenna side with the two signal headers
facing the same direction as the PCB drawing.

### Left header — top to bottom

```text
L1  GND
L2  GND
L3  3.3V
L4  RST
L5  DIO0
```

### Right header — top to bottom

```text
R1  GND
R2  NSS
R3  MOSI
R4  MISO
R5  SCK
```

Do not mirror or reverse these headers during PCB placement.

## PCB netlist

| PCB net | RA-02 header | XIAO ESP32-C3 connection | Function |
|---|---|---|---|
| GND | L1 | GND | Ground |
| GND | L2 | GND | Duplicate ground |
| +3V3 | L3 | 3V3 | RA-02 supply |
| RA02_RESET | L4 | D2 | Radio reset |
| RA02_DIO0 | L5 | D1 | Radio interrupt |
| GND | R1 | GND | Ground |
| RA02_NSS | R2 | D3 | SPI chip select |
| RA02_MOSI | R3 | D10 | SPI controller-to-radio |
| RA02_MISO | R4 | D7 | SPI radio-to-controller |
| RA02_SCK | R5 | D5 | SPI clock |

The SCK trace crosses to the C3 D5/GPIO7 side on the PCB. This is expected;
the signal must remain connected to GPIO7 even though it exits the right RA-02
header.

## LAY37 connector

Add a separate 2-pin connector labelled `ESTOP`:

| PCB connector | XIAO ESP32-C3 | Function |
|---|---|---|
| ESTOP.1 | D4 | LAY37 NC |
| ESTOP.2 | GND | LAY37 COM |

Leave the LAY37 NO contact unconnected.

The firmware uses the internal pull-up on GPIO6:

```text
NC closed / switch released = D4 LOW  = ESTOP,0
NC open / switch pressed     = D4 HIGH = ESTOP,1
```

## Power and protection

- Use the XIAO regulated 3V3 rail for the RA-02.
- Do not route 5V to the RA-02 VCC pin.
- Place a 100 nF ceramic capacitor directly between +3V3 and GND near the
  RA-02 header.
- Add a 10 uF ceramic or low-ESR capacitor near the RA-02 supply entry.
- Keep the 3V3 and GND traces short and wide.
- Use a continuous ground plane where possible.
- Keep the RA-02 antenna connector at the PCB edge.
- Keep copper, ground pour, tracks, and components clear beneath and directly
  in front of the antenna element.
- Add labelled test pads for `3V3`, `GND`, `GPIO6`, `NSS`, `MOSI`, `MISO`, and
  `SCK`.

## Firmware pin constants

These values must not be changed without revising this PCB:

```python
SPI_SCK = 7       # D5
SPI_MISO = 20     # D7
SPI_MOSI = 10     # D10
RA02_CS = 5       # D3
RA02_RESET = 4    # D2
RA02_DIO0 = 3     # D1
ESTOP_PIN = 6     # D4
```

## Pre-fabrication checks

1. Confirm the RA-02 carrier pin labels and pitch match the selected PCB
   footprint.
2. Confirm the XIAO ESP32-C3 socket/header footprint and USB connector
   orientation.
3. Check that L1/L2/R1 are all GND and cannot be connected to 3V3 by a
   footprint-mirroring error.
4. Check continuity for every net before inserting the modules.
5. Power the PCB without the RA-02 first and verify 3.3 V.
6. Verify there is no short between 3V3 and GND.
7. Insert the RA-02 only after the 3.3 V test passes.

This document defines the electrical Rev A baseline; it is not a Gerber file.
