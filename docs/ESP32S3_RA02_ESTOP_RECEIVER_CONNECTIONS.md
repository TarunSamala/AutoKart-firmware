# ESP32-S3 + RA-02 LoRa E-stop receiver

This is the vehicle-side receiver for the XIAO ESP32-C3 + RA-02 + LAY37
transmitter.

## Wiring

| RA-02 pin | ESP32-S3 pin |
|---|---:|
| VCC / 3.3V | 3V3 |
| GND | GND |
| SCK | GPIO4 |
| MOSI | GPIO5 |
| MISO | GPIO6 |
| NSS / CS | GPIO7 |
| RESET | GPIO10 |
| DIO0 | GPIO18 |

```text
ESP32-S3 3V3  ───────── RA-02 VCC
ESP32-S3 GND  ───────── RA-02 GND
ESP32-S3 IO4  ───────── RA-02 SCK
ESP32-S3 IO5  ───────── RA-02 MOSI
ESP32-S3 IO6  ───────── RA-02 MISO
ESP32-S3 IO7  ───────── RA-02 NSS / CS
ESP32-S3 IO10 ───────── RA-02 RESET
ESP32-S3 IO18 ───────── RA-02 DIO0
```

Connect a 433 MHz antenna before transmitting. Power the RA-02 from 3.3 V
only. Place a 100 nF ceramic capacitor and a 10–100 µF bulk capacitor close
to the RA-02 between VCC and GND. A 10 kΩ pull-up from NSS/CS to 3.3 V is
recommended so the radio remains deselected during ESP32-S3 startup.

GPIO4 and GPIO5 overlap with the temporary R86mini STEP/DIR motor tests.
Disconnect those motor-driver signal wires before connecting this radio.
The mapping leaves the BLDC GPIO13/14/15 controls untouched.

## Receiver behavior

The isolated receiver accepts only:

```text
ESTOP,0,sequence
ESTOP,1,sequence
```

It starts in `UNKNOWN`, changes to `CLEAR` after a valid state `0` packet,
and changes to `ACTIVE` after a valid state `1` packet. It does not apply a
heartbeat timeout and does not control any actuator.

## Upload

Upload the shared SX1278 driver and receiver program:

```bash
python3 -m mpremote connect "$PORT" fs cp \
  bench/ra02_lora_chat_common.py :ra02_lora_chat_common.py

python3 -m mpremote connect "$PORT" fs cp \
  bench/esp32s3_lora_estop_receiver_test.py :main.py

python3 -m mpremote connect "$PORT" soft-reset
python3 -m mpremote connect "$PORT" repl
```

Expected released output:

```text
RX: ESTOP,0,1
E-STOP CLEAR: SWITCH RELEASED
```

Expected pressed output:

```text
RX: ESTOP,1,2
*** E-STOP ACTIVE: STOP / BRAKE REQUIRED ***
```
