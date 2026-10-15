# ESP32-S3 + RA-02 LoRa E-stop receiver

This is the vehicle-side receiver for the Raspberry Pi Pico 2 + RA-02 +
LAY37 transmitter.

For the complete integrated vehicle allocation, see
`docs/AUTOKART_ESP32S3_MASTER_PINOUT.md`.

## Wiring

| RA-02 pin | ESP32-S3 pin |
|---|---:|
| VCC / 3.3V | 3V3 |
| GND | GND |
| SCK | GPIO12 |
| MOSI | GPIO11 |
| MISO | GPIO10 |
| NSS / CS | GPIO9 |
| RESET | GPIO8 |
| DIO0 | GPIO17 |

```text
ESP32-S3 3V3  ───────── RA-02 VCC
ESP32-S3 GND  ───────── RA-02 GND
ESP32-S3 IO12 ───────── RA-02 SCK
ESP32-S3 IO11 ───────── RA-02 MOSI
ESP32-S3 IO10 ───────── RA-02 MISO
ESP32-S3 IO9  ───────── RA-02 NSS / CS
ESP32-S3 IO8  ───────── RA-02 RESET
ESP32-S3 IO17 ───────── RA-02 DIO0
```

Connect a 433 MHz antenna before transmitting. Power the RA-02 from 3.3 V
only. Place a 100 nF ceramic capacitor and a 10–100 µF bulk capacitor close
to the RA-02 between VCC and GND. A 10 kΩ pull-up from NSS/CS to 3.3 V is
recommended so the radio remains deselected during ESP32-S3 startup.

This mapping leaves the R86mini GPIO4/5 and BLD750 GPIO13/14/15 controls
untouched. GPIO8/9 are no longer available for the integrated I2C bus; the
master allocation moves I2C to GPIO1/2.

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
