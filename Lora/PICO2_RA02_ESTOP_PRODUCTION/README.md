# Pico 2 + RA-02 production E-stop transmitter

This directory supersedes the XIAO ESP32-C3 transmitter for the AutoKart
LoRa E-stop. The vehicle-side receiver remains the ESP32-S3.

## Wiring

| RA-02 / LAY37 | Pico 2 |
|---|---|
| RA-02 VCC | 3V3(OUT) |
| RA-02 GND | GND |
| RA-02 MISO | GP16 |
| RA-02 NSS / CS | GP17 |
| RA-02 SCK | GP18 |
| RA-02 MOSI | GP19 |
| RA-02 RESET | GP20 |
| RA-02 DIO0 | GP21 |
| LAY37 NC | GP22 |
| LAY37 COM | GND |

Fit an external 4.7 kOhm pull-up from GP22 to 3V3. Power RA-02 from 3.3 V
only, fit local 100 nF and 10–100 uF supply capacitors, and connect the
433 MHz antenna before transmitting.

## Development ports

With both boards currently connected:

```text
/dev/ttyACM0 = Raspberry Pi Pico 2 transmitter
/dev/ttyACM1 = ESP32-S3 receiver
```

Rediscover the ports after unplugging or rebooting because Linux can assign
different ACM numbers.

## Install on Pico 2

```bash
cd /home/satvara/Github/AutoKart-firmware
source ~/python/bin/activate

python3 -m mpremote connect /dev/ttyACM0 fs cp \
  bench/ra02_lora_chat_common.py :ra02_lora_chat_common.py

python3 -m mpremote connect /dev/ttyACM0 fs cp \
  Lora/PICO2_RA02_ESTOP_PRODUCTION/main.py :main.py

python3 -m mpremote connect /dev/ttyACM0 soft-reset
python3 -m mpremote connect /dev/ttyACM0 repl
```

Expected startup output includes the SX1278 version `0x12` followed by five
`ESTOP,0,...` packets when the switch is released. Pressing or opening the
LAY37 NC circuit sends repeated `ESTOP,1,...` packets.

## Install receiver on ESP32-S3

```bash
python3 -m mpremote connect /dev/ttyACM1 fs cp \
  Lora/ESP32S3_RA02_ESTOP_RECEIVER_PRODUCTION/standalone_estop_receiver.py \
  :main.py

python3 -m mpremote connect /dev/ttyACM1 soft-reset
python3 -m mpremote connect /dev/ttyACM1 repl
```

This implementation deliberately has no healthy-state heartbeat or radio
disconnect timeout. The S3 changes E-stop state only after receiving a valid
`ESTOP,0,sequence` or `ESTOP,1,sequence` packet.
