# AutoKart RC automatic watchdog

The watchdog in `firmware/autokart_rc_s3_test.py` runs at startup and repeats
once per second. A missing item is displayed in the Wi-Fi dashboard but does
not prevent manual bench operation.

## Health meanings

| State | Meaning |
|---|---|
| `CONNECTED` | A real response, pulse, register value or heartbeat was received. |
| `MISSING` | The expected response was not detected. |
| `STALE` | A previously expected heartbeat is older than its timeout. |
| `UNVERIFIED` | The present wiring has no electrical feedback path, so firmware cannot honestly prove connection. |

## Automatic checks

| Item | Detection method |
|---|---|
| Steering motor driver | `UNVERIFIED`; R86mini STEP/DIR has no fault/readback wire. |
| Steering encoder | AS5600 response at I2C `0x36` plus magnet status. |
| Throttle motor driver | `UNVERIFIED`; BLD750 control wiring has no readback wire. |
| Throttle encoder | Enables W/PWM through the tested temporary SPI configuration, then checks for a valid PWM pulse on GPIO7. |
| GP8630N | I2C response at `0x58`. |
| Brake motor driver | `UNVERIFIED`; TB6600 STEP/DIR has no fault/readback wire. |
| Camera | Linux `/dev/video*` detection reported by the Raspberry Pi helper. |
| LoRa E-stop remote | `UNVERIFIED` until a valid remote packet is integrated/observed. |
| RA-02 | SX1278 version register `0x42` returns `0x12`. |
| Raspberry Pi 4B | Network heartbeat received within three seconds. |

## Raspberry Pi reporter

Connect the Pi to the ESP32 access point (`AutoKart-RC`) and run:

```bash
cd /home/satvara/Github/AutoKart-firmware
python3 tools/autokart_pi_health.py
```

The helper sends one report per second to `192.168.4.1`. It uses only the
Python standard library.

Motor-driver connection detection requires a wired `FAULT`, `ALARM`, output
readback or independent encoder response. Merely toggling an ESP32 output pin
cannot establish that a cable, driver or motor is physically connected.
