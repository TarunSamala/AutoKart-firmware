ESP32-S3 RA-02 E-STOP Receiver

Vehicle-side LoRa receiver.

Source:
Raspberry Pi Pico 2 transmitter in
`Lora/PICO2_RA02_ESTOP_PRODUCTION/`.

RA-02 receiver wiring on the ESP32-S3 terminal adapter:

| RA-02 | ESP32-S3 terminal adapter |
|---|---:|
| VCC | 3V3 |
| GND | GND |
| SCK | GPIO12 |
| MOSI | GPIO11 |
| MISO | GPIO10 |
| NSS / CS | GPIO9 |
| RESET | GPIO8 |
| DIO0 | GPIO17 |

Stop-request output: GPIO47 to an approved transistor/opto safety-interface
input. Never connect GPIO47 directly to a contactor or relay coil. GPIO15 is
reserved exclusively for the BLD-750 F/R control.

These pins avoid the BLDC control pins GPIO13, GPIO14, and GPIO15.

Receiver states:
UNKNOWN
CLEAR
ACTIVE
TIMEOUT

Safety rule:
No valid communication = STOP.

The GPIO output is only a stop request. It must connect through an approved
interface and does not by itself constitute an independent hardware emergency
stop or traction-power cut.
