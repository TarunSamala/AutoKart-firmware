ESP32-S3 RA-02 E-STOP Receiver

Vehicle-side LoRa receiver.

Source:
XIAO ESP32-C3 transmitter.

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

These pins avoid the BLDC control pins GPIO13, GPIO14, and GPIO15.

Receiver states:
UNKNOWN
CLEAR
ACTIVE
TIMEOUT

Safety rule:
No valid communication = STOP.

The GPIO output must connect to the independent safety chain,
not directly replace hardware emergency stop.
