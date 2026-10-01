ESP32-S3 RA-02 E-STOP Receiver

Vehicle-side LoRa receiver.

Source:
XIAO ESP32-C3 transmitter.

Receiver states:
UNKNOWN
CLEAR
ACTIVE
TIMEOUT

Safety rule:
No valid communication = STOP.

The GPIO output must connect to the independent safety chain,
not directly replace hardware emergency stop.
