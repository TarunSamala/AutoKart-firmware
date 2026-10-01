# ESP32-S3 RA-02 Receiver Validation

Hardware:
- ESP32-S3
- RA-02 433MHz
- XIAO C3 transmitter

Wiring verified:
- SCK GPIO4
- MOSI GPIO5
- MISO GPIO6
- CS GPIO7
- RESET GPIO10
- DIO0 GPIO18

Tests:

[ ] Receiver boots UNKNOWN
[ ] Valid CLEAR packet accepted
[ ] Valid ESTOP packet accepted
[ ] Invalid CRC rejected
[ ] Sequence changes detected
[ ] Transmitter power OFF causes STOP
[ ] RA-02 disconnected causes STOP
[ ] Reset causes safe output

Vehicle tests:
[ ] Bench only
[ ] Wheels lifted
[ ] Manual low speed
[ ] Fault injection

PASS:
Loss of communication must never allow motion.
