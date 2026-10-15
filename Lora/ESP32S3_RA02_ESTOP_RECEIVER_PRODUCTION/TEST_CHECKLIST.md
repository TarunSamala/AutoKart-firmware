# ESP32-S3 RA-02 Receiver Validation

Hardware:
- ESP32-S3
- RA-02 433MHz
- Raspberry Pi Pico 2 transmitter

Wiring verified:
- SCK GPIO12
- MOSI GPIO11
- MISO GPIO10
- CS GPIO9
- RESET GPIO8
- DIO0 GPIO17

Tests:

[ ] Receiver boots UNKNOWN
[ ] Valid CLEAR packet accepted
[ ] Valid ESTOP packet accepted
[ ] Invalid CRC rejected
[ ] Sequence changes detected
[ ] Pico startup sends its actual switch state
[ ] Pressed/open NC circuit sends STOP repeatedly
[ ] Releasing the switch sends CLEAR repeatedly
[ ] Transmitter power OFF does not create a false state transition
[ ] Receiver reset starts in UNKNOWN

Vehicle tests:
[ ] Bench only
[ ] Wheels lifted
[ ] Manual low speed
[ ] Fault injection

Current deliberate behavior:
There is no healthy-state heartbeat or disconnect timeout. The receiver
changes state only after a valid explicit packet. An independent hardware
safety chain must provide the final fail-safe behavior.
