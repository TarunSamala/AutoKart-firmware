# XIAO ESP32-C3 RA-02 E-STOP Validation

## Electrical
- [ ] Confirm RA-02 supply = 3.3V
- [ ] Antenna installed before TX
- [ ] CS pull-up installed
- [ ] Capacitors installed near radio
- [ ] NC switch wiring verified

## Functional
- [ ] Power transmitter
- [ ] Release mushroom = SAFE packets
- [ ] Press mushroom = ESTOP packets
- [ ] Disconnect NC wire = ESTOP
- [ ] Reset transmitter

## Communication
- [ ] Verify CRC rejection
- [ ] Verify sequence increment
- [ ] Switch off transmitter
- [ ] Receiver stops within timeout

## Vehicle integration
- [ ] Bench only
- [ ] Wheels lifted
- [ ] Manual low speed
- [ ] Fault injection
- [ ] Emergency stop measurement

PASS REQUIREMENT:
No communication or hardware fault may allow uncontrolled motion.
