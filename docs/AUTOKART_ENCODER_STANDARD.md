# AutoKart encoder and filter standard

## Final encoder roles

| Sensor | Mechanical mounting | Primary outputs |
|---|---|---|
| AS5600 #1 | NEMA34 steering/output shaft | Signed steering angle about calibrated center |
| AS5600 #2 | NEMA23 brake shaft | Clockwise brake travel from calibrated released position |
| AS5047P | BLDC motor shaft | Motor angle, motor RPM, estimated axle RPM, speed and distance |

## Integrated buses

The two AS5600 devices both have fixed address `0x36`; they must not be placed
on the same un-multiplexed bus. Use the available TCA9548A:

```text
ESP32-S3 GPIO1 SDA / GPIO2 SCL
  +-- GP8630N 0x58 (upstream bus)
  +-- TCA9548A 0x70
        +-- channel 0: steering AS5600 0x36
        +-- channel 1: brake AS5600 0x36

AS5047P SPI mode 1
  CS GPIO16 / SCK GPIO18 / MOSI GPIO41 / MISO GPIO42
```

The TCA9548A and GP8630N share the upstream I2C wires because their addresses
are different. Fit only the pull-ups needed for reliable 3.3 V operation.

## Filtering baseline

Implementation: `firmware/encoder_filters.py`.

| Signal | Acquisition target | Filter | Initial constant |
|---|---:|---|---:|
| Steering AS5600 | 200 Hz | unwrap, median 3, first-order LPF | 10 ms |
| Brake AS5600 | 200 Hz | unwrap, median 3, first-order LPF | 10 ms |
| BLDC AS5047P | 200 Hz | unwrap, count-difference RPM, first-order LPF | 30 ms |

Reject AS5600 samples when I2C fails, the magnet is invalid, or angular rate
exceeds the configured physical bound. Reject AS5047P samples on SPI parity,
sensor diagnostic/error, stale timing, or impossible RPM. Do not update a
filter with invalid data and do not treat its previous output as fresh.

The BLDC distance calculation assumes the encoder is on the motor shaft:

```text
axle_revolutions = motor_revolutions / motor_revolutions_per_axle_revolution
distance_m = axle_revolutions * measured_loaded_wheel_circumference_m
```

Current preliminary reduction is `44/14 = 3.142857` motor revolutions per
axle revolution. Replace nominal tyre circumference with a physically measured
loaded rolling circumference before using distance as vehicle telemetry.

## Calibration conventions

- Steering: mechanically centered position is `0 deg`; left/right signs must
  be frozen after observing the mounted sensor direction.
- Brake: mechanically released position is `0 deg`; configure direction so
  clockwise application is positive.
- BLDC: no positional zero is required for RPM/distance; cumulative distance
  starts at zero each VCU boot unless a separate odometer policy is adopted.
