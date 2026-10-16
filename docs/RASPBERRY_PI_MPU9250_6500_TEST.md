# Raspberry Pi 4B + MPU-9250/6500 test

This is a direct I2C smoke test. The ESP32-S3 and all actuators remain
disconnected from this test circuit.

## Wiring

| MPU-9250/6500 | Raspberry Pi 4B | Physical pin |
|---|---|---:|
| VCC | 3V3 | 1 |
| GND | GND | 6 |
| SDA | GPIO2 / SDA1 | 3 |
| SCL | GPIO3 / SCL1 | 5 |
| ADD / AD0 | GND for address `0x68` | 6 |
| NCS / CS, if present | 3V3 for I2C mode | 1 |
| INT | Not connected for this test | - |

Use 3.3 V only. Do not connect the sensor signal pins to the Pi's 5 V rail.
If the module already contains I2C pull-ups, do not add another strong set.

## Enable and inspect I2C

```bash
sudo raspi-config
```

Select `Interface Options` -> `I2C` -> `Enable`, then reboot if requested.

Install the test utilities:

```bash
sudo apt update
sudo apt install -y i2c-tools python3-smbus
```

Confirm the bus and scan it:

```bash
ls -l /dev/i2c-1
i2cdetect -y 1
```

Expected address with ADD/AD0 connected to GND:

```text
     0 1 2 3 4 5 6 7 8 9 a b c d e f
60:                         68
```

Read the identity register directly:

```bash
i2cget -y 1 0x68 0x75
```

Expected identity:

| Result | Device |
|---|---|
| `0x70` | MPU-6500 |
| `0x71` | MPU-9250 |
| `0x73` | MPU-9255 |

## Stream measurements

```bash
cd /home/satvara/Github/AutoKart-firmware
python3 raspberry_pi/mpu9250_6500_i2c_test.py
```

A stationary, level sensor should report approximately 1 g total acceleration
and gyroscope readings near zero. Exact axis signs depend on board orientation.

The MPU-6500 contains only the accelerometer and gyroscope. MPU-9250 magnetic
compass testing is a separate step because its AK8963 is a second internal I2C
device.
