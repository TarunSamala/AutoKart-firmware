# ESP32-S3 encoder test connections

These are standalone bench tests. They may overlap with AutoKart actuator pins during bring-up, but disconnect the actuator hardware while testing.

## AS5600 — I2C

The AS5600 uses I2C address `0x36` and exposes raw/mapped angle registers.

```text
AS5600 VCC  -> ESP32-S3 3V3
AS5600 GND  -> ESP32-S3 GND
AS5600 SDA  -> ESP32-S3 GPIO8
AS5600 SCL  -> ESP32-S3 GPIO9
```

Use 4.7 kOhm pull-ups from SDA and SCL to 3.3 V if the module does not already include them. Place the magnet centered above the sensor IC.

## AS5047P — SPI

The AS5047P uses SPI mode 1 and the `ANGLECOM` register (`0x3FFF`).

```text
AS5047P VCC   -> ESP32-S3 3V3 input
AS5047P GND   -> ESP32-S3 GND
AS5047P CLK   -> ESP32-S3 GPIO13
AS5047P MOSI  -> ESP32-S3 GPIO14
AS5047P MISO  -> ESP32-S3 GPIO15
AS5047P CSn   -> ESP32-S3 GPIO16
```

The magnet must be the correct diametrically magnetized type and centered over the AS5047P IC. The test does not write configuration or zero-position registers.

## Test files

```text
bench/as5600_esp32s3_test.py
bench/as5047p_esp32s3_test.py
```

