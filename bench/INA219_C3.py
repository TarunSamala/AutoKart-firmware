from machine import Pin, I2C
import time


class INA219:
    REG_CONFIG = 0x00
    REG_SHUNT_VOLTAGE = 0x01
    REG_BUS_VOLTAGE = 0x02
    REG_POWER = 0x03
    REG_CURRENT = 0x04
    REG_CALIBRATION = 0x05

    def __init__(self, i2c, address=0x40):
        self.i2c = i2c
        self.address = address

        # Calibration for common INA219 breakout:
        # Shunt resistor = 0.1 ohm
        # Max expected current ≈ 3.2 A
        self.current_lsb = 0.0001       # 100 uA per bit
        self.power_lsb = self.current_lsb * 20

        # calibration = 0.04096 / (current_lsb * shunt_resistance)
        self.calibration_value = 4096

        self.configure()

    def write_register(self, reg, value):
        data = bytes([
            (value >> 8) & 0xFF,
            value & 0xFF
        ])
        self.i2c.writeto_mem(self.address, reg, data)

    def read_register(self, reg):
        data = self.i2c.readfrom_mem(self.address, reg, 2)
        return (data[0] << 8) | data[1]

    def read_signed_register(self, reg):
        value = self.read_register(reg)

        if value > 32767:
            value -= 65536

        return value

    def configure(self):
        # 32 V bus range
        # +/-320 mV shunt range
        # 12-bit ADC
        # Continuous shunt + bus voltage measurement
        config = 0x399F

        self.write_register(self.REG_CONFIG, config)
        self.write_register(
            self.REG_CALIBRATION,
            self.calibration_value
        )

    def shunt_voltage_mv(self):
        raw = self.read_signed_register(self.REG_SHUNT_VOLTAGE)

        # 10 uV per bit
        return raw * 0.01

    def bus_voltage_v(self):
        raw = self.read_register(self.REG_BUS_VOLTAGE)

        # Ignore CNVR and OVF bits
        raw >>= 3

        # 4 mV per bit
        return raw * 0.004

    def current_a(self):
        # Rewrite calibration in case device reset
        self.write_register(
            self.REG_CALIBRATION,
            self.calibration_value
        )

        raw = self.read_signed_register(self.REG_CURRENT)

        return raw * self.current_lsb

    def current_ma(self):
        return self.current_a() * 1000

    def power_w(self):
        self.write_register(
            self.REG_CALIBRATION,
            self.calibration_value
        )

        raw = self.read_register(self.REG_POWER)

        return raw * self.power_lsb

    def load_voltage_v(self):
        return self.bus_voltage_v() + (
            self.shunt_voltage_mv() / 1000
        )


# --------------------------------------------------
# XIAO ESP32-C3 I2C
# D4 = GPIO6 = SDA
# D5 = GPIO7 = SCL
# --------------------------------------------------

i2c = I2C(
    0,
    sda=Pin(6),
    scl=Pin(7),
    freq=100000
)

print("\nINA219 Current Monitor")
print("----------------------")

devices = i2c.scan()

if 0x40 not in devices:
    print("ERROR: INA219 not detected!")
    print("Detected devices:", [hex(x) for x in devices])

    while True:
        time.sleep(1)

print("INA219 detected at address 0x40")

ina219 = INA219(i2c)

time.sleep(0.5)

while True:
    try:
        bus_voltage = ina219.bus_voltage_v()
        shunt_voltage = ina219.shunt_voltage_mv()
        current = ina219.current_ma()
        power = ina219.power_w()
        load_voltage = ina219.load_voltage_v()

        print("\n---------------------------")
        print("Bus Voltage   : {:.3f} V".format(bus_voltage))
        print("Shunt Voltage : {:.3f} mV".format(shunt_voltage))
        print("Load Voltage  : {:.3f} V".format(load_voltage))
        print("Current       : {:.2f} mA".format(current))
        print("Power         : {:.3f} W".format(power))

    except Exception as e:
        print("INA219 read error:", e)

    time.sleep(1)