"""Minimal ESP32-S3 BLD750 + LoRa + mechanical-brake bench test.

WHEELS-OFF-GROUND TESTING ONLY.

Before boot/reset, put the mechanical brake at its fully released/home
position. This deliberately simple test has no encoder, Wi-Fi, dashboard,
heartbeat timeout, command protocol, or automatic recovery.

Behaviour:
  Boot               -> run BLD750 at 2.5 V immediately
  ESTOP,0,sequence   -> ignored; motor keeps running
  ESTOP,1,sequence   -> latch BLD750 electric stop/brake, then push brake CW

After an E-stop, reset the ESP32 to perform another run. Releasing the Pico
button never restarts traction.
"""

import time
from machine import I2C, Pin, SPI
from ra02_lora_chat_common import SX1278


# GP8630N 0-10 V output.
GP_SDA_GPIO = 7
GP_SCL_GPIO = 21
GP_ADDRESS = 0x58
GP_MODE_REGISTER = 0x01
GP_DAC_REGISTER = 0x02
GP_MODE_0_TO_10_V = 0x1C
RUN_VOLTAGE = 2.5

# BLD750 through PN2222 open-collector stages.
# GPIO HIGH turns the transistor on and pulls the BLD input to COM.
BLD_BRK_GPIO = 13
BLD_EN_GPIO = 14
BLD_FR_GPIO = 15

# RA-02 SX1278 receiver.
RA02_RESET_GPIO = 8
RA02_CS_GPIO = 9
RA02_MISO_GPIO = 10
RA02_MOSI_GPIO = 11
RA02_SCK_GPIO = 12
RA02_DIO0_GPIO = 17

# Open-loop NEMA23/TB6600 mechanical brake.
BRAKE_STEP_GPIO = 38
BRAKE_DIR_GPIO = 39
BRAKE_ENABLE_GPIO = 40
BRAKE_ENABLE_LEVEL = 0
# Confirmed on the installed actuator: GPIO LOW pushes the brake pin inward.
BRAKE_CLOCKWISE_LEVEL = 0
BRAKE_APPLY_DEGREES = 30.0
BRAKE_PULSES_PER_REVOLUTION = 3200
BRAKE_STEP_HIGH_US = 20
BRAKE_STEP_PERIOD_US = 900


# Establish electrical stop before initialising I2C or LoRa.
bld_en = Pin(BLD_EN_GPIO, Pin.OUT, value=0)       # STOP
bld_brk = Pin(BLD_BRK_GPIO, Pin.OUT, value=0)    # BRAKE APPLIED
bld_direction = Pin(BLD_FR_GPIO, Pin.OUT, value=1)  # fixed forward

brake_step = Pin(BRAKE_STEP_GPIO, Pin.OUT, value=0)
brake_direction = Pin(BRAKE_DIR_GPIO, Pin.OUT, value=0)
brake_enable = Pin(
    BRAKE_ENABLE_GPIO,
    Pin.OUT,
    value=1 - BRAKE_ENABLE_LEVEL,
)


gp = I2C(
    1,
    sda=Pin(GP_SDA_GPIO),
    scl=Pin(GP_SCL_GPIO),
    freq=100_000,
)
if GP_ADDRESS not in gp.scan():
    raise RuntimeError("GP8630N NOT FOUND AT 0x58")
gp.writeto_mem(
    GP_ADDRESS,
    GP_MODE_REGISTER,
    bytes((GP_MODE_0_TO_10_V,)),
)
time.sleep_ms(100)


def set_voltage(volts):
    volts = max(0.0, min(10.0, float(volts)))
    raw = int(volts / 10.0 * 65535)
    gp.writeto_mem(
        GP_ADDRESS,
        GP_DAC_REGISTER,
        bytes((raw & 0xFF, (raw >> 8) & 0xFF)),
    )


set_voltage(0.0)


radio_spi = SPI(
    2,
    baudrate=2_000_000,
    polarity=0,
    phase=0,
    bits=8,
    firstbit=SPI.MSB,
    sck=Pin(RA02_SCK_GPIO),
    mosi=Pin(RA02_MOSI_GPIO),
    miso=Pin(RA02_MISO_GPIO),
)
radio = SX1278(
    radio_spi,
    Pin(RA02_CS_GPIO, Pin.OUT, value=1),
    Pin(RA02_RESET_GPIO, Pin.OUT, value=1),
    433_000_000,
)
radio_dio0 = Pin(RA02_DIO0_GPIO, Pin.IN)
radio.begin()


brake_pulses = int(round(
    BRAKE_APPLY_DEGREES * BRAKE_PULSES_PER_REVOLUTION / 360.0
))
mechanical_brake_applied = False
stop_latched = False
clear_packets = 0


def move_mechanical_brake(clockwise):
    brake_enable.value(BRAKE_ENABLE_LEVEL)
    brake_direction.value(
        BRAKE_CLOCKWISE_LEVEL
        if clockwise
        else 1 - BRAKE_CLOCKWISE_LEVEL
    )
    time.sleep_ms(2)
    for _ in range(brake_pulses):
        brake_step.value(1)
        time.sleep_us(BRAKE_STEP_HIGH_US)
        brake_step.value(0)
        time.sleep_us(BRAKE_STEP_PERIOD_US - BRAKE_STEP_HIGH_US)
    # Keep the driver enabled after the clockwise apply move so the actuator
    # holds the brake pin in its pushed-in position.


def apply_both_brakes():
    global mechanical_brake_applied
    # Electrical stop happens first; it is not delayed by mechanical movement.
    set_voltage(0.0)
    bld_en.value(0)       # PN2222 off -> BLD EN stop
    bld_brk.value(0)      # PN2222 off -> BLD electric brake applied
    if not mechanical_brake_applied:
        move_mechanical_brake(clockwise=True)
        mechanical_brake_applied = True
    print("STOP: SV=0.0 V | EN=STOP | BLD BRK=APPLIED | MECH=CW/APPLIED")


def start_bldc():
    # The brake must already be physically released/home before ESP32 reset.
    bld_brk.value(1)      # PN2222 on -> BLD brake released
    bld_en.value(1)       # PN2222 on -> BLD run
    set_voltage(RUN_VOLTAGE)
    print(
        "RUNNING: BRK GPIO={} | EN GPIO={} | SV COMMAND=2.5 V".format(
            bld_brk.value(), bld_en.value()
        )
    )


def packet_state(packet):
    try:
        fields = packet.decode().strip().split(",")
        if len(fields) != 3 or fields[0] != "ESTOP":
            return None
        state = int(fields[1])
        return state if state in (0, 1) else None
    except Exception:
        return None


print("SIMPLE BLD750 + LORA + MECHANICAL BRAKE BENCH TEST READY")
print("Mechanical brake must be physically RELEASED/HOME at boot")
print("Starting BLDC at 2.5 V now")
start_bldc()
print("ESTOP,0 packets are ignored")
print("ESTOP,1 -> LATCHED ELECTRIC BRAKE + CW MECHANICAL BRAKE")

while True:
    packet = radio.receive()
    if packet is None:
        time.sleep_ms(5)
        continue

    state = packet_state(packet)
    if state == 0:
        clear_packets += 1
        if clear_packets == 1 or clear_packets % 10 == 0:
            print("RX CLEAR:", packet.decode().strip(), "| BLDC remains at 2.5 V")
        continue
    if state != 1 or stop_latched:
        continue

    stop_latched = True
    print("RX:", packet.decode().strip())
    apply_both_brakes()
