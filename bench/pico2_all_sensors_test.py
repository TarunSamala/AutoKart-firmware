"""Pico 2 integrated sensor test: 2x AS5600, AS5047P, and RA-02.

No actuator output is initialized. The RA-02 is placed in receive mode.

PCA9548A/I2C0: SDA GP0, SCL GP1, address 0x70
  channel 0 -> steering AS5600 (0x36)
  channel 1 -> brake AS5600 (0x36)

AS5047P/SPI1: SCK GP10, MOSI GP11, MISO GP12, CS GP13 (SPI mode 1)
RA-02/SPI0:   MISO GP16, CS GP17, SCK GP18, MOSI GP19,
               RESET GP20, DIO0 GP21 (SPI mode 0)

Install encoder_filters.py and ra02_lora_chat_common.py beside main.py.
"""

from machine import I2C, Pin, SPI
import time

from encoder_filters import AS5600AngleFilter, AS5047KinematicsFilter, monotonic_us
from ra02_lora_chat_common import SX1278


# I2C multiplexer and AS5600 sensors
I2C_SDA = 0
I2C_SCL = 1
PCA_ADDRESS = 0x70
AS5600_ADDRESS = 0x36
STEERING_CHANNEL = 0
BRAKE_CHANNEL = 1

# AS5047P on SPI1
AS5047_SCK = 10
AS5047_MOSI = 11
AS5047_MISO = 12
AS5047_CS = 13

# RA-02 on the previously tested Pico SPI0 wiring
RA02_MISO = 16
RA02_CS = 17
RA02_SCK = 18
RA02_MOSI = 19
RA02_RESET = 20
RA02_DIO0 = 21

SAMPLE_INTERVAL_MS = 5
DISPLAY_INTERVAL_MS = 500

AS5600_STATUS = 0x0B
AS5600_ANGLE = 0x0E
AS5600_AGC = 0x1A
AS5600_MAGNITUDE = 0x1B

AS5600_MD = 0x20
AS5600_ML = 0x10
AS5600_MH = 0x08

AS5047_ERRFL = 0x0001
AS5047_DIAAGC = 0x3FFC
AS5047_MAG = 0x3FFD
AS5047_ANGLECOM = 0x3FFF

AS5047_MAGL = 1 << 11
AS5047_MAGH = 1 << 10
AS5047_COF = 1 << 9
AS5047_LF = 1 << 8


def read_u16_i2c(address, register):
    data = i2c.readfrom_mem(address, register, 2)
    return (data[0] << 8) | data[1]


def select_pca_channel(channel):
    if channel < 0 or channel > 7:
        raise ValueError("PCA channel must be 0-7")
    # Exactly one channel is enabled so duplicate AS5600 address 0x36 is safe.
    i2c.writeto(PCA_ADDRESS, bytes((1 << channel,)))


def disable_pca_channels():
    i2c.writeto(PCA_ADDRESS, b"\x00")


def as5600_state(status):
    if status & AS5600_MH:
        return "MAGNET TOO STRONG"
    if status & AS5600_ML:
        return "MAGNET TOO WEAK / NO MAGNET"
    if status & AS5600_MD:
        return "MAGNET OK"
    return "NO MAGNET"


def as5600_ok(status):
    return (
        bool(status & AS5600_MD)
        and not bool(status & AS5600_ML)
        and not bool(status & AS5600_MH)
    )


def read_as5600(channel, angle_filter):
    select_pca_channel(channel)
    status = i2c.readfrom_mem(AS5600_ADDRESS, AS5600_STATUS, 1)[0]
    raw = read_u16_i2c(AS5600_ADDRESS, AS5600_ANGLE) & 0x0FFF
    magnitude = read_u16_i2c(AS5600_ADDRESS, AS5600_MAGNITUDE) & 0x0FFF
    agc = i2c.readfrom_mem(AS5600_ADDRESS, AS5600_AGC, 1)[0]
    filtered = angle_filter.update(raw, magnet_ok=as5600_ok(status))
    return {
        "raw": raw,
        "absolute_deg": raw * 360.0 / 4096.0,
        "status": status,
        "state": as5600_state(status),
        "magnitude": magnitude,
        "agc": agc,
        "filtered": filtered,
    }


def parity_is_odd(value):
    return bool(bin(value & 0xFFFF).count("1") & 1)


def as5047_command(register):
    frame = 0x4000 | (register & 0x3FFF)
    if parity_is_odd(frame):
        frame |= 0x8000
    return frame


def as5047_transfer(frame):
    tx = bytes(((frame >> 8) & 0xFF, frame & 0xFF))
    rx = bytearray(2)
    as5047_cs.value(0)
    as5047_spi.write_readinto(tx, rx)
    as5047_cs.value(1)
    time.sleep_us(2)
    return (rx[0] << 8) | rx[1]


def as5047_read(register):
    as5047_transfer(as5047_command(register))
    response = as5047_transfer(0x0000)
    return {
        "data": response & 0x3FFF,
        "error": bool(response & 0x4000),
        "parity_error": parity_is_odd(response),
    }


def as5047_state(diaagc):
    if diaagc & AS5047_MAGH:
        return "MAGNET TOO STRONG"
    if diaagc & AS5047_MAGL:
        return "MAGNET TOO WEAK / NO MAGNET"
    if diaagc & AS5047_COF:
        return "MAGNET INVALID / CORDIC OVERFLOW"
    if not diaagc & AS5047_LF:
        return "MAGNET NOT READY"
    return "MAGNET OK"


def read_as5047():
    angle = as5047_read(AS5047_ANGLECOM)
    diagnostic = as5047_read(AS5047_DIAAGC)
    magnitude = as5047_read(AS5047_MAG)
    errors = as5047_read(AS5047_ERRFL)
    state = as5047_state(diagnostic["data"])
    spi_ok = not any(
        item["error"] or item["parity_error"]
        for item in (angle, diagnostic, magnitude)
    )
    filtered = throttle_filter.update(
        angle["data"],
        timestamp_us=monotonic_us(),
        sample_ok=spi_ok and state == "MAGNET OK",
    )
    return {
        "raw": angle["data"],
        "angle_deg": angle["data"] * 360.0 / 16384.0,
        "state": state,
        "magnitude": magnitude["data"],
        "agc": diagnostic["data"] & 0xFF,
        "errors": errors["data"],
        "spi_ok": spi_ok,
        "filtered": filtered,
    }


def filtered_angle(sample):
    result = sample["filtered"]
    return result["angle_deg"] if result is not None else None


def print_as5600(name, sample, angle_filter):
    angle = filtered_angle(sample)
    angle_text = "REJECTED" if angle is None else "{:+8.3f} deg".format(angle)
    print(
        "{} AS5600: raw {:4d} abs {:7.3f} filtered {} | "
        "MAG {:4d} AGC {:3d} | {} | valid {} rejected {}".format(
            name,
            sample["raw"],
            sample["absolute_deg"],
            angle_text,
            sample["magnitude"],
            sample["agc"],
            sample["state"],
            angle_filter.valid_samples,
            angle_filter.rejected_samples,
        )
    )


def print_as5047(sample):
    result = sample["filtered"]
    if result is None:
        motion = "FILTER REJECTED"
    else:
        turns = result["total_motor_counts"] / 16384.0
        motion = "RPM {:+8.2f} turns {:+9.4f}".format(
            result["motor_rpm"], turns
        )
    print(
        "THROTTLE AS5047P: raw {:5d} angle {:8.3f} | {} | "
        "MAG 0x{:04X} AGC 0x{:02X} ERR 0x{:04X} | {} | {}".format(
            sample["raw"],
            sample["angle_deg"],
            motion,
            sample["magnitude"],
            sample["agc"],
            sample["errors"],
            "SPI OK" if sample["spi_ok"] else "SPI ERROR",
            sample["state"],
        )
    )


def receive_lora():
    packet = radio.receive()
    if packet:
        try:
            text = packet.decode().strip()
        except Exception:
            text = repr(packet)
        print("LORA RX:", text)
        return text
    return None


def print_all():
    print_as5600("STEERING", latest_steering, steering_filter)
    print_as5600("BRAKE   ", latest_brake, brake_filter)
    print_as5047(latest_throttle)
    print("RA-02: version 0x{:02X} | RX mode | last packet: {}".format(
        ra02_version,
        "NONE" if last_lora_packet is None else last_lora_packet,
    ))


def collect(duration_ms, display=False):
    global last_lora_packet
    started = time.ticks_ms()
    last_display = started
    while time.ticks_diff(time.ticks_ms(), started) < duration_ms:
        latest = receive_lora()
        if latest is not None:
            last_lora_packet = latest
        acquire_all_without_lora()
        current = time.ticks_ms()
        if display and time.ticks_diff(current, last_display) >= DISPLAY_INTERVAL_MS:
            print_all()
            last_display = current
        time.sleep_ms(SAMPLE_INTERVAL_MS)


def acquire_all_without_lora():
    global latest_steering, latest_brake, latest_throttle
    latest_steering = read_as5600(STEERING_CHANNEL, steering_filter)
    latest_brake = read_as5600(BRAKE_CHANNEL, brake_filter)
    disable_pca_channels()
    latest_throttle = read_as5047()


def show_help():
    print("Commands:")
    print("  read / r       - collect for 1 second and show all sensors")
    print("  stream         - continuous sensor and LoRa monitoring")
    print("  zero steering  - set current steering position to 0 degrees")
    print("  zero brake     - set current brake position to 0 degrees")
    print("  zero throttle  - reset BLDC cumulative turns")
    print("  send <message> - transmit a LoRa test message")
    print("  status         - show all sensors")
    print("  help           - show commands")
    print("  q              - stop")


print()
print("========================================")
print(" PICO 2 AUTOKART ALL-SENSOR TEST")
print("========================================")
print("NO ACTUATOR OUTPUTS ARE INITIALIZED")

i2c = I2C(0, sda=Pin(I2C_SDA), scl=Pin(I2C_SCL), freq=400_000)
upstream_devices = i2c.scan()
print("I2C upstream:", [hex(device) for device in upstream_devices])
if PCA_ADDRESS not in upstream_devices:
    raise RuntimeError("PCA9548A not detected at 0x70")

for channel, name in ((STEERING_CHANNEL, "steering"), (BRAKE_CHANNEL, "brake")):
    select_pca_channel(channel)
    devices = i2c.scan()
    print("PCA channel {} {}: {}".format(
        channel, name, [hex(device) for device in devices]
    ))
    if AS5600_ADDRESS not in devices:
        raise RuntimeError("{} AS5600 not detected on PCA channel {}".format(
            name, channel
        ))
disable_pca_channels()

steering_filter = AS5600AngleFilter(time_constant_s=0.010, maximum_rate_dps=1000)
brake_filter = AS5600AngleFilter(time_constant_s=0.010, maximum_rate_dps=1000)
throttle_filter = AS5047KinematicsFilter(
    motor_revolutions_per_axle_revolution=1.0,
    wheel_circumference_m=1.0,
    time_constant_s=0.030,
    maximum_motor_rpm=6000,
)

as5047_spi = SPI(
    1,
    baudrate=1_000_000,
    polarity=0,
    phase=1,
    bits=8,
    firstbit=SPI.MSB,
    sck=Pin(AS5047_SCK),
    mosi=Pin(AS5047_MOSI),
    miso=Pin(AS5047_MISO),
)
as5047_cs = Pin(AS5047_CS, Pin.OUT, value=1)

ra02_spi = SPI(
    0,
    baudrate=2_000_000,
    polarity=0,
    phase=0,
    bits=8,
    firstbit=SPI.MSB,
    sck=Pin(RA02_SCK),
    mosi=Pin(RA02_MOSI),
    miso=Pin(RA02_MISO),
)
ra02_cs = Pin(RA02_CS, Pin.OUT, value=1)
ra02_reset = Pin(RA02_RESET, Pin.OUT, value=1)
ra02_dio0 = Pin(RA02_DIO0, Pin.IN)
radio = SX1278(ra02_spi, ra02_cs, ra02_reset, 433_000_000)
radio.begin()
ra02_version = radio.read_reg(radio.VERSION)

latest_steering = None
latest_brake = None
latest_throttle = None
last_lora_packet = None

collect(500)
print_all()
show_help()

while True:
    try:
        command = input("\nAll sensors > ").strip()
        lower = command.lower()

        if lower in ("r", "read"):
            collect(1000)
            print_all()

        elif lower == "stream":
            print("Sampling at 200 Hz; displaying at 2 Hz. Ctrl-C returns.")
            try:
                while True:
                    collect(DISPLAY_INTERVAL_MS)
                    print_all()
            except KeyboardInterrupt:
                print("Streaming stopped.")

        elif lower == "zero steering":
            collect(100)
            if steering_filter.calibrate_zero():
                print("Steering calibrated to 0 degrees (runtime only).")
            else:
                print("Steering zero rejected: no valid sample.")

        elif lower == "zero brake":
            collect(100)
            if brake_filter.calibrate_zero():
                print("Brake calibrated to 0 degrees (runtime only).")
            else:
                print("Brake zero rejected: no valid sample.")

        elif lower == "zero throttle":
            throttle_filter.reset()
            collect(100)
            print("Throttle cumulative turns reset to 0.")

        elif lower.startswith("send "):
            message = command[5:].strip()
            if message:
                radio.send(message)
                print("LORA TX:", message)
            else:
                print("Message cannot be empty.")

        elif lower == "status":
            collect(500)
            print_all()

        elif lower in ("help", "?"):
            show_help()

        elif lower in ("q", "quit", "exit"):
            disable_pca_channels()
            print("All-sensor test stopped.")
            break

        elif lower:
            print("Unknown command. Type help.")

    except KeyboardInterrupt:
        print("\nUse q to stop or help for commands.")
    except Exception as error:
        print("SENSOR TEST ERROR:", error)
