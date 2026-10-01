"""ESP32-S3 + RA-02 receive-only E-stop link test.

Compatible transmitter:
  XIAO ESP32-C3 + RA-02 + LAY37

Accepted packet format:
  ESTOP,0,sequence = switch released / clear
  ESTOP,1,sequence = switch pressed or NC wire open / stop

This isolated test does not import network and does not control the BLDC,
brake, steering, or traction outputs. It has no radio heartbeat timeout.
"""

from machine import Pin, SPI
import time
from ra02_lora_chat_common import SX1278


# Alternate ESP32-S3 pins: BLDC GPIO13/14/15 remain untouched.
SPI_SCK = 4
SPI_MOSI = 5
SPI_MISO = 6
RA02_CS = 7
RA02_RESET = 10
RA02_DIO0 = 18
FREQUENCY_HZ = 433_000_000


spi = SPI(
    2,
    baudrate=2_000_000,
    polarity=0,
    phase=0,
    bits=8,
    firstbit=SPI.MSB,
    sck=Pin(SPI_SCK),
    mosi=Pin(SPI_MOSI),
    miso=Pin(SPI_MISO),
)
cs = Pin(RA02_CS, Pin.OUT, value=1)
reset = Pin(RA02_RESET, Pin.OUT, value=1)
dio0 = Pin(RA02_DIO0, Pin.IN)
radio = SX1278(spi, cs, reset, FREQUENCY_HZ)


def parse_estop_packet(packet):
    try:
        text = packet.decode().strip()
        fields = text.split(",")

        if len(fields) != 3 or fields[0] != "ESTOP":
            return None

        state = int(fields[1])
        sequence = int(fields[2])

        if state not in (0, 1):
            return None

        if sequence < 0 or sequence > 0xFFFF:
            return None

        return text, state, sequence

    except Exception:
        return None


radio.begin()
estop_active = None
last_sequence = None

print()
print("ESP32-S3 LORA E-STOP RECEIVER TEST READY")
print("RA-02: SCK4 MOSI5 MISO6 CS7 RESET10 DIO0=18")
print("Waiting for the XIAO ESP32-C3 transmitter...")
print("Startup state: UNKNOWN; no actuator outputs are controlled.")

while True:
    packet = radio.receive()

    if packet:
        parsed = parse_estop_packet(packet)

        if parsed is None:
            try:
                print("IGNORED INVALID PACKET:", packet.decode().strip())
            except Exception:
                print("IGNORED INVALID BYTES:", packet)
        else:
            text, state, sequence = parsed

            # Always report the packet for link testing, but announce the
            # E-stop transition only when the state actually changes.
            print("RX:", text)

            if estop_active is None or state != estop_active:
                estop_active = state

                if estop_active:
                    print("*** E-STOP ACTIVE: STOP / BRAKE REQUIRED ***")
                else:
                    print("E-STOP CLEAR: SWITCH RELEASED")

            last_sequence = sequence

    time.sleep_ms(5)
