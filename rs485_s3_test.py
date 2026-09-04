"""ESP32-S3 RS485 responder test.

Replies PONG to PING packets from the XIAO ESP32-C3.
This test uses GPIOs not used by the current actuator/I2C mapping.
"""

from machine import Pin, UART
import time


TX_PIN = 17
RX_PIN = 18
DE_RE_PIN = 16    # tie MAX3485 DE and /RE together
USE_DIRECTION_PIN = True
BAUDRATE = 9600


uart = UART(
    1,
    baudrate=BAUDRATE,
    bits=8,
    parity=None,
    stop=1,
    tx=Pin(TX_PIN),
    rx=Pin(RX_PIN),
)

direction = Pin(DE_RE_PIN, Pin.OUT, value=0) if USE_DIRECTION_PIN else None


def send_line(text):
    if direction is not None:
        direction.value(1)
        time.sleep_us(100)

    uart.write((text + "\n").encode())
    uart.flush()
    # At 9600 baud a short packet needs several milliseconds on the wire.
    # Keep the driver enabled until the final stop bit has cleared.
    time.sleep_ms(20)

    if direction is not None:
        direction.value(0)


print("ESP32-S3 RS485 RESPONDER TEST")
print("UART TX GPIO", TX_PIN, "RX GPIO", RX_PIN)
print("Waiting for PING from XIAO ESP32-C3...")

rx_buffer = b""

while True:
    if uart.any():
        data = uart.read()
        if data:
            rx_buffer += data

            while b"\n" in rx_buffer:
                raw, rx_buffer = rx_buffer.split(b"\n", 1)
                raw = raw.strip()
                try:
                    line = raw.decode("utf-8")
                    print("RX:", line)
                except UnicodeError:
                    print("RX BYTES:", raw)
                    continue

                if line.startswith("PING,"):
                    sequence = line.split(",", 1)[1]
                    send_line("PONG," + sequence)
                    print("TX: PONG," + sequence)

    time.sleep_ms(10)
