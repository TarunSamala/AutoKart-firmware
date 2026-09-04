"""XIAO ESP32-C3 RS485 master test.

Sends PING packets and waits for PONG replies from the ESP32-S3.
"""

from machine import Pin, UART
import time


TX_PIN = 21       # XIAO D6 / TX
RX_PIN = 20       # XIAO D7 / RX
DE_RE_PIN = 10    # XIAO D10; tie MAX3485 DE and /RE together
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
    time.sleep_ms(3)

    if direction is not None:
        direction.value(0)


print("XIAO ESP32-C3 RS485 TEST")
print("UART TX GPIO", TX_PIN, "RX GPIO", RX_PIN)
print("Waiting for PONG from ESP32-S3...")

sequence = 0
last_ping = time.ticks_ms() - 1000

while True:
    now = time.ticks_ms()

    if time.ticks_diff(now, last_ping) >= 1000:
        sequence += 1
        send_line("PING," + str(sequence))
        print("TX: PING," + str(sequence))
        last_ping = now

    if uart.any():
        data = uart.readline()
        if data:
            line = data.decode("utf-8", "ignore").strip()
            print("RX:", line)

    time.sleep_ms(10)

