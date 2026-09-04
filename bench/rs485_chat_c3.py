"""XIAO ESP32-C3 USB-console to RS485 chat bridge.

Run as main.py on the C3. Type a line in the C3 serial terminal and it is
sent to the ESP32-S3. Lines from the S3 are printed locally.
"""

from machine import Pin, UART
import sys
import time
import uselect


TX_PIN = 21
RX_PIN = 20
EN_PIN = 10
BAUDRATE = 9600

uart = UART(1, baudrate=BAUDRATE, bits=8, parity=None, stop=1,
            tx=Pin(TX_PIN), rx=Pin(RX_PIN))
en = Pin(EN_PIN, Pin.OUT, value=0)
rx_buffer = b""


def send_line(text):
    en.value(1)
    time.sleep_us(100)
    uart.write(("C3|" + text + "\n").encode())
    uart.flush()
    time.sleep_ms(20)
    en.value(0)


def receive_lines():
    global rx_buffer
    data = uart.read()
    if not data:
        return

    rx_buffer += data
    while b"\n" in rx_buffer:
        raw, rx_buffer = rx_buffer.split(b"\n", 1)
        try:
            line = raw.decode("utf-8").strip()
        except UnicodeError:
            print("[RS485 INVALID BYTES]", raw)
            continue
        if line:
            print("[REMOTE]", line)


print("C3 RS485 CHAT READY")
print("Type a line and press Enter.")

console = uselect.poll()
console.register(sys.stdin, uselect.POLLIN)

while True:
    receive_lines()

    if console.poll(0):
        text = sys.stdin.readline()
        if text:
            text = text.strip()
            if text:
                send_line(text)
                print("[LOCAL]", text)

    time.sleep_ms(10)

