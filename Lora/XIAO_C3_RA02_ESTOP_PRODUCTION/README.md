XIAO ESP32-C3 + RA-02 Production E-Stop Firmware

Hardware:
- XIAO ESP32-C3
- SX1278 RA-02 433MHz
- LAY37 NC mushroom switch

Architecture:
Transmitter sends signed-state packets.
Receiver treats missing packets as STOP.

Important:
This wireless system is a safety command path.
Final vehicle safety requires independent motor disable/brake hardware.
