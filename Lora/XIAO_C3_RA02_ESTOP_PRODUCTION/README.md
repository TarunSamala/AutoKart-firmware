# Legacy XIAO ESP32-C3 + RA-02 E-stop firmware

This implementation is retained for reference but is no longer the selected
AutoKart transmitter. Use `Lora/PICO2_RA02_ESTOP_PRODUCTION/` for the final
Pico 2 transmitter.

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
