def crc16(data):
    crc = 0xFFFF

    for byte in data.encode():
        crc ^= byte

        for _ in range(8):
            if crc & 1:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1

    return crc


def build_packet(device, state, sequence, uptime_ms):

    payload = f"{device},{state},{sequence},{uptime_ms}"

    checksum = crc16(payload)

    return f"{payload},{checksum}"
