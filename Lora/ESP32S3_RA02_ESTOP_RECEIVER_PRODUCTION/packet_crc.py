def crc16(data):

    crc = 0xFFFF

    for b in data.encode():

        crc ^= b

        for i in range(8):

            if crc & 1:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1

    return crc


def verify_packet(packet):

    try:

        parts = packet.split(",")

        payload = ",".join(parts[:-1])
        received = int(parts[-1])

        return crc16(payload) == received

    except:

        return False
