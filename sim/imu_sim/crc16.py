"""
CRC-16/CCITT-FALSE -- poly 0x1021, init 0xFFFF, no reflection, no final XOR.
Same variant used by XMODEM and a lot of embedded serial protocols; picked
here because the bit-by-bit algorithm is simple enough to read once and
trust, not because it's cryptographically special.

This is deliberately reimplemented, byte-for-byte identical, in
../../host/include/utils/crc16.hpp (C++, for the real host app) and
../../firmware/imu_node/src/crc16.cpp (C++, for the ESP32 firmware) --
three independent implementations of one 12-line algorithm is cheap
insurance against a subtle porting bug silently breaking every frame's
integrity check.
"""


def crc16_ccitt_false(data: bytes) -> int:
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc
