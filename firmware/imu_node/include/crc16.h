#pragma once
#include <stdint.h>
#include <stddef.h>

// CRC-16/CCITT-FALSE -- poly 0x1021, init 0xFFFF, no reflection, no final
// XOR. Same algorithm, independently reimplemented, in
// ../../../sim/imu_sim/crc16.py (Python) and
// ../../host/include/utils/crc16.hpp (the C++ host app) -- see either of
// those for the full "why this variant, why three copies" explanation.
inline uint16_t crc16_ccitt_false(const uint8_t *data, size_t length) {
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < length; i++) {
    crc ^= (uint16_t)data[i] << 8;
    for (int bit = 0; bit < 8; bit++) {
      if (crc & 0x8000) {
        crc = (crc << 1) ^ 0x1021;
      } else {
        crc = crc << 1;
      }
    }
  }
  return crc;
}
