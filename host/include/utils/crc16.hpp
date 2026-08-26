#pragma once
#include <cstdint>
#include <cstddef>

// CRC-16/CCITT-FALSE -- poly 0x1021, init 0xFFFF, no reflection, no final
// XOR. Independently reimplemented, algorithm-for-algorithm identical, in
// ../../../sim/imu_sim/crc16.py (Python) and
// ../../../firmware/imu_node/include/crc16.h (the ESP32 firmware) --
// three copies of one 12-line function, on purpose, matching this repo's
// house convention of technique-reuse over cross-project code sharing.

namespace imu {

inline uint16_t crc16_ccitt_false(const uint8_t* data, size_t length) {
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < length; ++i) {
    crc ^= static_cast<uint16_t>(data[i]) << 8;
    for (int bit = 0; bit < 8; ++bit) {
      if (crc & 0x8000) {
        crc = static_cast<uint16_t>((crc << 1) ^ 0x1021);
      } else {
        crc = static_cast<uint16_t>(crc << 1);
      }
    }
  }
  return crc;
}

}  // namespace imu
