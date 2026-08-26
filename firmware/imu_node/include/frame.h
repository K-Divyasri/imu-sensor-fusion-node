#pragma once
#include <stdint.h>
#include <string.h>
#include "crc16.h"

// The exact 36-byte wire format one IMU sample travels in, over the UART
// link to the host. Field-for-field identical to
// ../../../sim/imu_sim/frame.py's IMUFrame (Python) and
// ../../host/include/core/frame.hpp (the C++ host app) -- see frame.py's
// module docstring for the full layout table and why a magic marker + CRC
// are needed at all (UART isn't message-atomic the way UDP is).
#pragma pack(push, 1)
struct WireFrame {
  uint8_t magic[2];   // 0xAA, 0x55
  uint32_t seq;
  uint32_t device_us;
  float accel_x, accel_y, accel_z;  // g
  float gyro_x, gyro_y, gyro_z;     // deg/s
  uint16_t crc;                     // over seq..gyro_z, NOT including magic or crc itself
};
#pragma pack(pop)

static_assert(sizeof(WireFrame) == 36, "WireFrame must stay wire-compatible with frame.py");

inline void encodeFrame(WireFrame &frame) {
  frame.magic[0] = 0xAA;
  frame.magic[1] = 0x55;
  // CRC covers everything AFTER magic, i.e. from `seq` through `gyro_z` --
  // that's offsetof(seq) through just before the crc field itself, which
  // is exactly sizeof(WireFrame) - sizeof(magic) - sizeof(crc) bytes.
  const uint8_t *body = reinterpret_cast<const uint8_t *>(&frame.seq);
  size_t body_len = sizeof(WireFrame) - sizeof(frame.magic) - sizeof(frame.crc);
  frame.crc = crc16_ccitt_false(body, body_len);
}
