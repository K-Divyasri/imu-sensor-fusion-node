#pragma once
#include <cstdint>
#include <cstddef>

// The exact 36-byte wire format one IMU sample travels in. Field-for-field
// identical to ../../../sim/imu_sim/frame.py (Python) and
// ../../../firmware/imu_node/include/frame.h (the ESP32 firmware) -- see
// frame.py's module docstring for the full layout table and rationale.

namespace imu {

#pragma pack(push, 1)
struct WireFrame {
  uint8_t magic[2];   // 0xAA, 0x55
  uint32_t seq;
  uint32_t device_us;
  float accel_x, accel_y, accel_z;  // g
  float gyro_x, gyro_y, gyro_z;     // deg/s
  uint16_t crc;                     // over seq..gyro_z only
};
#pragma pack(pop)

static_assert(sizeof(WireFrame) == 36, "WireFrame must stay wire-compatible with the firmware and imu_sim");

constexpr uint8_t kMagicByte0 = 0xAA;
constexpr uint8_t kMagicByte1 = 0x55;

}  // namespace imu
