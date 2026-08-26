#pragma once
#include <chrono>
#include <cstdint>

// Same nanosecond-timestamp idea as the existing C++ order-book engine's
// market::now_ns() -- a monotonic steady_clock reading, not wall-clock
// time, since only relative durations matter for latency measurement, and
// steady_clock (unlike system_clock) is guaranteed never to jump
// backwards from an NTP adjustment mid-measurement.

namespace imu {

inline uint64_t now_ns() {
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
             std::chrono::steady_clock::now().time_since_epoch())
      .count();
}

struct FusedState {
  double roll_deg = 0.0;
  double pitch_deg = 0.0;
  uint32_t seq = 0;
  uint32_t device_us = 0;
};

}  // namespace imu
