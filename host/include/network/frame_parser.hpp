#pragma once
#include <cstdint>
#include <cstddef>
#include <optional>
#include <vector>

#include "core/frame.hpp"

// The platform-INDEPENDENT half of receiving frames: given whatever bytes
// have arrived so far (from a real serial port, a replay file, or a unit
// test's synthetic corrupted stream), find and validate complete
// WireFrames, and recover cleanly from a dropped or corrupted byte
// instead of derailing the whole stream. See
// ../../../sim/imu_sim/frame.py's StreamParser docstring for the full
// "why UART needs this and UDP didn't" explanation -- this class is that
// same algorithm, in C++, unit-tested with GoogleTest in tests/.
//
// Deliberately has zero knowledge of where the bytes came from -- that's
// SerialPort's job (network/serial_port.hpp). This split is what makes
// the resync algorithm itself testable without a real serial port or
// real hardware at all.

namespace imu {

// Attempts to parse exactly one frame starting at buf[0..available).
// bytesConsumed is always set to how far the caller should advance before
// trying again -- 0 only when there simply isn't enough data yet.
std::optional<WireFrame> tryParseOne(const uint8_t* buf, size_t available, size_t& bytesConsumed);

class FrameParser {
 public:
  // Feed newly-arrived bytes; returns every complete, valid frame found.
  // Buffers any leftover partial data internally across calls.
  std::vector<WireFrame> feed(const uint8_t* data, size_t length);

  size_t framesParsed() const { return frames_parsed_; }
  size_t resyncByteCount() const { return resync_byte_count_; }

 private:
  std::vector<uint8_t> buffer_;
  size_t frames_parsed_ = 0;
  size_t resync_byte_count_ = 0;
};

}  // namespace imu
