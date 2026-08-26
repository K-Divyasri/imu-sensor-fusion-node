#include "network/frame_parser.hpp"
#include "utils/crc16.hpp"

#include <cstddef>
#include <cstring>

namespace imu {

std::optional<WireFrame> tryParseOne(const uint8_t* buf, size_t available, size_t& bytesConsumed) {
  if (available < sizeof(WireFrame)) {
    bytesConsumed = 0;  // not enough bytes buffered yet -- wait for more
    return std::nullopt;
  }

  if (buf[0] != kMagicByte0 || buf[1] != kMagicByte1) {
    bytesConsumed = 1;  // not a sync marker here -- slide forward one byte
    return std::nullopt;
  }

  WireFrame frame;
  std::memcpy(&frame, buf, sizeof(WireFrame));  // memcpy, not a cast -- buf may be misaligned

  const uint8_t* body = buf + offsetof(WireFrame, seq);
  const size_t bodyLen = sizeof(WireFrame) - sizeof(frame.magic) - sizeof(frame.crc);
  const uint16_t computedCrc = crc16_ccitt_false(body, bodyLen);

  if (computedCrc != frame.crc) {
    // Right magic bytes, wrong CRC -- either real corruption, or two
    // stray bytes that happened to look like a sync marker. Either way,
    // don't trust it: slide forward one byte and keep scanning.
    bytesConsumed = 1;
    return std::nullopt;
  }

  bytesConsumed = sizeof(WireFrame);
  return frame;
}

std::vector<WireFrame> FrameParser::feed(const uint8_t* data, size_t length) {
  buffer_.insert(buffer_.end(), data, data + length);

  std::vector<WireFrame> frames;
  size_t pos = 0;
  while (true) {
    size_t consumed = 0;
    auto frame = tryParseOne(buffer_.data() + pos, buffer_.size() - pos, consumed);
    if (consumed == 0) break;  // not enough data buffered for a decision -- stop here
    if (frame) {
      frames.push_back(*frame);
      ++frames_parsed_;
    } else {
      ++resync_byte_count_;
    }
    pos += consumed;
  }

  buffer_.erase(buffer_.begin(), buffer_.begin() + static_cast<long>(pos));
  return frames;
}

}  // namespace imu
