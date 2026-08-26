#include <gtest/gtest.h>
#include <cstring>
#include <vector>

#include "core/frame.hpp"
#include "network/frame_parser.hpp"
#include "utils/crc16.hpp"

using namespace imu;

namespace {

WireFrame makeSample() {
  WireFrame f{};
  f.magic[0] = kMagicByte0;
  f.magic[1] = kMagicByte1;
  f.seq = 42;
  f.device_us = 123456;
  f.accel_x = 0.25f;
  f.accel_y = -0.25f;
  f.accel_z = 0.5f;
  f.gyro_x = 1.5f;
  f.gyro_y = -0.375f;
  f.gyro_z = 0.125f;
  const uint8_t* body = reinterpret_cast<const uint8_t*>(&f.seq);
  size_t bodyLen = sizeof(WireFrame) - sizeof(f.magic) - sizeof(f.crc);
  f.crc = crc16_ccitt_false(body, bodyLen);
  return f;
}

std::vector<uint8_t> toBytes(const WireFrame& f) {
  std::vector<uint8_t> bytes(sizeof(WireFrame));
  std::memcpy(bytes.data(), &f, sizeof(WireFrame));
  return bytes;
}

bool framesEqual(const WireFrame& a, const WireFrame& b) {
  return a.seq == b.seq && a.device_us == b.device_us &&
         a.accel_x == b.accel_x && a.accel_y == b.accel_y && a.accel_z == b.accel_z &&
         a.gyro_x == b.gyro_x && a.gyro_y == b.gyro_y && a.gyro_z == b.gyro_z;
}

}  // namespace

TEST(FrameSize, Is36Bytes) {
  EXPECT_EQ(sizeof(WireFrame), 36u);
}

TEST(TryParseOne, RoundTripsExactly) {
  WireFrame sample = makeSample();
  auto bytes = toBytes(sample);

  size_t consumed = 0;
  auto frame = tryParseOne(bytes.data(), bytes.size(), consumed);
  ASSERT_TRUE(frame.has_value());
  EXPECT_EQ(consumed, sizeof(WireFrame));
  EXPECT_TRUE(framesEqual(*frame, sample));
}

TEST(TryParseOne, RejectsWrongMagic) {
  WireFrame sample = makeSample();
  auto bytes = toBytes(sample);
  bytes[0] = 0x00;

  size_t consumed = 0;
  auto frame = tryParseOne(bytes.data(), bytes.size(), consumed);
  EXPECT_FALSE(frame.has_value());
  EXPECT_EQ(consumed, 1u);
}

TEST(TryParseOne, RejectsBadCrc) {
  WireFrame sample = makeSample();
  auto bytes = toBytes(sample);
  bytes.back() ^= 0xFF;

  size_t consumed = 0;
  auto frame = tryParseOne(bytes.data(), bytes.size(), consumed);
  EXPECT_FALSE(frame.has_value());
  EXPECT_EQ(consumed, 1u);
}

TEST(TryParseOne, ReturnsZeroConsumedWhenNotEnoughBytes) {
  WireFrame sample = makeSample();
  auto bytes = toBytes(sample);
  bytes.resize(10);  // truncate -- an in-flight, not-yet-complete frame

  size_t consumed = 0;
  auto frame = tryParseOne(bytes.data(), bytes.size(), consumed);
  EXPECT_FALSE(frame.has_value());
  EXPECT_EQ(consumed, 0u);
}

TEST(FrameParser, HandlesTwoBackToBackFrames) {
  WireFrame first = makeSample();
  WireFrame second = makeSample();
  second.seq = 43;
  const uint8_t* body = reinterpret_cast<const uint8_t*>(&second.seq);
  second.crc = crc16_ccitt_false(body, sizeof(WireFrame) - sizeof(second.magic) - sizeof(second.crc));

  std::vector<uint8_t> stream;
  auto b1 = toBytes(first), b2 = toBytes(second);
  stream.insert(stream.end(), b1.begin(), b1.end());
  stream.insert(stream.end(), b2.begin(), b2.end());

  FrameParser parser;
  auto frames = parser.feed(stream.data(), stream.size());
  ASSERT_EQ(frames.size(), 2u);
  EXPECT_TRUE(framesEqual(frames[0], first));
  EXPECT_TRUE(framesEqual(frames[1], second));
  EXPECT_EQ(parser.framesParsed(), 2u);
  EXPECT_EQ(parser.resyncByteCount(), 0u);
}

TEST(FrameParser, HandlesAFrameSplitAcrossTwoFeeds) {
  WireFrame sample = makeSample();
  auto bytes = toBytes(sample);

  FrameParser parser;
  auto first = parser.feed(bytes.data(), 20);
  EXPECT_TRUE(first.empty());  // incomplete -- nothing yet

  auto second = parser.feed(bytes.data() + 20, bytes.size() - 20);
  ASSERT_EQ(second.size(), 1u);
  EXPECT_TRUE(framesEqual(second[0], sample));
}

TEST(FrameParser, ResyncsAfterGarbageBytes) {
  std::vector<uint8_t> garbage = {0x00, 0x11, 0x22};
  WireFrame sample = makeSample();
  auto sampleBytes = toBytes(sample);

  std::vector<uint8_t> stream = garbage;
  stream.insert(stream.end(), sampleBytes.begin(), sampleBytes.end());

  FrameParser parser;
  auto frames = parser.feed(stream.data(), stream.size());
  ASSERT_EQ(frames.size(), 1u);
  EXPECT_TRUE(framesEqual(frames[0], sample));
  EXPECT_EQ(parser.resyncByteCount(), garbage.size());
}

TEST(FrameParser, SurvivesADroppedByteMidStream) {
  WireFrame first = makeSample();

  WireFrame second = makeSample();
  second.seq = 1;
  second.device_us = 5000;
  const uint8_t* secondBody = reinterpret_cast<const uint8_t*>(&second.seq);
  second.crc = crc16_ccitt_false(secondBody, sizeof(WireFrame) - sizeof(second.magic) - sizeof(second.crc));

  WireFrame third = makeSample();
  third.seq = 2;
  third.device_us = 10000;
  const uint8_t* thirdBody = reinterpret_cast<const uint8_t*>(&third.seq);
  third.crc = crc16_ccitt_false(thirdBody, sizeof(WireFrame) - sizeof(third.magic) - sizeof(third.crc));

  auto firstBytes = toBytes(first);
  auto secondBytes = toBytes(second);
  auto thirdBytes = toBytes(third);
  secondBytes.erase(secondBytes.begin() + 10);  // drop one byte mid-frame

  std::vector<uint8_t> stream;
  stream.insert(stream.end(), firstBytes.begin(), firstBytes.end());
  stream.insert(stream.end(), secondBytes.begin(), secondBytes.end());
  stream.insert(stream.end(), thirdBytes.begin(), thirdBytes.end());

  FrameParser parser;
  auto frames = parser.feed(stream.data(), stream.size());
  ASSERT_EQ(frames.size(), 2u);  // corrupted middle frame correctly dropped
  EXPECT_TRUE(framesEqual(frames[0], first));
  EXPECT_TRUE(framesEqual(frames[1], third));
  EXPECT_GT(parser.resyncByteCount(), 0u);
}
