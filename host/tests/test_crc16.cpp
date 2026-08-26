#include <gtest/gtest.h>
#include "utils/crc16.hpp"

using namespace imu;

TEST(Crc16, KnownTestVector) {
  // The standard CRC-16/CCITT-FALSE check value for "123456789", the same
  // value ../../../sim/imu_sim's test_crc16.py asserts -- if this doesn't
  // match, the algorithm itself is wrong, not just this port of it.
  const uint8_t data[] = "123456789";
  EXPECT_EQ(crc16_ccitt_false(data, 9), 0x29B1);
}

TEST(Crc16, DifferentDataGivesDifferentCrc) {
  const uint8_t a[] = {1, 2, 3, 4};
  const uint8_t b[] = {1, 2, 3, 5};
  EXPECT_NE(crc16_ccitt_false(a, 4), crc16_ccitt_false(b, 4));
}

TEST(Crc16, MatchesPythonImplementationOnAKnownFrameBody) {
  // The exact 32-byte body struct.pack("<II6f", 42, 123456, 0.25, -0.25,
  // 0.5, 1.5, -0.375, 0.125) produces in Python, and the REAL CRC that
  // imu_sim.crc16.crc16_ccitt_false computes on those same bytes (checked
  // by actually running the Python function, not hand-derived) -- proves
  // the C++ and Python implementations genuinely agree on a real value,
  // not just that each one is internally consistent with itself.
  const uint8_t body[] = {0x2A, 0x00, 0x00, 0x00, 0x40, 0xE2, 0x01, 0x00,
                          0x00, 0x00, 0x80, 0x3E, 0x00, 0x00, 0x80, 0xBE,
                          0x00, 0x00, 0x00, 0x3F, 0x00, 0x00, 0xC0, 0x3F,
                          0x00, 0x00, 0xC0, 0xBE, 0x00, 0x00, 0x00, 0x3E};
  ASSERT_EQ(sizeof(body), 32u);
  EXPECT_EQ(crc16_ccitt_false(body, sizeof(body)), 0xE716);
}
