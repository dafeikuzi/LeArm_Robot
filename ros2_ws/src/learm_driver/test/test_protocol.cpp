#include <gtest/gtest.h>

#include <string>
#include <vector>

#include "learm_driver/protocol.hpp"

TEST(LeArmProtocol, ComputesStandardCrcVector)
{
  const std::string input = "123456789";
  EXPECT_EQ(learm_driver::crc16_ccitt(
      reinterpret_cast<const uint8_t *>(input.data()), input.size()), 0x29B1U);
}

TEST(LeArmProtocol, EncodesAndDecodesFrame)
{
  const auto bytes = learm_driver::encode_frame(7, learm_driver::kCommandGetStatus, {});
  learm_driver::Frame frame;
  std::string error;
  ASSERT_TRUE(learm_driver::decode_frame(bytes, frame, error));
  EXPECT_EQ(frame.version, learm_driver::kProtocolVersion);
  EXPECT_EQ(frame.sequence, 7);
  EXPECT_EQ(frame.command, learm_driver::kCommandGetStatus);
  EXPECT_TRUE(frame.payload.empty());
}

TEST(LeArmProtocol, RejectsInvalidCrc)
{
  auto bytes = learm_driver::encode_frame(1, learm_driver::kCommandEmergencyStop, {});
  bytes.back() ^= 0x01U;
  learm_driver::Frame frame;
  std::string error;
  EXPECT_FALSE(learm_driver::decode_frame(bytes, frame, error));
  EXPECT_EQ(error, "CRC mismatch");
}
