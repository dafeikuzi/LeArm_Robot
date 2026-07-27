#ifndef LEARM_DRIVER__PROTOCOL_HPP_
#define LEARM_DRIVER__PROTOCOL_HPP_

#include <cstdint>
#include <string>
#include <vector>

namespace learm_driver
{

constexpr uint8_t kFrameHeader1 = 0xA5;
constexpr uint8_t kFrameHeader2 = 0x5A;
constexpr uint8_t kProtocolVersion = 0x01;
constexpr std::size_t kFrameMinLength = 8;
constexpr std::size_t kFrameMaxLength = 64;

constexpr uint8_t kCommandMovePulses = 0x10;
constexpr uint8_t kCommandGetStatus = 0x11;
constexpr uint8_t kCommandEmergencyStop = 0x12;
constexpr uint8_t kCommandClearEstop = 0x13;
constexpr uint8_t kCommandAck = 0x80;
constexpr uint8_t kCommandStatus = 0x81;

constexpr uint8_t kStatusOk = 0;
constexpr uint8_t kStatusInvalidPayload = 3;
constexpr uint8_t kStatusEstopActive = 4;
constexpr uint8_t kStatusTransportError = 100;
constexpr uint8_t kStatusTimeout = 101;
constexpr uint8_t kStatusCalibrationDisabled = 102;
constexpr uint8_t kStatusCalibrationInvalid = 103;

struct Frame
{
  uint8_t version{};
  uint8_t sequence{};
  uint8_t command{};
  std::vector<uint8_t> payload;
};

uint16_t crc16_ccitt(const uint8_t * data, std::size_t size);
std::vector<uint8_t> encode_frame(uint8_t sequence, uint8_t command,
  const std::vector<uint8_t> & payload);
bool decode_frame(const std::vector<uint8_t> & bytes, Frame & frame, std::string & error);
std::string bytes_to_hex(const std::vector<uint8_t> & bytes);
bool hex_to_bytes(const std::string & text, std::vector<uint8_t> & bytes, std::string & error);

}  // namespace learm_driver

#endif  // LEARM_DRIVER__PROTOCOL_HPP_
