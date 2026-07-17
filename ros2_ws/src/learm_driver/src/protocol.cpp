#include "learm_driver/protocol.hpp"

#include <cctype>
#include <iomanip>
#include <sstream>
#include <stdexcept>

namespace learm_driver
{

uint16_t crc16_ccitt(const uint8_t * data, const std::size_t size)
{
  uint16_t crc = 0xFFFF;
  for (std::size_t index = 0; index < size; ++index) {
    crc ^= static_cast<uint16_t>(data[index]) << 8;
    for (uint8_t bit = 0; bit < 8; ++bit) {
      crc = (crc & 0x8000U) ? static_cast<uint16_t>((crc << 1) ^ 0x1021U) :
        static_cast<uint16_t>(crc << 1);
    }
  }
  return crc;
}

std::vector<uint8_t> encode_frame(const uint8_t sequence, const uint8_t command,
  const std::vector<uint8_t> & payload)
{
  if (payload.size() > kFrameMaxLength - kFrameMinLength) {
    throw std::invalid_argument("payload is too large");
  }

  std::vector<uint8_t> frame = {
    kFrameHeader1, kFrameHeader2, kProtocolVersion, sequence, command,
    static_cast<uint8_t>(payload.size())};
  frame.insert(frame.end(), payload.begin(), payload.end());
  const uint16_t crc = crc16_ccitt(frame.data() + 2, 4 + payload.size());
  frame.push_back(static_cast<uint8_t>(crc & 0xFFU));
  frame.push_back(static_cast<uint8_t>((crc >> 8) & 0xFFU));
  return frame;
}

bool decode_frame(const std::vector<uint8_t> & bytes, Frame & frame, std::string & error)
{
  if (bytes.size() < kFrameMinLength) {
    error = "frame is too short";
    return false;
  }
  if (bytes[0] != kFrameHeader1 || bytes[1] != kFrameHeader2) {
    error = "invalid frame header";
    return false;
  }
  const std::size_t payload_size = bytes[5];
  if (bytes.size() != kFrameMinLength + payload_size) {
    error = "frame length does not match payload length";
    return false;
  }
  const uint16_t expected_crc = static_cast<uint16_t>(bytes[bytes.size() - 2]) |
    (static_cast<uint16_t>(bytes[bytes.size() - 1]) << 8);
  const uint16_t actual_crc = crc16_ccitt(bytes.data() + 2, 4 + payload_size);
  if (actual_crc != expected_crc) {
    error = "CRC mismatch";
    return false;
  }
  frame.version = bytes[2];
  frame.sequence = bytes[3];
  frame.command = bytes[4];
  frame.payload.assign(bytes.begin() + 6, bytes.end() - 2);
  return true;
}

std::string bytes_to_hex(const std::vector<uint8_t> & bytes)
{
  std::ostringstream stream;
  stream << std::uppercase << std::hex << std::setfill('0');
  for (std::size_t index = 0; index < bytes.size(); ++index) {
    if (index != 0) {
      stream << ' ';
    }
    stream << std::setw(2) << static_cast<unsigned int>(bytes[index]);
  }
  return stream.str();
}

bool hex_to_bytes(const std::string & text, std::vector<uint8_t> & bytes, std::string & error)
{
  std::string compact;
  for (const char character : text) {
    if (!std::isspace(static_cast<unsigned char>(character))) {
      compact += character;
    }
  }
  if (compact.empty() || compact.size() % 2 != 0) {
    error = "hex text must contain complete byte pairs";
    return false;
  }

  bytes.clear();
  bytes.reserve(compact.size() / 2);
  try {
    for (std::size_t index = 0; index < compact.size(); index += 2) {
      std::size_t parsed = 0;
      const auto value = std::stoul(compact.substr(index, 2), &parsed, 16);
      if (parsed != 2 || value > 0xFFU) {
        error = "invalid hex byte";
        return false;
      }
      bytes.push_back(static_cast<uint8_t>(value));
    }
  } catch (const std::exception &) {
    error = "invalid hex byte";
    return false;
  }
  return true;
}

}  // namespace learm_driver
