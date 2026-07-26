#pragma once

#include "gps_sample.hpp"

#include <array>
#include <cstddef>
#include <cstdint>

namespace gpsmeter {

struct UbxFrame {
  static constexpr size_t kMaxPayloadLength = 256;

  uint8_t message_class = 0;
  uint8_t message_id = 0;
  uint16_t payload_length = 0;
  std::array<uint8_t, kMaxPayloadLength> payload{};
};

class UbxParser {
public:
  bool feed(uint8_t byte, UbxFrame &frame);
  void reset();

private:
  enum class State : uint8_t {
    Sync1,
    Sync2,
    MessageClass,
    MessageId,
    LengthLow,
    LengthHigh,
    Payload,
    ChecksumA,
    ChecksumB,
  };

  void add_checksum(uint8_t byte);

  State state_ = State::Sync1;
  UbxFrame working_{};
  uint16_t payload_offset_ = 0;
  uint8_t checksum_a_ = 0;
  uint8_t checksum_b_ = 0;
  uint8_t received_checksum_a_ = 0;
};

bool parse_ubx_nav_pvt(const UbxFrame &frame, uint64_t received_ms,
                       GpsSample &sample);

} // namespace gpsmeter
