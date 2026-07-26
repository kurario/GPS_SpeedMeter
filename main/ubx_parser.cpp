#include "ubx_parser.hpp"

#include <cmath>
#include <cstring>

namespace gpsmeter {
namespace {

uint16_t read_u16(const uint8_t *bytes) {
  return static_cast<uint16_t>(bytes[0]) |
         (static_cast<uint16_t>(bytes[1]) << 8U);
}

uint32_t read_u32(const uint8_t *bytes) {
  return static_cast<uint32_t>(bytes[0]) |
         (static_cast<uint32_t>(bytes[1]) << 8U) |
         (static_cast<uint32_t>(bytes[2]) << 16U) |
         (static_cast<uint32_t>(bytes[3]) << 24U);
}

int32_t read_i32(const uint8_t *bytes) {
  const uint32_t bits = read_u32(bytes);
  int32_t value = 0;
  static_assert(sizeof(value) == sizeof(bits));
  std::memcpy(&value, &bits, sizeof(value));
  return value;
}

bool utc_fields_valid(const UtcDateTime &utc) {
  return utc.year >= 2000 && utc.month >= 1 && utc.month <= 12 &&
         utc.day >= 1 && utc.day <= 31 && utc.hour <= 23 && utc.minute <= 59 &&
         utc.second <= 60;
}

} // namespace

void UbxParser::add_checksum(uint8_t byte) {
  checksum_a_ = static_cast<uint8_t>(checksum_a_ + byte);
  checksum_b_ = static_cast<uint8_t>(checksum_b_ + checksum_a_);
}

void UbxParser::reset() {
  state_ = State::Sync1;
  working_ = {};
  payload_offset_ = 0;
  checksum_a_ = 0;
  checksum_b_ = 0;
  received_checksum_a_ = 0;
}

bool UbxParser::feed(uint8_t byte, UbxFrame &frame) {
  switch (state_) {
  case State::Sync1:
    if (byte == 0xB5) {
      state_ = State::Sync2;
    }
    return false;
  case State::Sync2:
    if (byte == 0x62) {
      working_ = {};
      payload_offset_ = 0;
      checksum_a_ = 0;
      checksum_b_ = 0;
      state_ = State::MessageClass;
    } else {
      state_ = byte == 0xB5 ? State::Sync2 : State::Sync1;
    }
    return false;
  case State::MessageClass:
    working_.message_class = byte;
    add_checksum(byte);
    state_ = State::MessageId;
    return false;
  case State::MessageId:
    working_.message_id = byte;
    add_checksum(byte);
    state_ = State::LengthLow;
    return false;
  case State::LengthLow:
    working_.payload_length = byte;
    add_checksum(byte);
    state_ = State::LengthHigh;
    return false;
  case State::LengthHigh:
    working_.payload_length |= static_cast<uint16_t>(byte) << 8U;
    add_checksum(byte);
    if (working_.payload_length > working_.payload.size()) {
      reset();
      return false;
    }
    state_ = working_.payload_length == 0 ? State::ChecksumA : State::Payload;
    return false;
  case State::Payload:
    working_.payload[payload_offset_++] = byte;
    add_checksum(byte);
    if (payload_offset_ >= working_.payload_length) {
      state_ = State::ChecksumA;
    }
    return false;
  case State::ChecksumA:
    received_checksum_a_ = byte;
    state_ = State::ChecksumB;
    return false;
  case State::ChecksumB: {
    const bool valid =
        received_checksum_a_ == checksum_a_ && byte == checksum_b_;
    if (valid) {
      frame = working_;
    }
    reset();
    return valid;
  }
  }
  reset();
  return false;
}

bool parse_ubx_nav_pvt(const UbxFrame &frame, uint64_t received_ms,
                       GpsSample &sample) {
  constexpr uint8_t kNavClass = 0x01;
  constexpr uint8_t kNavPvtId = 0x07;
  constexpr uint16_t kNavPvtPayloadLength = 92;
  if (frame.message_class != kNavClass || frame.message_id != kNavPvtId ||
      frame.payload_length < kNavPvtPayloadLength) {
    return false;
  }

  const auto &payload = frame.payload;
  const uint8_t validity = payload[11];
  const uint8_t fix_type = payload[20];
  const bool fix_ok = (payload[21] & 0x01U) != 0;
  const bool coordinates_valid = (read_u16(&payload[78]) & 0x01U) == 0;
  const int32_t ground_speed_mm_s = read_i32(&payload[60]);
  const int32_t heading_1e5_degrees = read_i32(&payload[64]);

  GpsSample parsed{};
  parsed.speed_kmh = ground_speed_mm_s >= 0
                         ? static_cast<float>(ground_speed_mm_s) * 0.0036F
                         : 0.0F;
  parsed.speed_accuracy_kmh =
      static_cast<float>(read_u32(&payload[68])) * 0.0036F;
  parsed.course_degrees = static_cast<float>(heading_1e5_degrees) * 0.00001F;
  if (parsed.course_degrees < 0.0F) {
    parsed.course_degrees = std::fmod(parsed.course_degrees, 360.0F) + 360.0F;
  } else if (parsed.course_degrees >= 360.0F) {
    parsed.course_degrees = std::fmod(parsed.course_degrees, 360.0F);
  }
  parsed.horizontal_accuracy_m =
      static_cast<float>(read_u32(&payload[40])) * 0.001F;
  parsed.position_dop = static_cast<float>(read_u16(&payload[76])) * 0.01F;
  parsed.received_ms = received_ms;
  parsed.satellites = payload[23];
  parsed.fix_type = fix_type;
  parsed.position.longitude =
      static_cast<double>(read_i32(&payload[24])) * 0.0000001;
  parsed.position.latitude =
      static_cast<double>(read_i32(&payload[28])) * 0.0000001;
  parsed.utc.year = read_u16(&payload[4]);
  parsed.utc.month = payload[6];
  parsed.utc.day = payload[7];
  parsed.utc.hour = payload[8];
  parsed.utc.minute = payload[9];
  parsed.utc.second = payload[10];
  parsed.utc.valid =
      (validity & 0x03U) == 0x03U && utc_fields_valid(parsed.utc);
  parsed.gnss_fix_ok = fix_ok;
  parsed.position_valid =
      fix_ok && fix_type >= 2 && fix_type <= 4 && coordinates_valid &&
      parsed.position.latitude >= -90.0 && parsed.position.latitude <= 90.0 &&
      parsed.position.longitude >= -180.0 && parsed.position.longitude <= 180.0;
  parsed.valid = parsed.position_valid && ground_speed_mm_s >= 0 &&
                 std::isfinite(parsed.speed_kmh) &&
                 std::isfinite(parsed.speed_accuracy_kmh);
  parsed.speed_update = true;
  sample = parsed;
  return true;
}

} // namespace gpsmeter
