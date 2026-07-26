#include "nmea_parser.hpp"

#include <algorithm>
#include <cerrno>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <iterator>

namespace gpsmeter {
namespace {

constexpr float kKnotsToKmh = 1.852F;

int hex_value(char value) {
  if (value >= '0' && value <= '9') {
    return value - '0';
  }
  if (value >= 'A' && value <= 'F') {
    return value - 'A' + 10;
  }
  if (value >= 'a' && value <= 'f') {
    return value - 'a' + 10;
  }
  return -1;
}

bool validate_checksum(const char *sentence) {
  if (sentence == nullptr || sentence[0] != '$') {
    return false;
  }
  const char *star = std::strchr(sentence, '*');
  if (star == nullptr || star[1] == '\0' || star[2] == '\0') {
    return false;
  }
  uint8_t checksum = 0;
  for (const char *cursor = sentence + 1; cursor < star; ++cursor) {
    checksum ^= static_cast<uint8_t>(*cursor);
  }
  const int high = hex_value(star[1]);
  const int low = hex_value(star[2]);
  return high >= 0 && low >= 0 &&
         checksum == static_cast<uint8_t>((high << 4) | low);
}

size_t split_fields(char *sentence, char **fields, size_t capacity) {
  size_t count = 0;
  char *cursor = sentence;
  while (cursor != nullptr && count < capacity) {
    fields[count++] = cursor;
    char *comma = std::strchr(cursor, ',');
    if (comma == nullptr) {
      break;
    }
    *comma = '\0';
    cursor = comma + 1;
  }
  return count;
}

bool parse_float(const char *text, float &value) {
  if (text == nullptr || *text == '\0') {
    return false;
  }
  errno = 0;
  char *end = nullptr;
  const float parsed = std::strtof(text, &end);
  if (errno != 0 || end == text || *end != '\0' || !std::isfinite(parsed)) {
    return false;
  }
  value = parsed;
  return true;
}

bool parse_coordinate(const char *value_text, const char *hemisphere_text,
                      bool latitude, double &value) {
  if (value_text == nullptr || hemisphere_text == nullptr ||
      *value_text == '\0' || *hemisphere_text == '\0') {
    return false;
  }
  errno = 0;
  char *end = nullptr;
  const double raw = std::strtod(value_text, &end);
  if (errno != 0 || end == value_text || *end != '\0' ||
      !std::isfinite(raw)) {
    return false;
  }
  const double degrees = std::floor(raw / 100.0);
  const double minutes = raw - degrees * 100.0;
  if (minutes < 0.0 || minutes >= 60.0 ||
      degrees > (latitude ? 90.0 : 180.0)) {
    return false;
  }
  value = degrees + minutes / 60.0;
  const char hemisphere = hemisphere_text[0];
  if ((latitude && hemisphere == 'S') ||
      (!latitude && hemisphere == 'W')) {
    value = -value;
  } else if ((latitude && hemisphere != 'N') ||
             (!latitude && hemisphere != 'E')) {
    return false;
  }
  return true;
}

bool parse_two_digits(const char *text, uint8_t &value) {
  if (text == nullptr || text[0] < '0' || text[0] > '9' ||
      text[1] < '0' || text[1] > '9') {
    return false;
  }
  value = static_cast<uint8_t>((text[0] - '0') * 10 + text[1] - '0');
  return true;
}

bool parse_utc(const char *time_text, const char *date_text,
               UtcDateTime &utc) {
  if (time_text == nullptr || date_text == nullptr ||
      std::strlen(time_text) < 6 || std::strlen(date_text) != 6) {
    return false;
  }
  uint8_t year = 0;
  if (!parse_two_digits(time_text, utc.hour) ||
      !parse_two_digits(time_text + 2, utc.minute) ||
      !parse_two_digits(time_text + 4, utc.second) ||
      !parse_two_digits(date_text, utc.day) ||
      !parse_two_digits(date_text + 2, utc.month) ||
      !parse_two_digits(date_text + 4, year)) {
    return false;
  }
  utc.year = static_cast<uint16_t>(year >= 80 ? 1900 + year : 2000 + year);
  utc.valid = utc.hour <= 23 && utc.minute <= 59 && utc.second <= 60 &&
              utc.month >= 1 && utc.month <= 12 && utc.day >= 1 &&
              utc.day <= 31;
  return utc.valid;
}

bool sentence_type_is(const char *field, const char *suffix) {
  const size_t field_length = std::strlen(field);
  const size_t suffix_length = std::strlen(suffix);
  return field_length >= suffix_length &&
         std::strcmp(field + field_length - suffix_length, suffix) == 0;
}

} // namespace

bool NmeaSentenceDetector::feed(char byte) {
  if (byte == '$') {
    length_ = 0;
    line_[length_++] = byte;
    return false;
  }
  if (length_ == 0) {
    return false;
  }
  if (byte == '\r') {
    return false;
  }
  if (byte == '\n') {
    line_[length_] = '\0';
    const bool valid = NmeaParser::checksum_valid(line_);
    length_ = 0;
    return valid;
  }
  if (length_ + 1 >= kMaxSentenceLength) {
    length_ = 0;
    return false;
  }
  line_[length_++] = byte;
  return false;
}

void NmeaSentenceDetector::reset() { length_ = 0; }

bool NmeaParser::feed(char byte, uint64_t received_ms, GpsSample &sample) {
  if (byte == '$') {
    length_ = 0;
    line_[length_++] = byte;
    return false;
  }
  if (length_ == 0) {
    return false;
  }
  if (byte == '\r') {
    return false;
  }
  if (byte == '\n') {
    line_[length_] = '\0';
    GpsSample parsed = sample;
    const bool produced = parse_sentence(line_, received_ms, parsed);
    length_ = 0;
    if (produced) {
      if (!parsed.speed_update) {
        satellites_ = parsed.satellites;
        position_dop_ = parsed.position_dop;
        return false;
      }
      if (parsed.satellites == 0) {
        parsed.satellites = satellites_;
        parsed.position_dop = position_dop_;
      }
      sample = parsed;
    }
    return produced;
  }
  if (length_ + 1 >= kMaxSentenceLength) {
    length_ = 0;
    return false;
  }
  line_[length_++] = byte;
  return false;
}

bool NmeaParser::parse_sentence(const char *sentence, uint64_t received_ms,
                                GpsSample &sample) {
  if (!checksum_valid(sentence)) {
    return false;
  }

  char copy[kMaxSentenceLength]{};
  std::strncpy(copy, sentence + 1, sizeof(copy) - 1);
  char *star = std::strchr(copy, '*');
  if (star != nullptr) {
    *star = '\0';
  }

  char *fields[20]{};
  const size_t count = split_fields(copy, fields, std::size(fields));
  if (count == 0) {
    return false;
  }

  if (sentence_type_is(fields[0], "RMC")) {
    if (count <= 9) {
      return false;
    }
    float knots = 0.0F;
    const bool speed_ok = parse_float(fields[7], knots) && knots >= 0.0F;
    float course = 0.0F;
    const bool course_ok = parse_float(fields[8], course);
    GeoPoint position{};
    const bool position_ok =
        parse_coordinate(fields[3], fields[4], true, position.latitude) &&
        parse_coordinate(fields[5], fields[6], false, position.longitude);
    UtcDateTime utc{};
    const bool utc_ok = parse_utc(fields[1], fields[9], utc);
    sample.speed_kmh = speed_ok ? knots * kKnotsToKmh : 0.0F;
    sample.course_degrees = course_ok ? course : 0.0F;
    sample.received_ms = received_ms;
    sample.position = position;
    sample.utc = utc;
    sample.rmc_status = fields[2][0] != '\0' ? fields[2][0] : '?';
    sample.rmc_mode =
        count > 12 && fields[12][0] != '\0' ? fields[12][0] : '?';
    sample.position_valid = fields[2][0] == 'A' && position_ok;
    sample.valid = fields[2][0] == 'A' && speed_ok;
    if (!utc_ok) {
      sample.utc.valid = false;
    }
    sample.speed_update = true;
    return true;
  }

  if (sentence_type_is(fields[0], "GGA")) {
    if (count <= 8) {
      return false;
    }
    const long fix_quality = std::strtol(fields[6], nullptr, 10);
    const long satellites = std::strtol(fields[7], nullptr, 10);
    float hdop = 0.0F;
    (void)parse_float(fields[8], hdop);
    sample.received_ms = received_ms;
    sample.satellites = static_cast<uint8_t>(
        std::clamp(satellites, 0L, static_cast<long>(UINT8_MAX)));
    sample.position_dop = hdop;
    sample.speed_update = false;
    if (fix_quality == 0) {
      sample.valid = false;
    }
    return true;
  }
  return false;
}

bool NmeaParser::checksum_valid(const char *sentence) {
  return validate_checksum(sentence);
}

} // namespace gpsmeter
