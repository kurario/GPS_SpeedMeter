#pragma once

#include "gps_sample.hpp"

#include <cstddef>
#include <cstdint>

namespace gpsmeter {

class NmeaSentenceDetector {
public:
  bool feed(char byte);
  void reset();

private:
  static constexpr size_t kMaxSentenceLength = 128;
  char line_[kMaxSentenceLength]{};
  size_t length_ = 0;
};

class NmeaParser {
public:
  bool feed(char byte, uint64_t received_ms, GpsSample &sample);
  static bool parse_sentence(const char *sentence, uint64_t received_ms,
                             GpsSample &sample);
  static bool checksum_valid(const char *sentence);

private:
  static constexpr size_t kMaxSentenceLength = 128;
  char line_[kMaxSentenceLength]{};
  size_t length_ = 0;
  uint8_t satellites_ = 0;
  float position_dop_ = 0.0F;
};

} // namespace gpsmeter
