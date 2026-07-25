#pragma once

#include <cstddef>
#include <cstdint>

namespace gpsmeter {

struct GpsSample {
  float speed_kmh = 0.0F;
  uint64_t received_ms = 0;
  uint8_t satellites = 0;
  float hdop = 0.0F;
  char rmc_status = '?';
  char rmc_mode = '?';
  bool valid = false;
  bool speed_update = false;
};

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
  float hdop_ = 0.0F;
};

} // namespace gpsmeter
