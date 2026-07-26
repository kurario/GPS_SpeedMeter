#pragma once

#include "race_progress.hpp"

#include <cstdint>

namespace gpsmeter {

struct GpsSample {
  float speed_kmh = 0.0F;
  float speed_accuracy_kmh = 0.0F;
  float course_degrees = 0.0F;
  float horizontal_accuracy_m = 0.0F;
  float position_dop = 0.0F;
  uint64_t received_ms = 0;
  uint8_t satellites = 0;
  uint8_t fix_type = 0;
  GeoPoint position{};
  UtcDateTime utc{};
  char rmc_status = '?';
  char rmc_mode = '?';
  bool gnss_fix_ok = false;
  bool position_valid = false;
  bool valid = false;
  bool speed_update = false;
};

} // namespace gpsmeter
