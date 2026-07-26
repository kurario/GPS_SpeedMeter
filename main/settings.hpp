#pragma once

#include "race_progress.hpp"
#include "speed_policy.hpp"

#include <array>
#include <cstdint>

namespace gpsmeter {

struct AppSettings {
  static constexpr uint32_t kSchemaVersion = 3;

  uint32_t schema_version = kSchemaVersion;
  uint16_t pit_limit_kmh = 40;
  uint16_t caution_speed_kmh = 35;
  uint16_t warning_speed_kmh = 40;
  uint16_t alert_clear_margin_kmh = 2;
  uint16_t race_enter_speed_kmh = 60;
  uint16_t race_enter_hold_s = 3;
  uint16_t pit_return_speed_kmh = 40;
  uint16_t pit_return_hold_s = 3;
  uint16_t caution_tone_hz = 1200;
  uint16_t warning_tone_hz = 3200;
  uint8_t tone_volume = 192;
  uint8_t display_brightness = 180;
  bool pit_tone_enabled = true;
  bool pit_blink_enabled = true;
  RaceFormat race_format = RaceFormat::Timed;
  uint8_t race_finish_hour = 16;
  uint8_t race_finish_minute = 0;
  int8_t timezone_hours = 9;
  uint8_t excluded_passes = 0;
  uint16_t target_laps = 100;
  uint16_t minimum_lap_s = 30;
  uint16_t gate_radius_m = 25;
  uint16_t gate_exit_radius_m = 40;
  bool course_valid = false;
  std::array<char, 25> course_name{};
  double course_latitude = 0.0;
  double course_longitude = 0.0;
};

bool validate_settings(const AppSettings &settings);
void normalize_settings(AppSettings &settings);
PolicySettings make_policy(const AppSettings &settings);
RaceConfig make_race_config(const AppSettings &settings);
bool load_settings(AppSettings &settings);
bool save_settings(const AppSettings &settings);

} // namespace gpsmeter
