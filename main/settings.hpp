#pragma once

#include "speed_policy.hpp"

#include <cstdint>

namespace gpsmeter {

struct AppSettings {
  static constexpr uint32_t kSchemaVersion = 2;

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
  bool auto_mode_enabled = true;
};

bool validate_settings(const AppSettings &settings);
void normalize_settings(AppSettings &settings);
PolicySettings make_policy(const AppSettings &settings);
bool load_settings(AppSettings &settings);
bool save_settings(const AppSettings &settings);

} // namespace gpsmeter
