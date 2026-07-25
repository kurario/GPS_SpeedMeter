#include "settings.hpp"

#include "esp_log.h"
#include "nvs.h"

#include <algorithm>
#include <array>
#include <cstring>

namespace gpsmeter {
namespace {

constexpr const char *kTag = "settings";
constexpr const char *kNamespace = "gpsmeter";
constexpr const char *kKey = "config";

struct LegacyAppSettingsV1 {
  uint32_t schema_version = 1;
  uint16_t pit_limit_kmh = 40;
  uint16_t caution_margin_kmh = 0;
  uint16_t warning_margin_kmh = 0;
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

static_assert(sizeof(LegacyAppSettingsV1) == sizeof(AppSettings));

template <typename T>
T clamp_value(T value, T minimum, T maximum) {
  return std::max(minimum, std::min(value, maximum));
}

} // namespace

bool validate_settings(const AppSettings &settings) {
  return settings.schema_version == AppSettings::kSchemaVersion &&
         settings.pit_limit_kmh >= 20 && settings.pit_limit_kmh <= 80 &&
         settings.caution_speed_kmh >= 20 &&
         settings.caution_speed_kmh <= settings.warning_speed_kmh &&
         settings.warning_speed_kmh <= settings.pit_limit_kmh &&
         settings.alert_clear_margin_kmh <= 10 &&
         settings.race_enter_speed_kmh > settings.pit_limit_kmh &&
         settings.race_enter_speed_kmh <= 200 &&
         settings.race_enter_hold_s >= 1 &&
         settings.race_enter_hold_s <= 30 &&
         settings.pit_return_speed_kmh < settings.race_enter_speed_kmh &&
         settings.pit_return_speed_kmh <= 100 &&
         settings.pit_return_hold_s >= 1 &&
         settings.pit_return_hold_s <= 300 &&
         settings.caution_tone_hz >= 400 &&
         settings.caution_tone_hz <= 6000 &&
         settings.warning_tone_hz >= 400 &&
         settings.warning_tone_hz <= 6000;
}

void normalize_settings(AppSettings &settings) {
  settings.schema_version = AppSettings::kSchemaVersion;
  settings.pit_limit_kmh =
      clamp_value<uint16_t>(settings.pit_limit_kmh, 20, 80);
  settings.warning_speed_kmh =
      clamp_value<uint16_t>(settings.warning_speed_kmh, 20,
                            settings.pit_limit_kmh);
  settings.caution_speed_kmh =
      clamp_value<uint16_t>(settings.caution_speed_kmh, 20,
                            settings.warning_speed_kmh);
  settings.alert_clear_margin_kmh =
      clamp_value<uint16_t>(settings.alert_clear_margin_kmh, 0, 10);
  settings.race_enter_speed_kmh =
      clamp_value<uint16_t>(settings.race_enter_speed_kmh,
                            settings.pit_limit_kmh + 1, 200);
  settings.race_enter_hold_s =
      clamp_value<uint16_t>(settings.race_enter_hold_s, 1, 30);
  settings.pit_return_speed_kmh =
      clamp_value<uint16_t>(settings.pit_return_speed_kmh, 0,
                            settings.race_enter_speed_kmh - 1);
  settings.pit_return_hold_s =
      clamp_value<uint16_t>(settings.pit_return_hold_s, 1, 300);
  settings.caution_tone_hz =
      clamp_value<uint16_t>(settings.caution_tone_hz, 400, 6000);
  settings.warning_tone_hz =
      clamp_value<uint16_t>(settings.warning_tone_hz, 400, 6000);
}

PolicySettings make_policy(const AppSettings &settings) {
  return PolicySettings{
      .pit_limit_kmh = static_cast<float>(settings.pit_limit_kmh),
      .caution_speed_kmh =
          static_cast<float>(settings.caution_speed_kmh),
      .warning_speed_kmh =
          static_cast<float>(settings.warning_speed_kmh),
      .alert_clear_margin_kmh =
          static_cast<float>(settings.alert_clear_margin_kmh),
      .race_enter_speed_kmh =
          static_cast<float>(settings.race_enter_speed_kmh),
      .race_enter_hold_ms =
          static_cast<uint32_t>(settings.race_enter_hold_s) * 1000U,
      .pit_return_speed_kmh =
          static_cast<float>(settings.pit_return_speed_kmh),
      .pit_return_hold_ms =
          static_cast<uint32_t>(settings.pit_return_hold_s) * 1000U,
  };
}

bool load_settings(AppSettings &settings) {
  nvs_handle_t handle = 0;
  esp_err_t err = nvs_open(kNamespace, NVS_READONLY, &handle);
  if (err != ESP_OK) {
    ESP_LOGW(kTag, "No saved settings; using defaults: %s",
             esp_err_to_name(err));
    settings = AppSettings{};
    return false;
  }

  std::array<uint8_t, sizeof(AppSettings)> bytes{};
  size_t size = bytes.size();
  err = nvs_get_blob(handle, kKey, bytes.data(), &size);
  nvs_close(handle);
  if (err != ESP_OK || size != bytes.size()) {
    ESP_LOGW(kTag, "Saved settings invalid; using defaults");
    settings = AppSettings{};
    return false;
  }

  uint32_t schema_version = 0;
  std::memcpy(&schema_version, bytes.data(), sizeof(schema_version));
  if (schema_version == AppSettings::kSchemaVersion) {
    AppSettings stored{};
    std::memcpy(&stored, bytes.data(), sizeof(stored));
    if (validate_settings(stored)) {
      settings = stored;
      return true;
    }
  } else if (schema_version == 1) {
    LegacyAppSettingsV1 legacy{};
    std::memcpy(&legacy, bytes.data(), sizeof(legacy));
    const uint16_t legacy_pit_limit =
        clamp_value<uint16_t>(legacy.pit_limit_kmh, 20, 80);
    const uint16_t legacy_warning_margin =
        clamp_value<uint16_t>(legacy.warning_margin_kmh, 0, 20);
    const uint16_t legacy_caution_margin =
        clamp_value<uint16_t>(legacy.caution_margin_kmh, 0, 20);
    const uint16_t warning_speed = clamp_value<uint16_t>(
        legacy_warning_margin >= legacy_pit_limit
            ? 0
            : legacy_pit_limit - legacy_warning_margin,
        20, legacy_pit_limit);
    const bool caution_was_enabled =
        legacy_caution_margin > legacy_warning_margin;
    const uint16_t caution_speed = caution_was_enabled
                                        ? clamp_value<uint16_t>(
                                              legacy_pit_limit -
                                                  legacy_caution_margin,
                                              20, warning_speed)
                                        : warning_speed;
    settings = AppSettings{
        .pit_limit_kmh = legacy_pit_limit,
        .caution_speed_kmh = caution_speed,
        .warning_speed_kmh = warning_speed,
        .alert_clear_margin_kmh = legacy.alert_clear_margin_kmh,
        .race_enter_speed_kmh = legacy.race_enter_speed_kmh,
        .race_enter_hold_s = legacy.race_enter_hold_s,
        .pit_return_speed_kmh = legacy.pit_return_speed_kmh,
        .pit_return_hold_s = legacy.pit_return_hold_s,
        .caution_tone_hz = legacy.caution_tone_hz,
        .warning_tone_hz = legacy.warning_tone_hz,
        .tone_volume = legacy.tone_volume,
        .display_brightness = legacy.display_brightness,
        .pit_tone_enabled = legacy.pit_tone_enabled,
        .pit_blink_enabled = legacy.pit_blink_enabled,
        .auto_mode_enabled = legacy.auto_mode_enabled,
    };
    normalize_settings(settings);
    ESP_LOGI(kTag, "Migrated settings schema v1 to v2");
    (void)save_settings(settings);
    return true;
  }

  ESP_LOGW(kTag, "Saved settings invalid; using defaults");
  settings = AppSettings{};
  return false;
}

bool save_settings(const AppSettings &settings) {
  AppSettings normalized = settings;
  normalize_settings(normalized);
  if (!validate_settings(normalized)) {
    ESP_LOGE(kTag, "Refusing invalid settings");
    return false;
  }

  nvs_handle_t handle = 0;
  esp_err_t err = nvs_open(kNamespace, NVS_READWRITE, &handle);
  if (err == ESP_OK) {
    err = nvs_set_blob(handle, kKey, &normalized, sizeof(normalized));
  }
  if (err == ESP_OK) {
    err = nvs_commit(handle);
  }
  if (handle != 0) {
    nvs_close(handle);
  }
  if (err != ESP_OK) {
    ESP_LOGE(kTag, "Failed to save settings: %s", esp_err_to_name(err));
    return false;
  }
  ESP_LOGI(kTag, "Settings saved");
  return true;
}

} // namespace gpsmeter
