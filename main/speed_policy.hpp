#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>

namespace gpsmeter {

enum class DriveMode : uint8_t {
  PitLane = 0,
  Race = 1,
};

enum class SpeedAlert : uint8_t {
  None = 0,
  Caution = 1,
  Warning = 2,
};

struct PolicySettings {
  float pit_limit_kmh = 40.0F;
  float caution_speed_kmh = 35.0F;
  float warning_speed_kmh = 40.0F;
  float alert_clear_margin_kmh = 2.0F;
  float race_enter_speed_kmh = 60.0F;
  uint32_t race_enter_hold_ms = 3000;
  float pit_return_speed_kmh = 40.0F;
  uint32_t pit_return_hold_ms = 3000;
};

struct ModeRuntime {
  DriveMode mode = DriveMode::PitLane;
  uint32_t race_candidate_ms = 0;
  uint32_t pit_candidate_ms = 0;
};

constexpr uint32_t add_saturated(uint32_t value, uint32_t delta,
                                 uint32_t ceiling) {
  if (value >= ceiling || delta >= ceiling - value) {
    return ceiling;
  }
  return value + delta;
}

inline bool update_drive_mode(ModeRuntime &state, float speed_kmh,
                              uint32_t elapsed_ms, bool data_fresh,
                              const PolicySettings &cfg) {
  if (!data_fresh || !std::isfinite(speed_kmh)) {
    state.race_candidate_ms = 0;
    state.pit_candidate_ms = 0;
    return false;
  }

  if (state.mode == DriveMode::PitLane) {
    state.pit_candidate_ms = 0;
    if (speed_kmh >= cfg.race_enter_speed_kmh) {
      state.race_candidate_ms =
          add_saturated(state.race_candidate_ms, elapsed_ms,
                        cfg.race_enter_hold_ms);
      if (state.race_candidate_ms >= cfg.race_enter_hold_ms) {
        state.mode = DriveMode::Race;
        state.race_candidate_ms = 0;
        return true;
      }
    } else {
      state.race_candidate_ms = 0;
    }
    return false;
  }

  state.race_candidate_ms = 0;
  if (speed_kmh <= cfg.pit_return_speed_kmh) {
    state.pit_candidate_ms =
        add_saturated(state.pit_candidate_ms, elapsed_ms,
                      cfg.pit_return_hold_ms);
    if (state.pit_candidate_ms >= cfg.pit_return_hold_ms) {
      state.mode = DriveMode::PitLane;
      state.pit_candidate_ms = 0;
      return true;
    }
  } else {
    state.pit_candidate_ms = 0;
  }
  return false;
}

inline SpeedAlert classify_speed_alert(float speed_kmh, SpeedAlert previous,
                                       const PolicySettings &cfg) {
  if (!std::isfinite(speed_kmh)) {
    return SpeedAlert::None;
  }

  const float warning_threshold = cfg.warning_speed_kmh;
  const bool caution_enabled = cfg.caution_speed_kmh < cfg.warning_speed_kmh;
  const float caution_threshold = cfg.caution_speed_kmh;
  const float clear = std::max(0.0F, cfg.alert_clear_margin_kmh);

  if (previous == SpeedAlert::Warning &&
      speed_kmh >= warning_threshold - clear) {
    return SpeedAlert::Warning;
  }
  if (previous == SpeedAlert::Caution && caution_enabled &&
      speed_kmh >= caution_threshold - clear) {
    return speed_kmh >= warning_threshold ? SpeedAlert::Warning
                                          : SpeedAlert::Caution;
  }
  if (speed_kmh >= warning_threshold) {
    return SpeedAlert::Warning;
  }
  if (caution_enabled && speed_kmh >= caution_threshold) {
    return SpeedAlert::Caution;
  }
  return SpeedAlert::None;
}

inline int display_speed(float speed_kmh) {
  if (!std::isfinite(speed_kmh) || speed_kmh < 0.0F) {
    return 0;
  }
  return std::clamp(static_cast<int>(std::floor(speed_kmh)), 0, 999);
}

constexpr uint32_t alert_repeat_ms(SpeedAlert alert) {
  switch (alert) {
  case SpeedAlert::Caution:
    return 1500;
  case SpeedAlert::Warning:
    return 500;
  case SpeedAlert::None:
  default:
    return UINT32_MAX;
  }
}

inline bool should_emit_alert_tone(SpeedAlert alert, SpeedAlert previous,
                                   uint32_t now_ms, uint32_t last_tone_ms) {
  if (alert == SpeedAlert::None) {
    return false;
  }
  return alert != previous ||
         now_ms - last_tone_ms >= alert_repeat_ms(alert);
}

inline bool should_sound_pit_ng_tone(DriveMode mode, float speed_kmh,
                                     bool speed_fresh, bool tone_enabled,
                                     float pit_limit_kmh) {
  return tone_enabled && speed_fresh && mode == DriveMode::PitLane &&
         std::isfinite(speed_kmh) && speed_kmh >= pit_limit_kmh;
}

} // namespace gpsmeter
