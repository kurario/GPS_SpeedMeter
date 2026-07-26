#pragma once

#include <cstddef>
#include <cstdint>

namespace gpsmeter {

enum class RaceFormat : uint8_t {
  Timed = 0,
  Laps = 1,
};

enum class RacePhase : uint8_t {
  Idle = 0,
  WaitingForBaseline = 1,
  WaitingForStart = 2,
  Running = 3,
  Finished = 4,
};

struct GeoPoint {
  double latitude = 0.0;
  double longitude = 0.0;
};

struct UtcDateTime {
  uint16_t year = 0;
  uint8_t month = 0;
  uint8_t day = 0;
  uint8_t hour = 0;
  uint8_t minute = 0;
  uint8_t second = 0;
  bool valid = false;
};

struct RaceConfig {
  RaceFormat format = RaceFormat::Timed;
  uint8_t finish_hour = 16;
  uint8_t finish_minute = 0;
  int8_t timezone_hours = 9;
  uint16_t target_laps = 100;
  uint8_t excluded_passes = 0;
  float gate_radius_m = 25.0F;
  float gate_exit_radius_m = 40.0F;
  float direction_tolerance_deg = 90.0F;
  uint32_t minimum_lap_ms = 30000;
  GeoPoint gate{};
  bool gate_valid = false;
};

struct StintSummary {
  uint16_t number = 0;
  uint16_t completed_laps = 0;
  uint64_t elapsed_ms = 0;
};

struct RaceSnapshot {
  static constexpr size_t kMaximumStintHistory = 24;

  RaceConfig config{};
  RacePhase phase = RacePhase::Idle;
  bool baseline_established = false;
  bool inside_gate = false;
  bool direction_learned = false;
  uint8_t excluded_remaining = 0;
  uint16_t completed_laps = 0;
  uint16_t current_stint = 0;
  uint16_t current_stint_laps = 0;
  uint8_t stint_history_count = 0;
  uint64_t race_elapsed_ms = 0;
  uint64_t stint_elapsed_ms = 0;
  uint64_t last_lap_elapsed_ms = 0;
  float entry_heading_degrees = 0.0F;
  StintSummary stint_history[kMaximumStintHistory]{};
};

struct RaceUpdate {
  bool baseline_established = false;
  bool race_started = false;
  bool lap_completed = false;
  bool race_finished = false;
};

bool valid_geo_point(const GeoPoint &point);
double distance_m(const GeoPoint &lhs, const GeoPoint &rhs);
uint32_t timed_race_remaining_s(const RaceConfig &config,
                                const UtcDateTime &utc,
                                uint32_t elapsed_since_utc_s = 0);
void format_minutes_seconds(uint64_t seconds, char *buffer, size_t size);

class RaceProgress {
public:
  void begin_new(const RaceConfig &config, bool currently_inside_gate,
                 uint64_t now_ms);
  RaceUpdate update_position(const RaceConfig &config,
                             const GeoPoint &position, bool position_valid,
                             uint64_t now_ms);
  bool switch_stint(uint64_t now_ms);
  bool finish_if_needed(const RaceConfig &config, const UtcDateTime &utc,
                        uint64_t now_ms,
                        uint32_t elapsed_since_utc_s = 0);

  [[nodiscard]] RacePhase phase() const { return snapshot_.phase; }
  [[nodiscard]] uint16_t completed_laps() const {
    return snapshot_.completed_laps;
  }
  [[nodiscard]] uint16_t current_lap() const {
    return snapshot_.phase >= RacePhase::Running
               ? static_cast<uint16_t>(snapshot_.completed_laps + 1U)
               : 0;
  }
  [[nodiscard]] uint16_t current_stint() const {
    return snapshot_.current_stint;
  }
  [[nodiscard]] uint16_t current_stint_laps() const {
    return snapshot_.current_stint_laps;
  }
  [[nodiscard]] uint8_t stint_history_count() const {
    return snapshot_.stint_history_count;
  }
  [[nodiscard]] uint64_t race_elapsed_ms(uint64_t now_ms) const;
  [[nodiscard]] uint64_t stint_elapsed_ms(uint64_t now_ms) const;
  [[nodiscard]] uint16_t remaining_laps(const RaceConfig &config) const;
  [[nodiscard]] const RaceConfig &config() const { return snapshot_.config; }

private:
  void start_race(uint64_t now_ms);
  bool complete_lap(const RaceConfig &config, uint64_t now_ms);
  void freeze_elapsed(uint64_t now_ms);

  RaceSnapshot snapshot_{};
  GeoPoint previous_position_{};
  bool have_previous_position_ = false;
  uint64_t race_started_ms_ = 0;
  uint64_t stint_started_ms_ = 0;
};

} // namespace gpsmeter
