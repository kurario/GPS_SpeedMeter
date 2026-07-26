#include "race_progress.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>

namespace gpsmeter {
namespace {

constexpr double kEarthRadiusM = 6371008.8;
constexpr double kDegreesToRadians = 3.14159265358979323846 / 180.0;
constexpr double kRadiansToDegrees = 180.0 / 3.14159265358979323846;

float bearing_degrees(const GeoPoint &from, const GeoPoint &to) {
  const double lat1 = from.latitude * kDegreesToRadians;
  const double lat2 = to.latitude * kDegreesToRadians;
  const double longitude_delta =
      (to.longitude - from.longitude) * kDegreesToRadians;
  const double y = std::sin(longitude_delta) * std::cos(lat2);
  const double x = std::cos(lat1) * std::sin(lat2) -
                   std::sin(lat1) * std::cos(lat2) *
                       std::cos(longitude_delta);
  double bearing = std::atan2(y, x) * kRadiansToDegrees;
  if (bearing < 0.0) {
    bearing += 360.0;
  }
  return static_cast<float>(bearing);
}

float heading_difference(float lhs, float rhs) {
  const float difference = std::fabs(lhs - rhs);
  return std::min(difference, 360.0F - difference);
}

} // namespace

bool valid_geo_point(const GeoPoint &point) {
  return std::isfinite(point.latitude) && std::isfinite(point.longitude) &&
         point.latitude >= -90.0 && point.latitude <= 90.0 &&
         point.longitude >= -180.0 && point.longitude <= 180.0;
}

double distance_m(const GeoPoint &lhs, const GeoPoint &rhs) {
  if (!valid_geo_point(lhs) || !valid_geo_point(rhs)) {
    return INFINITY;
  }
  const double lat1 = lhs.latitude * kDegreesToRadians;
  const double lat2 = rhs.latitude * kDegreesToRadians;
  const double dlat = lat2 - lat1;
  const double dlon =
      (rhs.longitude - lhs.longitude) * kDegreesToRadians;
  const double sin_lat = std::sin(dlat / 2.0);
  const double sin_lon = std::sin(dlon / 2.0);
  const double a = sin_lat * sin_lat +
                   std::cos(lat1) * std::cos(lat2) * sin_lon * sin_lon;
  return 2.0 * kEarthRadiusM *
         std::asin(std::min(1.0, std::sqrt(std::max(0.0, a))));
}

uint32_t timed_race_remaining_s(const RaceConfig &config,
                                const UtcDateTime &utc,
                                uint32_t elapsed_since_utc_s) {
  if (!utc.valid || config.finish_hour > 23 || config.finish_minute > 59) {
    return UINT32_MAX;
  }
  int local_seconds =
      static_cast<int>(utc.hour) * 3600 +
      static_cast<int>(utc.minute) * 60 + utc.second +
      static_cast<int>(config.timezone_hours) * 3600 +
      static_cast<int>(std::min<uint32_t>(elapsed_since_utc_s, 86400));
  local_seconds %= 86400;
  if (local_seconds < 0) {
    local_seconds += 86400;
  }
  const int finish_seconds =
      static_cast<int>(config.finish_hour) * 3600 +
      static_cast<int>(config.finish_minute) * 60;
  return static_cast<uint32_t>(std::max(0, finish_seconds - local_seconds));
}

void format_minutes_seconds(uint64_t seconds, char *buffer, size_t size) {
  if (buffer == nullptr || size == 0) {
    return;
  }
  std::snprintf(buffer, size, "%llu:%02llu",
                static_cast<unsigned long long>(seconds / 60ULL),
                static_cast<unsigned long long>(seconds % 60ULL));
}

void RaceProgress::begin_new(const RaceConfig &config,
                             bool currently_inside_gate, uint64_t now_ms) {
  snapshot_ = RaceSnapshot{};
  snapshot_.config = config;
  snapshot_.phase = RacePhase::WaitingForBaseline;
  snapshot_.inside_gate = currently_inside_gate;
  snapshot_.excluded_remaining =
      std::min<uint8_t>(config.excluded_passes, 5);
  have_previous_position_ = false;
  race_started_ms_ = now_ms;
  stint_started_ms_ = now_ms;
}

RaceUpdate RaceProgress::update_position(const RaceConfig &config,
                                         const GeoPoint &position,
                                         bool position_valid,
                                         uint64_t now_ms) {
  RaceUpdate result{};
  if (snapshot_.phase == RacePhase::Idle ||
      snapshot_.phase == RacePhase::Finished || !position_valid ||
      !config.gate_valid || !valid_geo_point(position) ||
      !valid_geo_point(config.gate)) {
    return result;
  }

  const double distance = distance_m(position, config.gate);
  const bool entered = !snapshot_.inside_gate &&
                       distance <= config.gate_radius_m;
  const bool exited = snapshot_.inside_gate &&
                      distance >= config.gate_exit_radius_m;
  const bool have_entry_heading =
      entered && have_previous_position_ &&
      distance_m(previous_position_, position) >= 0.5;
  const float entry_heading =
      have_entry_heading
          ? bearing_degrees(previous_position_, config.gate)
          : 0.0F;
  previous_position_ = position;
  have_previous_position_ = true;

  if (entered) {
    snapshot_.inside_gate = true;
  } else if (exited) {
    snapshot_.inside_gate = false;
  }

  if (!snapshot_.baseline_established) {
    const bool baseline_event = entered || exited;
    if (!baseline_event) {
      return result;
    }
    snapshot_.baseline_established = true;
    if (entered && have_entry_heading) {
      snapshot_.direction_learned = true;
      snapshot_.entry_heading_degrees = entry_heading;
    }
    snapshot_.phase = RacePhase::WaitingForStart;
    result.baseline_established = true;
    if (snapshot_.excluded_remaining == 0) {
      start_race(now_ms);
      result.race_started = true;
    }
    return result;
  }

  if (!entered) {
    return result;
  }
  if (!snapshot_.direction_learned && have_entry_heading) {
    snapshot_.direction_learned = true;
    snapshot_.entry_heading_degrees = entry_heading;
  } else if (snapshot_.direction_learned &&
             (!have_entry_heading ||
              heading_difference(entry_heading,
                                 snapshot_.entry_heading_degrees) >
                  config.direction_tolerance_deg)) {
    return result;
  }
  if (snapshot_.phase == RacePhase::WaitingForStart) {
    if (snapshot_.excluded_remaining > 0) {
      --snapshot_.excluded_remaining;
    }
    if (snapshot_.excluded_remaining == 0) {
      start_race(now_ms);
      result.race_started = true;
    }
    return result;
  }

  if (snapshot_.phase == RacePhase::Running &&
      complete_lap(config, now_ms)) {
    result.lap_completed = true;
    if (config.format == RaceFormat::Laps &&
        snapshot_.completed_laps >= config.target_laps) {
      freeze_elapsed(now_ms);
      snapshot_.phase = RacePhase::Finished;
      result.race_finished = true;
    }
  }
  return result;
}

bool RaceProgress::switch_stint(uint64_t now_ms) {
  if (snapshot_.phase != RacePhase::Running) {
    return false;
  }
  snapshot_.stint_elapsed_ms = stint_elapsed_ms(now_ms);
  StintSummary completed{
      .number = snapshot_.current_stint,
      .completed_laps = snapshot_.current_stint_laps,
      .elapsed_ms = snapshot_.stint_elapsed_ms,
  };
  if (snapshot_.stint_history_count <
      RaceSnapshot::kMaximumStintHistory) {
    snapshot_.stint_history[snapshot_.stint_history_count++] = completed;
  } else {
    for (size_t index = 1; index < RaceSnapshot::kMaximumStintHistory;
         ++index) {
      snapshot_.stint_history[index - 1] =
          snapshot_.stint_history[index];
    }
    snapshot_.stint_history[RaceSnapshot::kMaximumStintHistory - 1] =
        completed;
  }
  snapshot_.current_stint =
      static_cast<uint16_t>(std::min<uint32_t>(
          static_cast<uint32_t>(snapshot_.current_stint) + 1U, UINT16_MAX));
  snapshot_.current_stint_laps = 0;
  snapshot_.stint_elapsed_ms = 0;
  stint_started_ms_ = now_ms;
  return true;
}

bool RaceProgress::finish_if_needed(const RaceConfig &config,
                                    const UtcDateTime &utc,
                                    uint64_t now_ms,
                                    uint32_t elapsed_since_utc_s) {
  if (snapshot_.phase != RacePhase::Running ||
      config.format != RaceFormat::Timed) {
    return false;
  }
  if (timed_race_remaining_s(config, utc, elapsed_since_utc_s) != 0) {
    return false;
  }
  freeze_elapsed(now_ms);
  snapshot_.phase = RacePhase::Finished;
  return true;
}

uint64_t RaceProgress::race_elapsed_ms(uint64_t now_ms) const {
  if (snapshot_.phase == RacePhase::Running) {
    return snapshot_.race_elapsed_ms + (now_ms - race_started_ms_);
  }
  return snapshot_.race_elapsed_ms;
}

uint64_t RaceProgress::stint_elapsed_ms(uint64_t now_ms) const {
  if (snapshot_.phase == RacePhase::Running) {
    return snapshot_.stint_elapsed_ms + (now_ms - stint_started_ms_);
  }
  return snapshot_.stint_elapsed_ms;
}

uint16_t RaceProgress::remaining_laps(const RaceConfig &config) const {
  if (snapshot_.completed_laps >= config.target_laps) {
    return 0;
  }
  return static_cast<uint16_t>(config.target_laps -
                               snapshot_.completed_laps);
}

void RaceProgress::start_race(uint64_t now_ms) {
  snapshot_.phase = RacePhase::Running;
  snapshot_.completed_laps = 0;
  snapshot_.current_stint = 1;
  snapshot_.current_stint_laps = 0;
  snapshot_.race_elapsed_ms = 0;
  snapshot_.stint_elapsed_ms = 0;
  snapshot_.last_lap_elapsed_ms = 0;
  race_started_ms_ = now_ms;
  stint_started_ms_ = now_ms;
}

bool RaceProgress::complete_lap(const RaceConfig &config, uint64_t now_ms) {
  const uint64_t elapsed = now_ms - race_started_ms_;
  if (snapshot_.last_lap_elapsed_ms != 0 &&
      elapsed - snapshot_.last_lap_elapsed_ms < config.minimum_lap_ms) {
    return false;
  }
  if (snapshot_.last_lap_elapsed_ms == 0 &&
      elapsed < config.minimum_lap_ms) {
    return false;
  }
  snapshot_.last_lap_elapsed_ms = elapsed;
  snapshot_.completed_laps =
      static_cast<uint16_t>(std::min<uint32_t>(
          static_cast<uint32_t>(snapshot_.completed_laps) + 1U, UINT16_MAX));
  snapshot_.current_stint_laps =
      static_cast<uint16_t>(std::min<uint32_t>(
          static_cast<uint32_t>(snapshot_.current_stint_laps) + 1U,
          UINT16_MAX));
  return true;
}

void RaceProgress::freeze_elapsed(uint64_t now_ms) {
  if (snapshot_.phase == RacePhase::Running) {
    snapshot_.race_elapsed_ms = race_elapsed_ms(now_ms);
    snapshot_.stint_elapsed_ms = stint_elapsed_ms(now_ms);
  }
}

} // namespace gpsmeter
