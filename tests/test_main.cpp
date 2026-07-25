#include "nmea_parser.hpp"
#include "speed_policy.hpp"

#include <cassert>
#include <cmath>
#include <cstdint>
#include <iostream>

namespace {

void test_nmea_rmc() {
  gpsmeter::GpsSample sample{};
  const bool parsed = gpsmeter::NmeaParser::parse_sentence(
      "$GPRMC,123519,A,4807.038,N,01131.000,E,21.6,084.4,230394,003.1,W*5B",
      1000, sample);
  assert(parsed);
  assert(sample.valid);
  assert(sample.speed_update);
  assert(std::fabs(sample.speed_kmh - 40.0032F) < 0.01F);
  assert(sample.received_ms == 1000);
}

void test_nmea_rmc_estimation_mode_is_exposed() {
  gpsmeter::GpsSample sample{};
  const bool parsed = gpsmeter::NmeaParser::parse_sentence(
      "$GNRMC,123519.00,A,4807.038,N,01131.000,E,21.6,084.4,230394,,,E*79",
      2000, sample);
  assert(parsed);
  assert(sample.rmc_status == 'A');
  assert(sample.rmc_mode == 'E');
}

void test_gga_does_not_refresh_speed() {
  gpsmeter::NmeaParser parser;
  gpsmeter::GpsSample sample{};
  constexpr const char *gga =
      "$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47\r\n";
  bool produced_speed = false;
  for (const char *cursor = gga; *cursor != '\0'; ++cursor) {
    produced_speed |= parser.feed(*cursor, 1000, sample);
  }
  assert(!produced_speed);
}

void test_nmea_rejects_bad_checksum() {
  gpsmeter::GpsSample sample{};
  assert(!gpsmeter::NmeaParser::parse_sentence(
      "$GPRMC,123519,A,4807.038,N,01131.000,E,21.6,084.4,230394,003.1,W*00",
      1000, sample));
}

void test_nmea_sentence_detector() {
  gpsmeter::NmeaSentenceDetector detector;
  constexpr const char *valid =
      "$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47\r\n";
  bool detected = false;
  for (const char *cursor = valid; *cursor != '\0'; ++cursor) {
    detected |= detector.feed(*cursor);
  }
  assert(detected);

  constexpr const char *invalid =
      "$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*00\r\n";
  detected = false;
  for (const char *cursor = invalid; *cursor != '\0'; ++cursor) {
    detected |= detector.feed(*cursor);
  }
  assert(!detected);
}

void test_exact_warning_threshold() {
  gpsmeter::PolicySettings settings{};
  assert(gpsmeter::classify_speed_alert(34.9F, gpsmeter::SpeedAlert::None,
                                        settings) ==
         gpsmeter::SpeedAlert::None);
  assert(gpsmeter::classify_speed_alert(35.0F, gpsmeter::SpeedAlert::None,
                                        settings) ==
         gpsmeter::SpeedAlert::Caution);
  assert(gpsmeter::classify_speed_alert(39.9F, gpsmeter::SpeedAlert::None,
                                        settings) ==
         gpsmeter::SpeedAlert::Caution);
  assert(gpsmeter::classify_speed_alert(40.0F, gpsmeter::SpeedAlert::None,
                                        settings) ==
         gpsmeter::SpeedAlert::Warning);
  assert(gpsmeter::display_speed(39.9F) == 39);
  assert(gpsmeter::display_speed(40.0F) == 40);
}

void test_direct_alert_thresholds() {
  gpsmeter::PolicySettings settings{};
  settings.caution_speed_kmh = 32.0F;
  settings.warning_speed_kmh = 38.0F;

  assert(gpsmeter::classify_speed_alert(31.9F, gpsmeter::SpeedAlert::None,
                                        settings) ==
         gpsmeter::SpeedAlert::None);
  assert(gpsmeter::classify_speed_alert(32.0F, gpsmeter::SpeedAlert::None,
                                        settings) ==
         gpsmeter::SpeedAlert::Caution);
  assert(gpsmeter::classify_speed_alert(37.9F, gpsmeter::SpeedAlert::None,
                                        settings) ==
         gpsmeter::SpeedAlert::Caution);
  assert(gpsmeter::classify_speed_alert(38.0F, gpsmeter::SpeedAlert::None,
                                        settings) ==
         gpsmeter::SpeedAlert::Warning);

  settings.caution_speed_kmh = 38.0F;
  assert(gpsmeter::classify_speed_alert(37.9F, gpsmeter::SpeedAlert::None,
                                        settings) ==
         gpsmeter::SpeedAlert::None);
  assert(gpsmeter::classify_speed_alert(38.0F, gpsmeter::SpeedAlert::None,
                                        settings) ==
         gpsmeter::SpeedAlert::Warning);
}

void test_mode_transitions() {
  gpsmeter::PolicySettings settings{};
  gpsmeter::ModeRuntime state{};
  assert(!gpsmeter::update_drive_mode(state, 60.0F, 1000, true, settings));
  assert(!gpsmeter::update_drive_mode(state, 65.0F, 1000, true, settings));
  assert(gpsmeter::update_drive_mode(state, 60.0F, 1000, true, settings));
  assert(state.mode == gpsmeter::DriveMode::Race);

  assert(!gpsmeter::update_drive_mode(state, 40.0F, 1000, true, settings));
  assert(!gpsmeter::update_drive_mode(state, 39.0F, 1000, true, settings));
  assert(gpsmeter::update_drive_mode(state, 40.0F, 1000, true, settings));
  assert(state.mode == gpsmeter::DriveMode::PitLane);
}

void test_stale_data_resets_timer() {
  gpsmeter::PolicySettings settings{};
  gpsmeter::ModeRuntime state{};
  assert(!gpsmeter::update_drive_mode(state, 70.0F, 2000, true, settings));
  assert(!gpsmeter::update_drive_mode(state, 70.0F, 5000, false, settings));
  assert(!gpsmeter::update_drive_mode(state, 70.0F, 1000, true, settings));
  assert(state.mode == gpsmeter::DriveMode::PitLane);
}

void test_continuous_pit_ng_tone_condition() {
  assert(!gpsmeter::should_sound_pit_ng_tone(
      gpsmeter::DriveMode::PitLane, 39.99F, true, true, 40.0F));
  assert(gpsmeter::should_sound_pit_ng_tone(
      gpsmeter::DriveMode::PitLane, 40.0F, true, true, 40.0F));
  assert(gpsmeter::should_sound_pit_ng_tone(
      gpsmeter::DriveMode::PitLane, 80.0F, true, true, 40.0F));
  assert(!gpsmeter::should_sound_pit_ng_tone(
      gpsmeter::DriveMode::Race, 40.0F, true, true, 40.0F));
  assert(!gpsmeter::should_sound_pit_ng_tone(
      gpsmeter::DriveMode::PitLane, 40.0F, false, true, 40.0F));
  assert(!gpsmeter::should_sound_pit_ng_tone(
      gpsmeter::DriveMode::PitLane, 40.0F, true, false, 40.0F));
}

} // namespace

int main() {
  test_nmea_rmc();
  test_nmea_rmc_estimation_mode_is_exposed();
  test_nmea_rejects_bad_checksum();
  test_nmea_sentence_detector();
  test_gga_does_not_refresh_speed();
  test_exact_warning_threshold();
  test_direct_alert_thresholds();
  test_mode_transitions();
  test_stale_data_resets_timer();
  test_continuous_pit_ng_tone_condition();
  std::cout << "All GPS Speed Meter tests passed\n";
  return 0;
}
