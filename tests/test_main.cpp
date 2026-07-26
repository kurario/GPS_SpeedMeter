#include "course_config.hpp"
#include "nmea_parser.hpp"
#include "race_progress.hpp"
#include "speed_policy.hpp"
#include "ubx_parser.hpp"

#include <array>
#include <cassert>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <vector>

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
  assert(sample.position_valid);
  assert(std::fabs(sample.position.latitude - 48.1173) < 0.000001);
  assert(std::fabs(sample.position.longitude - 11.5166667) < 0.000001);
  assert(sample.utc.valid);
  assert(sample.utc.year == 1994);
  assert(sample.utc.month == 3);
  assert(sample.utc.day == 23);
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

void write_u16(std::array<uint8_t, gpsmeter::UbxFrame::kMaxPayloadLength> &data,
               size_t offset, uint16_t value) {
  data[offset] = static_cast<uint8_t>(value);
  data[offset + 1] = static_cast<uint8_t>(value >> 8U);
}

void write_u32(std::array<uint8_t, gpsmeter::UbxFrame::kMaxPayloadLength> &data,
               size_t offset, uint32_t value) {
  for (size_t byte = 0; byte < 4; ++byte) {
    data[offset + byte] =
        static_cast<uint8_t>(value >> static_cast<uint32_t>(byte * 8U));
  }
}

void write_i32(std::array<uint8_t, gpsmeter::UbxFrame::kMaxPayloadLength> &data,
               size_t offset, int32_t value) {
  uint32_t bits = 0;
  static_assert(sizeof(bits) == sizeof(value));
  std::memcpy(&bits, &value, sizeof(bits));
  write_u32(data, offset, bits);
}

std::vector<uint8_t> serialize_ubx(const gpsmeter::UbxFrame &frame) {
  std::vector<uint8_t> bytes{
      0xB5,
      0x62,
      frame.message_class,
      frame.message_id,
      static_cast<uint8_t>(frame.payload_length),
      static_cast<uint8_t>(frame.payload_length >> 8U),
  };
  bytes.insert(bytes.end(), frame.payload.begin(),
               frame.payload.begin() + frame.payload_length);
  uint8_t checksum_a = 0;
  uint8_t checksum_b = 0;
  for (size_t index = 2; index < bytes.size(); ++index) {
    checksum_a = static_cast<uint8_t>(checksum_a + bytes[index]);
    checksum_b = static_cast<uint8_t>(checksum_b + checksum_a);
  }
  bytes.push_back(checksum_a);
  bytes.push_back(checksum_b);
  return bytes;
}

gpsmeter::UbxFrame make_nav_pvt_frame() {
  gpsmeter::UbxFrame frame{};
  frame.message_class = 0x01;
  frame.message_id = 0x07;
  frame.payload_length = 92;
  write_u16(frame.payload, 4, 2026);
  frame.payload[6] = 7;
  frame.payload[7] = 26;
  frame.payload[8] = 5;
  frame.payload[9] = 54;
  frame.payload[10] = 53;
  frame.payload[11] = 0x03;
  frame.payload[20] = 3;
  frame.payload[21] = 0x01;
  frame.payload[23] = 20;
  write_i32(frame.payload, 24, 1391234567);
  write_i32(frame.payload, 28, 351234567);
  write_u32(frame.payload, 40, 1500);
  write_i32(frame.payload, 60, 11111);
  write_i32(frame.payload, 64, 1234567);
  write_u32(frame.payload, 68, 250);
  write_u16(frame.payload, 76, 80);
  return frame;
}

void test_ubx_stream_and_nav_pvt() {
  const auto nav_pvt = make_nav_pvt_frame();
  auto bytes = serialize_ubx(nav_pvt);
  gpsmeter::UbxParser parser;
  gpsmeter::UbxFrame parsed_frame{};
  bool produced = false;
  for (const uint8_t byte : bytes) {
    produced |= parser.feed(byte, parsed_frame);
  }
  assert(produced);

  gpsmeter::GpsSample sample{};
  assert(gpsmeter::parse_ubx_nav_pvt(parsed_frame, 1234, sample));
  assert(sample.valid);
  assert(sample.position_valid);
  assert(sample.gnss_fix_ok);
  assert(sample.fix_type == 3);
  assert(sample.satellites == 20);
  assert(std::fabs(sample.speed_kmh - 39.9996F) < 0.001F);
  assert(std::fabs(sample.speed_accuracy_kmh - 0.9F) < 0.001F);
  assert(std::fabs(sample.course_degrees - 12.34567F) < 0.001F);
  assert(std::fabs(sample.horizontal_accuracy_m - 1.5F) < 0.001F);
  assert(std::fabs(sample.position_dop - 0.8F) < 0.001F);
  assert(std::fabs(sample.position.latitude - 35.1234567) < 0.0000001);
  assert(std::fabs(sample.position.longitude - 139.1234567) < 0.0000001);
  assert(sample.utc.valid);
  assert(sample.received_ms == 1234);

  bytes.back() ^= 0x01;
  parser.reset();
  produced = false;
  for (const uint8_t byte : bytes) {
    produced |= parser.feed(byte, parsed_frame);
  }
  assert(!produced);
}

void test_ubx_nav_pvt_rejects_invalid_fix() {
  auto nav_pvt = make_nav_pvt_frame();
  nav_pvt.payload[21] = 0;
  gpsmeter::GpsSample sample{};
  assert(gpsmeter::parse_ubx_nav_pvt(nav_pvt, 2000, sample));
  assert(!sample.valid);
  assert(!sample.position_valid);

  nav_pvt = make_nav_pvt_frame();
  nav_pvt.payload[78] = 0x01;
  assert(gpsmeter::parse_ubx_nav_pvt(nav_pvt, 2000, sample));
  assert(!sample.valid);
  assert(!sample.position_valid);
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

void test_course_file_parser() {
  constexpr std::string_view input =
      "# comment\n"
      "[COURSE01]\n"
      "latitude=36.1234567\n"
      "longitude=140.1234567\n"
      "\n"
      "[BROKEN]\n"
      "latitude=99\n"
      "longitude=140\n"
      "\n"
      "[COURSE02]\n"
      "latitude = 35.9876543\n"
      "longitude = 139.9876543\n";
  const auto courses = gpsmeter::parse_courses(input);
  assert(courses.count == 3);
  assert(courses.invalid_sections == 1);
  assert(courses.entries[0].valid);
  assert(!courses.entries[1].valid);
  assert(courses.entries[2].valid);

  const auto duplicates = gpsmeter::parse_courses(
      "[A]\nlatitude=35\nlongitude=139\n"
      "[B]\nlatitude=35\nlongitude=139\n");
  assert(duplicates.count == 2);
  assert(duplicates.entries[0].valid);
  assert(!duplicates.entries[1].valid);
}

void move_outside(gpsmeter::RaceProgress &progress,
                  const gpsmeter::RaceConfig &config, uint64_t now_ms) {
  auto outside = config.gate;
  outside.latitude += 0.001;
  (void)progress.update_position(config, outside, true, now_ms);
}

void move_inside(gpsmeter::RaceProgress &progress,
                 const gpsmeter::RaceConfig &config, uint64_t now_ms) {
  (void)progress.update_position(config, config.gate, true, now_ms);
}

void test_race_baseline_exclusion_and_laps() {
  gpsmeter::RaceConfig config{};
  config.format = gpsmeter::RaceFormat::Laps;
  config.target_laps = 2;
  config.excluded_passes = 1;
  config.minimum_lap_ms = 30000;
  config.gate = {35.0, 139.0};
  config.gate_valid = true;

  gpsmeter::RaceProgress progress;
  progress.begin_new(config, false, 0);
  move_inside(progress, config, 1000); // baseline
  assert(progress.phase() == gpsmeter::RacePhase::WaitingForStart);
  move_outside(progress, config, 2000);
  move_inside(progress, config, 5000); // excluded formation pass
  assert(progress.phase() == gpsmeter::RacePhase::Running);
  assert(progress.current_lap() == 1);
  move_outside(progress, config, 6000);
  move_inside(progress, config, 34000); // too early
  assert(progress.completed_laps() == 0);
  move_outside(progress, config, 35000);
  move_inside(progress, config, 36000);
  assert(progress.completed_laps() == 1);
  assert(progress.current_lap() == 2);
  assert(progress.current_stint_laps() == 1);
  assert(progress.switch_stint(40000));
  assert(progress.current_stint() == 2);
  assert(progress.current_stint_laps() == 0);
  assert(progress.stint_history_count() == 1);
  move_outside(progress, config, 65000);
  move_inside(progress, config, 67000);
  assert(progress.completed_laps() == 2);
  assert(progress.phase() == gpsmeter::RacePhase::Finished);
}

void test_timed_remaining_and_format() {
  gpsmeter::RaceConfig config{};
  config.finish_hour = 16;
  config.finish_minute = 0;
  config.timezone_hours = 9;
  gpsmeter::UtcDateTime utc{
      .year = 2026,
      .month = 7,
      .day = 26,
      .hour = 5,
      .minute = 54,
      .second = 53,
      .valid = true,
  };
  assert(gpsmeter::timed_race_remaining_s(config, utc) == 3907);
  assert(gpsmeter::timed_race_remaining_s(config, utc, 7) == 3900);
  char text[20]{};
  gpsmeter::format_minutes_seconds(7507, text, sizeof(text));
  assert(std::string_view(text) == "125:07");
}

void test_gate_direction() {
  gpsmeter::RaceConfig config{};
  config.format = gpsmeter::RaceFormat::Laps;
  config.target_laps = 5;
  config.minimum_lap_ms = 10000;
  config.gate = {35.0, 139.0};
  config.gate_valid = true;

  gpsmeter::RaceProgress progress;
  progress.begin_new(config, false, 0);
  move_outside(progress, config, 0);
  move_inside(progress, config, 1000); // learns southbound direction
  assert(progress.phase() == gpsmeter::RacePhase::Running);
  move_outside(progress, config, 2000);

  auto south = config.gate;
  south.latitude -= 0.001;
  (void)progress.update_position(config, south, true, 15000);
  move_inside(progress, config, 16000); // northbound: reject
  assert(progress.completed_laps() == 0);

  (void)progress.update_position(config, south, true, 17000);
  move_outside(progress, config, 20000);
  move_inside(progress, config, 21000); // southbound: accept
  assert(progress.completed_laps() == 1);

}

void test_starting_inside_gate_does_not_count_residence() {
  gpsmeter::RaceConfig config{};
  config.format = gpsmeter::RaceFormat::Laps;
  config.target_laps = 2;
  config.minimum_lap_ms = 10000;
  config.gate = {35.0, 139.0};
  config.gate_valid = true;

  gpsmeter::RaceProgress progress;
  progress.begin_new(config, true, 0);
  move_inside(progress, config, 1000);
  assert(progress.phase() == gpsmeter::RacePhase::WaitingForBaseline);
  move_outside(progress, config, 2000);
  assert(progress.phase() == gpsmeter::RacePhase::Running);
  assert(progress.completed_laps() == 0);
  assert(progress.current_lap() == 1);
}

void test_power_on_starts_fresh_race_session() {
  gpsmeter::RaceConfig config{};
  config.format = gpsmeter::RaceFormat::Laps;
  config.target_laps = 3;
  config.minimum_lap_ms = 10000;
  config.gate = {35.0, 139.0};
  config.gate_valid = true;

  gpsmeter::RaceProgress progress;
  progress.begin_new(config, false, 0);
  move_inside(progress, config, 1000);
  move_outside(progress, config, 2000);
  move_inside(progress, config, 12000);
  assert(progress.completed_laps() == 1);

  progress.begin_new(config, false, 20000);
  assert(progress.phase() == gpsmeter::RacePhase::WaitingForBaseline);
  assert(progress.completed_laps() == 0);
  assert(progress.current_stint() == 0);
  assert(progress.stint_history_count() == 0);
  assert(progress.race_elapsed_ms(20000) == 0);
}

} // namespace

int main() {
  test_nmea_rmc();
  test_nmea_rmc_estimation_mode_is_exposed();
  test_nmea_rejects_bad_checksum();
  test_nmea_sentence_detector();
  test_ubx_stream_and_nav_pvt();
  test_ubx_nav_pvt_rejects_invalid_fix();
  test_gga_does_not_refresh_speed();
  test_exact_warning_threshold();
  test_direct_alert_thresholds();
  test_mode_transitions();
  test_stale_data_resets_timer();
  test_continuous_pit_ng_tone_condition();
  test_course_file_parser();
  test_race_baseline_exclusion_and_laps();
  test_timed_remaining_and_format();
  test_gate_direction();
  test_starting_inside_gate_does_not_count_residence();
  test_power_on_starts_fresh_race_session();
  std::cout << "All Kuruma Race Assistant tests passed\n";
  return 0;
}
