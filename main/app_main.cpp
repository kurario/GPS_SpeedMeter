#include "course_storage.hpp"
#include "gps_sample.hpp"
#include "race_progress.hpp"
#include "settings.hpp"
#include "speed_policy.hpp"
#include "ubx_parser.hpp"

#include "M5Unified.h"
#include "driver/uart.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_task_wdt.h"
#include "esp_timer.h"
#include "nvs_flash.h"

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>

extern const uint8_t speed_font_vlw_start[]
    asm("_binary_speed_font_vlw_start");
extern const uint8_t startup_png_start[]
    asm("_binary_startup_png_start");
extern const uint8_t startup_png_end[]
    asm("_binary_startup_png_end");

namespace {

using gpsmeter::AppSettings;
using gpsmeter::DriveMode;
using gpsmeter::GpsSample;
using gpsmeter::ModeRuntime;
using gpsmeter::RaceFormat;
using gpsmeter::RacePhase;
using gpsmeter::RaceProgress;
using gpsmeter::SpeedAlert;
using gpsmeter::UbxFrame;
using gpsmeter::UbxParser;

constexpr const char *kTag = "kuruma_race_assistant";
constexpr uart_port_t kGpsUart = UART_NUM_2;
constexpr gpio_num_t kGpsRxPin = GPIO_NUM_13;
constexpr gpio_num_t kGpsTxPin = GPIO_NUM_15;
constexpr std::array<gpio_num_t, 4> kGpsRxPinCandidates{
    GPIO_NUM_13,
    GPIO_NUM_16,
    GPIO_NUM_34,
    GPIO_NUM_35,
};
constexpr int kGpsDefaultBaud = 38400;
constexpr int kGpsRuntimeBaud = 921600;
constexpr std::array<int, 8> kGpsBaudCandidates{
    921600, 38400, 460800, 230400, 115200, 57600, 19200, 9600,
};
constexpr uint32_t kGpsBaudProbeMs = 600;
constexpr uint32_t kGpsBaudRetryMs = 2000;
constexpr uint32_t kGpsStaleMs = 1000;
constexpr uint32_t kGpsDiagnosticPeriodMs = 5000;
constexpr uint32_t kUbxResponseTimeoutMs = 1200;
constexpr uint16_t kGpsAcquisitionPeriodMs = 100;
constexpr uint16_t kGpsRuntimePeriodMs = 40;
constexpr uint32_t kGpsStableFixBefore25HzMs = 3000;
constexpr uint8_t kGpsAutomotiveDynamicModel = 4;
constexpr uint8_t kGpsAutomaticFixMode = 3;
constexpr uint8_t kGpsStaticHoldThresholdCmS = 50;
constexpr uint32_t kStartupScreenDurationMs = 2000;
constexpr TickType_t kUiPeriod = pdMS_TO_TICKS(50);
constexpr uint32_t kCourseRegistrationMs = 10000;

QueueHandle_t g_gps_queue = nullptr;

uint64_t monotonic_ms() {
  return static_cast<uint64_t>(esp_timer_get_time()) / 1000ULL;
}

void play_tone(uint16_t frequency_hz, uint32_t duration_ms,
               uint8_t volume) {
  if (!M5.Speaker.isEnabled()) {
    return;
  }
  M5.Speaker.setVolume(volume);
  (void)M5.Speaker.tone(frequency_hz, duration_ms);
}

bool start_continuous_tone(uint16_t frequency_hz, uint8_t volume) {
  if (!M5.Speaker.isEnabled()) {
    return false;
  }
  M5.Speaker.setVolume(volume);
  return M5.Speaker.tone(frequency_hz);
}

void stop_continuous_tone() {
  if (M5.Speaker.isEnabled()) {
    M5.Speaker.stop();
  }
}

uint32_t read_le_u32(const uint8_t *bytes) {
  return static_cast<uint32_t>(bytes[0]) |
         (static_cast<uint32_t>(bytes[1]) << 8U) |
         (static_cast<uint32_t>(bytes[2]) << 16U) |
         (static_cast<uint32_t>(bytes[3]) << 24U);
}

struct UbxConfigItem {
  uint32_t key;
  uint32_t value;
  uint8_t value_size;
};

constexpr uint32_t kCfgUart1Baudrate = 0x40520001;
constexpr uint32_t kCfgUart1InUbx = 0x10730001;
constexpr uint32_t kCfgUart1InNmea = 0x10730002;
constexpr uint32_t kCfgUart1OutUbx = 0x10740001;
constexpr uint32_t kCfgUart1OutNmea = 0x10740002;
constexpr uint32_t kCfgRateMeas = 0x30210001;
constexpr uint32_t kCfgRateNav = 0x30210002;
constexpr uint32_t kCfgNavPvtUart1 = 0x20910007;
constexpr uint32_t kCfgFixMode = 0x20110011;
constexpr uint32_t kCfgDynamicModel = 0x20110021;
constexpr uint32_t kCfgStaticHoldSpeed = 0x20250038;
constexpr uint32_t kCfgStaticHoldDistance = 0x3025003B;

void append_le(std::array<uint8_t, 128> &payload, size_t &offset,
               uint32_t value, uint8_t size) {
  for (uint8_t byte = 0; byte < size; ++byte) {
    payload[offset++] = static_cast<uint8_t>(value >> (byte * 8U));
  }
}

bool send_ubx(uint8_t message_class, uint8_t message_id,
              const uint8_t *payload, size_t payload_length) {
  std::array<uint8_t, 136> packet{};
  const size_t packet_length = payload_length + 8U;
  if (payload_length > 128 || packet_length > packet.size()) {
    ESP_LOGE(kTag, "UBX packet too large: %u",
             static_cast<unsigned>(payload_length));
    return false;
  }

  packet[0] = 0xB5;
  packet[1] = 0x62;
  packet[2] = message_class;
  packet[3] = message_id;
  packet[4] = static_cast<uint8_t>(payload_length);
  packet[5] = static_cast<uint8_t>(payload_length >> 8U);
  if (payload_length > 0) {
    std::memcpy(&packet[6], payload, payload_length);
  }

  uint8_t checksum_a = 0;
  uint8_t checksum_b = 0;
  for (size_t index = 2; index < 6U + payload_length; ++index) {
    checksum_a = static_cast<uint8_t>(checksum_a + packet[index]);
    checksum_b = static_cast<uint8_t>(checksum_b + checksum_a);
  }
  packet[6U + payload_length] = checksum_a;
  packet[7U + payload_length] = checksum_b;

  const int written =
      uart_write_bytes(kGpsUart, packet.data(), packet_length);
  if (written != static_cast<int>(packet_length)) {
    ESP_LOGW(kTag, "UBX write incomplete: %d/%u", written,
             static_cast<unsigned>(packet_length));
    return false;
  }
  return uart_wait_tx_done(kGpsUart, pdMS_TO_TICKS(100)) == ESP_OK;
}

bool read_ubx_response(uint8_t expected_class, uint8_t expected_id,
                       UbxFrame &response, uint32_t timeout_ms) {
  UbxParser parser;
  UbxFrame frame{};
  std::array<uint8_t, 256> bytes{};
  const uint64_t deadline_ms = monotonic_ms() + timeout_ms;
  while (monotonic_ms() < deadline_ms) {
    const int count = uart_read_bytes(kGpsUart, bytes.data(), bytes.size(),
                                      pdMS_TO_TICKS(30));
    for (int index = 0; index < count; ++index) {
      if (parser.feed(bytes[static_cast<size_t>(index)], frame) &&
          frame.message_class == expected_class &&
          frame.message_id == expected_id) {
        response = frame;
        return true;
      }
    }
    (void)esp_task_wdt_reset();
  }
  return false;
}

bool wait_for_ubx_ack(uint8_t configured_class, uint8_t configured_id) {
  UbxFrame response{};
  if (!read_ubx_response(0x05, 0x01, response, kUbxResponseTimeoutMs)) {
    ESP_LOGW(kTag, "UBX ACK timeout: class=0x%02X id=0x%02X",
             configured_class, configured_id);
    return false;
  }
  const bool matches =
      response.payload_length >= 2 &&
      response.payload[0] == configured_class &&
      response.payload[1] == configured_id;
  if (!matches) {
    ESP_LOGW(kTag,
             "UBX ACK target mismatch: expected=0x%02X/0x%02X "
             "actual=0x%02X/0x%02X",
             configured_class, configured_id,
             response.payload_length >= 1 ? response.payload[0] : 0,
             response.payload_length >= 2 ? response.payload[1] : 0);
  }
  return matches;
}

bool send_valset(const UbxConfigItem *items, size_t item_count,
                 bool require_ack = true) {
  std::array<uint8_t, 128> payload{};
  size_t offset = 4;
  payload[0] = 0;    // Version 0.
  payload[1] = 0x01; // Apply to volatile RAM only.
  payload[2] = 0;    // No transaction.
  payload[3] = 0;
  for (size_t index = 0; index < item_count; ++index) {
    const auto &item = items[index];
    if ((item.value_size != 1 && item.value_size != 2 &&
         item.value_size != 4) ||
        offset + 4U + item.value_size > payload.size()) {
      ESP_LOGE(kTag, "Invalid UBX configuration item: key=0x%08lX",
               static_cast<unsigned long>(item.key));
      return false;
    }
    append_le(payload, offset, item.key, 4);
    append_le(payload, offset, item.value, item.value_size);
  }
  if (!send_ubx(0x06, 0x8A, payload.data(), offset)) {
    return false;
  }
  return !require_ack || wait_for_ubx_ack(0x06, 0x8A);
}

bool read_config_value(uint32_t key, uint8_t value_size, uint32_t &value) {
  std::array<uint8_t, 8> request{};
  request[0] = 0; // Version 0 request.
  request[1] = 0; // Read current RAM configuration.
  request[4] = static_cast<uint8_t>(key);
  request[5] = static_cast<uint8_t>(key >> 8U);
  request[6] = static_cast<uint8_t>(key >> 16U);
  request[7] = static_cast<uint8_t>(key >> 24U);
  if (!send_ubx(0x06, 0x8B, request.data(), request.size())) {
    return false;
  }

  UbxFrame response{};
  if (!read_ubx_response(0x06, 0x8B, response, kUbxResponseTimeoutMs) ||
      response.payload_length < 8U + value_size ||
      read_le_u32(&response.payload[4]) != key) {
    return false;
  }
  value = 0;
  for (uint8_t byte = 0; byte < value_size; ++byte) {
    value |= static_cast<uint32_t>(response.payload[8U + byte])
             << (byte * 8U);
  }
  return true;
}

bool verify_config_item(const UbxConfigItem &item, const char *name) {
  uint32_t actual = 0;
  const bool read = read_config_value(item.key, item.value_size, actual);
  const bool matches = read && actual == item.value;
  ESP_LOGI(kTag, "UBX config %-20s expected=%lu actual=%lu verify=%s",
           name, static_cast<unsigned long>(item.value),
           static_cast<unsigned long>(actual),
           matches ? "ok" : "failed");
  return matches;
}

bool probe_ubx_baud(int baud, uint32_t &received_bytes, bool &saw_nmea) {
  received_bytes = 0;
  saw_nmea = false;
  ESP_ERROR_CHECK(uart_set_baudrate(kGpsUart, baud));
  ESP_ERROR_CHECK(uart_flush_input(kGpsUart));
  if (!send_ubx(0x0A, 0x04, nullptr, 0)) {
    return false;
  }

  UbxParser parser;
  UbxFrame frame{};
  std::array<uint8_t, 256> bytes{};
  const uint64_t deadline_ms = monotonic_ms() + kGpsBaudProbeMs;
  while (monotonic_ms() < deadline_ms) {
    const int count = uart_read_bytes(kGpsUart, bytes.data(), bytes.size(),
                                      pdMS_TO_TICKS(30));
    if (count > 0) {
      received_bytes += static_cast<uint32_t>(count);
    }
    for (int index = 0; index < count; ++index) {
      const uint8_t byte = bytes[static_cast<size_t>(index)];
      saw_nmea = saw_nmea || byte == '$';
      if (parser.feed(byte, frame) && frame.message_class == 0x0A &&
          frame.message_id == 0x04) {
        return true;
      }
    }
    (void)esp_task_wdt_reset();
  }
  return false;
}

int detect_gps_baud() {
  for (;;) {
    for (const gpio_num_t rx_pin : kGpsRxPinCandidates) {
      ESP_ERROR_CHECK(uart_set_pin(kGpsUart, kGpsTxPin, rx_pin,
                                   UART_PIN_NO_CHANGE,
                                   UART_PIN_NO_CHANGE));
      for (const int baud : kGpsBaudCandidates) {
        uint32_t received_bytes = 0;
        bool saw_nmea = false;
        const bool detected =
            probe_ubx_baud(baud, received_bytes, saw_nmea);
        ESP_LOGI(
            kTag,
            "GNSS UBX probe: RX=G%d baud=%d result=%s raw=%lu nmea=%s",
            static_cast<int>(rx_pin), baud,
            detected ? "detected" : "none",
            static_cast<unsigned long>(received_bytes),
            saw_nmea ? "yes" : "no");
        if (detected) {
          ESP_ERROR_CHECK(uart_flush_input(kGpsUart));
          ESP_LOGI(kTag, "NEO-M9N UART route detected: RX=G%d baud=%d",
                   static_cast<int>(rx_pin), baud);
          return baud;
        }
        (void)esp_task_wdt_reset();
      }
    }
    ESP_LOGW(kTag, "NEO-M9N UBX stream not detected; retrying");
    vTaskDelay(pdMS_TO_TICKS(kGpsBaudRetryMs));
  }
}

bool configure_runtime_baud(int detected_baud) {
  const UbxConfigItem baud_item{
      kCfgUart1Baudrate, static_cast<uint32_t>(kGpsRuntimeBaud), 4};
  if (detected_baud != kGpsRuntimeBaud) {
    if (!send_valset(&baud_item, 1, false)) {
      return false;
    }
    vTaskDelay(pdMS_TO_TICKS(50));
    if (uart_set_baudrate(kGpsUart, kGpsRuntimeBaud) != ESP_OK) {
      return false;
    }
    ESP_ERROR_CHECK(uart_flush_input(kGpsUart));
  }
  if (!send_valset(&baud_item, 1)) {
    return false;
  }
  return verify_config_item(baud_item, "UART1 baud");
}

bool configure_ubx_for_vehicle_once(int detected_baud) {
  if (!configure_runtime_baud(detected_baud)) {
    return false;
  }

  const std::array<UbxConfigItem, 11> profile{{
      {kCfgUart1InUbx, 1, 1},
      {kCfgUart1InNmea, 0, 1},
      {kCfgUart1OutUbx, 1, 1},
      {kCfgUart1OutNmea, 0, 1},
      {kCfgRateMeas, kGpsAcquisitionPeriodMs, 2},
      {kCfgRateNav, 1, 2},
      {kCfgNavPvtUart1, 1, 1},
      {kCfgFixMode, kGpsAutomaticFixMode, 1},
      {kCfgDynamicModel, kGpsAutomotiveDynamicModel, 1},
      {kCfgStaticHoldSpeed, kGpsStaticHoldThresholdCmS, 1},
      {kCfgStaticHoldDistance, 0, 2},
  }};
  constexpr std::array<const char *, profile.size()> names{{
      "UART1 input UBX",
      "UART1 input NMEA",
      "UART1 output UBX",
      "UART1 output NMEA",
      "measurement period",
      "navigation ratio",
      "NAV-PVT UART1",
      "fix mode",
      "dynamic model",
      "static hold speed",
      "static hold distance",
  }};
  if (!send_valset(profile.data(), profile.size())) {
    return false;
  }

  bool verified = true;
  for (size_t index = 0; index < profile.size(); ++index) {
    verified &= verify_config_item(profile[index], names[index]);
  }
  ESP_LOGI(kTag,
           "NEO-M9N setup: automotive, auto 2D/3D, static hold %.2f m/s, "
           "NAV-PVT 10 Hz, UBX-only, %d bps, RAM-only; verify=%s",
           static_cast<double>(kGpsStaticHoldThresholdCmS * 0.01F),
           kGpsRuntimeBaud, verified ? "ok" : "failed");
  return verified;
}

void configure_ubx_for_vehicle(int detected_baud) {
  uint32_t attempt = 0;
  while (!configure_ubx_for_vehicle_once(
      attempt == 0 ? detected_baud : kGpsRuntimeBaud)) {
    ++attempt;
    ESP_LOGE(kTag,
             "NEO-M9N configuration failed (attempt %lu); retrying",
             static_cast<unsigned long>(attempt));
    ESP_ERROR_CHECK(uart_flush_input(kGpsUart));
    vTaskDelay(pdMS_TO_TICKS(1000));
    (void)esp_task_wdt_reset();
  }
}

bool configure_navigation_rate(uint16_t period_ms) {
  const UbxConfigItem rate_item{kCfgRateMeas, period_ms, 2};
  if (!send_valset(&rate_item, 1)) {
    return false;
  }
  return verify_config_item(rate_item, "measurement period");
}

void gps_task(void *) {
  ESP_ERROR_CHECK(esp_task_wdt_add(nullptr));
  const uart_config_t config{
      .baud_rate = kGpsDefaultBaud,
      .data_bits = UART_DATA_8_BITS,
      .parity = UART_PARITY_DISABLE,
      .stop_bits = UART_STOP_BITS_1,
      .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
      .rx_flow_ctrl_thresh = 0,
      .source_clk = UART_SCLK_DEFAULT,
      .flags = {},
  };
  ESP_ERROR_CHECK(uart_param_config(kGpsUart, &config));
  ESP_ERROR_CHECK(uart_set_pin(kGpsUart, kGpsTxPin, kGpsRxPin,
                               UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
  ESP_ERROR_CHECK(uart_driver_install(kGpsUart, 8192, 1024, 0, nullptr, 0));
  ESP_LOGI(kTag,
           "GNSS UART ready: port=%d RX=G%d(module TXD) "
           "TX=G%d(module RXD); probing UBX baud",
           static_cast<int>(kGpsUart), static_cast<int>(kGpsRxPin),
           static_cast<int>(kGpsTxPin));
  const int detected_baud = detect_gps_baud();
  ESP_LOGI(kTag, "NEO-M9N detected: baud=%d", detected_baud);
  configure_ubx_for_vehicle(detected_baud);

  UbxParser parser;
  UbxFrame frame{};
  GpsSample sample{};
  std::array<uint8_t, 512> bytes{};
  uint32_t received_byte_count = 0;
  uint32_t parsed_sample_count = 0;
  uint32_t speed_change_count = 0;
  float previous_raw_speed_kmh = NAN;
  float minimum_speed_kmh = INFINITY;
  float maximum_speed_kmh = -INFINITY;
  uint64_t stable_fix_since_ms = 0;
  uint64_t next_rate_retry_ms = 0;
  uint64_t next_diagnostic_ms = monotonic_ms() + kGpsDiagnosticPeriodMs;
  bool high_rate_active = false;

  for (;;) {
    const int count =
        uart_read_bytes(kGpsUart, bytes.data(), bytes.size(), pdMS_TO_TICKS(100));
    const uint64_t received_ms = monotonic_ms();
    if (count > 0) {
      received_byte_count += static_cast<uint32_t>(count);
    }
    for (int index = 0; index < count; ++index) {
      if (!parser.feed(bytes[static_cast<size_t>(index)], frame) ||
          !gpsmeter::parse_ubx_nav_pvt(frame, received_ms, sample)) {
        continue;
      }

      ++parsed_sample_count;
      if (!std::isfinite(previous_raw_speed_kmh) ||
          sample.speed_kmh != previous_raw_speed_kmh) {
        ++speed_change_count;
        previous_raw_speed_kmh = sample.speed_kmh;
      }
      minimum_speed_kmh = std::min(minimum_speed_kmh, sample.speed_kmh);
      maximum_speed_kmh = std::max(maximum_speed_kmh, sample.speed_kmh);
      (void)xQueueOverwrite(g_gps_queue, &sample);

      if (!sample.valid) {
        stable_fix_since_ms = 0;
      } else if (stable_fix_since_ms == 0) {
        stable_fix_since_ms = received_ms;
      }
      if (!high_rate_active && stable_fix_since_ms > 0 &&
          received_ms - stable_fix_since_ms >= kGpsStableFixBefore25HzMs &&
          received_ms >= next_rate_retry_ms) {
        const bool configured =
            configure_navigation_rate(kGpsRuntimePeriodMs);
        parser.reset();
        if (configured) {
          high_rate_active = true;
          ESP_LOGI(kTag,
                   "NEO-M9N stable fix; NAV-PVT promoted to 25 Hz "
                   "(40 ms, verified)");
        } else {
          next_rate_retry_ms = received_ms + 1000;
          ESP_LOGW(kTag, "NEO-M9N 25 Hz promotion failed; will retry");
        }
      }
    }

    if (received_ms >= next_diagnostic_ms) {
      ESP_LOGI(
          kTag,
          "GNSS UBX RX: rate=%s bytes=%u NAV-PVT=%u changes=%u/5s "
          "speed=%.2f sAcc=%.2f range=%.2f..%.2f fix=%u fixOk=%d "
          "sat=%u pDOP=%.2f hAcc=%.1f",
          high_rate_active ? "25Hz" : "10Hz",
          static_cast<unsigned>(received_byte_count),
          static_cast<unsigned>(parsed_sample_count),
          static_cast<unsigned>(speed_change_count),
          static_cast<double>(sample.speed_kmh),
          static_cast<double>(sample.speed_accuracy_kmh),
          static_cast<double>(std::isfinite(minimum_speed_kmh)
                                  ? minimum_speed_kmh
                                  : 0.0F),
          static_cast<double>(std::isfinite(maximum_speed_kmh)
                                  ? maximum_speed_kmh
                                  : 0.0F),
          static_cast<unsigned>(sample.fix_type),
          sample.gnss_fix_ok ? 1 : 0,
          static_cast<unsigned>(sample.satellites),
          static_cast<double>(sample.position_dop),
          static_cast<double>(sample.horizontal_accuracy_m));
      received_byte_count = 0;
      parsed_sample_count = 0;
      speed_change_count = 0;
      previous_raw_speed_kmh = NAN;
      minimum_speed_kmh = INFINITY;
      maximum_speed_kmh = -INFINITY;
      next_diagnostic_ms = received_ms + kGpsDiagnosticPeriodMs;
    }
    (void)esp_task_wdt_reset();
  }
}

struct UiRuntime {
  AppSettings settings{};
  ModeRuntime mode{};
  GpsSample sample{};
  RaceProgress race{};
  gpsmeter::CourseStorageResult course_storage{};
  SpeedAlert alert = SpeedAlert::None;
  SpeedAlert tone_level = SpeedAlert::None;
  uint64_t last_loop_ms = 0;
  uint64_t last_tone_ms = 0;
  uint64_t last_valid_utc_received_ms = 0;
  uint64_t notice_until_ms = 0;
  uint64_t registration_started_ms = 0;
  double registration_latitude_sum = 0.0;
  double registration_longitude_sum = 0.0;
  double registration_latitude_m2 = 0.0;
  double registration_longitude_m2 = 0.0;
  uint32_t registration_samples = 0;
  gpsmeter::UtcDateTime last_valid_utc{};
  size_t selected_course_index = 0;
  bool have_sample = false;
  bool ever_had_fix = false;
  bool previous_fresh = false;
  bool pit_ng_tone_active = false;
  bool c_hold_handled = false;
  bool registering_course = false;
  bool course_selection_initialized = false;
  bool setting_screen = false;
  uint8_t setting_index = 0;
  bool force_redraw = true;
  char notice[40]{};
};

struct SettingText {
  const char *name;
  const char *description_line1;
  const char *description_line2;
};

constexpr std::array<SettingText, 23> kSettingTexts{{
    {"ピットの制限速度", "この速度になると", "強い警告を出します"},
    {"注意を始める速度", "この速度になると", "黄色の注意を始めます"},
    {"警告を始める速度", "この速度になると", "赤色の強い警告を始めます"},
    {"警告を解除する幅", "速度が境目で上下したときの", "警告のちらつきを防ぎます"},
    {"レースへの切替速度", "この速度以上が続くと", "レースモードへ切り替えます"},
    {"レースへの切替時間", "切替速度を何秒続けたら", "レースモードにするか決めます"},
    {"ピットへの復帰速度", "この速度以下が続くと", "ピットモードへ戻ります"},
    {"ピットへの復帰時間", "復帰速度を何秒続けたら", "ピットモードに戻すか決めます"},
    {"警告音の大きさ", "ヘルメット着用中でも", "聞こえる音量に調整します"},
    {"画面の明るさ", "昼夜や取付場所に合わせて", "見やすい明るさに調整します"},
    {"注意音の高さ", "注意を知らせる音の高さを", "聞き分けやすく調整します"},
    {"警告音の高さ", "速度超過を知らせる音を", "聞き分けやすく調整します"},
    {"ピット警告音", "ピットモードの警告音を", "鳴らすか選びます"},
    {"警告時の画面点滅", "速度警告中に画面を", "点滅させるか選びます"},
    {"レース形式", "時間制または周回数制を", "レース前に選びます"},
    {"レース終了 時", "時間制レースの終了時刻を", "開催地の時刻で設定します"},
    {"レース終了 分", "終了時刻の分を", "1分刻みで設定します"},
    {"開催地の時差", "日本はUTC+9です", "開催地に合わせます"},
    {"規定周回数", "周回数制レースの", "ゴール周回数を設定します"},
    {"開始前に除外する周回", "フォーメーション等を", "0～5回まで除外します"},
    {"周回ポイントを選ぶ", "microSDのコース一覧から", "周回ポイントを選びます"},
    {"この場所を登録", "停止してCを押すと10秒間", "平均してmicroSDへ保存します"},
    {"設定を保存して終了", "Bボタンを長押しすると", "設定を保存して戻ります"},
}};

constexpr uint16_t rgb565(uint8_t red, uint8_t green, uint8_t blue) {
  return (static_cast<uint16_t>(red & 0xF8U) << 8U) |
         (static_cast<uint16_t>(green & 0xFCU) << 3U) |
         static_cast<uint16_t>(blue >> 3U);
}

namespace ui_color {
constexpr uint16_t kBackground = rgb565(0, 0, 0);
constexpr uint16_t kSurface = rgb565(17, 23, 34);
constexpr uint16_t kSurfaceRaised = rgb565(25, 33, 47);
constexpr uint16_t kBorder = rgb565(62, 74, 94);
constexpr uint16_t kText = rgb565(238, 244, 252);
constexpr uint16_t kMuted = rgb565(132, 148, 170);
constexpr uint16_t kPit = rgb565(35, 196, 132);
constexpr uint16_t kRace = rgb565(45, 133, 230);
constexpr uint16_t kCaution = rgb565(255, 188, 61);
constexpr uint16_t kWarning = rgb565(246, 55, 73);
constexpr uint16_t kWarningDim = rgb565(122, 24, 38);
constexpr uint16_t kInvalid = rgb565(148, 157, 173);
} // namespace ui_color

struct UiVisualState {
  uint16_t accent = ui_color::kInvalid;
  const char *status = "GPS WAIT";
};

UiVisualState visual_state_for(const UiRuntime &runtime, bool fresh,
                               uint64_t now_ms) {
  if (!fresh) {
    return UiVisualState{
        .accent = ui_color::kInvalid,
        .status = runtime.ever_had_fix ? "GPS LOST" : "GPS WAIT",
    };
  }
  if (runtime.mode.mode == DriveMode::Race) {
    return UiVisualState{
        .accent = ui_color::kRace,
        .status = nullptr,
    };
  }
  if (runtime.alert == SpeedAlert::Warning) {
    const bool dimmed = runtime.settings.pit_blink_enabled &&
                        ((now_ms / 125U) % 2U) != 0U;
    return UiVisualState{
        .accent = dimmed ? ui_color::kWarningDim : ui_color::kWarning,
        .status = "SLOW",
    };
  }
  if (runtime.alert == SpeedAlert::Caution) {
    const bool dimmed = runtime.settings.pit_blink_enabled &&
                        ((now_ms / 250U) % 2U) != 0U;
    return UiVisualState{
        .accent = dimmed ? rgb565(126, 85, 20) : ui_color::kCaution,
        .status = "CAUTION",
    };
  }
  return UiVisualState{
      .accent = ui_color::kPit,
      .status = nullptr,
  };
}

void set_ui_font(lgfx::v1::LGFXBase &display, const lgfx::IFont *font,
                 uint8_t size = 1) {
  display.setFont(font);
  display.setTextSize(size);
}

void draw_startup_screen(lgfx::v1::LGFXBase &display) {
  constexpr uint16_t background = rgb565(5, 8, 13);
  const size_t png_size =
      static_cast<size_t>(startup_png_end - startup_png_start);
  if (display.drawPng(startup_png_start, png_size, 0, 0)) {
    return;
  }

  ESP_LOGW(kTag, "Startup PNG decode failed; using text fallback");
  display.fillScreen(background);
  display.setTextDatum(middle_center);
  display.setTextColor(ui_color::kRace, background);
  set_ui_font(display, &fonts::FreeSansBold18pt7b);
  display.drawString("KURUMA", 160, 88);
  display.setTextColor(ui_color::kText, background);
  display.drawString("Speed Meter", 160, 128);
  display.setTextColor(ui_color::kMuted, background);
  set_ui_font(display, &fonts::FreeSans9pt7b);
  display.drawString("by kurumario", 160, 170);
  display.fillRect(26, 229, 267, 2, ui_color::kSurfaceRaised);
}

void draw_startup_progress(lgfx::v1::LGFXBase &display,
                           uint32_t elapsed_ms) {
  constexpr int32_t x = 26;
  constexpr int32_t width = 267;
  const int32_t completed = static_cast<int32_t>(
      std::min<uint64_t>(
          static_cast<uint64_t>(elapsed_ms) * width /
              kStartupScreenDurationMs,
          width));
  // Repaint the complete rail on every frame. The previous implementation
  // left each moving white endpoint behind and produced a broken-looking bar.
  display.fillRect(x, 229, width, 2, ui_color::kSurfaceRaised);
  if (completed <= 0) {
    return;
  }
  display.fillRect(x, 229, completed, 2, ui_color::kRace);
}

void adjust_setting(AppSettings &settings, uint8_t index, int direction) {
  const auto add_u16 = [direction](uint16_t &value, int step) {
    const int next = static_cast<int>(value) + direction * step;
    value = static_cast<uint16_t>(std::max(0, next));
  };
  switch (index) {
  case 0:
    add_u16(settings.pit_limit_kmh, 1);
    break;
  case 1:
    add_u16(settings.caution_speed_kmh, 1);
    break;
  case 2:
    add_u16(settings.warning_speed_kmh, 1);
    break;
  case 3:
    add_u16(settings.alert_clear_margin_kmh, 1);
    break;
  case 4:
    add_u16(settings.race_enter_speed_kmh, 1);
    break;
  case 5:
    add_u16(settings.race_enter_hold_s, 1);
    break;
  case 6:
    add_u16(settings.pit_return_speed_kmh, 1);
    break;
  case 7:
    add_u16(settings.pit_return_hold_s, 1);
    break;
  case 8: {
    const int next = static_cast<int>(settings.tone_volume) + direction * 8;
    settings.tone_volume =
        static_cast<uint8_t>(std::clamp(next, 0, 255));
    break;
  }
  case 9: {
    const int next =
        static_cast<int>(settings.display_brightness) + direction * 8;
    settings.display_brightness =
        static_cast<uint8_t>(std::clamp(next, 8, 255));
    M5.Display.setBrightness(settings.display_brightness);
    break;
  }
  case 10:
    add_u16(settings.caution_tone_hz, 100);
    break;
  case 11:
    add_u16(settings.warning_tone_hz, 100);
    break;
  case 12:
    settings.pit_tone_enabled = !settings.pit_tone_enabled;
    break;
  case 13:
    settings.pit_blink_enabled = !settings.pit_blink_enabled;
    break;
  case 14:
    settings.race_format = settings.race_format == RaceFormat::Timed
                               ? RaceFormat::Laps
                               : RaceFormat::Timed;
    break;
  case 15:
    settings.race_finish_hour = static_cast<uint8_t>(
        (static_cast<int>(settings.race_finish_hour) + direction + 24) % 24);
    break;
  case 16:
    settings.race_finish_minute = static_cast<uint8_t>(
        (static_cast<int>(settings.race_finish_minute) + direction + 60) %
        60);
    break;
  case 17:
    settings.timezone_hours = static_cast<int8_t>(
        static_cast<int>(settings.timezone_hours) + direction);
    break;
  case 18:
    add_u16(settings.target_laps, 1);
    break;
  case 19: {
    const int next =
        static_cast<int>(settings.excluded_passes) + direction;
    settings.excluded_passes =
        static_cast<uint8_t>(std::clamp(next, 0, 5));
    break;
  }
  default:
    break;
  }
  gpsmeter::normalize_settings(settings);
}

void format_setting_value(const UiRuntime &runtime, uint8_t index,
                          char *buffer, size_t size) {
  const AppSettings &settings = runtime.settings;
  switch (index) {
  case 0:
    std::snprintf(buffer, size, "%u km/h", settings.pit_limit_kmh);
    break;
  case 1:
    std::snprintf(buffer, size, "%u km/h", settings.caution_speed_kmh);
    break;
  case 2:
    std::snprintf(buffer, size, "%u km/h", settings.warning_speed_kmh);
    break;
  case 3:
    std::snprintf(buffer, size, "%u km/h", settings.alert_clear_margin_kmh);
    break;
  case 4:
    std::snprintf(buffer, size, "%u km/h", settings.race_enter_speed_kmh);
    break;
  case 5:
    std::snprintf(buffer, size, "%u 秒", settings.race_enter_hold_s);
    break;
  case 6:
    std::snprintf(buffer, size, "%u km/h", settings.pit_return_speed_kmh);
    break;
  case 7:
    std::snprintf(buffer, size, "%u 秒", settings.pit_return_hold_s);
    break;
  case 8:
    std::snprintf(buffer, size, "%u%%",
                  (static_cast<unsigned>(settings.tone_volume) * 100U + 127U) /
                      255U);
    break;
  case 9:
    std::snprintf(
        buffer, size, "%u%%",
        (static_cast<unsigned>(settings.display_brightness) * 100U + 127U) /
            255U);
    break;
  case 10:
    std::snprintf(buffer, size, "%u Hz", settings.caution_tone_hz);
    break;
  case 11:
    std::snprintf(buffer, size, "%u Hz", settings.warning_tone_hz);
    break;
  case 12:
    std::snprintf(buffer, size, "%s",
                  settings.pit_tone_enabled ? "オン" : "オフ");
    break;
  case 13:
    std::snprintf(buffer, size, "%s",
                  settings.pit_blink_enabled ? "オン" : "オフ");
    break;
  case 14:
    std::snprintf(buffer, size, "%s",
                  settings.race_format == RaceFormat::Timed ? "時間制"
                                                            : "周回数制");
    break;
  case 15:
    std::snprintf(buffer, size, "%02u 時", settings.race_finish_hour);
    break;
  case 16:
    std::snprintf(buffer, size, "%02u 分", settings.race_finish_minute);
    break;
  case 17:
    std::snprintf(buffer, size, "UTC%+d", settings.timezone_hours);
    break;
  case 18:
    std::snprintf(buffer, size, "%u 周", settings.target_laps);
    break;
  case 19:
    std::snprintf(buffer, size, "%u 回", settings.excluded_passes);
    break;
  case 20:
    if (settings.course_valid) {
      std::snprintf(buffer, size, "%s", settings.course_name.data());
    } else {
      std::snprintf(buffer, size, "%s",
                    gpsmeter::course_storage_status_japanese(
                        runtime.course_storage.status));
    }
    break;
  case 21:
    if (runtime.registering_course) {
      const uint64_t elapsed =
          monotonic_ms() - runtime.registration_started_ms;
      std::snprintf(buffer, size, "計測中 %llu秒",
                    static_cast<unsigned long long>(
                        std::min<uint64_t>(elapsed / 1000U, 10)));
    } else {
      std::snprintf(buffer, size, "Cで登録開始");
    }
    break;
  default:
    std::snprintf(buffer, size, "Bを長押し");
    break;
  }
}

void draw_settings(lgfx::v1::LGFXBase &display, const UiRuntime &runtime) {
  display.fillScreen(ui_color::kBackground);
  display.fillRect(0, 0, 320, 32, ui_color::kSurface);
  display.fillRect(0, 30, 320, 2, ui_color::kRace);

  display.setTextDatum(middle_left);
  display.setTextColor(ui_color::kText, ui_color::kSurface);
  set_ui_font(display, &fonts::lgfxJapanGothicP_16);
  display.drawString("設定", 12, 16);

  char index[12]{};
  std::snprintf(index, sizeof(index), "%02u / %02u",
                static_cast<unsigned>(runtime.setting_index + 1U),
                static_cast<unsigned>(kSettingTexts.size()));
  display.setTextDatum(middle_right);
  display.setTextColor(ui_color::kMuted, ui_color::kSurface);
  set_ui_font(display, &fonts::FreeMono9pt7b);
  display.drawString(index, 308, 16);

  display.setTextDatum(middle_center);
  display.setTextColor(ui_color::kText, ui_color::kBackground);
  set_ui_font(display, &fonts::lgfxJapanGothicP_16);
  display.drawString(kSettingTexts[runtime.setting_index].name, 160, 48);

  display.fillRoundRect(12, 62, 296, 52, 8, ui_color::kSurfaceRaised);
  display.drawRoundRect(12, 62, 296, 52, 8, ui_color::kBorder);
  char value[32]{};
  format_setting_value(runtime, runtime.setting_index, value, sizeof(value));
  display.setTextDatum(middle_center);
  display.setTextColor(ui_color::kText, ui_color::kSurfaceRaised);
  const bool japanese_value =
      runtime.setting_index == 5 || runtime.setting_index == 7 ||
      runtime.setting_index >= 12;
  if (japanese_value) {
    set_ui_font(display, &fonts::lgfxJapanGothicP_16,
                runtime.setting_index == 23 ? 1 : 2);
  } else {
    set_ui_font(display, &fonts::FreeSansBold18pt7b);
  }
  if (display.textWidth(value) > 272) {
    set_ui_font(display, &fonts::lgfxJapanGothicP_16);
  }
  display.drawString(value, 160, 88);

  display.setTextDatum(middle_center);
  display.setTextColor(ui_color::kMuted, ui_color::kBackground);
  set_ui_font(display, &fonts::lgfxJapanGothicP_16);
  if (runtime.notice_until_ms > monotonic_ms() &&
      runtime.notice[0] != '\0') {
    display.setTextColor(ui_color::kCaution, ui_color::kBackground);
    display.drawString(runtime.notice, 160, 140);
  } else {
    display.drawString(kSettingTexts[runtime.setting_index].description_line1,
                       160, 130);
    display.drawString(kSettingTexts[runtime.setting_index].description_line2,
                       160, 150);
  }

  display.fillRoundRect(8, 165, 96, 40, 7, ui_color::kSurface);
  display.fillRoundRect(112, 165, 96, 40, 7, ui_color::kSurface);
  display.fillRoundRect(216, 165, 96, 40, 7, ui_color::kSurface);
  display.drawRoundRect(8, 165, 96, 40, 7, ui_color::kBorder);
  display.drawRoundRect(112, 165, 96, 40, 7, ui_color::kBorder);
  display.drawRoundRect(216, 165, 96, 40, 7, ui_color::kBorder);
  display.setTextDatum(middle_center);
  display.setTextColor(ui_color::kText, ui_color::kSurface);
  set_ui_font(display, &fonts::lgfxJapanGothicP_16);
  display.drawString(runtime.setting_index >= 21 ? "A 戻る" : "A 減らす",
                     56, 185);
  display.drawString("B 次へ", 160, 185);
  display.drawString(runtime.setting_index >= 21 ? "C 実行" : "C 増やす",
                     264, 185);

  display.setTextColor(ui_color::kMuted, ui_color::kBackground);
  display.drawString("B長押し：保存して戻る", 160, 224);
}

void draw_race(lgfx::v1::LGFXBase &display, const UiRuntime &runtime,
               bool fresh, uint64_t now_ms) {
  const auto config =
      runtime.race.phase() == RacePhase::Idle
          ? gpsmeter::make_race_config(runtime.settings)
          : runtime.race.config();
  display.fillScreen(ui_color::kBackground);
  display.fillRect(0, 0, 320, 5, ui_color::kRace);
  display.setTextDatum(top_left);
  display.setTextColor(ui_color::kRace, ui_color::kBackground);
  set_ui_font(display, &fonts::FreeSansBold12pt7b);
  display.drawString("RACE", 12, 10);

  char speed[20]{};
  if (fresh) {
    std::snprintf(speed, sizeof(speed), "%d km/h",
                  gpsmeter::display_speed(runtime.sample.speed_kmh));
  } else {
    std::snprintf(speed, sizeof(speed), "--- km/h");
  }
  display.setTextDatum(top_right);
  display.setTextColor(fresh ? ui_color::kMuted : ui_color::kWarning,
                       ui_color::kBackground);
  set_ui_font(display, &fonts::FreeSans9pt7b);
  display.drawString(speed, 308, 12);

  const RacePhase phase = runtime.race.phase();
  const bool active =
      phase == RacePhase::Running || phase == RacePhase::Finished;
  if (!active) {
    display.setTextDatum(middle_center);
    display.setTextColor(ui_color::kText, ui_color::kBackground);
    set_ui_font(display, &fonts::lgfxJapanGothicP_16, 2);
    display.drawString(phase == RacePhase::WaitingForBaseline
                           ? "周回計測 待機中"
                           : phase == RacePhase::WaitingForStart
                                 ? "開始前周回を除外中"
                                 : "レース設定待ち",
                       160, 103);
    display.setTextColor(ui_color::kMuted, ui_color::kBackground);
    set_ui_font(display, &fonts::lgfxJapanGothicP_16);
    display.drawString("停止中にB長押しで設定", 160, 151);
    return;
  }

  char primary[24]{};
  const char *label = nullptr;
  if (config.format == RaceFormat::Timed) {
    const bool estimated_time =
        runtime.last_valid_utc_received_ms != 0 &&
        now_ms - runtime.last_valid_utc_received_ms > 10000U;
    label = estimated_time ? "残り時間 ※推定" : "残り時間";
    const uint32_t remaining =
        gpsmeter::timed_race_remaining_s(
            config, runtime.last_valid_utc,
            runtime.last_valid_utc_received_ms == 0
                ? 0
                : static_cast<uint32_t>(
                      (now_ms - runtime.last_valid_utc_received_ms) /
                      1000ULL));
    if (remaining == UINT32_MAX) {
      std::snprintf(primary, sizeof(primary), "--:--");
    } else {
      gpsmeter::format_minutes_seconds(remaining, primary, sizeof(primary));
    }
  } else {
    label = "残り周回";
    std::snprintf(primary, sizeof(primary), "%u",
                  runtime.race.remaining_laps(config));
  }

  display.setTextDatum(middle_center);
  display.setTextColor(ui_color::kMuted, ui_color::kBackground);
  set_ui_font(display, &fonts::lgfxJapanGothicP_16);
  display.drawString(label, 160, 45);
  display.setTextColor(phase == RacePhase::Finished ? ui_color::kCaution
                                                    : ui_color::kText,
                       ui_color::kBackground);
  set_ui_font(display, &fonts::FreeSansBold24pt7b, 2);
  if (display.textWidth(primary) > 280) {
    set_ui_font(display, &fonts::FreeSansBold24pt7b);
  }
  display.drawString(primary, 160, 91);

  char lap[24]{};
  std::snprintf(lap, sizeof(lap), "LAP %u", runtime.race.current_lap());
  display.setTextColor(ui_color::kRace, ui_color::kBackground);
  set_ui_font(display, &fonts::FreeSansBold18pt7b);
  display.drawString(phase == RacePhase::Finished ? "FINISH" : lap, 160, 145);

  char stint_time[20]{};
  gpsmeter::format_minutes_seconds(
      runtime.race.stint_elapsed_ms(now_ms) / 1000ULL, stint_time,
      sizeof(stint_time));
  char stint[64]{};
  std::snprintf(stint, sizeof(stint), "STINT %u   %s   %u LAP",
                runtime.race.current_stint(), stint_time,
                runtime.race.current_stint_laps());
  display.fillRoundRect(10, 185, 300, 38, 8, ui_color::kSurface);
  display.setTextColor(ui_color::kText, ui_color::kSurface);
  set_ui_font(display, &fonts::FreeSansBold9pt7b);
  display.drawString(stint, 160, 204);

  if (runtime.notice_until_ms > now_ms && runtime.notice[0] != '\0') {
    display.fillRoundRect(32, 151, 256, 28, 6, ui_color::kSurfaceRaised);
    display.setTextColor(ui_color::kText, ui_color::kSurfaceRaised);
    set_ui_font(display, &fonts::lgfxJapanGothicP_16);
    display.drawString(runtime.notice, 160, 165);
  }
}

void draw_main(lgfx::v1::LGFXBase &display, const UiRuntime &runtime,
               bool fresh, uint64_t now_ms,
               const lgfx::IFont *smooth_speed_font) {
  if (runtime.mode.mode == DriveMode::Race) {
    draw_race(display, runtime, fresh, now_ms);
    return;
  }
  const UiVisualState visual = visual_state_for(runtime, fresh, now_ms);
  display.fillScreen(ui_color::kBackground);
  display.fillRect(0, 0, 320, 5, visual.accent);
  display.fillRect(0, 235, 320, 5, visual.accent);
  display.fillRect(0, 0, 5, 240, visual.accent);
  display.fillRect(315, 0, 5, 240, visual.accent);

  char mode[16]{};
  if (runtime.mode.mode == DriveMode::PitLane) {
    std::snprintf(mode, sizeof(mode), "PIT %u",
                  static_cast<unsigned>(runtime.settings.pit_limit_kmh));
  } else {
    std::snprintf(mode, sizeof(mode), "RACE");
  }
  display.setTextDatum(top_left);
  display.setTextColor(visual.accent, ui_color::kBackground);
  set_ui_font(display, &fonts::FreeSansBold12pt7b);
  display.drawString(mode, 13, 11);

  if (visual.status != nullptr) {
    display.setTextDatum(top_right);
    display.setTextColor(visual.accent, ui_color::kBackground);
    display.drawString(visual.status, 307, 11);
  }
  if (runtime.notice_until_ms > now_ms && runtime.notice[0] != '\0') {
    display.setTextDatum(top_right);
    display.setTextColor(ui_color::kText, ui_color::kBackground);
    set_ui_font(display, &fonts::lgfxJapanGothicP_16);
    display.drawString(runtime.notice, 307, 11);
  }

  char speed[8]{};
  if (fresh) {
    std::snprintf(speed, sizeof(speed), "%d",
                  gpsmeter::display_speed(runtime.sample.speed_kmh));
  } else {
    std::snprintf(speed, sizeof(speed), "---");
  }
  display.setTextDatum(middle_center);
  display.setTextColor(fresh ? ui_color::kText : ui_color::kInvalid,
                       ui_color::kBackground);
  if (smooth_speed_font != nullptr) {
    set_ui_font(display, smooth_speed_font);
  } else {
    set_ui_font(display, &fonts::FreeSansBold24pt7b, 3);
  }
  if (smooth_speed_font == nullptr && display.textWidth(speed) > 300) {
    set_ui_font(display, &fonts::FreeSansBold24pt7b, 2);
  }
  display.drawString(speed, 160, 119);
  display.setTextSize(1);

  display.setTextDatum(middle_center);
  display.setTextColor(ui_color::kMuted, ui_color::kBackground);
  set_ui_font(display, &fonts::FreeSansBold12pt7b);
  display.drawString("km/h", 160, 215);
}

void set_notice(UiRuntime &runtime, const char *text, uint64_t now_ms,
                uint32_t duration_ms = 2500) {
  std::snprintf(runtime.notice, sizeof(runtime.notice), "%s", text);
  runtime.notice_until_ms = now_ms + duration_ms;
  runtime.force_redraw = true;
}

bool select_course(UiRuntime &runtime, int direction) {
  const auto &courses = runtime.course_storage.courses;
  if (runtime.course_storage.status !=
          gpsmeter::CourseStorageStatus::Ok ||
      courses.count == 0) {
    return false;
  }
  for (size_t attempt = 0; attempt < courses.count; ++attempt) {
    const int count = static_cast<int>(courses.count);
    const int candidate = runtime.course_selection_initialized
                              ? (static_cast<int>(
                                     runtime.selected_course_index) +
                                 direction + count) %
                                    count
                              : (direction > 0 ? static_cast<int>(attempt)
                                               : count - 1 -
                                                     static_cast<int>(attempt));
    runtime.selected_course_index = static_cast<size_t>(candidate);
    const auto &entry = courses.entries[runtime.selected_course_index];
    if (entry.valid) {
      runtime.course_selection_initialized = true;
      runtime.settings.course_valid = true;
      runtime.settings.course_latitude = entry.point.latitude;
      runtime.settings.course_longitude = entry.point.longitude;
      std::snprintf(runtime.settings.course_name.data(),
                    runtime.settings.course_name.size(), "%s",
                    entry.name.data());
      return true;
    }
  }
  return false;
}

void handle_buttons(UiRuntime &runtime, bool fresh) {
  const uint64_t now_ms = monotonic_ms();
  if (runtime.setting_screen) {
    if (M5.BtnA.wasClicked()) {
      if (runtime.setting_index == 20) {
        (void)select_course(runtime, -1);
      } else {
        adjust_setting(runtime.settings, runtime.setting_index, -1);
      }
      runtime.force_redraw = true;
    }
    if (M5.BtnC.wasClicked()) {
      if (runtime.setting_index == 20) {
        (void)select_course(runtime, 1);
      } else if (runtime.setting_index == 21) {
        if (!runtime.registering_course && fresh &&
            runtime.sample.position_valid &&
            runtime.sample.speed_kmh <= 5.0F &&
            runtime.sample.satellites >= 6 &&
            runtime.sample.position_dop > 0.0F &&
            runtime.sample.position_dop <= 2.5F) {
          runtime.registering_course = true;
          runtime.registration_started_ms = now_ms;
          runtime.registration_latitude_sum = 0.0;
          runtime.registration_longitude_sum = 0.0;
          runtime.registration_latitude_m2 = 0.0;
          runtime.registration_longitude_m2 = 0.0;
          runtime.registration_samples = 0;
          set_notice(runtime, "10秒間その場で待ってください", now_ms);
        } else if (!runtime.registering_course) {
          set_notice(runtime, "停止してGPS安定後に実行", now_ms);
        }
      } else {
        adjust_setting(runtime.settings, runtime.setting_index, 1);
      }
      runtime.force_redraw = true;
    }
    if (M5.BtnB.wasHold()) {
      gpsmeter::normalize_settings(runtime.settings);
      (void)gpsmeter::save_settings(runtime.settings);
      if (runtime.race.phase() == RacePhase::WaitingForBaseline ||
          runtime.race.phase() == RacePhase::WaitingForStart) {
        const auto race_config =
            gpsmeter::make_race_config(runtime.settings);
        const bool inside =
            fresh && runtime.sample.position_valid &&
            gpsmeter::distance_m(runtime.sample.position,
                                 race_config.gate) <=
                race_config.gate_radius_m;
        runtime.race.begin_new(race_config, inside, now_ms);
      }
      runtime.setting_screen = false;
      runtime.force_redraw = true;
      play_tone(1800, 50, runtime.settings.tone_volume);
    } else if (M5.BtnB.wasClicked()) {
      runtime.setting_index =
          static_cast<uint8_t>((runtime.setting_index + 1) %
                               kSettingTexts.size());
      if (runtime.setting_index == kSettingTexts.size() - 1) {
        gpsmeter::normalize_settings(runtime.settings);
      }
      runtime.force_redraw = true;
    }
    return;
  }

  if (M5.BtnB.wasHold() &&
      (!fresh || runtime.sample.speed_kmh <= 5.0F)) {
    runtime.setting_screen = true;
    runtime.setting_index = 0;
    runtime.force_redraw = true;
    return;
  }
  const uint32_t required_hold_ms = fresh ? 1000U : 3000U;
  const bool can_switch_stint =
      runtime.mode.mode == DriveMode::PitLane &&
      (!fresh || runtime.sample.speed_kmh <= 5.0F);
  if (!runtime.c_hold_handled && can_switch_stint &&
      M5.BtnC.pressedFor(required_hold_ms)) {
    runtime.c_hold_handled = true;
    if (runtime.race.switch_stint(now_ms)) {
      char message[32]{};
      std::snprintf(message, sizeof(message), "STINT %u を開始",
                    runtime.race.current_stint());
      set_notice(runtime, message, now_ms, 2500);
      play_tone(1800, 70, runtime.settings.tone_volume);
    }
  }
  if (M5.BtnC.wasReleased()) {
    runtime.c_hold_handled = false;
  }
  if (M5.BtnC.wasClicked() && !runtime.c_hold_handled) {
    play_tone(runtime.settings.warning_tone_hz, 120,
              runtime.settings.tone_volume);
  }
  if (M5.BtnA.wasHold() && fresh && runtime.sample.speed_kmh <= 5.0F) {
    runtime.mode = ModeRuntime{};
    runtime.alert = SpeedAlert::None;
    runtime.force_redraw = true;
    ESP_LOGI(kTag, "Manual PIT mode reset");
  }
}

void ui_task(void *) {
  ESP_ERROR_CHECK(esp_task_wdt_add(nullptr));
  UiRuntime runtime{};
  (void)gpsmeter::load_settings(runtime.settings);
  gpsmeter::normalize_settings(runtime.settings);
  ESP_LOGI(kTag, "Settings active: pit=%u caution=%u warning=%u km/h",
           runtime.settings.pit_limit_kmh,
           runtime.settings.caution_speed_kmh,
           runtime.settings.warning_speed_kmh);

  M5.BtnC.setHoldThresh(1000);
  M5.Display.setRotation(1);
  M5.Display.setBrightness(runtime.settings.display_brightness);
  draw_startup_screen(M5.Display);
  const TickType_t startup_screen_started_at = xTaskGetTickCount();

  const bool speaker_ready =
      M5.Speaker.isEnabled() && M5.Speaker.begin();
  ESP_LOGI(kTag, "M5 board=%d display=%dx%d speaker=%s",
           static_cast<int>(M5.getBoard()), M5.Display.width(),
           M5.Display.height(), speaker_ready ? "ready" : "unavailable");
  if (speaker_ready) {
    play_tone(1400, 60, runtime.settings.tone_volume);
    vTaskDelay(pdMS_TO_TICKS(100));
    play_tone(2200, 80, runtime.settings.tone_volume);
  }
  const TickType_t startup_duration =
      pdMS_TO_TICKS(kStartupScreenDurationMs);
  for (;;) {
    const TickType_t startup_elapsed =
        xTaskGetTickCount() - startup_screen_started_at;
    draw_startup_progress(
        M5.Display,
        static_cast<uint32_t>(
            std::min<TickType_t>(startup_elapsed, startup_duration)) *
            portTICK_PERIOD_MS);
    if (startup_elapsed >= startup_duration) {
      break;
    }
    vTaskDelay(std::min<TickType_t>(
        pdMS_TO_TICKS(33), startup_duration - startup_elapsed));
  }

  runtime.course_storage = gpsmeter::load_courses_from_sd();
  if (runtime.course_storage.status == gpsmeter::CourseStorageStatus::Ok) {
    for (size_t index = 0;
         index < runtime.course_storage.courses.count; ++index) {
      const auto &entry = runtime.course_storage.courses.entries[index];
      if (entry.valid && runtime.settings.course_valid &&
          std::strcmp(entry.name.data(),
                      runtime.settings.course_name.data()) == 0) {
        runtime.selected_course_index = index;
        runtime.course_selection_initialized = true;
        break;
      }
    }
  }
  const uint64_t race_power_on_ms = monotonic_ms();
  runtime.race.begin_new(gpsmeter::make_race_config(runtime.settings),
                         false, race_power_on_ms);
  set_notice(runtime, "新しいレースを開始", race_power_on_ms, 3000);
  ESP_LOGI(kTag, "New race armed at power-on; previous progress ignored");

  M5Canvas ui_canvas(&M5.Display);
  ui_canvas.setPsram(false);
  ui_canvas.setColorDepth(16);
  int ui_canvas_depth = 16;
  bool ui_canvas_ready =
      ui_canvas.createSprite(M5.Display.width(), M5.Display.height()) != nullptr;
  if (!ui_canvas_ready) {
    ui_canvas.setColorDepth(8);
    ui_canvas_depth = 8;
    ui_canvas_ready =
        ui_canvas.createSprite(M5.Display.width(), M5.Display.height()) !=
        nullptr;
  }
  lgfx::v1::LGFXBase *render_target =
      ui_canvas_ready
          ? static_cast<lgfx::v1::LGFXBase *>(&ui_canvas)
          : static_cast<lgfx::v1::LGFXBase *>(&M5.Display);
  M5Canvas speed_font_owner(&M5.Display);
  speed_font_owner.setTextSize(1);
  const bool smooth_speed_font_ready =
      speed_font_owner.loadFont(speed_font_vlw_start);
  const lgfx::IFont *smooth_speed_font =
      smooth_speed_font_ready ? speed_font_owner.getFont() : nullptr;
  if (ui_canvas_ready) {
    ESP_LOGI(kTag, "UI framebuffer=ready depth=%d", ui_canvas_depth);
  } else {
    ESP_LOGW(kTag, "UI framebuffer unavailable; using direct draw");
  }
  ESP_LOGI(kTag, "Smooth speed font=%s",
           smooth_speed_font_ready ? "ready" : "fallback");

  runtime.last_loop_ms = monotonic_ms();
  TickType_t wake = xTaskGetTickCount();
  bool smooth_font_stack_logged = false;
  for (;;) {
    M5.update();
    GpsSample incoming{};
    if (xQueueReceive(g_gps_queue, &incoming, 0) == pdTRUE) {
      const int previous_speed =
          runtime.have_sample && runtime.sample.valid
              ? gpsmeter::display_speed(runtime.sample.speed_kmh)
              : -1;
      const int incoming_speed =
          incoming.valid ? gpsmeter::display_speed(incoming.speed_kmh) : -1;
      const bool sample_changed =
          !runtime.have_sample || runtime.sample.valid != incoming.valid ||
          runtime.sample.satellites != incoming.satellites ||
          previous_speed != incoming_speed;
      runtime.sample = incoming;
      runtime.have_sample = true;
      if (incoming.valid) {
        runtime.ever_had_fix = true;
      }
      runtime.force_redraw = runtime.force_redraw || sample_changed;
      if (incoming.utc.valid) {
        runtime.last_valid_utc = incoming.utc;
        runtime.last_valid_utc_received_ms = incoming.received_ms;
      }

      const uint64_t sample_now_ms = monotonic_ms();
      const auto race_config =
          runtime.race.phase() == RacePhase::Idle
              ? gpsmeter::make_race_config(runtime.settings)
              : runtime.race.config();
      const auto race_update = runtime.race.update_position(
          race_config, incoming.position,
          incoming.position_valid && incoming.valid &&
              incoming.satellites >= 4 && incoming.position_dop > 0.0F &&
              incoming.position_dop <= 4.0F,
          sample_now_ms);
      if (race_update.baseline_established || race_update.race_started ||
          race_update.lap_completed || race_update.race_finished) {
        runtime.force_redraw = true;
        if (race_update.race_started) {
          set_notice(runtime, "LAP 1 / STINT 1 開始", sample_now_ms, 2500);
        } else if (race_update.lap_completed) {
          ESP_LOGI(kTag, "Lap completed: %u",
                   runtime.race.completed_laps());
        }
      }

      if (runtime.registering_course && incoming.position_valid &&
          incoming.valid && incoming.speed_kmh <= 5.0F &&
          incoming.satellites >= 6 && incoming.position_dop > 0.0F &&
          incoming.position_dop <= 2.5F) {
        const uint32_t next_count = runtime.registration_samples + 1U;
        const double latitude_delta =
            incoming.position.latitude -
            runtime.registration_latitude_sum;
        runtime.registration_latitude_sum +=
            latitude_delta / static_cast<double>(next_count);
        runtime.registration_latitude_m2 +=
            latitude_delta *
            (incoming.position.latitude -
             runtime.registration_latitude_sum);
        const double longitude_delta =
            incoming.position.longitude -
            runtime.registration_longitude_sum;
        runtime.registration_longitude_sum +=
            longitude_delta / static_cast<double>(next_count);
        runtime.registration_longitude_m2 +=
            longitude_delta *
            (incoming.position.longitude -
             runtime.registration_longitude_sum);
        runtime.registration_samples = next_count;
      }
    }

    const uint64_t now_ms = monotonic_ms();
    const uint32_t elapsed_ms = static_cast<uint32_t>(
        std::min<uint64_t>(now_ms - runtime.last_loop_ms, 1000U));
    runtime.last_loop_ms = now_ms;
    const bool fresh =
        runtime.have_sample && runtime.sample.valid &&
        now_ms - runtime.sample.received_ms <= kGpsStaleMs;

    if (runtime.previous_fresh && !fresh && runtime.ever_had_fix) {
      if (runtime.pit_ng_tone_active) {
        stop_continuous_tone();
        runtime.pit_ng_tone_active = false;
        runtime.tone_level = SpeedAlert::None;
        runtime.last_tone_ms = now_ms;
        ESP_LOGI(kTag, "Pit NG continuous tone stopped");
      }
      play_tone(700, 180, runtime.settings.tone_volume);
      ESP_LOGW(kTag, "GNSS fix lost");
      runtime.force_redraw = true;
    }
    if (!runtime.previous_fresh && fresh) {
      ESP_LOGI(kTag, "GNSS fix acquired: speed=%.2f sat=%u",
               runtime.sample.speed_kmh, runtime.sample.satellites);
      runtime.force_redraw = true;
    }
    runtime.previous_fresh = fresh;

    if (runtime.registering_course &&
        now_ms - runtime.registration_started_ms >=
            kCourseRegistrationMs) {
      runtime.registering_course = false;
      if (runtime.registration_samples >= 20) {
        const gpsmeter::GeoPoint averaged{
            runtime.registration_latitude_sum,
            runtime.registration_longitude_sum,
        };
        const double longitude_scale =
            std::cos(averaged.latitude *
                     3.14159265358979323846 / 180.0);
        const double mean_square_degrees =
            (runtime.registration_latitude_m2 +
             runtime.registration_longitude_m2 * longitude_scale *
                 longitude_scale) /
            static_cast<double>(runtime.registration_samples);
        const double position_rms_m =
            std::sqrt(std::max(0.0, mean_square_degrees)) * 111320.0;
        if (position_rms_m > 10.0) {
          set_notice(runtime, "位置がばらつきました 再試行", now_ms, 3000);
          runtime.force_redraw = true;
          continue;
        }
        char created_name[16]{};
        (void)esp_task_wdt_reset();
        const auto status = gpsmeter::append_course_to_sd(
            averaged, created_name, sizeof(created_name));
        runtime.course_storage.status = status;
        if (status == gpsmeter::CourseStorageStatus::Ok) {
          runtime.settings.course_valid = true;
          runtime.settings.course_latitude = averaged.latitude;
          runtime.settings.course_longitude = averaged.longitude;
          std::snprintf(runtime.settings.course_name.data(),
                        runtime.settings.course_name.size(), "%s",
                        created_name);
          runtime.course_storage = gpsmeter::load_courses_from_sd();
          (void)gpsmeter::save_settings(runtime.settings);
          set_notice(runtime, "周回ポイントを保存しました", now_ms, 3000);
        } else {
          set_notice(runtime,
                     gpsmeter::course_storage_status_japanese(status),
                     now_ms, 3000);
        }
      } else {
        set_notice(runtime, "GPSが不安定です 再試行", now_ms, 3000);
      }
      runtime.force_redraw = true;
    }

    const auto policy = gpsmeter::make_policy(runtime.settings);
    const bool mode_changed = gpsmeter::update_drive_mode(
        runtime.mode, runtime.sample.speed_kmh, elapsed_ms, fresh, policy);
    if (mode_changed) {
      runtime.alert = SpeedAlert::None;
      runtime.tone_level = SpeedAlert::None;
      runtime.force_redraw = true;
      ESP_LOGI(kTag, "Drive mode changed: %s",
               runtime.mode.mode == DriveMode::PitLane ? "PIT" : "RACE");
    }

    SpeedAlert next_alert = SpeedAlert::None;
    if (fresh && runtime.mode.mode == DriveMode::PitLane) {
      next_alert = gpsmeter::classify_speed_alert(
          runtime.sample.speed_kmh, runtime.alert, policy);
    }
    if (next_alert != runtime.alert) {
      runtime.alert = next_alert;
      runtime.force_redraw = true;
      ESP_LOGI(kTag, "Pit alert changed: %d speed=%.2f",
               static_cast<int>(runtime.alert), runtime.sample.speed_kmh);
    }

    const bool should_sound_ng_tone = gpsmeter::should_sound_pit_ng_tone(
        runtime.mode.mode, runtime.sample.speed_kmh, fresh,
        runtime.settings.pit_tone_enabled,
        static_cast<float>(runtime.settings.pit_limit_kmh));
    if (should_sound_ng_tone) {
      if (!runtime.pit_ng_tone_active || !M5.Speaker.isPlaying()) {
        const bool started = start_continuous_tone(
            runtime.settings.warning_tone_hz,
            runtime.settings.tone_volume);
        if (started && !runtime.pit_ng_tone_active) {
          ESP_LOGI(kTag, "Pit NG continuous tone started: speed=%.2f limit=%u",
                   runtime.sample.speed_kmh,
                   runtime.settings.pit_limit_kmh);
        }
        runtime.pit_ng_tone_active = started;
      }
      runtime.tone_level = SpeedAlert::Warning;
    } else {
      if (runtime.pit_ng_tone_active) {
        stop_continuous_tone();
        runtime.pit_ng_tone_active = false;
        runtime.tone_level = SpeedAlert::None;
        runtime.last_tone_ms = now_ms;
        ESP_LOGI(kTag, "Pit NG continuous tone stopped");
      }
      if (runtime.settings.pit_tone_enabled &&
          runtime.mode.mode == DriveMode::PitLane &&
          gpsmeter::should_emit_alert_tone(
              runtime.alert, runtime.tone_level,
              static_cast<uint32_t>(now_ms),
              static_cast<uint32_t>(runtime.last_tone_ms))) {
        const uint16_t frequency =
            runtime.alert == SpeedAlert::Warning
                ? runtime.settings.warning_tone_hz
                : runtime.settings.caution_tone_hz;
        play_tone(frequency, 90, runtime.settings.tone_volume);
        runtime.tone_level = runtime.alert;
        runtime.last_tone_ms = now_ms;
      }
      if (runtime.alert == SpeedAlert::None) {
        runtime.tone_level = SpeedAlert::None;
      }
    }

    handle_buttons(runtime, fresh);

    const auto race_config =
        runtime.race.phase() == RacePhase::Idle
            ? gpsmeter::make_race_config(runtime.settings)
            : runtime.race.config();
    const uint32_t utc_elapsed_s =
        runtime.last_valid_utc_received_ms == 0
            ? 0
            : static_cast<uint32_t>(
                  (now_ms - runtime.last_valid_utc_received_ms) / 1000ULL);
    if (runtime.race.finish_if_needed(
            race_config, runtime.last_valid_utc, now_ms, utc_elapsed_s)) {
      runtime.force_redraw = true;
      set_notice(runtime, "レース終了", now_ms, 5000);
    }

    if (runtime.mode.mode == DriveMode::Race &&
        (runtime.race.phase() == RacePhase::Running ||
         runtime.race.phase() == RacePhase::Finished) &&
        now_ms / 1000U != (now_ms - elapsed_ms) / 1000U) {
      runtime.force_redraw = true;
    }
    if (runtime.setting_screen && runtime.registering_course &&
        now_ms / 1000U != (now_ms - elapsed_ms) / 1000U) {
      runtime.force_redraw = true;
    }
    if (runtime.notice_until_ms != 0 &&
        now_ms >= runtime.notice_until_ms) {
      runtime.notice_until_ms = 0;
      runtime.notice[0] = '\0';
      runtime.force_redraw = true;
    }
    if (runtime.setting_screen) {
      if (runtime.force_redraw) {
        draw_settings(*render_target, runtime);
        if (ui_canvas_ready) {
          ui_canvas.pushSprite(0, 0);
        }
        runtime.force_redraw = false;
      }
    } else {
      const bool animation_active =
          runtime.settings.pit_blink_enabled &&
          runtime.alert != SpeedAlert::None;
      if (runtime.force_redraw || animation_active) {
        draw_main(*render_target, runtime, fresh, now_ms,
                  smooth_speed_font);
        if (ui_canvas_ready) {
          ui_canvas.pushSprite(0, 0);
        }
        if (smooth_speed_font_ready && !smooth_font_stack_logged) {
          ESP_LOGI(kTag, "UI stack free after smooth draw: %u bytes",
                   static_cast<unsigned>(
                       uxTaskGetStackHighWaterMark(nullptr)));
          smooth_font_stack_logged = true;
        }
        runtime.force_redraw = false;
      }
    }

    (void)esp_task_wdt_reset();
    vTaskDelayUntil(&wake, kUiPeriod);
  }
}

} // namespace

extern "C" void app_main(void) {
  esp_err_t err = nvs_flash_init();
  if (err == ESP_ERR_NVS_NO_FREE_PAGES ||
      err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    ESP_ERROR_CHECK(nvs_flash_erase());
    err = nvs_flash_init();
  }
  ESP_ERROR_CHECK(err);

  // M5GFX board auto-detection temporarily uses several GPIOs, including the
  // Module GNSS UART choices. Finish all M5 hardware discovery before routing
  // UART2 to G13/G15.
  auto m5_config = M5.config();
  m5_config.fallback_board = m5::board_t::board_M5Stack;
  m5_config.internal_spk = true;
  M5.begin(m5_config);
  const bool module_bmi270_detected = M5.In_I2C.scanID(0x69, 100000);
  const bool module_bmp280_detected = M5.In_I2C.scanID(0x76, 100000);
  ESP_LOGI(kTag, "Module GNSS seat probe: BMI270=%s BMP280=%s",
           module_bmi270_detected ? "detected" : "none",
           module_bmp280_detected ? "detected" : "none");

  g_gps_queue = xQueueCreate(1, sizeof(GpsSample));
  if (g_gps_queue == nullptr) {
    ESP_LOGE(kTag, "Failed to create GNSS queue");
    return;
  }

  BaseType_t created =
      xTaskCreatePinnedToCore(gps_task, "gnss_uart", 4096, nullptr, 12, nullptr,
                              0);
  if (created != pdPASS) {
    ESP_LOGE(kTag, "Failed to create GNSS task");
    return;
  }
  created =
      xTaskCreatePinnedToCore(ui_task, "ui_control", 24576, nullptr, 8,
                              nullptr, 1);
  if (created != pdPASS) {
    ESP_LOGE(kTag, "Failed to create UI task");
    return;
  }
  ESP_LOGI(kTag, "Kuruma Race Assistant started");
}
