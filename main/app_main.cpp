#include "nmea_parser.hpp"
#include "settings.hpp"
#include "speed_policy.hpp"

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

namespace {

using gpsmeter::AppSettings;
using gpsmeter::DriveMode;
using gpsmeter::GpsSample;
using gpsmeter::ModeRuntime;
using gpsmeter::NmeaSentenceDetector;
using gpsmeter::SpeedAlert;

constexpr const char *kTag = "gps_speed_meter";
constexpr uart_port_t kGpsUart = UART_NUM_2;
constexpr gpio_num_t kGpsRxPin = GPIO_NUM_16;
constexpr gpio_num_t kGpsTxPin = GPIO_NUM_17;
constexpr int kGpsDefaultBaud = 115200;
constexpr std::array<int, 8> kGpsBaudCandidates{
    460800, 230400, 115200, 57600, 38400, 19200, 9600, 4800,
};
constexpr uint32_t kGpsAutobaudMeasureMs = 1500;
constexpr uint32_t kGpsBaudProbeMs = 1200;
constexpr uint32_t kGpsBaudRetryMs = 2000;
constexpr uint32_t kGpsStaleMs = 1000;
constexpr uint32_t kGpsDiagnosticPeriodMs = 5000;
constexpr uint32_t kCasicResponseTimeoutMs = 1200;
constexpr uint16_t kCasicFixIntervalMs = 100;
constexpr uint8_t kCasicFixRateHz = 10;
constexpr uint8_t kCasicVehicleDynamicModel = 3;
constexpr uint8_t kCasicAutomaticFixMode = 3;
constexpr uint32_t kStartupScreenDurationMs = 2000;
constexpr TickType_t kUiPeriod = pdMS_TO_TICKS(50);

QueueHandle_t g_gps_queue = nullptr;

int nearest_supported_baud(uint32_t estimate) {
  const auto closest = std::min_element(
      kGpsBaudCandidates.begin(), kGpsBaudCandidates.end(),
      [estimate](int lhs, int rhs) {
        return std::abs(static_cast<int64_t>(lhs) -
                        static_cast<int64_t>(estimate)) <
               std::abs(static_cast<int64_t>(rhs) -
                        static_cast<int64_t>(estimate));
      });
  return *closest;
}

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

void send_casic_command(const char *body) {
  uint8_t checksum = 0;
  for (const char *cursor = body; *cursor != '\0'; ++cursor) {
    checksum ^= static_cast<uint8_t>(*cursor);
  }
  char sentence[64]{};
  const int length = std::snprintf(sentence, sizeof(sentence), "$%s*%02X\r\n",
                                   body, checksum);
  if (length <= 0 || static_cast<size_t>(length) >= sizeof(sentence)) {
    ESP_LOGE(kTag, "CASIC command is too long");
    return;
  }
  const int written = uart_write_bytes(kGpsUart, sentence, length);
  if (written != length) {
    ESP_LOGW(kTag, "CASIC command write incomplete: %d/%d", written, length);
  }
  ESP_ERROR_CHECK(uart_wait_tx_done(kGpsUart, pdMS_TO_TICKS(100)));
}

uint16_t read_le_u16(const uint8_t *bytes) {
  return static_cast<uint16_t>(bytes[0]) |
         (static_cast<uint16_t>(bytes[1]) << 8U);
}

uint32_t read_le_u32(const uint8_t *bytes) {
  return static_cast<uint32_t>(bytes[0]) |
         (static_cast<uint32_t>(bytes[1]) << 8U) |
         (static_cast<uint32_t>(bytes[2]) << 16U) |
         (static_cast<uint32_t>(bytes[3]) << 24U);
}

float read_le_float(const uint8_t *bytes) {
  const uint32_t bits = read_le_u32(bytes);
  float value = 0.0F;
  static_assert(sizeof(value) == sizeof(bits));
  std::memcpy(&value, &bits, sizeof(value));
  return value;
}

uint32_t casic_checksum(uint8_t message_class, uint8_t message_id,
                        const uint8_t *payload, size_t payload_length) {
  uint32_t checksum =
      static_cast<uint32_t>(payload_length) |
      (static_cast<uint32_t>(message_class) << 16U) |
      (static_cast<uint32_t>(message_id) << 24U);
  for (size_t offset = 0; offset < payload_length; offset += 4U) {
    uint32_t word = 0;
    for (size_t part = 0;
         part < 4U && offset + part < payload_length; ++part) {
      word |= static_cast<uint32_t>(payload[offset + part])
              << (part * 8U);
    }
    checksum += word;
  }
  return checksum;
}

bool send_casic_packet(uint8_t message_class, uint8_t message_id,
                       const uint8_t *payload, size_t payload_length) {
  std::array<uint8_t, 128> packet{};
  const size_t packet_length = 6U + payload_length + 4U;
  if (payload_length > UINT16_MAX || packet_length > packet.size()) {
    ESP_LOGE(kTag, "CASIC packet is too large: %u",
             static_cast<unsigned>(payload_length));
    return false;
  }

  packet[0] = 0xBA;
  packet[1] = 0xCE;
  packet[2] = static_cast<uint8_t>(payload_length);
  packet[3] = static_cast<uint8_t>(payload_length >> 8U);
  packet[4] = message_class;
  packet[5] = message_id;
  if (payload_length > 0) {
    std::memcpy(&packet[6], payload, payload_length);
  }
  const uint32_t checksum =
      casic_checksum(message_class, message_id, payload, payload_length);
  const size_t checksum_offset = 6U + payload_length;
  packet[checksum_offset] = static_cast<uint8_t>(checksum);
  packet[checksum_offset + 1U] = static_cast<uint8_t>(checksum >> 8U);
  packet[checksum_offset + 2U] = static_cast<uint8_t>(checksum >> 16U);
  packet[checksum_offset + 3U] = static_cast<uint8_t>(checksum >> 24U);

  const int written =
      uart_write_bytes(kGpsUart, packet.data(), packet_length);
  if (written != static_cast<int>(packet_length)) {
    ESP_LOGW(kTag, "CASIC packet write incomplete: %d/%u", written,
             static_cast<unsigned>(packet_length));
    return false;
  }
  ESP_ERROR_CHECK(uart_wait_tx_done(kGpsUart, pdMS_TO_TICKS(100)));
  return true;
}

void send_casic_poll(uint8_t message_class, uint8_t message_id) {
  (void)send_casic_packet(message_class, message_id, nullptr, 0);
}

bool read_casic_response(uint8_t expected_class, uint8_t expected_id,
                         uint8_t *payload, size_t payload_capacity,
                         size_t &payload_length, uint32_t timeout_ms) {
  std::array<uint8_t, 128> packet{};
  std::array<uint8_t, 64> bytes{};
  size_t packet_size = 0;
  size_t expected_size = 0;
  const uint64_t deadline_ms = monotonic_ms() + timeout_ms;

  while (monotonic_ms() < deadline_ms) {
    const int count = uart_read_bytes(kGpsUart, bytes.data(), bytes.size(),
                                      pdMS_TO_TICKS(50));
    for (int index = 0; index < count; ++index) {
      const uint8_t byte = bytes[static_cast<size_t>(index)];
      if (packet_size == 0) {
        if (byte == 0xBA) {
          packet[packet_size++] = byte;
        }
        continue;
      }
      if (packet_size == 1) {
        if (byte == 0xCE) {
          packet[packet_size++] = byte;
        } else {
          packet_size = byte == 0xBA ? 1 : 0;
        }
        continue;
      }

      packet[packet_size++] = byte;
      if (packet_size == 4) {
        const size_t binary_payload_length = read_le_u16(&packet[2]);
        expected_size = 6U + binary_payload_length + 4U;
        if (expected_size > packet.size()) {
          ESP_LOGW(kTag, "CASIC response too large: %u",
                   static_cast<unsigned>(binary_payload_length));
          packet_size = 0;
          expected_size = 0;
        }
      }
      if (expected_size == 0 || packet_size < expected_size) {
        continue;
      }

      const size_t binary_payload_length = read_le_u16(&packet[2]);
      uint32_t calculated =
          static_cast<uint32_t>(binary_payload_length) |
          (static_cast<uint32_t>(packet[4]) << 16U) |
          (static_cast<uint32_t>(packet[5]) << 24U);
      for (size_t offset = 0; offset < binary_payload_length; offset += 4U) {
        uint32_t word = 0;
        for (size_t part = 0;
             part < 4U && offset + part < binary_payload_length; ++part) {
          word |= static_cast<uint32_t>(packet[6U + offset + part])
                  << (part * 8U);
        }
        calculated += word;
      }
      const uint32_t received = read_le_u32(&packet[6U + binary_payload_length]);
      const bool matches = packet[4] == expected_class &&
                           packet[5] == expected_id;
      if (calculated == received && matches &&
          binary_payload_length <= payload_capacity) {
        std::memcpy(payload, &packet[6], binary_payload_length);
        payload_length = binary_payload_length;
        return true;
      }
      if (calculated != received) {
        ESP_LOGW(kTag, "CASIC checksum mismatch: class=0x%02X id=0x%02X",
                 packet[4], packet[5]);
      }
      packet_size = 0;
      expected_size = 0;
    }
    (void)esp_task_wdt_reset();
  }
  return false;
}

const char *casic_dynamic_model_name(uint8_t model) {
  constexpr std::array<const char *, 8> names{
      "portable", "static", "walking", "car",
      "nautical", "flight<1g", "flight<2g", "flight<4g",
  };
  return model < names.size() ? names[model] : "unknown";
}

bool wait_for_casic_ack(uint8_t configured_class, uint8_t configured_id) {
  std::array<uint8_t, 8> payload{};
  size_t payload_length = 0;
  if (!read_casic_response(0x05, 0x01, payload.data(), payload.size(),
                           payload_length, kCasicResponseTimeoutMs)) {
    ESP_LOGW(kTag, "CASIC CFG ACK timeout: class=0x%02X id=0x%02X",
             configured_class, configured_id);
    return false;
  }
  const bool matches = payload_length >= 2 &&
                       payload[0] == configured_class &&
                       payload[1] == configured_id;
  if (!matches) {
    ESP_LOGW(kTag,
             "CASIC CFG ACK target mismatch: expected=0x%02X/0x%02X "
             "actual=0x%02X/0x%02X",
             configured_class, configured_id,
             payload_length >= 1 ? payload[0] : 0,
             payload_length >= 2 ? payload[1] : 0);
  }
  return matches;
}

bool read_casic_configuration(uint8_t message_id, uint8_t *payload,
                              size_t payload_capacity,
                              size_t &payload_length) {
  send_casic_poll(0x06, message_id);
  return read_casic_response(0x06, message_id, payload, payload_capacity,
                             payload_length, kCasicResponseTimeoutMs);
}

bool configure_casic_for_vehicle_once() {
  const std::array<uint8_t, 16> navigation_mode{
      kCasicVehicleDynamicModel,
      kCasicAutomaticFixMode,
      1, // Initial fix must be 3D.
      0, // Do not continue autonomous dead reckoning without satellites.
      0, 0, 0, 0, // Fixed altitude: 0.0 m.
      0, 0, 0, 0, // Fixed altitude accuracy: 0.0 m.
      1, // Enable altitude assistance.
      0, 0, 0,
  };
  const std::array<uint8_t, 4> navigation_rate{
      static_cast<uint8_t>(kCasicFixIntervalMs),
      static_cast<uint8_t>(kCasicFixIntervalMs >> 8U),
      kCasicFixRateHz,
      0,
  };

  const bool nav_sent =
      send_casic_packet(0x06, 0x0B, navigation_mode.data(),
                        navigation_mode.size());
  const bool nav_ack = nav_sent && wait_for_casic_ack(0x06, 0x0B);
  const bool rate_sent =
      send_casic_packet(0x06, 0x04, navigation_rate.data(),
                        navigation_rate.size());
  const bool rate_ack = rate_sent && wait_for_casic_ack(0x06, 0x04);

  std::array<uint8_t, 32> readback{};
  size_t readback_length = 0;
  const bool nav_read =
      read_casic_configuration(0x0B, readback.data(), readback.size(),
                               readback_length);
  const bool nav_verified =
      nav_read && readback_length >= navigation_mode.size() &&
      readback[0] == kCasicVehicleDynamicModel &&
      readback[1] == kCasicAutomaticFixMode &&
      readback[2] == 1 && readback[3] == 0 && readback[12] == 1;
  if (nav_read && !nav_verified) {
    ESP_LOGW(kTag,
             "CASIC CFG-NAVMODE verify mismatch: dynamic=%u fixMode=%u "
             "initFix3D=%u drLimit=%u altAid=%u",
             static_cast<unsigned>(readback[0]),
             static_cast<unsigned>(readback[1]),
             static_cast<unsigned>(readback[2]),
             static_cast<unsigned>(readback[3]),
             static_cast<unsigned>(readback[12]));
  }

  readback.fill(0);
  readback_length = 0;
  const bool rate_read =
      read_casic_configuration(0x04, readback.data(), readback.size(),
                               readback_length);
  const bool rate_verified =
      rate_read && readback_length >= navigation_rate.size() &&
      read_le_u16(readback.data()) == kCasicFixIntervalMs &&
      readback[2] == kCasicFixRateHz;
  if (rate_read && !rate_verified) {
    ESP_LOGW(kTag,
             "CASIC CFG-RATE verify mismatch: interval=%u rate=%u raw=%02X",
             static_cast<unsigned>(read_le_u16(readback.data())),
             static_cast<unsigned>(readback[2]),
             readback[3]);
  }

  ESP_LOGI(kTag,
           "CASIC vehicle setup: NAVMODE ack=%s verify=%s; "
           "RATE ack=%s verify=%s",
           nav_ack ? "ok" : "failed",
           nav_verified ? "ok" : "failed",
           rate_ack ? "ok" : "failed",
           rate_verified ? "ok" : "failed");
  return nav_ack && rate_ack && nav_verified && rate_verified;
}

void configure_casic_for_vehicle() {
  uint32_t attempt = 0;
  while (!configure_casic_for_vehicle_once()) {
    ++attempt;
    ESP_LOGE(kTag,
             "CASIC vehicle configuration failed (attempt %lu); retrying",
             static_cast<unsigned long>(attempt));
    ESP_ERROR_CHECK(uart_flush_input(kGpsUart));
    vTaskDelay(pdMS_TO_TICKS(1000));
    (void)esp_task_wdt_reset();
  }
  ESP_LOGI(kTag,
           "CASIC vehicle configuration verified: car, automatic 2D/3D, "
           "%u ms/%u Hz (volatile)",
           static_cast<unsigned>(kCasicFixIntervalMs),
           static_cast<unsigned>(kCasicFixRateHz));
}

void log_casic_navigation_configuration() {
  std::array<uint8_t, 64> payload{};
  size_t payload_length = 0;

  send_casic_poll(0x06, 0x04);
  if (read_casic_response(0x06, 0x04, payload.data(), payload.size(),
                          payload_length, 1200) &&
      payload_length >= 4) {
    ESP_LOGI(kTag,
             "CASIC CFG-RATE: interval=%u ms rate=%u Hz raw=%02X %02X %02X %02X",
             static_cast<unsigned>(read_le_u16(payload.data())),
             static_cast<unsigned>(payload[2]),
             payload[0], payload[1], payload[2], payload[3]);
  } else {
    ESP_LOGW(kTag, "CASIC CFG-RATE read failed (configuration unchanged)");
  }

  payload.fill(0);
  payload_length = 0;
  send_casic_poll(0x06, 0x0B);
  if (read_casic_response(0x06, 0x0B, payload.data(), payload.size(),
                          payload_length, 1200) &&
      payload_length >= 16) {
    ESP_LOGI(
        kTag,
        "CASIC CFG-NAVMODE: dynamic=%u(%s) fixMode=%u initFix3D=%u "
        "drLimit=%u s fixedAlt=%.1f m fixedAltAcc=%.1f m altAid=%u",
        static_cast<unsigned>(payload[0]),
        casic_dynamic_model_name(payload[0]),
        static_cast<unsigned>(payload[1]),
        static_cast<unsigned>(payload[2]),
        static_cast<unsigned>(payload[3]),
        static_cast<double>(read_le_float(&payload[4])),
        static_cast<double>(read_le_float(&payload[8])),
        static_cast<unsigned>(payload[12]));
  } else {
    ESP_LOGW(kTag, "CASIC CFG-NAVMODE read failed (configuration unchanged)");
  }

  payload.fill(0);
  payload_length = 0;
  send_casic_poll(0x06, 0x0A);
  if (read_casic_response(0x06, 0x0A, payload.data(), payload.size(),
                          payload_length, 1200) &&
      payload_length >= 8) {
    ESP_LOGI(kTag,
             "CASIC CFG-NAVLIMIT: minSV=%u maxSV=%u minCNO=%u minElev=%d",
             static_cast<unsigned>(payload[0]),
             static_cast<unsigned>(payload[1]),
             static_cast<unsigned>(payload[2]),
             static_cast<int>(static_cast<int8_t>(payload[3])));
  } else {
    ESP_LOGW(kTag, "CASIC CFG-NAVLIMIT read failed (configuration unchanged)");
  }

  payload.fill(0);
  payload_length = 0;
  send_casic_poll(0x06, 0x0C);
  if (read_casic_response(0x06, 0x0C, payload.data(), payload.size(),
                          payload_length, 1200) &&
      payload_length >= 20) {
    ESP_LOGI(kTag,
             "CASIC CFG-NAVFLT: maxPDOP=%.1f maxTDOP=%.1f maxPAcc=%.1f m "
             "maxTAcc=%.1f m staticSpeed=%.3f m/s",
             static_cast<double>(read_le_float(&payload[0])),
             static_cast<double>(read_le_float(&payload[4])),
             static_cast<double>(read_le_float(&payload[8])),
             static_cast<double>(read_le_float(&payload[12])),
             static_cast<double>(read_le_float(&payload[16])));
  } else {
    ESP_LOGW(kTag, "CASIC CFG-NAVFLT read failed (configuration unchanged)");
  }

  payload.fill(0);
  payload_length = 0;
  send_casic_poll(0x06, 0x12);
  if (read_casic_response(0x06, 0x12, payload.data(), payload.size(),
                          payload_length, 1200) &&
      payload_length >= 8) {
    ESP_LOGI(kTag,
             "CASIC CFG-NMEA: version=%u latLonResolution=%u "
             "heightResolution=%u gsaPlus=%u validOutput=0x%02X",
             static_cast<unsigned>(payload[0]),
             static_cast<unsigned>(payload[1]),
             static_cast<unsigned>(payload[2]),
             static_cast<unsigned>(payload[3]),
             payload[4]);
  } else {
    ESP_LOGW(kTag, "CASIC CFG-NMEA read failed (configuration unchanged)");
  }
}

bool probe_gps_baud(int baud, uint32_t duration_ms, uint32_t &byte_count,
                    uint32_t &valid_sentence_count) {
  ESP_ERROR_CHECK(uart_set_baudrate(kGpsUart, baud));
  ESP_ERROR_CHECK(uart_flush_input(kGpsUart));
  send_casic_command("PCAS06,0");

  NmeaSentenceDetector detector;
  std::array<uint8_t, 256> bytes{};
  byte_count = 0;
  valid_sentence_count = 0;
  const uint64_t deadline_ms = monotonic_ms() + duration_ms;
  while (monotonic_ms() < deadline_ms) {
    const int count = uart_read_bytes(kGpsUart, bytes.data(), bytes.size(),
                                      pdMS_TO_TICKS(50));
    if (count > 0) {
      byte_count += static_cast<uint32_t>(count);
    }
    for (int index = 0; index < count; ++index) {
      if (detector.feed(
              static_cast<char>(bytes[static_cast<size_t>(index)]))) {
        ++valid_sentence_count;
      }
    }
    (void)esp_task_wdt_reset();
  }
  return valid_sentence_count > 0;
}

int detect_gps_baud() {
  for (;;) {
    uart_bitrate_res_t bitrate{};
    ESP_ERROR_CHECK(uart_detect_bitrate_start(kGpsUart, nullptr));
    vTaskDelay(pdMS_TO_TICKS(kGpsAutobaudMeasureMs));
    ESP_ERROR_CHECK(uart_detect_bitrate_stop(kGpsUart, false, &bitrate));
    (void)esp_task_wdt_reset();

    const uint32_t estimate_ideal =
        bitrate.low_period + bitrate.high_period > 0
            ? static_cast<uint32_t>(
                  (static_cast<uint64_t>(bitrate.clk_freq_hz) * 2U) /
                  (bitrate.low_period + bitrate.high_period))
            : 0;
    const uint32_t estimate_positive =
        bitrate.pos_period > 0
            ? static_cast<uint32_t>(
                  (static_cast<uint64_t>(bitrate.clk_freq_hz) * 2U) /
                  bitrate.pos_period)
            : 0;
    const uint32_t estimate_negative =
        bitrate.neg_period > 0
            ? static_cast<uint32_t>(
                  (static_cast<uint64_t>(bitrate.clk_freq_hz) * 2U) /
                  bitrate.neg_period)
            : 0;
    ESP_LOGI(kTag,
             "GNSS edge probe: edges=%u low=%u high=%u pos=%u neg=%u "
             "estimate=%u/%u/%u",
             static_cast<unsigned>(bitrate.edge_cnt),
             static_cast<unsigned>(bitrate.low_period),
             static_cast<unsigned>(bitrate.high_period),
             static_cast<unsigned>(bitrate.pos_period),
             static_cast<unsigned>(bitrate.neg_period),
             static_cast<unsigned>(estimate_ideal),
             static_cast<unsigned>(estimate_positive),
             static_cast<unsigned>(estimate_negative));

    if (bitrate.edge_cnt > 0) {
      const std::array<uint32_t, 3> estimates{
          estimate_ideal, estimate_positive, estimate_negative};
      uint32_t previous = 0;
      for (const uint32_t estimate : estimates) {
        const uint32_t difference =
            estimate > previous ? estimate - previous : previous - estimate;
        if (estimate < 1200 || estimate > 5000000 ||
            (previous > 0 && difference < estimate / 100U)) {
          continue;
        }
        previous = estimate;
        const int candidate_baud = nearest_supported_baud(estimate);
        uint32_t byte_count = 0;
        uint32_t valid_sentence_count = 0;
        const bool detected = probe_gps_baud(
            candidate_baud, kGpsBaudProbeMs, byte_count, valid_sentence_count);
        ESP_LOGI(kTag,
                 "GNSS estimated baud probe: measured=%u candidate=%d "
                 "bytes=%u valid_nmea=%u",
                 static_cast<unsigned>(estimate),
                 candidate_baud,
                 static_cast<unsigned>(byte_count),
                 static_cast<unsigned>(valid_sentence_count));
        if (detected) {
          ESP_LOGI(kTag, "GNSS baud detected from edges: %d", candidate_baud);
          ESP_ERROR_CHECK(uart_flush_input(kGpsUart));
          return candidate_baud;
        }
      }
    }

    for (const int baud : kGpsBaudCandidates) {
      uint32_t byte_count = 0;
      uint32_t valid_sentence_count = 0;
      const bool detected = probe_gps_baud(
          baud, kGpsBaudProbeMs, byte_count, valid_sentence_count);
      ESP_LOGI(kTag, "GNSS baud probe: baud=%d bytes=%u valid_nmea=%u", baud,
               static_cast<unsigned>(byte_count),
               static_cast<unsigned>(valid_sentence_count));
      if (detected) {
        ESP_LOGI(kTag, "GNSS baud detected: %d", baud);
        ESP_ERROR_CHECK(uart_flush_input(kGpsUart));
        return baud;
      }
    }
    ESP_LOGW(kTag, "GNSS baud not detected; retrying");
    vTaskDelay(pdMS_TO_TICKS(kGpsBaudRetryMs));
  }
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
  ESP_ERROR_CHECK(uart_driver_install(kGpsUart, 4096, 0, 0, nullptr, 0));
  ESP_LOGI(kTag, "GNSS UART ready: port=%d RX=%d TX=%d; probing baud",
           static_cast<int>(kGpsUart), static_cast<int>(kGpsRxPin),
           static_cast<int>(kGpsTxPin));
  const int detected_baud = detect_gps_baud();
  ESP_LOGI(kTag, "GNSS receive started: baud=%d", detected_baud);
  configure_casic_for_vehicle();
  log_casic_navigation_configuration();
  send_casic_command("PCAS07,RMC,1");
  send_casic_command("PCAS07,GGA,1");
  ESP_LOGI(kTag, "GNSS RMC/GGA output requested for this power cycle");

  gpsmeter::NmeaParser parser;
  GpsSample sample{};
  std::array<uint8_t, 256> bytes{};
  uint32_t received_byte_count = 0;
  uint32_t parsed_sample_count = 0;
  uint32_t speed_change_count = 0;
  float previous_raw_speed_kmh = NAN;
  float minimum_speed_kmh = INFINITY;
  float maximum_speed_kmh = -INFINITY;
  uint64_t next_diagnostic_ms = monotonic_ms() + kGpsDiagnosticPeriodMs;
  for (;;) {
    const int count =
        uart_read_bytes(kGpsUart, bytes.data(), bytes.size(), pdMS_TO_TICKS(100));
    const uint64_t received_ms = monotonic_ms();
    if (count > 0) {
      received_byte_count += static_cast<uint32_t>(count);
    }
    for (int index = 0; index < count; ++index) {
      if (parser.feed(static_cast<char>(bytes[static_cast<size_t>(index)]),
                      received_ms, sample)) {
        ++parsed_sample_count;
        if (!std::isfinite(previous_raw_speed_kmh) ||
            sample.speed_kmh != previous_raw_speed_kmh) {
          ++speed_change_count;
          previous_raw_speed_kmh = sample.speed_kmh;
        }
        minimum_speed_kmh = std::min(minimum_speed_kmh, sample.speed_kmh);
        maximum_speed_kmh = std::max(maximum_speed_kmh, sample.speed_kmh);
        (void)xQueueOverwrite(g_gps_queue, &sample);
      }
    }
    if (received_ms >= next_diagnostic_ms) {
      ESP_LOGI(kTag,
               "GNSS RX: bytes=%u parsed_samples=%u changes=%u/5s speed=%.2f "
               "range=%.2f..%.2f status=%c mode=%c valid=%d sat=%u hdop=%.1f",
               static_cast<unsigned>(received_byte_count),
               static_cast<unsigned>(parsed_sample_count),
               static_cast<unsigned>(speed_change_count),
               static_cast<double>(sample.speed_kmh),
               static_cast<double>(std::isfinite(minimum_speed_kmh)
                                       ? minimum_speed_kmh
                                       : 0.0F),
               static_cast<double>(std::isfinite(maximum_speed_kmh)
                                       ? maximum_speed_kmh
                                       : 0.0F),
               sample.rmc_status, sample.rmc_mode,
               sample.valid ? 1 : 0,
               static_cast<unsigned>(sample.satellites),
               static_cast<double>(sample.hdop));
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
  SpeedAlert alert = SpeedAlert::None;
  SpeedAlert tone_level = SpeedAlert::None;
  uint64_t last_loop_ms = 0;
  uint64_t last_tone_ms = 0;
  bool have_sample = false;
  bool ever_had_fix = false;
  bool previous_fresh = false;
  bool pit_ng_tone_active = false;
  bool setting_screen = false;
  uint8_t setting_index = 0;
  bool force_redraw = true;
};

struct SettingText {
  const char *name;
  const char *description_line1;
  const char *description_line2;
};

constexpr std::array<SettingText, 16> kSettingTexts{{
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
    {"自動モード切替", "速度に応じてピットとレースを", "自動で切り替えます"},
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
  display.fillScreen(ui_color::kBackground);
  display.fillRect(0, 0, 320, 4, ui_color::kRace);

  display.fillRoundRect(109, 23, 102, 30, 15, ui_color::kSurface);
  display.drawRoundRect(109, 23, 102, 30, 15, ui_color::kBorder);
  display.setTextDatum(middle_center);
  display.setTextColor(ui_color::kText, ui_color::kSurface);
  set_ui_font(display, &fonts::FreeSansBold9pt7b);
  display.drawString("M5Stack", 160, 38);

  display.setTextColor(ui_color::kRace, ui_color::kBackground);
  set_ui_font(display, &fonts::FreeSansBold18pt7b);
  display.drawString("GPS SPEED", 160, 88);

  display.setTextColor(ui_color::kText, ui_color::kBackground);
  set_ui_font(display, &fonts::FreeSansBold18pt7b);
  display.drawString("METER", 160, 127);

  display.fillRect(92, 154, 60, 2, ui_color::kRace);
  display.fillRect(168, 154, 60, 2, ui_color::kWarning);
  display.fillCircle(160, 155, 3, ui_color::kText);

  display.setTextColor(ui_color::kMuted, ui_color::kBackground);
  set_ui_font(display, &fonts::FreeSans9pt7b);
  display.drawString("by kurumario", 160, 181);

  display.setTextColor(ui_color::kInvalid, ui_color::kBackground);
  set_ui_font(display, &fonts::Font0, 1);
  display.drawString("INITIALIZING GNSS", 160, 218);
  display.fillCircle(145, 231, 2, ui_color::kSurfaceRaised);
  display.fillCircle(160, 231, 2, ui_color::kRace);
  display.fillCircle(175, 231, 2, ui_color::kSurfaceRaised);
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
    settings.auto_mode_enabled = !settings.auto_mode_enabled;
    break;
  default:
    break;
  }
  gpsmeter::normalize_settings(settings);
}

void format_setting_value(const AppSettings &settings, uint8_t index,
                          char *buffer, size_t size) {
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
                  settings.auto_mode_enabled ? "オン" : "オフ");
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
  format_setting_value(runtime.settings, runtime.setting_index, value,
                       sizeof(value));
  display.setTextDatum(middle_center);
  display.setTextColor(ui_color::kText, ui_color::kSurfaceRaised);
  const bool japanese_value =
      runtime.setting_index == 5 || runtime.setting_index == 7 ||
      runtime.setting_index >= 12;
  if (japanese_value) {
    set_ui_font(display, &fonts::lgfxJapanGothicP_16,
                runtime.setting_index == 15 ? 1 : 2);
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
  display.drawString(kSettingTexts[runtime.setting_index].description_line1,
                     160, 130);
  display.drawString(kSettingTexts[runtime.setting_index].description_line2,
                     160, 150);

  display.fillRoundRect(8, 165, 96, 40, 7, ui_color::kSurface);
  display.fillRoundRect(112, 165, 96, 40, 7, ui_color::kSurface);
  display.fillRoundRect(216, 165, 96, 40, 7, ui_color::kSurface);
  display.drawRoundRect(8, 165, 96, 40, 7, ui_color::kBorder);
  display.drawRoundRect(112, 165, 96, 40, 7, ui_color::kBorder);
  display.drawRoundRect(216, 165, 96, 40, 7, ui_color::kBorder);
  display.setTextDatum(middle_center);
  display.setTextColor(ui_color::kText, ui_color::kSurface);
  set_ui_font(display, &fonts::lgfxJapanGothicP_16);
  display.drawString("A 減らす", 56, 185);
  display.drawString("B 次へ", 160, 185);
  display.drawString("C 増やす", 264, 185);

  display.setTextColor(ui_color::kMuted, ui_color::kBackground);
  display.drawString("B長押し：保存して戻る", 160, 224);
}

void draw_main(lgfx::v1::LGFXBase &display, const UiRuntime &runtime,
               bool fresh, uint64_t now_ms,
               const lgfx::IFont *smooth_speed_font) {
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

void handle_buttons(UiRuntime &runtime, bool fresh) {
  if (runtime.setting_screen) {
    if (M5.BtnA.wasClicked()) {
      adjust_setting(runtime.settings, runtime.setting_index, -1);
      runtime.force_redraw = true;
    }
    if (M5.BtnC.wasClicked()) {
      adjust_setting(runtime.settings, runtime.setting_index, 1);
      runtime.force_redraw = true;
    }
    if (M5.BtnB.wasHold()) {
      gpsmeter::normalize_settings(runtime.settings);
      (void)gpsmeter::save_settings(runtime.settings);
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

  if (M5.BtnB.wasHold()) {
    runtime.setting_screen = true;
    runtime.setting_index = 0;
    runtime.force_redraw = true;
    return;
  }
  if (M5.BtnC.wasClicked()) {
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

  auto config = M5.config();
  config.fallback_board = m5::board_t::board_M5Stack;
  config.internal_spk = true;
  M5.begin(config);
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
  const TickType_t startup_elapsed =
      xTaskGetTickCount() - startup_screen_started_at;
  const TickType_t startup_duration =
      pdMS_TO_TICKS(kStartupScreenDurationMs);
  if (startup_elapsed < startup_duration) {
    vTaskDelay(startup_duration - startup_elapsed);
  }

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

    const auto policy = gpsmeter::make_policy(runtime.settings);
    bool mode_changed = false;
    if (runtime.settings.auto_mode_enabled) {
      mode_changed = gpsmeter::update_drive_mode(
          runtime.mode, runtime.sample.speed_kmh, elapsed_ms, fresh, policy);
    } else if (runtime.mode.mode != DriveMode::Race) {
      runtime.mode.mode = DriveMode::Race;
      runtime.mode.race_candidate_ms = 0;
      runtime.mode.pit_candidate_ms = 0;
      mode_changed = true;
    }
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
  ESP_LOGI(kTag, "GPS Speed Meter started");
}
