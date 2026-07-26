#include "course_storage.hpp"

#include "driver/sdspi_host.h"
#include "driver/spi_common.h"
#include "driver/spi_master.h"
#include "esp_log.h"
#include "esp_vfs_fat.h"
#include "sdmmc_cmd.h"

#include <array>
#include <cerrno>
#include <cstdio>
#include <cstring>
#include <sys/stat.h>
#include <unistd.h>

namespace gpsmeter {
namespace {

constexpr const char *kTag = "course_storage";
constexpr const char *kMountPoint = "/sdcard";
constexpr const char *kDirectory = "/sdcard/gps_speed_meter";
constexpr const char *kCourseFile = "/sdcard/gps_speed_meter/courses.txt";
constexpr const char *kTemporaryFile =
    "/sdcard/gps_speed_meter/courses.tmp";
constexpr const char *kBackupFile = "/sdcard/gps_speed_meter/courses.bak";
constexpr size_t kMaximumFileSize = 8192;

struct MountedCard {
  sdmmc_card_t *card = nullptr;
  bool bus_owned = false;

  ~MountedCard() {
    if (card != nullptr) {
      esp_vfs_fat_sdcard_unmount(kMountPoint, card);
    }
    if (bus_owned) {
      (void)spi_bus_free(SPI3_HOST);
    }
  }
};

bool mount_card(MountedCard &mounted) {
  const spi_bus_config_t bus{
      .mosi_io_num = GPIO_NUM_23,
      .miso_io_num = GPIO_NUM_19,
      .sclk_io_num = GPIO_NUM_18,
      .quadwp_io_num = -1,
      .quadhd_io_num = -1,
      .data4_io_num = -1,
      .data5_io_num = -1,
      .data6_io_num = -1,
      .data7_io_num = -1,
      .data_io_default_level = false,
      .max_transfer_sz = 4096,
      .flags = 0,
      .isr_cpu_id = ESP_INTR_CPU_AFFINITY_AUTO,
      .intr_flags = 0,
  };
  size_t maximum_transaction_size = 0;
  esp_err_t err =
      spi_bus_get_max_transaction_len(SPI3_HOST,
                                      &maximum_transaction_size);
  if (err != ESP_OK) {
    err = spi_bus_initialize(SPI3_HOST, &bus, SPI_DMA_CH_AUTO);
    if (err != ESP_OK) {
      ESP_LOGW(kTag, "SD SPI bus unavailable: %s",
               esp_err_to_name(err));
      return false;
    }
    mounted.bus_owned = true;
  }

  sdmmc_host_t host = SDSPI_HOST_DEFAULT();
  host.slot = SPI3_HOST;
  host.max_freq_khz = 10000;
  sdspi_device_config_t slot = SDSPI_DEVICE_CONFIG_DEFAULT();
  slot.gpio_cs = GPIO_NUM_4;
  slot.host_id = SPI3_HOST;
  const esp_vfs_fat_sdmmc_mount_config_t mount{
      .format_if_mount_failed = false,
      .max_files = 4,
      .allocation_unit_size = 16 * 1024,
      .disk_status_check_enable = false,
      .use_one_fat = false,
  };
  err = esp_vfs_fat_sdspi_mount(kMountPoint, &host, &slot, &mount,
                                &mounted.card);
  if (err != ESP_OK) {
    ESP_LOGW(kTag, "microSD mount failed: %s", esp_err_to_name(err));
    return false;
  }
  return true;
}

bool read_file(const char *path,
               std::array<char, kMaximumFileSize + 1> &buffer,
               size_t &length, CourseStorageStatus &status) {
  FILE *file = std::fopen(path, "rb");
  if (file == nullptr) {
    status = errno == ENOENT ? CourseStorageStatus::FileMissing
                             : CourseStorageStatus::ReadError;
    return false;
  }
  length = std::fread(buffer.data(), 1, kMaximumFileSize, file);
  const bool read_failed = std::ferror(file) != 0;
  const int extra = std::fgetc(file);
  std::fclose(file);
  if (read_failed) {
    status = CourseStorageStatus::ReadError;
    return false;
  }
  if (extra != EOF) {
    status = CourseStorageStatus::FileTooLarge;
    return false;
  }
  buffer[length] = '\0';
  return true;
}

} // namespace

CourseStorageResult load_courses_from_sd() {
  CourseStorageResult result{};
  MountedCard mounted{};
  if (!mount_card(mounted)) {
    return result;
  }
  std::array<char, kMaximumFileSize + 1> contents{};
  size_t length = 0;
  if (!read_file(kCourseFile, contents, length, result.status)) {
    return result;
  }
  result.courses =
      parse_courses(std::string_view(contents.data(), length));
  bool have_valid = false;
  for (size_t index = 0; index < result.courses.count; ++index) {
    have_valid = have_valid || result.courses.entries[index].valid;
  }
  result.status = have_valid ? CourseStorageStatus::Ok
                             : CourseStorageStatus::NoValidCourses;
  return result;
}

CourseStorageStatus append_course_to_sd(const GeoPoint &point,
                                        char *created_name,
                                        size_t created_name_size) {
  if (!valid_geo_point(point)) {
    return CourseStorageStatus::WriteError;
  }
  MountedCard mounted{};
  if (!mount_card(mounted)) {
    return CourseStorageStatus::CardUnavailable;
  }
  if (::mkdir(kDirectory, 0775) != 0 && errno != EEXIST) {
    return CourseStorageStatus::WriteError;
  }

  std::array<char, kMaximumFileSize + 1> contents{};
  size_t length = 0;
  CourseStorageStatus read_status = CourseStorageStatus::Ok;
  if (!read_file(kCourseFile, contents, length, read_status) &&
      read_status != CourseStorageStatus::FileMissing) {
    return read_status;
  }
  const CourseList existing =
      parse_courses(std::string_view(contents.data(), length));
  unsigned sequence = 1;
  char course_name[16]{};
  for (; sequence <= 99; ++sequence) {
    std::snprintf(course_name, sizeof(course_name), "COURSE%02u", sequence);
    bool used = false;
    for (size_t index = 0; index < existing.count; ++index) {
      used = used ||
             std::strcmp(existing.entries[index].name.data(), course_name) ==
                 0;
    }
    if (!used) {
      break;
    }
  }
  if (sequence > 99) {
    return CourseStorageStatus::WriteError;
  }
  char section[128]{};
  if (!format_course_section(course_name, point, section, sizeof(section)) ||
      length + std::strlen(section) > kMaximumFileSize) {
    return CourseStorageStatus::FileTooLarge;
  }

  FILE *temporary = std::fopen(kTemporaryFile, "wb");
  if (temporary == nullptr) {
    return CourseStorageStatus::WriteError;
  }
  const bool write_ok =
      (length == 0 ||
       std::fwrite(contents.data(), 1, length, temporary) == length) &&
      std::fwrite(section, 1, std::strlen(section), temporary) ==
          std::strlen(section) &&
      std::fflush(temporary) == 0 && ::fsync(fileno(temporary)) == 0;
  const bool close_ok = std::fclose(temporary) == 0;
  if (!write_ok || !close_ok) {
    (void)::unlink(kTemporaryFile);
    return CourseStorageStatus::WriteError;
  }
  std::array<char, kMaximumFileSize + 1> verified_contents{};
  size_t verified_length = 0;
  CourseStorageStatus verify_status = CourseStorageStatus::Ok;
  if (!read_file(kTemporaryFile, verified_contents, verified_length,
                 verify_status)) {
    (void)::unlink(kTemporaryFile);
    return CourseStorageStatus::WriteError;
  }
  const CourseList verified = parse_courses(
      std::string_view(verified_contents.data(), verified_length));
  bool created_entry_valid = false;
  for (size_t index = 0; index < verified.count; ++index) {
    created_entry_valid =
        created_entry_valid ||
        (verified.entries[index].valid &&
         std::strcmp(verified.entries[index].name.data(), course_name) == 0);
  }
  if (!created_entry_valid) {
    (void)::unlink(kTemporaryFile);
    return CourseStorageStatus::WriteError;
  }
  (void)::unlink(kBackupFile);
  if (::access(kCourseFile, F_OK) == 0 &&
      ::rename(kCourseFile, kBackupFile) != 0) {
    (void)::unlink(kTemporaryFile);
    return CourseStorageStatus::WriteError;
  }
  if (::rename(kTemporaryFile, kCourseFile) != 0) {
    (void)::rename(kBackupFile, kCourseFile);
    return CourseStorageStatus::WriteError;
  }
  if (created_name != nullptr && created_name_size > 0) {
    std::snprintf(created_name, created_name_size, "%s", course_name);
  }
  return CourseStorageStatus::Ok;
}

const char *course_storage_status_japanese(CourseStorageStatus status) {
  switch (status) {
  case CourseStorageStatus::Ok:
    return "microSD読込済み";
  case CourseStorageStatus::CardUnavailable:
    return "microSDがありません";
  case CourseStorageStatus::FileMissing:
    return "コースファイルなし";
  case CourseStorageStatus::ReadError:
    return "コース読込エラー";
  case CourseStorageStatus::FileTooLarge:
    return "コースファイル過大";
  case CourseStorageStatus::NoValidCourses:
    return "有効なコースなし";
  case CourseStorageStatus::WriteError:
  default:
    return "microSD書込エラー";
  }
}

} // namespace gpsmeter
