#pragma once

#include "course_config.hpp"

#include <cstddef>

namespace gpsmeter {

enum class CourseStorageStatus : uint8_t {
  Ok = 0,
  CardUnavailable,
  FileMissing,
  ReadError,
  FileTooLarge,
  NoValidCourses,
  WriteError,
};

struct CourseStorageResult {
  CourseStorageStatus status = CourseStorageStatus::CardUnavailable;
  CourseList courses{};
};

CourseStorageResult load_courses_from_sd();
CourseStorageStatus append_course_to_sd(const GeoPoint &point,
                                        char *created_name,
                                        size_t created_name_size);
const char *course_storage_status_japanese(CourseStorageStatus status);

} // namespace gpsmeter
