#pragma once

#include "race_progress.hpp"

#include <array>
#include <cstddef>
#include <string_view>

namespace gpsmeter {

constexpr size_t kMaxCourses = 24;
constexpr size_t kCourseNameLength = 24;

struct CourseEntry {
  std::array<char, kCourseNameLength + 1> name{};
  GeoPoint point{};
  bool valid = false;
};

struct CourseList {
  std::array<CourseEntry, kMaxCourses> entries{};
  size_t count = 0;
  size_t invalid_sections = 0;
  bool truncated = false;
};

CourseList parse_courses(std::string_view text);
bool format_course_section(const char *name, const GeoPoint &point,
                           char *buffer, size_t size);

} // namespace gpsmeter
