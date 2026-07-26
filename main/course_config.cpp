#include "course_config.hpp"

#include <algorithm>
#include <cerrno>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>

namespace gpsmeter {
namespace {

std::string_view trim(std::string_view value) {
  while (!value.empty() &&
         (value.front() == ' ' || value.front() == '\t' ||
          value.front() == '\r')) {
    value.remove_prefix(1);
  }
  while (!value.empty() &&
         (value.back() == ' ' || value.back() == '\t' ||
          value.back() == '\r')) {
    value.remove_suffix(1);
  }
  return value;
}

bool parse_double(std::string_view text, double &value) {
  if (text.empty() || text.size() >= 40) {
    return false;
  }
  char copy[40]{};
  std::memcpy(copy, text.data(), text.size());
  errno = 0;
  char *end = nullptr;
  const double parsed = std::strtod(copy, &end);
  if (errno != 0 || end == copy || *end != '\0' ||
      !std::isfinite(parsed)) {
    return false;
  }
  value = parsed;
  return true;
}

void finalize(CourseList &list, CourseEntry &entry, bool have_section,
              bool have_latitude, bool have_longitude) {
  if (!have_section) {
    return;
  }
  entry.valid =
      have_latitude && have_longitude && valid_geo_point(entry.point);
  if (!entry.valid) {
    ++list.invalid_sections;
  }
  if (list.count < list.entries.size()) {
    list.entries[list.count++] = entry;
  } else {
    list.truncated = true;
  }
}

} // namespace

CourseList parse_courses(std::string_view text) {
  CourseList result{};
  CourseEntry current{};
  bool have_section = false;
  bool have_latitude = false;
  bool have_longitude = false;

  size_t offset = 0;
  while (offset <= text.size()) {
    const size_t next = text.find('\n', offset);
    std::string_view line =
        text.substr(offset, next == std::string_view::npos
                                ? text.size() - offset
                                : next - offset);
    line = trim(line);
    if (!line.empty() && line.front() != '#') {
      if (line.size() >= 3 && line.front() == '[' && line.back() == ']') {
        finalize(result, current, have_section, have_latitude,
                 have_longitude);
        current = CourseEntry{};
        have_latitude = false;
        have_longitude = false;
        const std::string_view name = trim(line.substr(1, line.size() - 2));
        have_section = !name.empty() && name.size() <= kCourseNameLength;
        if (have_section) {
          std::memcpy(current.name.data(), name.data(), name.size());
        } else {
          ++result.invalid_sections;
        }
      } else if (have_section) {
        const size_t equals = line.find('=');
        if (equals != std::string_view::npos) {
          const std::string_view key = trim(line.substr(0, equals));
          const std::string_view value = trim(line.substr(equals + 1));
          if (key == "latitude") {
            have_latitude = parse_double(value, current.point.latitude);
          } else if (key == "longitude") {
            have_longitude = parse_double(value, current.point.longitude);
          }
        }
      }
    }
    if (next == std::string_view::npos) {
      break;
    }
    offset = next + 1;
  }
  finalize(result, current, have_section, have_latitude, have_longitude);

  for (size_t index = 0; index < result.count; ++index) {
    if (!result.entries[index].valid) {
      continue;
    }
    for (size_t prior = 0; prior < index; ++prior) {
      if (result.entries[prior].valid &&
          (std::strcmp(result.entries[index].name.data(),
                       result.entries[prior].name.data()) == 0 ||
           (std::fabs(result.entries[index].point.latitude -
                      result.entries[prior].point.latitude) <
                0.0000001 &&
            std::fabs(result.entries[index].point.longitude -
                      result.entries[prior].point.longitude) <
                0.0000001))) {
        result.entries[index].valid = false;
        ++result.invalid_sections;
        break;
      }
    }
  }
  return result;
}

bool format_course_section(const char *name, const GeoPoint &point,
                           char *buffer, size_t size) {
  if (name == nullptr || buffer == nullptr || size == 0 ||
      std::strlen(name) == 0 || std::strlen(name) > kCourseNameLength ||
      !valid_geo_point(point)) {
    return false;
  }
  const int length =
      std::snprintf(buffer, size, "\n[%s]\nlatitude=%.7f\nlongitude=%.7f\n",
                    name, point.latitude, point.longitude);
  return length > 0 && static_cast<size_t>(length) < size;
}

} // namespace gpsmeter
