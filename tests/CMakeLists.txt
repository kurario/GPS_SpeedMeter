cmake_minimum_required(VERSION 3.16)
project(gps_speed_meter_tests LANGUAGES CXX)

enable_testing()

add_executable(gps_speed_meter_tests
    test_main.cpp
    ../main/nmea_parser.cpp
)
target_include_directories(gps_speed_meter_tests PRIVATE ../main)
target_compile_features(gps_speed_meter_tests PRIVATE cxx_std_20)
target_compile_options(gps_speed_meter_tests PRIVATE -Wall -Wextra -Werror)

add_test(NAME gps_speed_meter_tests COMMAND gps_speed_meter_tests)
