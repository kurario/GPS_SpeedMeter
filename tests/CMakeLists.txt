cmake_minimum_required(VERSION 3.16)
project(kuruma_race_assistant_tests LANGUAGES CXX)

enable_testing()

add_executable(kuruma_race_assistant_tests
    test_main.cpp
    ../main/course_config.cpp
    ../main/nmea_parser.cpp
    ../main/race_progress.cpp
    ../main/ubx_parser.cpp
)
target_include_directories(kuruma_race_assistant_tests PRIVATE ../main)
target_compile_features(kuruma_race_assistant_tests PRIVATE cxx_std_20)
target_compile_options(kuruma_race_assistant_tests PRIVATE -Wall -Wextra -Werror)

add_test(NAME kuruma_race_assistant_tests COMMAND kuruma_race_assistant_tests)
