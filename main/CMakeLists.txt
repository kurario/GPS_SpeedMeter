idf_component_register(
    SRCS
        "app_main.cpp"
        "nmea_parser.cpp"
        "settings.cpp"
    INCLUDE_DIRS "."
    PRIV_REQUIRES
        driver
        esp_timer
        nvs_flash
        m5stack__m5unified
    EMBED_FILES
        "speed_font.vlw"
)

target_compile_options(${COMPONENT_LIB} PRIVATE -Wall -Wextra -Werror)
