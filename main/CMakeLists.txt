idf_component_register(
    SRCS
        "app_main.cpp"
        "course_config.cpp"
        "course_storage.cpp"
        "race_progress.cpp"
        "settings.cpp"
        "ubx_parser.cpp"
    INCLUDE_DIRS "."
    PRIV_REQUIRES
        driver
        fatfs
        esp_timer
        nvs_flash
        m5stack__m5unified
    EMBED_FILES
        "startup.png"
        "speed_font.vlw"
)

target_compile_options(${COMPONENT_LIB} PRIVATE -Wall -Wextra -Werror)
