idf_component_register(
    SRCS
        "signal_db.c"
        "signal_decoder.c"
    INCLUDE_DIRS
        "."
    REQUIRES
        driver
        esp_driver_spi
        esp_timer
        log
        json          # cJSON is bundled in ESP-IDF as component "json"
        spiffs        # esp_vfs_spiffs_register
        esp_common
        freertos
)