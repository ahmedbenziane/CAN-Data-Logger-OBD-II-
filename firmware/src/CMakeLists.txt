cmake_minimum_required(VERSION 3.16)
 
idf_component_register(
    SRCS
        "main.c"
    INCLUDE_DIRS
        "."
    REQUIRES
        driver
        esp_timer
        freertos
        diagnostic
        isotp
        UDS
        dispatcher
        decoder
        GPS
        shutdown
        sim
        sniffer
        signal
        log
        mcp2515
)
