/* ============================================================
 *  gps_app.h  —  NEO-6M GPS driver for Node A
 *
 *  Wiring (UART2 defaults):
 *    NEO-6M TX  →  ESP32 GPIO 16  (UART2 RX)
 *    NEO-6M RX  →  ESP32 GPIO 17  (UART2 TX)
 *    NEO-6M VCC →  3.3V
 *    NEO-6M GND →  GND
 *
 *  Rate: 10 Hz via UBX CFG-RATE command at boot.
 *  Parses NMEA GGA + RMC. Thread-safe gps_get_fix().
 *  Emits {"type":"gps",...} JSON lines to serial.
 * ============================================================ */
#pragma once
#include "esp_err.h"
#include <stdint.h>
#include <stdbool.h>

typedef struct
{
    double lat;            /* decimal degrees, +N/-S        */
    double lon;            /* decimal degrees, +E/-W        */
    float speed_kmh;       /* from RMC (knots × 1.852)      */
    float heading_deg;     /* true course 0–360 from RMC    */
    float altitude_m;      /* MSL altitude from GGA         */
    float hdop;            /* horizontal dilution           */
    uint8_t satellites;    /* satellites in use             */
    uint8_t fix_quality;   /* 0=none 1=GPS 2=DGPS           */
    bool valid;            /* true when fix acquired        */
    uint32_t timestamp_ms; /* esp_timer ms of last fix      */
    uint32_t fix_count;    /* total valid fixes received    */
} gps_fix_t;

/**
 * Initialise UART2, configure NEO-6M at 10 Hz via UBX, start
 * parser task. Call once from decoder_app_start().
 */
esp_err_t gps_app_start(void);

/** Thread-safe copy of latest fix. */
gps_fix_t gps_get_fix(void);

/** True if a valid fix arrived in the last 3 s. */
bool gps_is_fresh(void);