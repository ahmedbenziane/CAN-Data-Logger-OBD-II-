/* ============================================================
 *  sd_logger.h  —  Zero-loss CAN + GPS + Dead Reckoning logger
 *
 *  SD Card wiring (HSPI — separate bus from MCP2515 VSPI):
 *    MOSI = GPIO 13    MISO = GPIO 27
 *    CLK  = GPIO 14    CS   = GPIO 26
 *
 *  NOTE: GPIO12 and GPIO15 are ESP32 strapping pins — never use
 *  them for SD. GPIO12 sets flash voltage at boot (HIGH = crash),
 *  GPIO15 controls boot logging.
 *
 *  Session files in /sdcard/can/session_NNNNN/ :
 *    raw.bin        Every raw CAN frame        @ ~100 Hz (20 B each)
 *    decoded.csv    Decoded signals            @ 100 Hz
 *    gps.csv        GPS fixes                  @ 10 Hz
 *    dr.csv         Dead-reckoning position    @ 100 Hz
 *    uds.csv        UDS DID results            (on-demand)
 *
 *  Sample rate rationale for smooth dead reckoning:
 *    100 Hz CAN  → 10 ms integration step → < 0.3 m/step at 100 km/h
 *    10 Hz GPS   → drift correction every 100 ms
 *    Error bound: < 2 m per 200 m at highway speed
 * ============================================================ */
#pragma once
#include "esp_err.h"
#include "mcp2515.h"
#include <stdint.h>
#include <stdbool.h>

/* ── Raw CAN record (20 bytes, packed) ─────────────────────── */
typedef struct __attribute__((packed))
{
    uint32_t timestamp_ms;
    uint32_t can_id;
    uint8_t dlc;
    uint8_t flags;
    uint8_t data[8];
    uint16_t seq;
} raw_frame_t;

#define LOG_FLAG_UDS 0x01
#define LOG_FLAG_OBD 0x02
#define LOG_FLAG_CORRUPT 0x04
#define LOG_FLAG_TIMEOUT 0x08

/* ── Decoded CAN snapshot (100 Hz) ─────────────────────────── */
typedef struct
{
    uint32_t timestamp_ms;
    int16_t rpm;
    int16_t steer_deg_x10; /* ×10: 1234 = 123.4° */
    uint8_t wheel_fl;      /* km/h integer        */
    uint8_t wheel_fr;
    uint8_t wheel_rl;
    uint8_t wheel_rr;
    uint8_t brake_bar;
    uint8_t valid_mask;
} decoded_snapshot_t;

#define VALID_RPM 0x01
#define VALID_STEER 0x02
#define VALID_WHEELS 0x04
#define VALID_BRAKE 0x08

/* ── GPS snapshot (10 Hz) ───────────────────────────────────── */
typedef struct
{
    uint32_t timestamp_ms;
    double lat;
    double lon;
    float speed_kmh;
    float heading_deg;
    float altitude_m;
    uint8_t satellites;
    float hdop;
    bool valid;
} gps_snapshot_t;

/* ── Dead-reckoning position (100 Hz) ──────────────────────── */
typedef struct
{
    uint32_t timestamp_ms;
    float x_m;         /* metres east  from origin */
    float y_m;         /* metres north from origin */
    float heading_deg; /* 0=North CW+              */
    float speed_kmh;
    uint8_t gps_corrected; /* 1 = GPS anchor this tick */
} dr_snapshot_t;

/* ── Statistics ─────────────────────────────────────────────── */
typedef struct
{
    uint16_t queue_used;
    uint16_t queue_cap;
    uint32_t dropped;
    uint32_t written_raw;
    uint32_t written_decoded;
    uint32_t written_gps;
    uint32_t written_dr;
    uint32_t written_uds;
    uint32_t session_num;
    bool sd_ready;
} sd_logger_stats_t;

/* ── API ────────────────────────────────────────────────────── */
esp_err_t sd_logger_init(void);
void sd_logger_log_raw(const can_frame_t *f, uint8_t flags);
void sd_logger_log_decoded(const decoded_snapshot_t *s);
void sd_logger_log_gps(const gps_snapshot_t *g);
void sd_logger_log_dr(const dr_snapshot_t *dr);
void sd_logger_log_uds(uint16_t did, const uint8_t *data,
                       size_t len, uint32_t elapsed_ms, bool corrupt);
sd_logger_stats_t sd_logger_get_stats(void);

/* ── Serial JSON emitters ────────────────────────────────────── */
void sd_logger_serial_emit_decoded(const decoded_snapshot_t *s);
void sd_logger_serial_emit_gps(const gps_snapshot_t *g);
void sd_logger_serial_emit_dr(const dr_snapshot_t *dr);
void sd_logger_serial_emit_uds(uint16_t did, const uint8_t *data,
                               size_t len, uint32_t elapsed_ms, bool corrupt);
void sd_logger_serial_emit_stats(void);
void sd_logger_deinit(void);