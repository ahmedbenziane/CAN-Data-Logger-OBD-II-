/**
 * @file sniffer_app.h
 * @brief Passive CAN Sniffer — LISTEN-ONLY mode, CSV logger, stats engine
 *
 * Hardware guarantee:
 *   MCP2515 in LISTEN-ONLY mode (0b011) never drives CANH/CANL.
 *   It receives frames silently — no ACK bits, no error frames.
 *   Safe to connect to a live vehicle bus without affecting ECU behaviour.
 *
 * CSV output format (one line per frame, written to UART):
 *   timestamp_us,id_hex,dlc,b0,b1,b2,b3,b4,b5,b6,b7
 *   Example:
 *     1234567,7E8,8,04,41,0C,1A,F0,00,00,00
 *
 * Configurable via /spiffs/system.json (loaded at boot):
 *   {
 *     "sniffer": {
 *       "csv_header":    true,
 *       "id_filter_min": "0x000",
 *       "id_filter_max": "0x7FF",
 *       "stats_interval_ms": 10000,
 *       "max_unique_ids":    256
 *     }
 *   }
 *
 * Python analyzer:
 *   Redirect UART output to a file, then run:
 *     pio device monitor --environment node_b > capture.csv
 *     python tools/signal_analyzer.py capture.csv
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "mcp2515.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ── Limits ─────────────────────────────────────────────────────────────── */

#define SNIFFER_MAX_UNIQUE_IDS   256u   /**< Max distinct IDs tracked in RAM */
#define SNIFFER_BYTE_HISTORY     8u     /**< CAN frame bytes (standard CAN)  */

/* ── Per-ID statistics ──────────────────────────────────────────────────── */

/**
 * @brief Statistics accumulated for one CAN ID.
 *
 * Updated on every received frame. Kept in a flat array sorted by first-seen
 * order (insertion order). Lookup is linear scan — with ≤256 IDs on a typical
 * car bus this costs ~10 µs worst case, acceptable for 500 kbps CAN.
 */
typedef struct {
    uint32_t  can_id;                        /**< CAN frame ID                */
    uint64_t  frame_count;                   /**< Total frames received        */

    /* Timing */
    uint32_t  first_seen_us;                 /**< esp_timer timestamp          */
    uint32_t  last_seen_us;
    uint32_t  min_period_us;                 /**< Minimum inter-frame gap      */
    uint32_t  max_period_us;
    uint64_t  period_sum_us;                 /**< For mean calculation         */
    uint32_t  period_count;                  /**< Samples in period_sum        */

    /* Per-byte statistics */
    uint8_t   byte_min[SNIFFER_BYTE_HISTORY];
    uint8_t   byte_max[SNIFFER_BYTE_HISTORY];
    uint8_t   byte_last[SNIFFER_BYTE_HISTORY];
    bool      byte_seen[SNIFFER_BYTE_HISTORY]; /**< false until first frame   */

    /* Counter byte detection */
    int8_t    counter_delta[SNIFFER_BYTE_HISTORY]; /**< +1,-1,0,or UNKNOWN    */
    bool      counter_candidate[SNIFFER_BYTE_HISTORY];

    bool      valid;                          /**< true = slot occupied        */
} sniffer_id_stats_t;

/* ── Sniffer config ─────────────────────────────────────────────────────── */

typedef struct {
    bool      csv_header;            /**< Print CSV header line at boot       */
    uint32_t  id_filter_min;         /**< Log only IDs >= this (default 0x000)*/
    uint32_t  id_filter_max;         /**< Log only IDs <= this (default 0x7FF)*/
    uint32_t  stats_interval_ms;     /**< How often to print stats (ms)       */
    uint16_t  max_unique_ids;        /**< Stop tracking new IDs above this    */
} sniffer_config_t;

/* ── Sniffer instance ───────────────────────────────────────────────────── */

typedef struct {
    sniffer_config_t   cfg;
    sniffer_id_stats_t id_stats[SNIFFER_MAX_UNIQUE_IDS];
    uint16_t           id_count;         /**< Unique IDs seen so far          */
    uint64_t           total_frames;     /**< All frames received             */
    uint32_t           dropped_frames;   /**< Ring buffer overflows           */
    bool               running;
} sniffer_t;

/* ── API ────────────────────────────────────────────────────────────────── */

/**
 * @brief Initialise sniffer with default config.
 *
 * Call before sniffer_load_config() or sniffer_start().
 */
void sniffer_init(sniffer_t *s);

/**
 * @brief Load sniffer config from /spiffs/system.json.
 *
 * Optional — falls back to defaults if file missing or key absent.
 * Must be called after SPIFFS is mounted (i.e. after signal_db_load or
 * explicit esp_vfs_spiffs_register).
 */
esp_err_t sniffer_load_config(sniffer_t *s, const char *json_path);

/**
 * @brief Start the sniffer.
 *
 * - Sets MCP2515 to LISTEN-ONLY mode (electrically passive)
 * - Configures hardware filters to accept all IDs
 * - Spawns RX task (notify+drain pattern, Core 0, prio TASK_PRIO_CAN_RX)
 * - Spawns stats task (Core 1, prio TASK_PRIO_APP)
 * - Prints CSV header to UART if cfg.csv_header == true
 *
 * @return ESP_OK on success.
 */
esp_err_t sniffer_start(sniffer_t *s);

/**
 * @brief Print accumulated per-ID statistics to UART.
 *
 * Called automatically by the stats task every cfg.stats_interval_ms.
 * Can also be called manually (e.g. on button press).
 */
void sniffer_print_stats(const sniffer_t *s);

#ifdef __cplusplus
}
#endif