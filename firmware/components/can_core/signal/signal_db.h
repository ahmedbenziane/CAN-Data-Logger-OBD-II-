/**
 * @file signal_db.h
 * @brief Signal Database — load and query CAN signal definitions from JSON
 *
 * Loads a JSON file from SPIFFS at runtime using cJSON (bundled with ESP-IDF).
 * Each signal definition describes how to extract a physical value from a
 * raw CAN frame: which bits to read, endianness, scaling, unit, etc.
 *
 * Memory model:
 *   All signal_def_t entries live in a single static pool (SIGNAL_DB_MAX_SIGNALS).
 *   No heap allocation after signal_db_load(). Safe for RTOS use.
 *
 * Typical usage:
 *   signal_db_t db;
 *   signal_db_init(&db);
 *   signal_db_load(&db, "/spiffs/renault_clio4.json");
 *   const signal_def_t *rpm = signal_db_find_by_name(&db, "engine_rpm");
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ── Limits ─────────────────────────────────────────────────────────────── */

#define SIGNAL_DB_MAX_SIGNALS   64u
#define SIGNAL_NAME_LEN         32u
#define SIGNAL_DESC_LEN         64u
#define SIGNAL_UNIT_LEN         16u
#define SIGNAL_ECU_LEN          24u

/* ── Enumerations ───────────────────────────────────────────────────────── */

typedef enum {
    BYTE_ORDER_BIG_ENDIAN    = 0,   /**< Motorola / big-endian (MSB first) */
    BYTE_ORDER_LITTLE_ENDIAN = 1,   /**< Intel / little-endian (LSB first) */
} signal_byte_order_t;

typedef enum {
    VALUE_TYPE_UNSIGNED = 0,        /**< Unsigned integer raw value        */
    VALUE_TYPE_SIGNED   = 1,        /**< Two's complement signed integer   */
    VALUE_TYPE_STRING   = 2,        /**< Raw bytes as ASCII string (DIDs)  */
} signal_value_type_t;

/* ── Signal definition ──────────────────────────────────────────────────── */

/**
 * @brief Complete definition of one CAN signal.
 *
 * Describes how to extract a physical value from a raw frame payload.
 * All fields are populated by signal_db_load() from the JSON file.
 */
typedef struct {
    char                 name[SIGNAL_NAME_LEN];   /**< Unique signal name  */
    char                 desc[SIGNAL_DESC_LEN];   /**< Human description   */
    char                 unit[SIGNAL_UNIT_LEN];   /**< Physical unit       */
    char                 ecu[SIGNAL_ECU_LEN];     /**< Source ECU name     */

    uint32_t             frame_id;    /**< CAN frame ID this signal lives in*/
    uint16_t             pid;         /**< OBD PID (0 if UDS DID)          */
    uint16_t             did;         /**< UDS DID (0 if OBD PID)          */
    uint8_t              service;     /**< 0x01=OBD, 0x22=UDS ReadDID      */

    uint8_t              start_bit;   /**< Bit offset in payload (0=MSB)   */
    uint8_t              length;      /**< Signal width in bits (1–64)     */

    signal_byte_order_t  byte_order;  /**< Big or little endian            */
    signal_value_type_t  value_type;  /**< Unsigned / signed / string      */

    float                factor;      /**< physical = raw * factor + offset */
    float                offset;
    float                min;         /**< Clamp floor (after scaling)     */
    float                max;         /**< Clamp ceiling (after scaling)   */

    bool                 valid;       /**< true = slot in use              */
} signal_def_t;

/* ── Database ───────────────────────────────────────────────────────────── */

typedef struct {
    signal_def_t  signals[SIGNAL_DB_MAX_SIGNALS];
    uint16_t      count;              /**< Number of loaded signals        */
    char          vehicle[64];        /**< Vehicle name from JSON          */
    char          vin[20];            /**< VIN from JSON                   */
    uint8_t       version;            /**< DB version field                */
    bool          loaded;             /**< true after successful load      */
} signal_db_t;

/* ── API ────────────────────────────────────────────────────────────────── */

/**
 * @brief Zero-initialise the database. Call before signal_db_load().
 */
void signal_db_init(signal_db_t *db);

/**
 * @brief Load signal definitions from a JSON file on SPIFFS.
 *
 * Mounts SPIFFS (base_path="/spiffs") if not already mounted, reads the
 * file, parses it with cJSON, and populates the signal pool.
 *
 * @param db        Database instance (must be initialised with signal_db_init).
 * @param json_path SPIFFS path, e.g. "/spiffs/renault_clio4.json".
 * @return ESP_OK on success.
 * @return ESP_ERR_NOT_FOUND if file does not exist.
 * @return ESP_ERR_NO_MEM if signal pool is full.
 * @return ESP_FAIL on JSON parse error.
 */
esp_err_t signal_db_load(signal_db_t *db, const char *json_path);

/**
 * @brief Find a signal definition by exact name.
 *
 * @return Pointer to signal_def_t, or NULL if not found.
 */
const signal_def_t *signal_db_find_by_name(const signal_db_t *db,
                                            const char *name);

/**
 * @brief Find all signals that belong to a given CAN frame ID.
 *
 * Fills @p out_defs with up to @p max_out matching pointers.
 *
 * @return Number of matches written into out_defs.
 */
uint8_t signal_db_find_by_frame_id(const signal_db_t *db,
                                    uint32_t frame_id,
                                    const signal_def_t **out_defs,
                                    uint8_t max_out);

/**
 * @brief Find a signal by OBD PID.
 * @return Pointer to signal_def_t, or NULL if not found.
 */
const signal_def_t *signal_db_find_by_pid(const signal_db_t *db,
                                           uint16_t pid);

/**
 * @brief Find a signal by UDS DID.
 * @return Pointer to signal_def_t, or NULL if not found.
 */
const signal_def_t *signal_db_find_by_did(const signal_db_t *db,
                                           uint16_t did);

/**
 * @brief Print the entire loaded database to UART (ESP_LOGI).
 */
void signal_db_print(const signal_db_t *db);

#ifdef __cplusplus
}
#endif