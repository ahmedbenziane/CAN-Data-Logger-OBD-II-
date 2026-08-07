/**
 * @file signal_decoder.h
 * @brief Signal Decoder — extract physical values from raw CAN frame bytes
 *
 * The decoder is stateless. It takes a signal_def_t (from the DB) and a
 * raw payload buffer and returns a float physical value.
 *
 * Decoding pipeline:
 *   raw bytes → bit extraction → endian normalisation →
 *   sign extension → scale (factor + offset) → clamp (min/max) → float
 *
 * Supported:
 *   - Big-endian (Motorola): bits numbered MSB-first, common in Renault ECUs
 *   - Little-endian (Intel): bits numbered LSB-first
 *   - Signed two's complement (e.g. fuel trim, timing advance)
 *   - Unsigned (most OBD PIDs)
 *   - String signals (DID reads) — returned as 0.0f, use signal_decode_str()
 *   - Signal widths 1–64 bits
 *
 * Key formula:
 *   physical = clamp(raw_integer * factor + offset, min, max)
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "signal_db.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ── Result type ────────────────────────────────────────────────────────── */

/**
 * @brief Decode result — physical value plus validity flag.
 *
 * Check .valid before using .value. A decode can fail if:
 *   - def is NULL
 *   - dlc is too small to contain the signal
 *   - signal length > 64 bits
 */
typedef struct {
    float   value;   /**< Physical value after factor + offset + clamp    */
    int64_t raw;     /**< Raw integer before scaling (for diagnostics)    */
    bool    valid;   /**< false = extraction failed, value is undefined   */
} decode_result_t;

/* ── API ────────────────────────────────────────────────────────────────── */

/**
 * @brief Decode one signal from a raw CAN payload.
 *
 * This is the primary entry point. Handles all endianness and sign modes.
 *
 * @param def   Signal definition from signal_db (must not be NULL).
 * @param data  Raw CAN frame payload bytes.
 * @param dlc   Number of valid bytes in data (0–8 for standard CAN).
 * @return      decode_result_t with physical value and validity flag.
 *
 * Example — OBD RPM (PID 0x0C, big-endian 16-bit, factor=0.25):
 *   data = {0x1A, 0xF0}  →  raw = 0x1AF0 = 6896  →  6896 * 0.25 = 1724 rpm
 */
decode_result_t signal_decode(const signal_def_t *def,
                               const uint8_t      *data,
                               uint8_t             dlc);

/**
 * @brief Decode a string signal (VALUE_TYPE_STRING) into a char buffer.
 *
 * Copies raw bytes as printable ASCII. Non-printable bytes are replaced
 * with '.'. Always null-terminates out_str.
 *
 * @param def      Signal definition (value_type must be VALUE_TYPE_STRING).
 * @param data     Raw CAN/ISO-TP payload.
 * @param dlc      Payload length.
 * @param out_str  Output buffer to receive the decoded string.
 * @param out_len  Size of out_str (including null terminator).
 * @return true on success, false if def is wrong type or dlc too small.
 */
bool signal_decode_str(const signal_def_t *def,
                        const uint8_t      *data,
                        uint8_t             dlc,
                        char               *out_str,
                        uint8_t             out_len);

/**
 * @brief Decode all signals matching a given frame ID from the database.
 *
 * Convenience function — looks up all signals for frame_id and decodes each.
 * Results are written into out_results alongside corresponding signal defs.
 *
 * @param db          Signal database (must be loaded).
 * @param frame_id    CAN frame ID.
 * @param data        Raw frame payload.
 * @param dlc         Payload length.
 * @param out_defs    Array to receive matching signal_def_t pointers.
 * @param out_results Array to receive decode_result_t for each signal.
 * @param max_out     Size of both output arrays.
 * @return            Number of signals decoded.
 */
uint8_t signal_decode_frame(const signal_db_t  *db,
                              uint32_t            frame_id,
                              const uint8_t      *data,
                              uint8_t             dlc,
                              const signal_def_t **out_defs,
                              decode_result_t    *out_results,
                              uint8_t             max_out);

/**
 * @brief Format a decode_result_t as a printable string.
 *
 * Writes "1724.00 rpm" style output into buf.
 *
 * @param result  Decode result.
 * @param def     Signal definition (for unit string).
 * @param buf     Output buffer.
 * @param len     Buffer size.
 */
void signal_result_to_str(const decode_result_t *result,
                           const signal_def_t    *def,
                           char *buf, uint8_t len);

#ifdef __cplusplus
}
#endif