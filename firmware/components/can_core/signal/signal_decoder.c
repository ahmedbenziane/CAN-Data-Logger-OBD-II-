/**
 * @file signal_decoder.c
 * @brief Signal Decoder implementation
 *
 * Bit extraction algorithm:
 *
 *   BIG-ENDIAN (Motorola):
 *     start_bit is the MSB position, numbered within the frame as:
 *       byte 0: bits  7..0
 *       byte 1: bits 15..8
 *       byte 2: bits 23..16 ...
 *     We extract `length` bits starting from start_bit downward.
 *     This matches the Renault/Bosch convention used by all ECUs in this car.
 *
 *   LITTLE-ENDIAN (Intel):
 *     start_bit is the LSB position, numbered linearly:
 *       bit 0 = byte[0] bit 0, bit 8 = byte[1] bit 0, etc.
 *     We extract `length` bits starting from start_bit upward.
 *
 *   Both paths produce a uint64_t raw value, which is then sign-extended
 *   (if VALUE_TYPE_SIGNED) and scaled with factor + offset.
 */

#include "signal_decoder.h"
#include "esp_log.h"
#include <string.h>
#include <math.h>
#include <stdio.h>

static const char *TAG = "SIG_DEC";

/* ── Bit extraction ─────────────────────────────────────────────────────── */

/**
 * Extract @p length bits from @p data starting at @p start_bit,
 * big-endian (Motorola) convention.
 *
 * In Motorola layout, start_bit is the MSB of the signal.
 * Bits are numbered: byte[0] contains bits 7..0, byte[1] bits 15..8, etc.
 *
 * We iterate from MSB to LSB, accumulating bits into result.
 */
static uint64_t _extract_big_endian(const uint8_t *data, uint8_t dlc,
                                     uint8_t start_bit, uint8_t length)
{
    uint64_t result = 0u;

    /* Convert start_bit (MSB in Motorola) to byte/bit coordinates.
     * In Motorola format: byte = start_bit / 8, bit = 7 - (start_bit % 8) */
    int cur_byte = (int)(start_bit / 8u);
    int cur_bit  = (int)(7u - (start_bit % 8u));   /* bit within byte, LSB=0 */

    for (uint8_t i = 0; i < length; i++) {
        if (cur_byte < 0 || cur_byte >= (int)dlc) {
            ESP_LOGW(TAG, "big-endian extraction OOB at byte %d (dlc=%u)",
                     cur_byte, dlc);
            break;
        }

        /* Extract this bit and shift into result */
        uint8_t bit_val = (data[cur_byte] >> cur_bit) & 0x01u;
        result = (result << 1u) | (uint64_t)bit_val;

        /* Advance to next bit: go right within byte, wrap to MSB of next byte */
        cur_bit--;
        if (cur_bit < 0) {
            cur_bit = 7;
            cur_byte++;
        }
    }

    return result;
}

/**
 * Extract @p length bits from @p data starting at @p start_bit,
 * little-endian (Intel) convention.
 *
 * start_bit is the LSB. Bits are numbered linearly:
 *   bit N → byte[N/8], bit position (N%8).
 */
static uint64_t _extract_little_endian(const uint8_t *data, uint8_t dlc,
                                        uint8_t start_bit, uint8_t length)
{
    uint64_t result = 0u;

    for (uint8_t i = 0; i < length; i++) {
        uint8_t abs_bit  = start_bit + i;
        uint8_t byte_idx = abs_bit / 8u;
        uint8_t bit_idx  = abs_bit % 8u;

        if (byte_idx >= dlc) {
            ESP_LOGW(TAG, "little-endian extraction OOB at byte %u (dlc=%u)",
                     byte_idx, dlc);
            break;
        }

        uint8_t bit_val = (data[byte_idx] >> bit_idx) & 0x01u;
        result |= ((uint64_t)bit_val << i);
    }

    return result;
}

/**
 * Sign-extend a @p length-bit unsigned value to int64_t.
 *
 * If the MSB of the raw value (bit length-1) is 1, the value is negative
 * in two's complement. We extend the sign bit to fill the full 64 bits.
 */
static int64_t _sign_extend(uint64_t raw, uint8_t length)
{
    if (length == 0u || length >= 64u) return (int64_t)raw;

    uint64_t sign_bit = 1ULL << (length - 1u);
    if (raw & sign_bit) {
        /* Set all bits above length to 1 */
        raw |= ~(sign_bit - 1u) & ~(sign_bit);
        raw |= ~((sign_bit << 1u) - 1u);
    }
    return (int64_t)raw;
}

/* ── Clamp helper ───────────────────────────────────────────────────────── */

static inline float _clampf(float v, float lo, float hi)
{
    if (lo == 0.0f && hi == 0.0f) return v;   /* 0/0 = no clamp defined */
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

/* ── Public API ─────────────────────────────────────────────────────────── */

decode_result_t signal_decode(const signal_def_t *def,
                               const uint8_t      *data,
                               uint8_t             dlc)
{
    decode_result_t res = { .value = 0.0f, .raw = 0, .valid = false };

    if (!def || !data) return res;
    if (!def->valid)   return res;

    /* String signals don't produce a float */
    if (def->value_type == VALUE_TYPE_STRING) {
        res.valid = true;
        res.value = 0.0f;
        return res;
    }

    /* Sanity: signal must fit within the payload */
    uint8_t bits_needed = def->start_bit + def->length;
    uint8_t bytes_needed = (bits_needed + 7u) / 8u;
    if (bytes_needed > dlc) {
        ESP_LOGW(TAG, "'%s': needs %u bytes, dlc=%u — skipping",
                 def->name, bytes_needed, dlc);
        return res;
    }

    if (def->length > 64u) {
        ESP_LOGE(TAG, "'%s': length %u > 64 bits — unsupported", def->name, def->length);
        return res;
    }

    /* ── Step 1: extract raw bits ──────────────────────────────────────── */
    uint64_t raw_u;
    if (def->byte_order == BYTE_ORDER_BIG_ENDIAN) {
        raw_u = _extract_big_endian(data, dlc, def->start_bit, def->length);
    } else {
        raw_u = _extract_little_endian(data, dlc, def->start_bit, def->length);
    }

    /* ── Step 2: sign extension ────────────────────────────────────────── */
    int64_t raw_s;
    if (def->value_type == VALUE_TYPE_SIGNED) {
        raw_s = _sign_extend(raw_u, def->length);
    } else {
        raw_s = (int64_t)raw_u;
    }
    res.raw = raw_s;

    /* ── Step 3: scale ─────────────────────────────────────────────────── */
    float physical = (float)raw_s * def->factor + def->offset;

    /* ── Step 4: clamp ─────────────────────────────────────────────────── */
    physical = _clampf(physical, def->min, def->max);

    res.value = physical;
    res.valid = true;
    return res;
}

bool signal_decode_str(const signal_def_t *def,
                        const uint8_t      *data,
                        uint8_t             dlc,
                        char               *out_str,
                        uint8_t             out_len)
{
    if (!def || !data || !out_str || out_len == 0) return false;
    if (def->value_type != VALUE_TYPE_STRING)       return false;

    uint8_t byte_start = def->start_bit / 8u;
    uint8_t byte_count = def->length    / 8u;

    if (byte_start + byte_count > dlc) {
        ESP_LOGW(TAG, "'%s' string: needs bytes %u–%u, dlc=%u",
                 def->name, byte_start, byte_start + byte_count, dlc);
        return false;
    }

    uint8_t copy = (byte_count < (uint8_t)(out_len - 1u))
                   ? byte_count
                   : (uint8_t)(out_len - 1u);

    for (uint8_t i = 0; i < copy; i++) {
        uint8_t b = data[byte_start + i];
        out_str[i] = (b >= 0x20u && b < 0x7Fu) ? (char)b : '.';
    }
    out_str[copy] = '\0';
    return true;
}

uint8_t signal_decode_frame(const signal_db_t  *db,
                              uint32_t            frame_id,
                              const uint8_t      *data,
                              uint8_t             dlc,
                              const signal_def_t **out_defs,
                              decode_result_t    *out_results,
                              uint8_t             max_out)
{
    if (!db || !data || !out_defs || !out_results || max_out == 0) return 0;

    /* Find all signals matching this frame ID */
    uint8_t n = signal_db_find_by_frame_id(db, frame_id,
                                            out_defs, max_out);

    /* Decode each one */
    for (uint8_t i = 0; i < n; i++) {
        out_results[i] = signal_decode(out_defs[i], data, dlc);
    }

    return n;
}

void signal_result_to_str(const decode_result_t *result,
                           const signal_def_t    *def,
                           char *buf, uint8_t len)
{
    if (!result || !def || !buf || len == 0) return;

    if (!result->valid) {
        snprintf(buf, len, "INVALID");
        return;
    }

    if (def->value_type == VALUE_TYPE_STRING) {
        snprintf(buf, len, "(string)");
        return;
    }

    /* Choose decimal places based on factor magnitude */
    int decimals = 2;
    if (def->factor >= 1.0f)   decimals = 1;
    if (def->factor >= 10.0f)  decimals = 0;
    if (def->factor < 0.01f)   decimals = 4;

    if (def->unit[0] != '\0') {
        snprintf(buf, len, "%.*f %s", decimals, (double)result->value, def->unit);
    } else {
        snprintf(buf, len, "%.*f", decimals, (double)result->value);
    }
}