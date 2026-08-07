#pragma once
/* ============================================================
 *  obd.h  —  Step 6: OBD-II Request/Response Engine
 *
 *  Implements OBD-II Service 01 (Show Current Data) over ISO-TP.
 *  Talks to a real ECU via 0x7DF (functional request broadcast).
 *  Receives response on 0x7E8 (engine ECU).
 *
 *  Supported PIDs (all confirmed on Renault Clio IV):
 *    0x0C  Engine RPM          (uint16, rpm = raw / 4)
 *    0x0D  Vehicle Speed       (uint8,  km/h = raw)
 *    0x04  Engine Load         (uint8,  % = raw * 100 / 255)
 *    0x05  Coolant Temperature (uint8,  °C = raw - 40)
 *    0x11  Throttle Position   (uint8,  % = raw * 100 / 255)
 *    0x0F  Intake Air Temp     (uint8,  °C = raw - 40)
 *
 *  Request format (Service 01):
 *    [0x02, 0x01, PID, 0x00, 0x00, 0x00, 0x00, 0x00]
 *    Byte 0: length = 2
 *    Byte 1: service = 0x01
 *    Byte 2: PID
 *
 *  Response format:
 *    [len, 0x41, PID, data_byte_A, data_byte_B?, ...]
 *    Byte 1: 0x41 = positive response to service 0x01
 *    Byte 2: echoed PID
 *    Bytes 3+: data (1-4 bytes depending on PID)
 * ============================================================ */

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "isotp.h"

/* ── PID definitions ─────────────────────────────────────── */
#define OBD_PID_ENGINE_LOAD 0x04
#define OBD_PID_COOLANT_TEMP 0x05
#define OBD_PID_INTAKE_TEMP 0x0F
#define OBD_PID_RPM 0x0C
#define OBD_PID_SPEED 0x0D
#define OBD_PID_THROTTLE 0x11
#define OBD_PID_SUPPORTED_01_20 0x00 /* bitmask of supported PIDs */

/* ── OBD service byte ─────────────────────────────────────── */
#define OBD_SERVICE_01 0x01
#define OBD_POSITIVE_RESPONSE 0x41 /* 0x01 + 0x40 */

/* ── CAN IDs ──────────────────────────────────────────────── */
#define OBD_REQ_ID 0x7E0  /* functional broadcast — all ECUs listen */
#define OBD_RESP_ID 0x7E8 /* engine ECU response                    */

/* ── Timeout ─────────────────────────────────────────────── */
#define OBD_RESPONSE_TIMEOUT_MS 500

/* ── Decoded signal ─────────────────────────────────────── */
typedef struct
{
    uint8_t pid;
    float value;
    char unit[8];
    bool valid;
    int64_t timestamp_us;
} obd_signal_t;

/* ── Engine: one context per ECU channel ─────────────────── */
typedef struct
{
    isotp_ctx_t isotp;            /* underlying ISO-TP channel    */
    volatile bool response_ready; /* written Core 0, read Core 1  */
    uint8_t response_buf[32];
    size_t response_len;
    uint32_t req_count;
    uint32_t ok_count;
    uint32_t err_count;
    uint32_t timeout_count;
} obd_engine_t;

/* ── API ─────────────────────────────────────────────────── */

/**
 * @brief  Initialise OBD engine (wraps isotp_init).
 */
void obd_init(obd_engine_t *eng, isotp_tx_fn_t tx_fn);

/**
 * @brief  Send one OBD-II request and wait for response.
 *         Blocking up to OBD_RESPONSE_TIMEOUT_MS.
 *
 * @param eng   Engine context
 * @param pid   OBD-II PID (e.g. OBD_PID_RPM)
 * @param out   Decoded signal output
 * @return ESP_OK on success, ESP_ERR_TIMEOUT, ESP_FAIL
 */
esp_err_t obd_request(obd_engine_t *eng, uint8_t pid, obd_signal_t *out);

/**
 * @brief  Feed an incoming CAN frame into the OBD engine.
 *         Call from your RX task for frames with ID == OBD_RESP_ID.
 */
void obd_process_frame(obd_engine_t *eng, const can_frame_t *frame);

/**
 * @brief  Print current signal value via ESP_LOGI.
 */
void obd_print_signal(const obd_signal_t *sig, const char *tag);

/**
 * @brief  Print engine statistics.
 */
void obd_print_stats(const obd_engine_t *eng, const char *tag);