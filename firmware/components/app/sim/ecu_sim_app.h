/**
 * @file ecu_sim_app.h
 * @brief Step 13 — Multi-ECU Vehicle Simulator (Node B)
 *
 * Node B impersonates a multi-ECU vehicle bus. Periodically broadcasts:
 *   ID 0x7E8 every 100 ms  — OBD-II RPM/Load response (mode 0x41)
 *   ID 0x0A6 every  10 ms  — Steering angle (proprietary)
 *   ID 0x1A0 every  10 ms  — Wheel speeds, 4-wheel (ABS)
 *   ID 0x3B0 every  20 ms  — Brake pressure (BCM)
 *
 * Also responds (request/response) to:
 *   OBD-II requests on 0x7DF (functional) and 0x7E0 (physical) → 0x7E8
 *   UDS requests to 0x740/0x736/0x745/0x743/0x731/0x755 → +0x08
 *
 * Error injection (configurable via JSON or time schedule):
 *   - TIMEOUT      : drop one response (no reply)
 *   - SLOW_FC      : delay multi-frame flow control by 500 ms
 *   - WRONG_DID    : corrupt one DID response payload
 *   - TX_FLOOD     : burst 50 frames in 50 ms (queue overflow test)
 *   - BUS_OFF      : force TEC to 250 (near bus-off) and recover
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "mcp2515.h"
#include "isotp.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ── Simulated signal values (live, mutable) ────────────────────────── */
typedef struct {
    uint16_t rpm;              /* 0–8000      */
    uint8_t  engine_load_pct;  /* 0–100       */
    int16_t  steering_deg;     /* -540..+540  */
    uint16_t wheel_fl_kmh;     /* 0–250 (×100 internal) */
    uint16_t wheel_fr_kmh;
    uint16_t wheel_rl_kmh;
    uint16_t wheel_rr_kmh;
    uint16_t brake_pressure_bar; /* 0–200     */
} ecu_sim_signals_t;

/* ── Error injection types ──────────────────────────────────────────── */
typedef enum {
    ECU_FAULT_NONE     = 0,
    ECU_FAULT_TIMEOUT,     /* drop next OBD/UDS response */
    ECU_FAULT_SLOW_FC,     /* 500 ms FC delay on next multi-frame */
    ECU_FAULT_WRONG_DID,   /* corrupt next DID response */
    ECU_FAULT_TX_FLOOD,    /* burst frames to overflow queue */
    ECU_FAULT_BUS_OFF,     /* force TEC near bus-off */
} ecu_fault_type_t;

/* ── Config ─────────────────────────────────────────────────────────── */
typedef struct {
    /* Broadcast periods (ms) — 0 disables that stream */
    uint32_t obd_period_ms;       /* default 100 */
    uint32_t steering_period_ms;  /* default  10 */
    uint32_t wheel_period_ms;     /* default  10 */
    uint32_t brake_period_ms;     /* default  20 */

    /* Initial signal values */
    ecu_sim_signals_t initial;

    /* Error injection schedule (seconds since boot, 0 disables) */
    uint32_t fault_timeout_at_s;
    uint32_t fault_slow_fc_at_s;
    uint32_t fault_wrong_did_at_s;
    uint32_t fault_tx_flood_at_s;
    uint32_t fault_bus_off_at_s;

    /* Toggle full simulator ECU map (true) or just broadcasts (false) */
    bool     enable_uds_responder;
} ecu_sim_config_t;

/* ── Runtime state ──────────────────────────────────────────────────── */
typedef struct {
    ecu_sim_config_t  cfg;
    ecu_sim_signals_t live;
    uint64_t          tx_count;
    uint64_t          rx_request_count;
    uint64_t          tx_drop_count;
    bool              running;
    isotp_ctx_t       uds_isotp;  /* ISO-TP context for UDS responses */
} ecu_sim_t;

/* ── API ────────────────────────────────────────────────────────────── */

/** Initialise with sane defaults. Call before load_config / start. */
void ecu_sim_init(ecu_sim_t *s);

/** Optional: load config from /spiffs/system.json (key "ecu_sim"). */
esp_err_t ecu_sim_load_config(ecu_sim_t *s, const char *json_path);

/** Set MCP2515 to NORMAL mode and spawn TX/RX/fault tasks. */
esp_err_t ecu_sim_start(ecu_sim_t *s);

/** Manually trigger a fault (used by fault scheduler internally too). */
void ecu_sim_inject_fault(ecu_sim_t *s, ecu_fault_type_t type);

#ifdef __cplusplus
}
#endif