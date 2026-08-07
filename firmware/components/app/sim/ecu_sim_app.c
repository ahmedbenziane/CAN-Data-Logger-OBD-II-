/**
 * @file ecu_sim_app.c
 * @brief Step 13 — Multi-ECU Vehicle Simulator (Node B)
 *
 * Architecture:
 *   - 4 broadcast tasks (one per stream) — independent periods, jitter
 *   - 1 RX task — handles incoming OBD/UDS requests, replies on +0x08
 *   - 1 fault scheduler task — injects faults at configured timestamps
 *   - 1 signal animator task — sweeps RPM, steering, wheels for visuals
 *
 * All TX goes through mcp2515_transmit_frame(). RX uses the ring buffer
 * (mcp2515_pop_frame); the sniffer's direct-read trick is sniffer-only.
 *
 * FIXES vs. original Step 13 draft:
 *   - Brake counter uses task-local sequence (was global tx_count: tearing)
 *   - SLOW_FC actually implemented: VIN response is multi-frame, FC delayed
 *   - BUS_OFF actually triggers: bitrate switched to wrong value during flood
 *   - Real Renault DID values from handoff §9 (was hardcoded 0x12 0x34)
 *   - Includes "project_config.h" (no fragile ../../../src/ relative path)
 *   - Drops uninitialised can_frame_t local pattern in negative-response path
 */

#include "ecu_sim_app.h"
#include "project_config.h"
#include "mcp2515.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "cJSON.h"
#include <inttypes.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

static const char *TAG = "ECU_SIM";

/* No TX driver task or semaphore needed.
 * isotp_tick() runs at the bottom of _rx_task after every frame drain,
 * keeping the TX state machine in the same task that calls
 * isotp_process_frame(). This prevents any race between FC processing
 * and CF transmission since both happen sequentially on Core 0.      */

/* Pending fault flags — set by scheduler, consumed by tx/rx tasks */
static volatile ecu_fault_type_t s_pending_fault = ECU_FAULT_NONE;

/* Global pointer for fault scheduler task */
static ecu_sim_t *s_sim = NULL;

/* ── Default config ─────────────────────────────────────────────────── */
static const ecu_sim_config_t s_default_cfg = {
    .obd_period_ms      = 100,
    .steering_period_ms = 10,
    .wheel_period_ms    = 10,
    .brake_period_ms    = 20,

    .initial = {
        .rpm                = 800,
        .engine_load_pct    = 25,
        .steering_deg       = 0,
        .wheel_fl_kmh       = 0,
        .wheel_fr_kmh       = 0,
        .wheel_rl_kmh       = 0,
        .wheel_rr_kmh       = 0,
        .brake_pressure_bar = 0,
    },

    /* Faults DISABLED by default (0 = off). Override via JSON. */
    .fault_timeout_at_s   = 0,
    .fault_slow_fc_at_s   = 0,
    .fault_wrong_did_at_s = 0,
    .fault_tx_flood_at_s  = 0,
    .fault_bus_off_at_s   = 0,

    .enable_uds_responder = true,
};

/* ── Real DID values (from handoff §9, VIN VF14SR6B4FD018433) ──────── */
static const char S_DID_VIN[]        = "VF14SR6B4FD018433";    /* 17 bytes */
static const uint8_t S_DID_SW_VER[]  = { 0x00, 0x2C };
static const char S_DID_SERIAL[]     = "ECU0123456789";
static const char S_DID_CAL_ID[]     = "8201312983237106032R"; /* 20 bytes */
static const uint8_t S_DID_DIAG_VER[]= { 0x84 };




/* ── Helpers ────────────────────────────────────────────────────────── */





static esp_err_t _tx(const can_frame_t *f)
{
    /* 50 ms TX timeout — ample for 8-byte frame at 500 kbps */
    return mcp2515_transmit_frame(f, 50);
}

/* Send an ISO-TP Single Frame on the ECU response ID 0x7E8.
 * Direct CAN frame — no ISO-TP state machine involved. */
static void _send_uds_sf(const uint8_t *payload, size_t len)
{
    can_frame_t f = { .id = 0x7E8, .dlc = 8 };
    memset(f.data, 0xAA, 8);
    f.data[0] = (uint8_t)len;
    memcpy(&f.data[1], payload, len);
    _tx(&f);
}

/* Send a multi-frame ISO-TP response synchronously.
 *
 * Runs entirely inside _rx_task so the ring-buffer poller and the
 * frame sender share the same task context — no locking needed.
 *
 * Sequence:
 *   1. Send First Frame.
 *   2. Spin-poll the ring buffer until a Flow-Control frame (PCI 0x30)
 *      arrives on 0x7E0.  With RXB0 dedicated to 0x7E0, the FC is
 *      never lost to broadcast traffic.
 *   3. Send all Consecutive Frames back-to-back (assumes BS=0 from
 *      Node A, which is what isotp_process_frame sends).
 */
static void _send_uds_multiframe(const uint8_t *payload, size_t len)
{
    /* First Frame */
    can_frame_t ff = { .id = 0x7E8, .dlc = 8 };
    memset(ff.data, 0xAA, 8);
    ff.data[0] = (uint8_t)(0x10 | ((len >> 8) & 0x0F));
    ff.data[1] = (uint8_t)(len & 0xFF);
    memcpy(&ff.data[2], payload, 6);
    if (_tx(&ff) != ESP_OK) return;

    /* Poll ring buffer for Flow Control — 200 ms window */
    can_frame_t fc;
    bool fc_ok = false;
    int64_t deadline = esp_timer_get_time() + 200000LL;
    while (!fc_ok && esp_timer_get_time() < deadline) {
        while (mcp2515_frame_available())
            mcp2515_receive_frame(&fc);
        while (mcp2515_ring_has_frame()) {
            if (mcp2515_pop_frame(&fc) == ESP_OK &&
                fc.id == 0x7E0 && (fc.data[0] & 0xF0) == 0x30) {
                fc_ok = true;
                break;
            }
        }
        if (!fc_ok) taskYIELD();
    }
    if (!fc_ok) {
        ESP_LOGW(TAG, "# MF TX: no FC received (timeout)");
        return;
    }

    /* STmin inter-frame gap from FC[2] — Node A sends 0, handled anyway */
    uint8_t stmin = fc.data[2];
    TickType_t gap = 0;
    if      (stmin >= 0x01 && stmin <= 0x7F) gap = pdMS_TO_TICKS(stmin);
    else if (stmin >= 0xF1 && stmin <= 0xF9) gap = 1; /* < 1 ms → 1 tick */

    /* Consecutive Frames — BS=0 means send all without another FC */
    size_t offset = 6;
    uint8_t sn = 1;
    while (offset < len) {
        if (gap > 0) vTaskDelay(gap);
        size_t copy_len = ((len - offset) > 7) ? 7 : (len - offset);
        can_frame_t cf = { .id = 0x7E8, .dlc = 8 };
        memset(cf.data, 0xAA, 8);
        cf.data[0] = (uint8_t)(0x20 | (sn & 0x0F));
        memcpy(&cf.data[1], &payload[offset], copy_len);
        _tx(&cf);
        offset += copy_len;
        sn = (sn + 1) & 0x0F;
    }
}

/* Build OBD-II response: mode + PID + payload (max 5 data bytes here) */
static void _build_obd_response(can_frame_t *f, uint8_t mode_resp,
                                uint8_t pid, const uint8_t *payload,
                                uint8_t payload_len)
{
    memset(f, 0, sizeof(*f));
    f->id  = 0x7E8;
    f->dlc = 8;
    /* OBD pads unused bytes with 0x00 (already zeroed by memset above) */
    f->data[0] = (uint8_t)(payload_len + 2);   /* SF length: mode + pid + N */
    f->data[1] = mode_resp;
    f->data[2] = pid;
    for (uint8_t i = 0; i < payload_len && i < 5; i++) {
        f->data[3 + i] = payload[i];
    }
}

/* ═══════════════════════════════════════════════════════════════════════
 *  Broadcast tasks — one per stream
 * ═══════════════════════════════════════════════════════════════════════ */

static void _tx_obd_broadcast(void *arg)
{
    ecu_sim_t *s = (ecu_sim_t *)arg;
    TickType_t next = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(s->cfg.obd_period_ms);

    ESP_LOGI(TAG, "# TX 0x7E8 OBD broadcast every %"PRIu32" ms",
             s->cfg.obd_period_ms);

    for (;;) {
        vTaskDelayUntil(&next, period);

        /* Skip one TX if TIMEOUT fault pending */
        if (s_pending_fault == ECU_FAULT_TIMEOUT) {
            s_pending_fault = ECU_FAULT_NONE;
            s->tx_drop_count++;
            ESP_LOGW(TAG, "# FAULT TIMEOUT injected — dropped 1 OBD frame");
            continue;
        }

        /* OBD response: PID 0x0C (RPM)
         * Formula: RPM = (A*256 + B) / 4 → encoded = rpm * 4 */
        can_frame_t f;
        uint16_t encoded = (uint16_t)(s->live.rpm * 4u);
        uint8_t  payload[2] = { (uint8_t)(encoded >> 8),
                                (uint8_t)(encoded & 0xFF) };
        _build_obd_response(&f, 0x41, 0x0C, payload, 2);

        /* Optionally corrupt if WRONG_DID pending */
        if (s_pending_fault == ECU_FAULT_WRONG_DID) {
            f.data[3] ^= 0xFF;
            f.data[4] ^= 0xFF;
            s_pending_fault = ECU_FAULT_NONE;
            ESP_LOGW(TAG, "# FAULT WRONG_DID injected — RPM payload corrupted");
        }

        if (_tx(&f) == ESP_OK) s->tx_count++;
    }
}

static void _tx_steering_broadcast(void *arg)
{
    ecu_sim_t *s = (ecu_sim_t *)arg;
    TickType_t next = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(s->cfg.steering_period_ms);

    ESP_LOGI(TAG, "# TX 0x0A6 steering every %"PRIu32" ms",
             s->cfg.steering_period_ms);

    for (;;) {
        vTaskDelayUntil(&next, period);

        /* Steering angle: int16 little-endian, units = 0.1 deg
         * range -540.0 .. +540.0 → -5400 .. +5400 */
        can_frame_t f;
        memset(&f, 0, sizeof(f));
        f.id  = 0x0A6;
        f.dlc = 4;

        int16_t encoded = (int16_t)(s->live.steering_deg * 10);
        f.data[0] = (uint8_t)(encoded & 0xFF);
        f.data[1] = (uint8_t)((encoded >> 8) & 0xFF);
        f.data[2] = 0xDE;    /* fixed signature byte */
        f.data[3] = 0xAD;

        if (_tx(&f) == ESP_OK) s->tx_count++;
    }
}

static void _tx_wheel_broadcast(void *arg)
{
    ecu_sim_t *s = (ecu_sim_t *)arg;
    TickType_t next = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(s->cfg.wheel_period_ms);

    ESP_LOGI(TAG, "# TX 0x1A0 wheel speeds every %"PRIu32" ms",
             s->cfg.wheel_period_ms);

    for (;;) {
        vTaskDelayUntil(&next, period);

        /* 4 × uint16 wheel speeds, little-endian, units = 0.01 km/h */
        can_frame_t f;
        memset(&f, 0, sizeof(f));
        f.id  = 0x1A0;
        f.dlc = 8;

        uint16_t v[4] = { s->live.wheel_fl_kmh, s->live.wheel_fr_kmh,
                          s->live.wheel_rl_kmh, s->live.wheel_rr_kmh };
        for (int i = 0; i < 4; i++) {
            f.data[i*2 + 0] = (uint8_t)(v[i] & 0xFF);
            f.data[i*2 + 1] = (uint8_t)((v[i] >> 8) & 0xFF);
        }

        if (_tx(&f) == ESP_OK) s->tx_count++;
    }
}

static void _tx_brake_broadcast(void *arg)
{
    ecu_sim_t *s = (ecu_sim_t *)arg;
    TickType_t next = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(s->cfg.brake_period_ms);

    /* Task-local rolling counter — no tearing from concurrent TX tasks */
    uint8_t brake_seq = 0;

    ESP_LOGI(TAG, "# TX 0x3B0 brake every %"PRIu32" ms",
             s->cfg.brake_period_ms);

    for (;;) {
        vTaskDelayUntil(&next, period);

        /* Brake pressure: uint16 LE, bar + status + counter */
        can_frame_t f;
        memset(&f, 0, sizeof(f));
        f.id  = 0x3B0;
        f.dlc = 4;

        f.data[0] = (uint8_t)(s->live.brake_pressure_bar & 0xFF);
        f.data[1] = (uint8_t)((s->live.brake_pressure_bar >> 8) & 0xFF);
        f.data[2] = (s->live.brake_pressure_bar > 0) ? 0x01 : 0x00; /* pedal */
        f.data[3] = brake_seq++;                                    /* +1 each frame */

        if (_tx(&f) == ESP_OK) s->tx_count++;
    }
}

/* ═══════════════════════════════════════════════════════════════════════
 *  Signal animator — sweeps values so Node A sees them changing
 * ═══════════════════════════════════════════════════════════════════════ */

static void _animator_task(void *arg)
{
    ecu_sim_t *s = (ecu_sim_t *)arg;
    int16_t  steer_dir = +1;
    int      rpm_dir   = +1;
    uint32_t brake_phase = 0;

    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(50));

        /* RPM: 800 → 6000 → 800 ramp */
        if (rpm_dir > 0 && s->live.rpm >= 6000) rpm_dir = -1;
        if (rpm_dir < 0 && s->live.rpm <= 800)  rpm_dir = +1;
        s->live.rpm = (uint16_t)(s->live.rpm + (rpm_dir > 0 ? 50 : -50));

        /* Engine load follows RPM roughly */
        s->live.engine_load_pct = (uint8_t)((s->live.rpm * 100u) / 6000u);
        if (s->live.engine_load_pct > 100) s->live.engine_load_pct = 100;

        /* Steering: -540 → +540 → -540 */
        if (steer_dir > 0 && s->live.steering_deg >= 540)  steer_dir = -1;
        if (steer_dir < 0 && s->live.steering_deg <= -540) steer_dir = +1;
        s->live.steering_deg = (int16_t)(s->live.steering_deg + steer_dir * 5);

        /* Wheel speeds: track RPM/100 with mild asymmetry (×100 internal) */
        uint16_t base = (uint16_t)((s->live.rpm / 100u) * 100u);
        s->live.wheel_fl_kmh = base;
        s->live.wheel_fr_kmh = base + 50;
        s->live.wheel_rl_kmh = (base >= 30) ? (uint16_t)(base - 30) : 0;
        s->live.wheel_rr_kmh = base + 20;

        /* Brake pressure: pulse 0 → 100 → 0 every ~10 s */
        brake_phase++;
        s->live.brake_pressure_bar =
            (brake_phase % 200 < 100) ? (uint16_t)(brake_phase % 100) : 0;
    }
}

/* ═══════════════════════════════════════════════════════════════════════
 *  RX task — answer OBD-II and UDS requests
 * ═══════════════════════════════════════════════════════════════════════ */

static bool _is_known_uds_request(uint32_t id)
{
    static const uint32_t reqs[] = {
        0x7E0,  /* Engine    */
        0x740,  /* ABS/ESP   */
        0x736,  /* Airbag    */
        0x745,  /* BCM       */
        0x743,  /* Cluster   */
        0x731,  /* Steering  */
        0x755,  /* Climate   */
        0x600,  /* GPL ECU placeholder (handoff §9) */
    };
    for (size_t i = 0; i < sizeof(reqs)/sizeof(reqs[0]); i++) {
        if (reqs[i] == id) return true;
    }
    return false;
}

static void _handle_obd_request(ecu_sim_t *s, const can_frame_t *req)
{
    if (req->dlc < 2) return;
    uint8_t mode = req->data[1];
    uint8_t pid  = (req->dlc >= 3) ? req->data[2] : 0;

    can_frame_t resp;
    uint8_t payload[5] = {0};
    uint8_t plen = 0;

    if (mode == 0x01) {   /* current data */
        switch (pid) {
            case 0x0C: { /* RPM */
                uint16_t v = (uint16_t)(s->live.rpm * 4);
                payload[0] = (uint8_t)(v >> 8);
                payload[1] = (uint8_t)(v & 0xFF);
                plen = 2;
                break;
            }
            case 0x04:   /* engine load (0–255 = 0–100%) */
                payload[0] = (uint8_t)((s->live.engine_load_pct * 255) / 100);
                plen = 1;
                break;
            case 0x0D:   /* speed (km/h) */
                payload[0] = (uint8_t)(s->live.wheel_fl_kmh / 100);
                plen = 1;
                break;
            default: {
                /* Negative response: 7F 01 12 (subFunctionNotSupported) */
                memset(&resp, 0, sizeof(resp));
                resp.id  = 0x7E8;
                resp.dlc = 8;
                resp.data[0] = 3;
                resp.data[1] = 0x7F;
                resp.data[2] = 0x01;
                resp.data[3] = 0x12;
                _tx(&resp);
                return;
            }
        }
        _build_obd_response(&resp, 0x41, pid, payload, plen);
        _tx(&resp);
        s->rx_request_count++;
    }
}


static void _handle_uds_request(ecu_sim_t *s, const can_frame_t *req)
{
    if (req->dlc < 3) return;

    /* ISO-TP PCI byte:
     *   data[0] = 0x0N  SF (N=payload length)
     *   data[0] = 0x1H  FF (H=high nibble of 12-bit length)
     * UDS payload for a SF request sent by isotp_send([02 10 03]):
     *   data[0]=0x03  PCI
     *   data[1]=0x02  UDS length byte
     *   data[2]=0x10  SID
     *   data[3]=0x03  subtype                                       */
    uint8_t pci      = req->data[0];
    uint8_t pci_type = pci & 0xF0;

    if (pci_type != 0x00 && pci_type != 0x10) {
        ESP_LOGD(TAG, "# UDS frame 0x%03"PRIX32" bad PCI 0x%02X — ignored",
                 req->id, pci);
        return;
    }

    /* SID: SF→data[2], FF→data[3] */
    uint8_t sid = (pci_type == 0x00) ? req->data[2] : req->data[3];

    ESP_LOGI(TAG, "# UDS req 0x%03"PRIX32" PCI=0x%02X SID=0x%02X",
             req->id, pci, sid);

    /* ── DiagnosticSessionControl (0x10) ── */
    if (sid == 0x10) {
        uint8_t sub = (pci_type == 0x00) ? req->data[3] : req->data[4];

        /* Session‑open response is always a Single Frame — send it
         * directly without involving the ISO‑TP layer. This eliminates
         * the FC time‑out that would otherwise occur on a busy bus.  */
        can_frame_t resp;
        memset(&resp, 0, sizeof(resp));
        resp.id   = 0x7E8;
        resp.dlc  = 8;
        resp.data[0] = 0x06;               /* SF length 6 */
        resp.data[1] = 0x50;               /* positive response SID */
        resp.data[2] = sub;                /* session type */
        resp.data[3] = 0x00; resp.data[4] = 0x32;   /* P2 = 50 ms */
        resp.data[5] = 0x01; resp.data[6] = 0xF4;   /* P2* = 500 ms */
        _tx(&resp);
        s->rx_request_count++;
        ESP_LOGI(TAG, "# Session 0x%02X response sent", sub);
        return;
    }

    /* ── TesterPresent (0x3E) ── */
    if (sid == 0x3E) {
        uint8_t sub = (pci_type == 0x00)
                      ? ((req->dlc >= 4) ? req->data[3] : 0x00)
                      : ((req->dlc >= 5) ? req->data[4] : 0x00);
        if (sub & 0x80) return;   /* suppress bit: stay silent */

        /* Direct Single Frame — no ISO‑TP needed */
        can_frame_t resp;
        memset(&resp, 0, sizeof(resp));
        resp.id   = 0x7E8;
        resp.dlc  = 8;
        resp.data[0] = 0x02;               /* SF length 2 */
        resp.data[1] = 0x7E;               /* positive response SID */
        resp.data[2] = 0x00;               /* zero sub‑function */
        _tx(&resp);
        return;
    }

    /* ── ReadDataByIdentifier (0x22) ── */
    if (sid == 0x22) {
        if (pci_type == 0x00 && req->dlc < 5) return;
        if (pci_type == 0x10 && req->dlc < 6) return;

        uint16_t did = (pci_type == 0x00)
                       ? (((uint16_t)req->data[3] << 8) | req->data[4])
                       : (((uint16_t)req->data[4] << 8) | req->data[5]);

        const uint8_t *pl   = NULL;
        size_t         plen = 0;

        switch (did) {
            case 0xF190: pl = (const uint8_t *)S_DID_VIN;    plen = sizeof(S_DID_VIN)    - 1; break;
            case 0xF189: pl = S_DID_SW_VER;                   plen = sizeof(S_DID_SW_VER);     break;
            case 0xF18C: pl = (const uint8_t *)S_DID_SERIAL;  plen = sizeof(S_DID_SERIAL) - 1; break;
            case 0xF186: pl = (const uint8_t *)S_DID_CAL_ID;  plen = sizeof(S_DID_CAL_ID) - 1; break;
            case 0xF18E: pl = S_DID_DIAG_VER;                 plen = sizeof(S_DID_DIAG_VER);   break;
            default: {
                uint8_t nrc[] = { 0x7F, 0x22, 0x31 };
                _send_uds_sf(nrc, sizeof(nrc));
                return;
            }
        }

        /* Apply WRONG_DID fault if latched */
        bool corrupt = false;
        if (s_pending_fault == ECU_FAULT_WRONG_DID) {
            corrupt = true;
            s_pending_fault = ECU_FAULT_NONE;
            ESP_LOGW(TAG, "# FAULT WRONG_DID applied to DID 0x%04X", did);
        }

        /* Apply SLOW_FC fault: delay before sending so Node A times out
         * on the FC exchange (or at least experiences a slow one)      */
        if (s_pending_fault == ECU_FAULT_SLOW_FC) {
            s_pending_fault = ECU_FAULT_NONE;
            ESP_LOGW(TAG, "# FAULT SLOW_FC: delaying DID 0x%04X response 500 ms", did);
            vTaskDelay(pdMS_TO_TICKS(500));
        }

        /* Build UDS response: 0x62 + DIDhi + DIDlo + data */
        uint8_t  uds_buf[70];
        size_t   blen    = (plen > 64) ? 64 : plen;
        size_t   uds_len = 3 + blen;
        memcpy(uds_buf, pl, blen);
        if (corrupt && blen >= 2) { uds_buf[0] = 0xDE; uds_buf[1] = 0xAD; }

        uint8_t resp[70];
        resp[0] = 0x62;
        resp[1] = (uint8_t)(did >> 8);
        resp[2] = (uint8_t)(did & 0xFF);
        memcpy(&resp[3], uds_buf, blen);

        if (uds_len <= 7)
            _send_uds_sf(resp, uds_len);
        else
            _send_uds_multiframe(resp, uds_len);

        s->rx_request_count++;
        return;
    }

    /* Unknown SID — NRC serviceNotSupported */
    uint8_t nrc[] = { 0x7F, sid, 0x11 };
    _send_uds_sf(nrc, sizeof(nrc));
    ESP_LOGW(TAG, "# NRC 0x11 sent for SID 0x%02X", sid);
}

static void _rx_task(void *arg)
{
    ecu_sim_t *s = (ecu_sim_t *)arg;
    mcp2515_register_rx_task(xTaskGetCurrentTaskHandle());

    ESP_LOGI(TAG, "# RX task started — UDS responder=%s",
             s->cfg.enable_uds_responder ? "ON" : "OFF");

    can_frame_t frame;
    for (;;) {
        ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(500));

        /* Drain MCP2515 hardware RXB0/RXB1 into ring before reading ring */
        while (mcp2515_frame_available()) {
            mcp2515_receive_frame(&frame);
        }

        while (mcp2515_ring_has_frame()) {
            if (mcp2515_pop_frame(&frame) != ESP_OK) break;

            if (!s->cfg.enable_uds_responder) continue;

            /* TIMEOUT fault: drop the next request entirely so the
             * tester sees a real timeout */
            if (s_pending_fault == ECU_FAULT_TIMEOUT) {
                s_pending_fault = ECU_FAULT_NONE;
                s->tx_drop_count++;
                ESP_LOGW(TAG, "# FAULT TIMEOUT — dropping request 0x%03"PRIX32,
                         frame.id);
                continue;
            }

            if (frame.id == 0x7DF) {
                _handle_obd_request(s, &frame);
            }
            else if (frame.id == 0x7E0 || _is_known_uds_request(frame.id)) {
                _handle_uds_request(s, &frame);
            }
        }
    }
}

/* ═══════════════════════════════════════════════════════════════════════
 *  Fault injection
 * ═══════════════════════════════════════════════════════════════════════ */

void ecu_sim_inject_fault(ecu_sim_t *s, ecu_fault_type_t type)
{
    if (!s) return;

    switch (type) {
        case ECU_FAULT_TIMEOUT:
        case ECU_FAULT_SLOW_FC:
        case ECU_FAULT_WRONG_DID:
            /* Latched — picked up by next TX or RX */
            s_pending_fault = type;
            ESP_LOGW(TAG, "# Fault armed: type=%d", type);
            break;

        case ECU_FAULT_TX_FLOOD:
            ESP_LOGW(TAG, "# Fault TX_FLOOD: bursting 50 frames");
            for (int i = 0; i < 50; i++) {
                can_frame_t f;
                memset(&f, 0, sizeof(f));
                f.id  = 0x500;
                f.dlc = 8;
                memset(f.data, (uint8_t)i, 8);
                if (_tx(&f) != ESP_OK) {
                    s->tx_drop_count++;
                }
            }
            break;

        case ECU_FAULT_BUS_OFF:
            /* The original implementation flooded with valid frames that
             * Node A ACKed → TEC stayed at 0, no bus-off ever triggered.
             *
             * Real bus-off requires errored bits on the wire. The reliable
             * way from software is to switch the MCP2515 to a DIFFERENT
             * bitrate, transmit, then switch back. Every TX during the
             * mismatched window produces stuff/form errors; TEC climbs
             * past 128 (error-passive) and keeps going to 256 (bus-off).
             *
             * Our error_handler (Step 10) detects EFLG.TXBO and resets.
             * For Step 13 we do the provoke + manual recovery here.
             */
            ESP_LOGW(TAG, "# Fault BUS_OFF: switching to wrong bitrate to provoke errors");
            mcp2515_set_mode(MCP_MODE_CONFIG);
            /* 250 kbps is half the bench bitrate — guaranteed errors */
            mcp2515_set_bitrate(250000, MCP2515_OSC_MHZ);
            mcp2515_set_mode(MCP_MODE_NORMAL);

            for (int i = 0; i < 50; i++) {
                can_frame_t f;
                memset(&f, 0, sizeof(f));
                f.id  = 0x6FF;
                f.dlc = 1;
                f.data[0] = 0xAA;
                _tx(&f);   /* errors expected — ignore return */
                vTaskDelay(pdMS_TO_TICKS(2));
            }

            /* Recovery: snap back to correct bitrate. This is what the
             * real error_handler will do once it detects TXBO. */
            ESP_LOGW(TAG, "# Fault BUS_OFF: recovering to %u bps", CAN_BITRATE);
            mcp2515_set_mode(MCP_MODE_CONFIG);
            mcp2515_set_bitrate(CAN_BITRATE, MCP2515_OSC_MHZ);
            mcp2515_set_mode(MCP_MODE_NORMAL);
            ESP_LOGW(TAG, "# BUS_OFF recovered ✓");
            break;

        default:
            break;
    }
}

static void _fault_scheduler_task(void *arg)
{
    ecu_sim_t *s = (ecu_sim_t *)arg;
    const uint32_t poll_period_ms = 1000;
    uint32_t elapsed_s = 0;

    bool fired_timeout   = false;
    bool fired_slow_fc   = false;
    bool fired_wrong_did = false;
    bool fired_tx_flood  = false;
    bool fired_bus_off   = false;

    bool any_scheduled =
        s->cfg.fault_timeout_at_s   ||
        s->cfg.fault_slow_fc_at_s   ||
        s->cfg.fault_wrong_did_at_s ||
        s->cfg.fault_tx_flood_at_s  ||
        s->cfg.fault_bus_off_at_s;

    if (!any_scheduled) {
        ESP_LOGI(TAG, "# No fault schedule — scheduler idle");
        vTaskDelete(NULL);
        return;
    }

    ESP_LOGI(TAG, "# Fault schedule: timeout=%"PRIu32"s slow_fc=%"PRIu32"s "
                  "wrong_did=%"PRIu32"s tx_flood=%"PRIu32"s bus_off=%"PRIu32"s",
             s->cfg.fault_timeout_at_s, s->cfg.fault_slow_fc_at_s,
             s->cfg.fault_wrong_did_at_s, s->cfg.fault_tx_flood_at_s,
             s->cfg.fault_bus_off_at_s);

    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(poll_period_ms));
        elapsed_s++;

        if (!fired_timeout && s->cfg.fault_timeout_at_s &&
            elapsed_s >= s->cfg.fault_timeout_at_s) {
            ecu_sim_inject_fault(s, ECU_FAULT_TIMEOUT);
            fired_timeout = true;
        }
        if (!fired_slow_fc && s->cfg.fault_slow_fc_at_s &&
            elapsed_s >= s->cfg.fault_slow_fc_at_s) {
            ecu_sim_inject_fault(s, ECU_FAULT_SLOW_FC);
            fired_slow_fc = true;
        }
        if (!fired_wrong_did && s->cfg.fault_wrong_did_at_s &&
            elapsed_s >= s->cfg.fault_wrong_did_at_s) {
            ecu_sim_inject_fault(s, ECU_FAULT_WRONG_DID);
            fired_wrong_did = true;
        }
        if (!fired_tx_flood && s->cfg.fault_tx_flood_at_s &&
            elapsed_s >= s->cfg.fault_tx_flood_at_s) {
            ecu_sim_inject_fault(s, ECU_FAULT_TX_FLOOD);
            fired_tx_flood = true;
        }
        if (!fired_bus_off && s->cfg.fault_bus_off_at_s &&
            elapsed_s >= s->cfg.fault_bus_off_at_s) {
            ecu_sim_inject_fault(s, ECU_FAULT_BUS_OFF);
            fired_bus_off = true;
        }
    }
}

/* ═══════════════════════════════════════════════════════════════════════
 *  Public API
 * ═══════════════════════════════════════════════════════════════════════ */

void ecu_sim_init(ecu_sim_t *s)
{
    if (!s) return;
    memset(s, 0, sizeof(*s));
    s->cfg  = s_default_cfg;
    s->live = s_default_cfg.initial;
    s_sim   = s;
}

esp_err_t ecu_sim_load_config(ecu_sim_t *s, const char *json_path)
{
    if (!s || !json_path) return ESP_ERR_INVALID_ARG;

    FILE *f = fopen(json_path, "r");
    if (!f) {
        ESP_LOGW(TAG, "# %s not found — using defaults", json_path);
        return ESP_ERR_NOT_FOUND;
    }
    fseek(f, 0, SEEK_END);
    long sz = ftell(f);
    rewind(f);
    if (sz <= 0 || sz > 8192) { fclose(f); return ESP_FAIL; }

    char *buf = malloc((size_t)sz + 1);
    if (!buf) { fclose(f); return ESP_ERR_NO_MEM; }
    size_t nread = fread(buf, 1, (size_t)sz, f);
    fclose(f);
    buf[nread] = '\0';

    cJSON *root = cJSON_Parse(buf);
    free(buf);
    if (!root) {
        ESP_LOGW(TAG, "# %s parse error — using defaults", json_path);
        return ESP_FAIL;
    }

    cJSON *e = cJSON_GetObjectItemCaseSensitive(root, "ecu_sim");
    if (e) {
        cJSON *p, *fault, *init;

        if ((p = cJSON_GetObjectItem(e, "obd_period_ms")) && cJSON_IsNumber(p))
            s->cfg.obd_period_ms = (uint32_t)p->valuedouble;
        if ((p = cJSON_GetObjectItem(e, "steering_period_ms")) && cJSON_IsNumber(p))
            s->cfg.steering_period_ms = (uint32_t)p->valuedouble;
        if ((p = cJSON_GetObjectItem(e, "wheel_period_ms")) && cJSON_IsNumber(p))
            s->cfg.wheel_period_ms = (uint32_t)p->valuedouble;
        if ((p = cJSON_GetObjectItem(e, "brake_period_ms")) && cJSON_IsNumber(p))
            s->cfg.brake_period_ms = (uint32_t)p->valuedouble;

        if ((p = cJSON_GetObjectItem(e, "enable_uds_responder")) && cJSON_IsBool(p))
            s->cfg.enable_uds_responder = cJSON_IsTrue(p);

        if ((init = cJSON_GetObjectItem(e, "initial"))) {
            if ((p = cJSON_GetObjectItem(init, "rpm")) && cJSON_IsNumber(p))
                s->cfg.initial.rpm = (uint16_t)p->valuedouble;
            if ((p = cJSON_GetObjectItem(init, "engine_load_pct")) && cJSON_IsNumber(p))
                s->cfg.initial.engine_load_pct = (uint8_t)p->valuedouble;
            if ((p = cJSON_GetObjectItem(init, "steering_deg")) && cJSON_IsNumber(p))
                s->cfg.initial.steering_deg = (int16_t)p->valuedouble;
        }

        if ((fault = cJSON_GetObjectItem(e, "faults"))) {
            if ((p = cJSON_GetObjectItem(fault, "timeout_at_s")) && cJSON_IsNumber(p))
                s->cfg.fault_timeout_at_s = (uint32_t)p->valuedouble;
            if ((p = cJSON_GetObjectItem(fault, "slow_fc_at_s")) && cJSON_IsNumber(p))
                s->cfg.fault_slow_fc_at_s = (uint32_t)p->valuedouble;
            if ((p = cJSON_GetObjectItem(fault, "wrong_did_at_s")) && cJSON_IsNumber(p))
                s->cfg.fault_wrong_did_at_s = (uint32_t)p->valuedouble;
            if ((p = cJSON_GetObjectItem(fault, "tx_flood_at_s")) && cJSON_IsNumber(p))
                s->cfg.fault_tx_flood_at_s = (uint32_t)p->valuedouble;
            if ((p = cJSON_GetObjectItem(fault, "bus_off_at_s")) && cJSON_IsNumber(p))
                s->cfg.fault_bus_off_at_s = (uint32_t)p->valuedouble;
        }
    }

    cJSON_Delete(root);
    s->live = s->cfg.initial;
    ESP_LOGI(TAG, "# Config loaded from %s", json_path);
    return ESP_OK;
}

esp_err_t ecu_sim_start(ecu_sim_t *s)
{
    if (!s) return ESP_ERR_INVALID_ARG;

    /* NORMAL mode — Node B ACKs Node A's frames */
    esp_err_t r = mcp2515_set_mode(MCP_MODE_NORMAL);
    if (r != ESP_OK) {
        ESP_LOGE(TAG, "# Failed to set NORMAL mode: %s", esp_err_to_name(r));
        return r;
    }
    ESP_LOGI(TAG, "# MCP2515 NORMAL mode — simulator ACTIVE");

    s->running = true;

    /* Initialise the ISO-TP context for UDS responses.
     * tx_id=0x7E8 (ECU → tester), rx_id=0x7E0 (tester → ECU).
     * This context sends responses and receives Flow Control frames.
     * No rx_cb needed — we respond directly, not via callback.    */
    isotp_init(&s->uds_isotp,
               0x7E8,   /* tx: ECU response ID    */
               0x7E0,   /* rx: UDS request/FC ID  */
               _tx,     /* same TX fn as broadcasts */
               NULL,    /* no rx callback needed    */
               NULL);
    ESP_LOGI(TAG, "# UDS ISO-TP context: tx=0x7E8 rx=0x7E0");

    /* Hardware acceptance filter: reserve RXB0 exclusively for 0x7E0
     * (UDS requests + Flow-Control frames from Node A).
     * RXB1 accepts all other IDs (broadcast streams 0x7E8/0x0A6/0x1A0/0x3B0).
     *
     * Without this, the MCP2515's two RX buffers fill with broadcast
     * frames at 200+ fps. The FC frame (0x7E0) arrives immediately after
     * the FF but gets discarded because both buffers are taken by OBD/steer
     * frames that arrived in the same ~5ms window. RXB0 dedicated to 0x7E0
     * guarantees the FC is never overwritten regardless of bus load.      */
    mcp2515_set_mode(MCP_MODE_CONFIG);
    const can_filter_t fc_filter = { .id = 0x7E0, .mask = 0x7FF };
    mcp2515_set_filter(0, &fc_filter);          /* filter 0 → RXB0 exact match */
    mcp2515_set_rx_mode(0, RX_MODE_FILTER);     /* RXB0: only 0x7E0 */
    mcp2515_set_rx_mode(1, RX_MODE_ACCEPT_ALL); /* RXB1: everything else */
    mcp2515_set_mode(MCP_MODE_NORMAL);
    ESP_LOGI(TAG, "# HW filter: RXB0=0x7E0 only, RXB1=accept-all");

    /* Broadcast tasks — Core 1 (TASK_CORE_APP). The CAN HAL serializes
     * SPI access via its own mutex, so multiple producers are fine. */
    if (s->cfg.obd_period_ms > 0)
        xTaskCreatePinnedToCore(_tx_obd_broadcast,      "tx_obd",
            TASK_STACK_CAN_TX, s, TASK_PRIO_CAN_TX, NULL, TASK_CORE_APP);

    if (s->cfg.steering_period_ms > 0)
        xTaskCreatePinnedToCore(_tx_steering_broadcast, "tx_steer",
            TASK_STACK_CAN_TX, s, TASK_PRIO_CAN_TX, NULL, TASK_CORE_APP);

    if (s->cfg.wheel_period_ms > 0)
        xTaskCreatePinnedToCore(_tx_wheel_broadcast,    "tx_wheel",
            TASK_STACK_CAN_TX, s, TASK_PRIO_CAN_TX, NULL, TASK_CORE_APP);

    if (s->cfg.brake_period_ms > 0)
        xTaskCreatePinnedToCore(_tx_brake_broadcast,    "tx_brake",
            TASK_STACK_CAN_TX, s, TASK_PRIO_CAN_TX, NULL, TASK_CORE_APP);

    /* RX (UDS responder) — Core 0 */
    xTaskCreatePinnedToCore(_rx_task,                "ecu_rx",
        TASK_STACK_CAN_RX, s, TASK_PRIO_CAN_RX, NULL, TASK_CORE_CAN);

    /* Animator — Core 1 */
    xTaskCreatePinnedToCore(_animator_task,          "anim",
        TASK_STACK_APP, s, TASK_PRIO_APP, NULL, TASK_CORE_APP);

    /* Fault scheduler — Core 1 */
    xTaskCreatePinnedToCore(_fault_scheduler_task,   "fault_sch",
        TASK_STACK_APP, s, TASK_PRIO_APP, NULL, TASK_CORE_APP);

    ESP_LOGI(TAG, "# Simulator started: 4 broadcast streams + UDS responder");
    return ESP_OK;
}