/**
 * @file decoder_app.c
 * @brief Step 14 — Refactored decoder with hash dispatcher + OBD-II engine
 *
 * ARCHITECTURE
 * ────────────
 * RX task drains MCP2515 → ring buffer → dispatcher_dispatch(&frame)
 *   - Dispatcher routes to per-ID handlers via O(1) hash lookup
 *   - Handlers registered at startup based on REAL_CAR_MODE flag
 *
 * BENCH MODE (REAL_CAR_MODE undefined):
 *   Registered IDs: 0x7E8 (OBD broadcast), 0x0A6, 0x1A0, 0x3B0, UDS resp IDs
 *   MCP2515 mode  : NORMAL  (ACKs needed for Node B simulator)
 *   UDS task      : ON   (runs scorecard test sequence)
 *   OBD engine    : OFF  (not needed — Node B simulates 0x7E8 broadcasts)
 *
 * CAR MODE (REAL_CAR_MODE defined):
 *   Registered IDs: 0x7E8 only (routed to OBD engine)
 *   MCP2515 mode  : NORMAL  (must transmit OBD-II queries)
 *   UDS task      : OFF  (would conflict with OBD on 0x7E8)
 *   OBD engine    : ON   (polls RPM/speed/throttle/temp/load via ISO-TP)
 *   OBD poller task: queries one PID every 100ms (round-robin schedule)
 *
 * Both modes share the same dispatcher, same RX task, same dashboard task.
 * Mode-specific behaviour is gated by #ifdef REAL_CAR_MODE only.
 */

#include "decoder_app.h"
#include "project_config.h" /* TASK_STACK_*, TASK_PRIO_*, TASK_CORE_*  */
#include "mcp2515.h"
#include "isotp.h"
#include "uds.h" /* renamed from uds_client.h               */
#include "sd_logger.h"
#include "gps_app.h"
#include "dispatcher.h"
#include "obd.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_timer.h"
#include <string.h>
#include <stdio.h>
#include <inttypes.h>

#if defined(PASSIVE_SNIFF_MODE) && defined(REAL_CAR_MODE)
#error "PASSIVE_SNIFF_MODE and REAL_CAR_MODE are mutually exclusive — define only one"
#endif

/* ─── Global OBD engine pointer (set in decoder_app_start) ──────────── */
obd_engine_t *g_obd = NULL;

static const char *TAG = "DECODER";

/* ─── Shared state ──────────────────────────────────────────────────── */

/* Mutex protecting ISO-TP state. Both UDS and OBD use the same ISO-TP
 * layer; the RX-side callbacks feed frames in while a foreground task
 * is sending a request. */
static SemaphoreHandle_t s_isotp_mutex = NULL;

/* Hash dispatcher — replaces the old switch-statement _dispatch_frame() */
static dispatcher_t s_disp;

/* ─── Dashboard state (bench mode signals) ──────────────────────────── */
static struct
{
    uint16_t rpm;
    bool rpm_valid;
    int16_t steering_deg10;
    bool steering_valid;
    uint16_t wheels_kmh100[4];
    bool wheels_valid;
    uint8_t brake_bar;
    uint8_t brake_counter;
    bool brake_valid;
} s_dash;

/* ─── Stream tracking (for health check) ────────────────────────────── */
typedef struct
{
    uint32_t can_id;
    const char *name;
    uint32_t frame_count;
    uint64_t last_seen_us;
    uint32_t expected_rate_hz;
} decoder_stream_t;

static decoder_stream_t s_streams[] = {
    {0x7E8, "OBD/RPM ", 0, 0, 10},
    {0x0A6, "STEER   ", 0, 0, 100},
    {0x1A0, "WHEELS  ", 0, 0, 100},
    {0x3B0, "BRAKE   ", 0, 0, 50},
};
#define NUM_STREAMS (sizeof(s_streams) / sizeof(s_streams[0]))

static decoder_stream_t *_find_stream(uint32_t id)
{
    for (size_t i = 0; i < NUM_STREAMS; i++)
        if (s_streams[i].can_id == id)
            return &s_streams[i];
    return NULL;
}

/* ─── Passive sniff mode: per-ID frame histogram ─────────────────────── */
#ifdef PASSIVE_SNIFF_MODE
#define SNIFF_MAX_IDS 128

typedef struct {
    uint32_t id;
    uint32_t count_total;
    uint32_t count_window;   /* frames in current 5 s health window */
    uint64_t last_seen_us;
    uint8_t  last_data[8];
    uint8_t  last_dlc;
    bool     active;
} sniff_entry_t;

static sniff_entry_t     s_sniff_table[SNIFF_MAX_IDS];
static uint32_t          s_sniff_unique = 0;
static SemaphoreHandle_t s_sniff_mutex  = NULL;
#endif /* PASSIVE_SNIFF_MODE */

/* ─── ISO-TP TX adapter (shared by UDS and OBD) ─────────────────────── */
static esp_err_t _isotp_tx(const can_frame_t *frame)
{
    return mcp2515_transmit_frame(frame, 150); /* 150ms: survives MLOA bursts at high bus load */
}

/* ─── Bench-mode clients (UDS) ─────────────────────────────────────── */
#ifndef REAL_CAR_MODE
static uds_client_t s_uds;
#endif

/* ─── Car-mode clients (OBD-II engine) ─────────────────────────────── */
#ifdef REAL_CAR_MODE
static obd_engine_t s_obd;

/* OBD round-robin schedule. 10 slots × 100 ms = 1 s cycle.
 * RPM appears 4×, speed 2× to favour fast-changing signals. */
static const uint8_t s_obd_schedule[] = {
    OBD_PID_RPM,
    OBD_PID_SPEED,
    OBD_PID_RPM,
    OBD_PID_THROTTLE,
    OBD_PID_RPM,
    OBD_PID_SPEED,
    OBD_PID_ENGINE_LOAD,
    OBD_PID_RPM,
    OBD_PID_COOLANT_TEMP,
    OBD_PID_INTAKE_TEMP,
};
#define OBD_SCHEDULE_LEN (sizeof(s_obd_schedule) / sizeof(s_obd_schedule[0]))

/* Latest decoded OBD values (read by dashboard task) */
static struct
{
    obd_signal_t rpm;
    obd_signal_t speed;
    obd_signal_t throttle;
    obd_signal_t load;
    obd_signal_t coolant;
    obd_signal_t intake;
} s_obd_dash;
static SemaphoreHandle_t s_obd_dash_mutex = NULL;
#endif

/* ─── Scorecard (bench) ─────────────────────────────────────────────── */
typedef struct
{
    bool obd_pid_decoded;
    bool all_streams_seen;
    bool uds_did_received;
    bool timeout_detected;
    bool wrong_did_detected;
    bool slow_fc_detected;
    bool tx_overflow_detected;
} scorecard_t;
static scorecard_t s_score;

/* ─── Decoders for bench broadcast IDs ──────────────────────────────── */

static void _decode_obd_broadcast(const can_frame_t *f, void *ctx)
{
    (void)ctx;

    decoder_stream_t *st = _find_stream(f->id);
    if (st)
    {
        st->frame_count++;
        st->last_seen_us = (uint64_t)esp_timer_get_time();
    }

#ifdef REAL_CAR_MODE
    /* CAR MODE: 0x7E8 frames are OBD-II responses — feed directly.
     * No mutex needed: only the RX task calls this handler, so there
     * is no concurrent access to the ISO-TP state from this path.
     * The poller task does NOT hold any mutex while waiting, so there
     * is no deadlock risk. Holding a mutex here would silently drop
     * frames if acquisition ever timed out. */
    ESP_LOGD(TAG, "0x7E8 dlc=%u [%02X %02X %02X %02X %02X %02X %02X %02X]",
             f->dlc,
             f->data[0], f->data[1], f->data[2], f->data[3],
             f->data[4], f->data[5], f->data[6], f->data[7]);
    obd_process_frame(&s_obd, f);
#else
    /* BENCH MODE: 0x7E8 is a hand-crafted OBD broadcast from Node B. */
    if (f->dlc >= 5 && f->data[1] == 0x41 && f->data[2] == 0x0C)
    {
        uint8_t A = f->data[3];
        uint8_t B = f->data[4];
        s_dash.rpm = ((uint16_t)A * 256 + B) / 4;
        s_dash.rpm_valid = true;
        s_score.obd_pid_decoded = true;
    }

    /* Forward to UDS only if NOT an OBD broadcast (0x41-0x49 service codes) */
    bool is_obd_broadcast = (f->dlc >= 2 &&
                             f->data[1] >= 0x40 && f->data[1] <= 0x49);
    if (!is_obd_broadcast && s_isotp_mutex &&
        xSemaphoreTake(s_isotp_mutex, pdMS_TO_TICKS(10)) == pdTRUE)
    {
        uds_process_frame(&s_uds, f);
        xSemaphoreGive(s_isotp_mutex);
    }
#endif
}

#ifndef REAL_CAR_MODE
/* Bench-mode proprietary stream decoders */
static void _decode_steering(const can_frame_t *f, void *ctx)
{
    (void)ctx;
    decoder_stream_t *st = _find_stream(f->id);
    if (st)
    {
        st->frame_count++;
        st->last_seen_us = esp_timer_get_time();
    }

    if (f->dlc >= 2)
    {
        /* Little-endian: low byte first — matches ecu_sim_app encoding */
        int16_t raw = (int16_t)((uint16_t)f->data[0] | ((uint16_t)f->data[1] << 8));
        s_dash.steering_deg10 = raw;
        s_dash.steering_valid = true;
    }
}

static void _decode_wheels(const can_frame_t *f, void *ctx)
{
    (void)ctx;
    decoder_stream_t *st = _find_stream(f->id);
    if (st)
    {
        st->frame_count++;
        st->last_seen_us = esp_timer_get_time();
    }

    if (f->dlc >= 8)
    {
        /* Little-endian: low byte first — matches ecu_sim_app encoding */
        for (int i = 0; i < 4; i++)
        {
            s_dash.wheels_kmh100[i] =
                (uint16_t)f->data[i * 2] | ((uint16_t)f->data[i * 2 + 1] << 8);
        }
        s_dash.wheels_valid = true;
    }
}

static void _decode_brake(const can_frame_t *f, void *ctx)
{
    (void)ctx;
    decoder_stream_t *st = _find_stream(f->id);
    if (st)
    {
        st->frame_count++;
        st->last_seen_us = esp_timer_get_time();
    }

    if (f->dlc >= 4)
    {
        /* Little-endian uint16 for pressure, counter at byte 3 */
        s_dash.brake_bar = (uint16_t)f->data[0] | ((uint16_t)f->data[1] << 8);
        s_dash.brake_counter = f->data[3];
        s_dash.brake_valid = true;
    }
}
#endif /* !REAL_CAR_MODE */

/* ─── Passive sniff handler ──────────────────────────────────────────── */
#ifdef PASSIVE_SNIFF_MODE
static void _sniff_handler(const can_frame_t *f, void *ctx)
{
    (void)ctx;
    if (!s_sniff_mutex) return;
    /* Non-blocking take: skip the update on contention rather than
     * stalling the RX task while hardware buffers fill up. */
    if (xSemaphoreTake(s_sniff_mutex, 0) != pdTRUE) return;

    sniff_entry_t *entry = NULL;
    for (int si = 0; si < SNIFF_MAX_IDS; si++)
    {
        if (s_sniff_table[si].active && s_sniff_table[si].id == f->id)
        {
            entry = &s_sniff_table[si];
            break;
        }
        if (!entry && !s_sniff_table[si].active)
            entry = &s_sniff_table[si];  /* first free slot */
    }
    if (entry && !entry->active)
    {
        entry->active       = true;
        entry->id           = f->id;
        entry->count_total  = 0;
        entry->count_window = 0;
        s_sniff_unique++;
    }
    if (entry)
    {
        entry->count_total++;
        entry->count_window++;
        entry->last_seen_us = (uint64_t)f->timestamp_us;
        entry->last_dlc     = f->dlc;
        uint8_t copy = (f->dlc > 8) ? 8 : f->dlc;
        memcpy(entry->last_data, f->data, copy);
    }
    xSemaphoreGive(s_sniff_mutex);
}
#endif /* PASSIVE_SNIFF_MODE */

/* Default handler — fires for unregistered IDs (debug log only) */
static void _default_handler(const can_frame_t *f, void *ctx)
{
    (void)ctx;
    ESP_LOGD(TAG, "Unregistered ID 0x%03lX dlc=%d",
             (unsigned long)f->id, f->dlc);
}

/* ─── RX task ──────────────────────────────────────────────────────── */

static void _rx_task(void *arg)
{
    (void)arg;
    mcp2515_register_rx_task(xTaskGetCurrentTaskHandle());
    ESP_LOGI(TAG, "# RX task started");

    can_frame_t frame;
    for (;;)
    {
        ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(500));

        /* Stage 1: drain hardware RXB0/RXB1 into the ring buffer. */
        while (mcp2515_frame_available())
        {
            mcp2515_receive_frame(&frame);
        }

        /* Stage 2: dispatch every frame in the ring */
        while (mcp2515_ring_has_frame())
        {
            if (mcp2515_pop_frame(&frame) != ESP_OK)
                break;

            dispatcher_dispatch(&s_disp, &frame);

#ifdef CANDUMP_OUTPUT
        {
            /* candump format: "(seconds.microseconds) vcan0 ID#DDDDDDDD" */
            uint32_t ts_s    = (uint32_t)(frame.timestamp_us / 1000000LL);
            uint32_t ts_frac = (uint32_t)(frame.timestamp_us % 1000000LL);
            printf("(%lu.%06lu) vcan0 %03lX#",
                   (unsigned long)ts_s, (unsigned long)ts_frac,
                   (unsigned long)frame.id);
            for (uint8_t ci = 0; ci < frame.dlc; ci++)
                printf("%02X", frame.data[ci]);
            putchar('\n');
        }
#endif /* CANDUMP_OUTPUT */

        /* Log every raw frame — non-blocking */
            uint8_t flags = 0;
            if (frame.id >= 0x7E8 && frame.id <= 0x7EF)
                flags |= LOG_FLAG_UDS;
            if (frame.id == 0x7E8 && frame.dlc >= 2 &&
                frame.data[1] >= 0x40 && frame.data[1] <= 0x49)
                flags |= LOG_FLAG_OBD;
            sd_logger_log_raw(&frame, flags);
        }

        if (mcp2515_get_overflow_count() > 0)
        {
            s_score.tx_overflow_detected = true;
        }
    }
}

/* ─── Dashboard task ───────────────────────────────────────────────── */

static void _dashboard_task(void *arg)
{
    (void)arg;
    ESP_LOGI(TAG, "# Dashboard refresh every 500 ms");
#if defined(PASSIVE_SNIFF_MODE)
    ESP_LOGI(TAG, "# SNIFF MODE: passive capture (listen-only, no TX)");
#elif defined(REAL_CAR_MODE)
    ESP_LOGI(TAG, "# CAR MODE: RPM|SPD|THR|LOAD|CLT|INT (OBD-II)");
#else
    ESP_LOGI(TAG, "# BENCH MODE: RPM | STEER | FL/FR/RL/RR | BRAKE");
#endif

    for (;;)
    {
        vTaskDelay(pdMS_TO_TICKS(500));

#if defined(PASSIVE_SNIFF_MODE)
        /* ── SNIFF MODE DASHBOARD ────────────────────────────────── */
        {
            uint32_t total = 0;
            xSemaphoreTake(s_sniff_mutex, portMAX_DELAY);
            for (int sj = 0; sj < SNIFF_MAX_IDS; sj++)
                if (s_sniff_table[sj].active)
                    total += s_sniff_table[sj].count_total;
            xSemaphoreGive(s_sniff_mutex);
            printf("[SNIFF] unique_ids=%lu  total_rx=%lu\n",
                   (unsigned long)s_sniff_unique, (unsigned long)total);
        }
#elif defined(REAL_CAR_MODE)
        /* ── CAR MODE DASHBOARD ──────────────────────────────────── */
        obd_signal_t rpm, spd, thr, load, clt, intake;
        xSemaphoreTake(s_obd_dash_mutex, portMAX_DELAY);
        rpm = s_obd_dash.rpm;
        spd = s_obd_dash.speed;
        thr = s_obd_dash.throttle;
        load = s_obd_dash.load;
        clt = s_obd_dash.coolant;
        intake = s_obd_dash.intake;
        xSemaphoreGive(s_obd_dash_mutex);

        printf("[DASH] RPM=%4.0f SPD=%3.0f THR=%3.0f%% LOAD=%3.0f%% "
               "CLT=%3.0fC INT=%3.0fC\n",
               rpm.valid ? rpm.value : 0.0f,
               spd.valid ? spd.value : 0.0f,
               thr.valid ? thr.value : 0.0f,
               load.valid ? load.value : 0.0f,
               clt.valid ? clt.value : 0.0f,
               intake.valid ? intake.value : 0.0f);

        /* JSON for laptop capture */
        printf("{\"t\":%lu,\"type\":\"obd\","
               "\"rpm\":%.0f,\"spd\":%.0f,\"thr\":%.1f,\"load\":%.1f,"
               "\"clt\":%.0f,\"intake\":%.0f,"
               "\"rpm_v\":%d,\"spd_v\":%d,\"thr_v\":%d,"
               "\"load_v\":%d,\"clt_v\":%d,\"int_v\":%d,"
               "\"req\":%lu,\"ok\":%lu,\"err\":%lu,\"to\":%lu}\n",
               (unsigned long)(esp_timer_get_time() / 1000),
               rpm.value, spd.value, thr.value, load.value,
               clt.value, intake.value,
               rpm.valid, spd.valid, thr.valid,
               load.valid, clt.valid, intake.valid,
               (unsigned long)s_obd.req_count,
               (unsigned long)s_obd.ok_count,
               (unsigned long)s_obd.err_count,
               (unsigned long)s_obd.timeout_count);

        /* SD log a decoded snapshot — reuse decoded_snapshot_t */
        decoded_snapshot_t snap = {
            .timestamp_ms = (uint32_t)(esp_timer_get_time() / 1000),
            .rpm = (uint16_t)(rpm.valid ? rpm.value : 0),
            .steer_deg_x10 = 0,
            .wheel_fl = (uint8_t)(spd.valid ? spd.value : 0),
            .wheel_fr = (uint8_t)(spd.valid ? spd.value : 0),
            .wheel_rl = (uint8_t)(spd.valid ? spd.value : 0),
            .wheel_rr = (uint8_t)(spd.valid ? spd.value : 0),
            .brake_bar = (uint8_t)(thr.valid ? thr.value : 0),
            .valid_mask = (rpm.valid ? VALID_RPM : 0) |
                          (spd.valid ? VALID_WHEELS : 0),
        };
        sd_logger_log_decoded(&snap);

#else
        /* ── BENCH MODE DASHBOARD ────────────────────────────────── */
        char rpm_str[16] = "  ---";
        char steer_str[16] = "    ---";
        char wheels_str[40] = "  ---/---/---/--- km/h";
        char brake_str[24] = " --- bar";

        if (s_dash.rpm_valid)
            snprintf(rpm_str, sizeof(rpm_str), " %4u", s_dash.rpm);
        if (s_dash.steering_valid)
            snprintf(steer_str, sizeof(steer_str), " %+6.1f",
                     s_dash.steering_deg10 / 10.0);
        if (s_dash.wheels_valid)
            snprintf(wheels_str, sizeof(wheels_str),
                     " %3u/%3u/%3u/%3u km/h",
                     s_dash.wheels_kmh100[0] / 100, s_dash.wheels_kmh100[1] / 100,
                     s_dash.wheels_kmh100[2] / 100, s_dash.wheels_kmh100[3] / 100);
        if (s_dash.brake_valid)
            snprintf(brake_str, sizeof(brake_str), " %3u bar (cnt=%u)",
                     s_dash.brake_bar, s_dash.brake_counter);

        printf("[DASH] RPM=%s | STEER=%s deg | WHEELS=%s | BRAKE=%s\n",
               rpm_str, steer_str, wheels_str, brake_str);

        decoded_snapshot_t snap = {
            .timestamp_ms = (uint32_t)(esp_timer_get_time() / 1000),
            .rpm = s_dash.rpm,
            .steer_deg_x10 = s_dash.steering_deg10,
            .wheel_fl = s_dash.wheels_kmh100[0] / 100,
            .wheel_fr = s_dash.wheels_kmh100[1] / 100,
            .wheel_rl = s_dash.wheels_kmh100[2] / 100,
            .wheel_rr = s_dash.wheels_kmh100[3] / 100,
            .brake_bar = s_dash.brake_bar,
            .valid_mask = (s_dash.rpm_valid ? VALID_RPM : 0) |
                          (s_dash.steering_valid ? VALID_STEER : 0) |
                          (s_dash.wheels_valid ? VALID_WHEELS : 0) |
                          (s_dash.brake_valid ? VALID_BRAKE : 0),
        };
        sd_logger_log_decoded(&snap);
#endif
    }
}

/* ─── Health task ──────────────────────────────────────────────────── */

static void _health_task(void *arg)
{
    (void)arg;
    const TickType_t period = pdMS_TO_TICKS(5000);
    TickType_t last = xTaskGetTickCount();
    uint32_t prev_counts[NUM_STREAMS] = {0};

    for (;;)
    {
        vTaskDelayUntil(&last, period);

        ESP_LOGI(TAG, "# ── Health check (last 5 s) ──");

#ifndef PASSIVE_SNIFF_MODE
        bool all_seen = true;
        for (size_t i = 0; i < NUM_STREAMS; i++)
        {
            decoder_stream_t *st = &s_streams[i];
            uint32_t rx_5s = st->frame_count - prev_counts[i];
            prev_counts[i] = st->frame_count;
            uint32_t expect = st->expected_rate_hz * 5;
            const char *stat = (rx_5s == 0) ? "SILENT" : (rx_5s < expect * 80 / 100) ? "STALE"
                                                                                     : "OK";
            if (rx_5s == 0)
                all_seen = false;

            ESP_LOGI(TAG,
                     "#   %s 0x%03lX : %4lu rx (expect ~%lu) total=%lu  status=%s",
                     st->name, (unsigned long)st->can_id,
                     (unsigned long)rx_5s, (unsigned long)expect,
                     (unsigned long)st->frame_count, stat);
        }
        s_score.all_streams_seen = all_seen;

#ifdef REAL_CAR_MODE
        ESP_LOGI(TAG, "#   OBD: req=%lu ok=%lu err=%lu timeout=%lu",
                 (unsigned long)s_obd.req_count,
                 (unsigned long)s_obd.ok_count,
                 (unsigned long)s_obd.err_count,
                 (unsigned long)s_obd.timeout_count);
#else
        ESP_LOGI(TAG, "#   UDS: req=%lu timeout=%lu corrupt=0  ring_overflows=%lu",
                 (unsigned long)s_uds.req_count,
                 (unsigned long)s_uds.timeout_count,
                 (unsigned long)mcp2515_get_overflow_count());
#endif

        ESP_LOGI(TAG, "#   SCORECARD: streams=%c  pid=%c  did=%c  timeout=%c  "
                      "wrong_did=%c  slow_fc=%c  tx_ovf=%c",
                 s_score.all_streams_seen ? 'Y' : '.',
                 s_score.obd_pid_decoded ? 'Y' : '.',
                 s_score.uds_did_received ? 'Y' : '.',
                 s_score.timeout_detected ? 'Y' : '.',
                 s_score.wrong_did_detected ? 'Y' : '.',
                 s_score.slow_fc_detected ? 'Y' : '.',
                 s_score.tx_overflow_detected ? 'Y' : '.');
#endif /* !PASSIVE_SNIFF_MODE */

        /* Hardware RX buffer overflow diagnostic (all modes) */
        uint8_t eflg = mcp2515_read_eflg();
        if (eflg & 0xC0)
            ESP_LOGW(TAG, "#   EFLG=0x%02X  RXBO=%c  BUS_OFF=%c",
                     eflg,
                     (eflg & 0x40) ? 'Y' : '.',
                     (eflg & 0x80) ? 'Y' : '.');

#ifdef PASSIVE_SNIFF_MODE
        xSemaphoreTake(s_sniff_mutex, portMAX_DELAY);
        ESP_LOGI(TAG, "#   SNIFF: %lu unique IDs  (last 5 s activity):",
                 (unsigned long)s_sniff_unique);
        uint32_t active_in_window = 0;
        for (int sj = 0; sj < SNIFF_MAX_IDS; sj++)
        {
            sniff_entry_t *e = &s_sniff_table[sj];
            if (!e->active || e->count_window == 0) continue;
            active_in_window++;
            ESP_LOGI(TAG,
                     "#     0x%03lX  %4lu/5s  [%02X %02X %02X %02X %02X %02X %02X %02X]  dlc=%u",
                     (unsigned long)e->id, (unsigned long)e->count_window,
                     e->last_data[0], e->last_data[1], e->last_data[2], e->last_data[3],
                     e->last_data[4], e->last_data[5], e->last_data[6], e->last_data[7],
                     e->last_dlc);
            e->count_window = 0;
        }
        if (active_in_window == 0)
            ESP_LOGI(TAG, "#     (no activity in last 5 s)");
        xSemaphoreGive(s_sniff_mutex);
#endif /* PASSIVE_SNIFF_MODE */
    }
}

/* ─── CAR-MODE: TesterPresent keep-alive ───────────────────────────── */
#ifdef REAL_CAR_MODE
/* Send UDS TesterPresent (0x3E, sub-fn 0x80 = suppress response) on the
 * functional address 0x7DF so every ECU on the bus resets its S3 timer.
 *
 * Why 0x7DF, not 0x7E0?  We want ALL ECUs to stay in their diagnostic
 * sessions (ABS, BCM, etc.) even though we only poll the engine ECU.
 * Using the functional address costs nothing extra.
 *
 * The suppress-response sub-function (bit 7 = 1) means no ECU will
 * reply, so this never confuses the OBD response queue.
 *
 * Renault S3 timer is typically 2–5 s.  Sending every ~3 s (once per
 * full 10-slot cycle) provides comfortable margin. */
static void _send_tester_present(void)
{
    can_frame_t tp = {
        .id  = 0x7DF,
        .dlc = 8,
        .data = {0x02, 0x3E, 0x80, 0x55, 0x55, 0x55, 0x55, 0x55}
    };
    mcp2515_transmit_frame(&tp, 50);
    ESP_LOGD(TAG, "# TesterPresent sent (keep-alive)");
}

/* ─── CAR-MODE: OBD poller task ────────────────────────────────────── */
static void _obd_poller_task(void *arg)
{
    (void)arg;
    ESP_LOGI(TAG, "# OBD poller task started — 10 slot × 100 ms + 500 ms pause = ~1.5 s cycle");

    /* Wake-up burst: 3 rapid RPM queries to wake Renault diagnostic CAN.
     * DO NOT hold s_isotp_mutex across obd_request() — it blocks up to
     * 200ms waiting for a response. While waiting, the RX task needs
     * the mutex to call obd_process_frame() and deliver the response.
     * Holding the mutex here = instant deadlock = 100% timeouts.        */
    for (int i = 0; i < 3; i++)
    {
        obd_signal_t sig;
        obd_request(&s_obd, OBD_PID_RPM, &sig);
        vTaskDelay(pdMS_TO_TICKS(50));
    }
    /* Send TesterPresent immediately after the wake burst so the ECU's
     * S3 diagnostic-session timer starts from a clean baseline. */
    _send_tester_present();
    ESP_LOGI(TAG, "# OBD wake-up burst sent — entering normal polling");

    uint32_t slot = 0;
    for (;;)
    {
        uint8_t pid = s_obd_schedule[slot];
        obd_signal_t sig;

        /* No mutex here — see comment above. */
        esp_err_t r = obd_request(&s_obd, pid, &sig);

        /* Single retry on timeout — the ECU may have been busy serving
         * another request or recovering from a bus burst.  A 50 ms gap
         * before the retry is enough for the ECU to finish and become
         * ready; back-to-back retries without a gap hit the same busy
         * window and waste a slot. */
        if (r == ESP_ERR_TIMEOUT)
        {
            vTaskDelay(pdMS_TO_TICKS(50));
            r = obd_request(&s_obd, pid, &sig);
        }

        if (r == ESP_OK && sig.valid)
        {
            xSemaphoreTake(s_obd_dash_mutex, portMAX_DELAY);
            switch (pid)
            {
            case OBD_PID_RPM:
                s_obd_dash.rpm = sig;
                break;
            case OBD_PID_SPEED:
                s_obd_dash.speed = sig;
                break;
            case OBD_PID_THROTTLE:
                s_obd_dash.throttle = sig;
                break;
            case OBD_PID_ENGINE_LOAD:
                s_obd_dash.load = sig;
                break;
            case OBD_PID_COOLANT_TEMP:
                s_obd_dash.coolant = sig;
                break;
            case OBD_PID_INTAKE_TEMP:
                s_obd_dash.intake = sig;
                break;
            }
            xSemaphoreGive(s_obd_dash_mutex);
        }

        slot = (slot + 1) % OBD_SCHEDULE_LEN;

        if (slot == 0)
        {
            /* Full 10-slot cycle complete.
             * 1. Send TesterPresent to reset the ECU's S3 session timer.
             * 2. Pause 500 ms — brief idle gap that lets the ECU drain any
             *    pending internal work before the next cycle starts.
             * Cycle budget: 10 × 100 ms + 500 ms = 1.5 s → ~6.7 req/s
             * (not counting retry slots).  The ECU caps throughput at
             * ~4 resp/s regardless; keeping the request rate above that
             * keeps its pipeline full and maximises actual update rate. */
            _send_tester_present();
            vTaskDelay(pdMS_TO_TICKS(500));
        }
        else
        {
            vTaskDelay(pdMS_TO_TICKS(100));
        }
    }
}
#endif

/* ─── BENCH-MODE: UDS test task ────────────────────────────────────── */
#ifndef REAL_CAR_MODE
static const uint16_t s_test_dids[] = {
    UDS_DID_SW_VERSION,
    UDS_DID_ECU_SERIAL,
    UDS_DID_CAL_ID,
    UDS_DID_DIAG_VERSION,
    UDS_DID_VIN,
};

static void _uds_test_task(void *arg)
{
    (void)arg;
    vTaskDelay(pdMS_TO_TICKS(2000));

    /* Open extended session.
     * IMPORTANT: do NOT hold s_isotp_mutex across uds_open_session or
     * uds_read_did. Both functions block internally waiting for a response.
     * While they wait, the RX task needs s_isotp_mutex to call
     * uds_process_frame() — which delivers the response (and sends the FC
     * for multi-frame DIDs). Holding the mutex here would starve the RX
     * task → no FC sent → Node B times out → every DID times out.
     * Architecture rule from Step 12 handoff §8: no outer mutex around
     * uds_read_did. The 10ms window inside _decode_obd_broadcast is
     * sufficient to protect the brief ISO-TP state machine access.    */
    uds_flush_rx(&s_uds);
    esp_err_t r = uds_open_session(&s_uds, UDS_SESSION_EXTENDED);
    if (r == ESP_OK)
    {
        vTaskDelay(pdMS_TO_TICKS(50)); /* let ECU state machine settle */
        ESP_LOGI(TAG, "# UDS extended session opened ✓");
    }
    else
    {
        ESP_LOGW(TAG, "# UDS session open failed (%s) — continuing",
                 esp_err_to_name(r));
    }

    size_t idx = 0;
    uds_did_result_t result;

    for (;;)
    {
        vTaskDelay(pdMS_TO_TICKS(2500));

        uint16_t did = s_test_dids[idx];
        idx = (idx + 1) % (sizeof(s_test_dids) / sizeof(s_test_dids[0]));

        memset(&result, 0, sizeof(result));
        uds_flush_rx(&s_uds); /* discard any stale partial frames */

        uint64_t t0 = esp_timer_get_time();
        r = uds_read_did(&s_uds, did, &result); /* NO mutex held here */
        uint32_t elapsed_ms = (uint32_t)((esp_timer_get_time() - t0) / 1000);

        if (r == ESP_OK && result.valid)
        {
            s_score.uds_did_received = true;
            ESP_LOGI(TAG, "# UDS DID 0x%04X OK in %lu ms (%u bytes)",
                     did, (unsigned long)elapsed_ms, (unsigned)result.length);
            uds_print_did(&result, TAG);
            sd_logger_log_uds(did, result.data, result.length, elapsed_ms, false);
        }
        else if (r == ESP_ERR_TIMEOUT)
        {
            s_score.timeout_detected = true;
            ESP_LOGW(TAG, "# UDS DID 0x%04X TIMEOUT after %lu ms (criterion 4 ✓)",
                     did, (unsigned long)elapsed_ms);
            sd_logger_log_uds(did, NULL, 0, elapsed_ms, false);
        }
        else
        {
            ESP_LOGW(TAG, "# UDS DID 0x%04X failed: %s", did, esp_err_to_name(r));
        }

        /* TesterPresent with suppress bit — keeps ECU session alive,
         * sends no response so it doesn't collide with the next DID. */
        uds_tester_present(&s_uds);
    }
}
#endif

/* ─── decoder_app_start() ──────────────────────────────────────────── */

esp_err_t decoder_app_start(void)
{
    /* MCP2515 mode: LISTEN_ONLY for passive sniff, NORMAL otherwise. */
#if defined(PASSIVE_SNIFF_MODE)
    esp_err_t r = mcp2515_set_mode(MCP_MODE_LISTEN_ONLY);
    if (r != ESP_OK)
    {
        ESP_LOGE(TAG, "# Failed to set LISTEN_ONLY mode: %s", esp_err_to_name(r));
        return r;
    }
    ESP_LOGI(TAG, "# MCP2515 LISTEN_ONLY mode — passive sniff (no TX)");
#else
    esp_err_t r = mcp2515_set_mode(MCP_MODE_NORMAL);
    if (r != ESP_OK)
    {
        ESP_LOGE(TAG, "# Failed to set NORMAL mode: %s", esp_err_to_name(r));
        return r;
    }
#ifdef REAL_CAR_MODE
    ESP_LOGI(TAG, "# MCP2515 NORMAL mode — OBD-II polling (real car)");
#else
    ESP_LOGI(TAG, "# MCP2515 NORMAL mode — decoder + UDS tester (bench)");
#endif
#endif /* PASSIVE_SNIFF_MODE */

    /* Dispatcher setup */
    dispatcher_init(&s_disp);

#if defined(PASSIVE_SNIFF_MODE)
    /* All frames go to _sniff_handler via the default slot — no ID
     * registrations needed; the histogram tracks every ID it sees. */
    s_sniff_mutex = xSemaphoreCreateMutex();
    if (!s_sniff_mutex)
        return ESP_ERR_NO_MEM;
    memset(s_sniff_table, 0, sizeof(s_sniff_table));
    s_sniff_unique = 0;
    dispatcher_set_default(&s_disp, _sniff_handler, NULL);
#else
    /* ISO-TP mutex — not needed in PASSIVE_SNIFF_MODE (no TX/UDS/OBD) */
    s_isotp_mutex = xSemaphoreCreateMutex();
    if (!s_isotp_mutex)
        return ESP_ERR_NO_MEM;

    dispatcher_set_default(&s_disp, _default_handler, NULL);

    /* 0x7E8 is registered in BOTH modes — handler routes internally. */
    dispatcher_register(&s_disp, 0x7E8, _decode_obd_broadcast, NULL);

#ifndef REAL_CAR_MODE
    dispatcher_register(&s_disp, 0x0A6, _decode_steering, NULL);
    dispatcher_register(&s_disp, 0x1A0, _decode_wheels, NULL);
    dispatcher_register(&s_disp, 0x3B0, _decode_brake, NULL);
#endif

    /* Client init (mode-specific) */
#ifdef REAL_CAR_MODE
    s_obd_dash_mutex = xSemaphoreCreateMutex();
    if (!s_obd_dash_mutex)
        return ESP_ERR_NO_MEM;
    memset(&s_obd_dash, 0, sizeof(s_obd_dash));
    obd_init(&s_obd, _isotp_tx);

    g_obd = &s_obd; /* publish for external access if needed */
    ESP_LOGI(TAG, "# OBD-II engine initialised (req=0x%03X resp=0x%03X)",
             OBD_REQ_ID, OBD_RESP_ID);
#else
    uds_init(&s_uds, _isotp_tx);
    ESP_LOGI(TAG, "# UDS client initialised");
#endif
#endif /* PASSIVE_SNIFF_MODE */

    /* GPS module */
    r = gps_app_start();
    if (r != ESP_OK)
    {
        ESP_LOGW(TAG, "# GPS start failed (%s) — continuing",
                 esp_err_to_name(r));
    }

    /* Tasks */
    xTaskCreatePinnedToCore(_rx_task, "dec_rx",
                            TASK_STACK_CAN_RX, NULL, TASK_PRIO_CAN_RX, NULL, TASK_CORE_CAN);

    xTaskCreatePinnedToCore(_dashboard_task, "dec_dash",
                            TASK_STACK_APP, NULL, TASK_PRIO_APP, NULL, TASK_CORE_APP);

    xTaskCreatePinnedToCore(_health_task, "dec_hlth",
                            TASK_STACK_APP, NULL, TASK_PRIO_APP, NULL, TASK_CORE_APP);

#if !defined(PASSIVE_SNIFF_MODE)
#ifdef REAL_CAR_MODE
    xTaskCreatePinnedToCore(_obd_poller_task, "dec_obd",
                            TASK_STACK_APP, NULL, TASK_PRIO_APP, NULL, TASK_CORE_APP);
#else
    xTaskCreatePinnedToCore(_uds_test_task, "dec_uds",
                            TASK_STACK_APP, NULL, TASK_PRIO_APP, NULL, TASK_CORE_APP);
#endif
#endif /* !PASSIVE_SNIFF_MODE */

    return ESP_OK;
}