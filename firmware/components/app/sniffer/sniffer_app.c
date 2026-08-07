/**
 * @file sniffer_app.c
 * @brief Passive CAN Sniffer implementation
 *
 * CSV output goes to UART0 via printf() — redirect with:
 *   pio device monitor --environment node_b --raw > capture.csv
 *
 * Stats output is prefixed with '#' so the Python analyzer can skip it:
 *   Lines starting with '#' are comments, ignored by the CSV parser.
 */

#include "sniffer_app.h"
#include "../../../src/project_config.h"
#include "mcp2515.h"
#include "mcp2515_regs.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "cJSON.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

static const char *TAG = "SNIFFER";

/* ── Default config ─────────────────────────────────────────────────────── */

static const sniffer_config_t s_default_cfg = {
    .csv_header         = true,
    .id_filter_min      = 0x000u,
    .id_filter_max      = 0x7FFu,
    .stats_interval_ms  = 60000u,  /* 60s — avoids interrupting short captures */
    .max_unique_ids     = SNIFFER_MAX_UNIQUE_IDS,
};

/* ── Global sniffer pointer (used by tasks) ─────────────────────────────── */

static sniffer_t *s_sniffer = NULL;

/* ── Internal: ID stats lookup / insert ────────────────────────────────── */

static sniffer_id_stats_t *_get_or_create(sniffer_t *s, uint32_t can_id)
{
    /* Linear scan — ≤256 entries, fast enough */
    for (uint16_t i = 0; i < s->id_count; i++) {
        if (s->id_stats[i].valid && s->id_stats[i].can_id == can_id) {
            return &s->id_stats[i];
        }
    }

    /* New ID */
    if (s->id_count >= s->cfg.max_unique_ids ||
        s->id_count >= SNIFFER_MAX_UNIQUE_IDS) {
        return NULL;   /* Table full — still log CSV, just no stats */
    }

    sniffer_id_stats_t *e = &s->id_stats[s->id_count++];
    memset(e, 0, sizeof(*e));
    e->can_id       = can_id;
    e->valid        = true;
    e->min_period_us = UINT32_MAX;

    /* Initialise byte min/max to sentinel */
    for (uint8_t b = 0; b < SNIFFER_BYTE_HISTORY; b++) {
        e->byte_min[b] = 0xFF;
        e->byte_max[b] = 0x00;
        e->counter_delta[b] = 0;
    }
    return e;
}

/* ── Internal: update per-byte stats ───────────────────────────────────── */

static void _update_byte_stats(sniffer_id_stats_t *e,
                                const uint8_t *data, uint8_t dlc)
{
    for (uint8_t b = 0; b < dlc && b < SNIFFER_BYTE_HISTORY; b++) {
        uint8_t v = data[b];

        if (!e->byte_seen[b]) {
            e->byte_min[b]  = v;
            e->byte_max[b]  = v;
            e->byte_last[b] = v;
            e->byte_seen[b] = true;
            /* Can't determine delta yet — need two frames */
            e->counter_candidate[b] = true;   /* optimistic until disproved */
            e->counter_delta[b]     = 0;       /* unknown */
            continue;
        }

        /* Update range */
        if (v < e->byte_min[b]) e->byte_min[b] = v;
        if (v > e->byte_max[b]) e->byte_max[b] = v;

        /* Counter detection: check if delta is consistent */
        if (e->counter_candidate[b]) {
            int16_t delta = (int16_t)v - (int16_t)e->byte_last[b];

            /* Wrap-around: 0xFF→0x00 is delta=-255 → treat as +1 */
            if (delta == -255) delta = 1;
            /* Wrap-around: 0x00→0xFF is delta=+255 → treat as -1 */
            if (delta == 255)  delta = -1;

            if (e->frame_count == 1) {
                /* First comparison — set expected delta */
                if (delta == 1 || delta == -1) {
                    e->counter_delta[b] = (int8_t)delta;
                } else {
                    /* Non-unit delta on second frame — not a counter */
                    e->counter_candidate[b] = false;
                }
            } else {
                /* Subsequent frames — delta must be consistent */
                if (delta != e->counter_delta[b]) {
                    e->counter_candidate[b] = false;
                }
            }
        }

        e->byte_last[b] = v;
    }
}

/* ── Internal: update timing stats ─────────────────────────────────────── */

static void _update_timing(sniffer_id_stats_t *e, uint32_t now_us)
{
    if (e->frame_count == 0) {
        e->first_seen_us = now_us;
        e->last_seen_us  = now_us;
        return;
    }

    uint32_t period = now_us - e->last_seen_us;
    e->last_seen_us  = now_us;

    /* Sanity: ignore periods > 60 s (startup / gap artefacts) */
    if (period > 60000000u) return;

    if (period < e->min_period_us) e->min_period_us = period;
    if (period > e->max_period_us) e->max_period_us = period;
    e->period_sum_us += period;
    e->period_count++;
}

/* ── Internal: emit one CSV line ────────────────────────────────────────── */

static void _emit_csv(uint32_t ts_us, const can_frame_t *f)
{
    /*
     * Format: timestamp_us,id_hex,dlc,b0,b1,b2,b3,b4,b5,b6,b7
     * Unused bytes (dlc < 8) printed as 00.
     *
     * MUST use printf() — NOT ESP_LOGI.
     * ESP_LOGI adds "I (NNN) TAG: " prefix AND batches UART output,
     * both of which corrupt timestamps and inflate jitter by ~100ms.
     * printf() writes directly to UART0, one line at a time.
     */
    uint8_t b[8] = {0};
    for (uint8_t i = 0; i < f->dlc && i < 8; i++) b[i] = f->data[i];

    printf("%"PRIu32",%03"PRIX32",%u,"
           "%02X,%02X,%02X,%02X,%02X,%02X,%02X,%02X\n",
           ts_us, f->id, f->dlc,
           b[0], b[1], b[2], b[3], b[4], b[5], b[6], b[7]);
}

/* ── RX task ────────────────────────────────────────────────────────────── */

static void _rx_task(void *arg)
{
    sniffer_t *s = (sniffer_t *)arg;

    /* Critical: register THIS task handle — never NULL */
    mcp2515_register_rx_task(xTaskGetCurrentTaskHandle());

    ESP_LOGI(TAG, "# RX task started on core %d (LISTEN-ONLY mode)",
             xPortGetCoreID());

    for (;;) {
        /* Block until MCP2515 INT fires (500 ms timeout) */
        ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(500));

        /*
         * Drain loop — read CANINTF directly each iteration.
         *
         * We do NOT call mcp2515_receive_frame() here because that function
         * pushes frames into the ring buffer. If we then also use the frame
         * directly, each physical CAN frame gets processed twice (once from
         * the direct read, once left in the ring buffer for the next drain).
         *
         * Instead: read CANINTF, pick the right RX buffer, read registers
         * manually, clear the flag, then process. This is a direct path:
         * MCP2515 hardware → CSV line. The ring buffer is not used by the
         * sniffer — it is used by steps that have a separate app task.
         */
        uint8_t intf;
        while ((intf = mcp2515_read_reg(MCP2515_REG_CANINTF)) &
               (MCP2515_INT_RX0IF | MCP2515_INT_RX1IF)) {

            can_frame_t frame;
            uint8_t sidh_reg, sidl_reg, dlc_reg, d0_reg, clr_bit;

            if (intf & MCP2515_INT_RX0IF) {
                sidh_reg = MCP2515_REG_RXB0SIDH;
                sidl_reg = MCP2515_REG_RXB0SIDL;
                dlc_reg  = MCP2515_REG_RXB0DLC;
                d0_reg   = MCP2515_REG_RXB0D0;
                clr_bit  = MCP2515_INT_RX0IF;
            } else {
                sidh_reg = MCP2515_REG_RXB1SIDH;
                sidl_reg = MCP2515_REG_RXB1SIDL;
                dlc_reg  = MCP2515_REG_RXB1DLC;
                d0_reg   = MCP2515_REG_RXB1D0;
                clr_bit  = MCP2515_INT_RX1IF;
            }

            /* Read frame from hardware */
            uint8_t sidh = mcp2515_read_reg(sidh_reg);
            uint8_t sidl = mcp2515_read_reg(sidl_reg);
            frame.id  = ((uint32_t)sidh << 3) | ((sidl >> 5) & 0x07);
            frame.dlc = mcp2515_read_reg(dlc_reg) & 0x0F;
            if (frame.dlc > 8) frame.dlc = 8;
            for (uint8_t i = 0; i < frame.dlc; i++)
                frame.data[i] = mcp2515_read_reg(d0_reg + i);
            for (uint8_t i = frame.dlc; i < 8; i++)
                frame.data[i] = 0x00;

            /*
             * Clear flags — CRITICAL: must clear ERRIF (bit 5) too.
             *
             * In LISTEN-ONLY mode, ERRIF fires constantly because the chip
             * detects bus activity but cannot send ACK/error frames.
             * If ERRIF stays set after clearing RX0IF, the INT pin stays LOW,
             * the ISR fires again immediately, the task is woken again, and
             * the drain loop re-runs — producing 3-8 duplicate reads of the
             * same frame at ~3.3ms intervals (one SPI burst per duplicate).
             *
             * Fix: write CANINTF keeping ONLY the other RX flag (if pending).
             * This clears RX0IF/RX1IF, ERRIF, MERRF, WAKIF, TXnIF all at once.
             * The INT pin goes HIGH and stays HIGH until the next real frame.
             */
            {
                uint8_t keep = (clr_bit == MCP2515_INT_RX0IF)
                               ? (intf & MCP2515_INT_RX1IF)   /* keep RX1 if also pending */
                               : (intf & MCP2515_INT_RX0IF);  /* keep RX0 if also pending */
                mcp2515_write_reg(MCP2515_REG_CANINTF, keep);
            }

            /* Timestamp after hardware read, before UART */
            uint32_t now_us = (uint32_t)esp_timer_get_time();

            /* Apply ID filter */
            if (frame.id < s->cfg.id_filter_min ||
                frame.id > s->cfg.id_filter_max) {
                continue;
            }

            s->total_frames++;

            /* Emit CSV */
            _emit_csv(now_us, &frame);
            fflush(stdout);

            /* Update stats */
            sniffer_id_stats_t *e = _get_or_create(s, frame.id);
            if (e) {
                _update_timing(e, now_us);
                _update_byte_stats(e, frame.data, frame.dlc);
                e->frame_count++;
            }
        }
    }
}

/* ── Stats task ─────────────────────────────────────────────────────────── */

static void _stats_task(void *arg)
{
    sniffer_t *s = (sniffer_t *)arg;

    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(s->cfg.stats_interval_ms));
        sniffer_print_stats(s);
    }
}

/* ── Public API ─────────────────────────────────────────────────────────── */

void sniffer_init(sniffer_t *s)
{
    if (!s) return;
    memset(s, 0, sizeof(*s));
    s->cfg = s_default_cfg;
}

esp_err_t sniffer_load_config(sniffer_t *s, const char *json_path)
{
    if (!s || !json_path) return ESP_ERR_INVALID_ARG;

    FILE *f = fopen(json_path, "r");
    if (!f) {
        ESP_LOGW(TAG, "# system.json not found at %s — using defaults", json_path);
        return ESP_ERR_NOT_FOUND;
    }

    fseek(f, 0, SEEK_END);
    long size = ftell(f);
    rewind(f);

    if (size <= 0 || size > 4096) {
        fclose(f);
        ESP_LOGW(TAG, "# system.json size %ld invalid — using defaults", size);
        return ESP_FAIL;
    }

    char *buf = (char *)malloc((size_t)size + 1u);
    if (!buf) { fclose(f); return ESP_ERR_NO_MEM; }
    fread(buf, 1, (size_t)size, f);
    fclose(f);
    buf[size] = '\0';

    cJSON *root = cJSON_Parse(buf);
    free(buf);
    if (!root) { ESP_LOGW(TAG, "# system.json parse error — using defaults"); return ESP_FAIL; }

    const cJSON *sn = cJSON_GetObjectItemCaseSensitive(root, "sniffer");
    if (sn) {
        const cJSON *item;

        item = cJSON_GetObjectItemCaseSensitive(sn, "csv_header");
        if (cJSON_IsBool(item)) s->cfg.csv_header = cJSON_IsTrue(item);

        item = cJSON_GetObjectItemCaseSensitive(sn, "id_filter_min");
        if (cJSON_IsString(item)) s->cfg.id_filter_min = (uint32_t)strtoul(item->valuestring, NULL, 0);
        if (cJSON_IsNumber(item)) s->cfg.id_filter_min = (uint32_t)item->valuedouble;

        item = cJSON_GetObjectItemCaseSensitive(sn, "id_filter_max");
        if (cJSON_IsString(item)) s->cfg.id_filter_max = (uint32_t)strtoul(item->valuestring, NULL, 0);
        if (cJSON_IsNumber(item)) s->cfg.id_filter_max = (uint32_t)item->valuedouble;

        item = cJSON_GetObjectItemCaseSensitive(sn, "stats_interval_ms");
        if (cJSON_IsNumber(item)) s->cfg.stats_interval_ms = (uint32_t)item->valuedouble;

        item = cJSON_GetObjectItemCaseSensitive(sn, "max_unique_ids");
        if (cJSON_IsNumber(item)) s->cfg.max_unique_ids = (uint16_t)item->valuedouble;
    }

    cJSON_Delete(root);

    ESP_LOGI(TAG, "# Config loaded from %s", json_path);
    ESP_LOGI(TAG, "#   ID filter : 0x%03"PRIX32" – 0x%03"PRIX32,
             s->cfg.id_filter_min, s->cfg.id_filter_max);
    ESP_LOGI(TAG, "#   Stats every %"PRIu32" ms", s->cfg.stats_interval_ms);
    return ESP_OK;
}

esp_err_t sniffer_start(sniffer_t *s)
{
    if (!s) return ESP_ERR_INVALID_ARG;
    s_sniffer = s;

    /*
     * LISTEN-ONLY mode: MCP2515 mode 0b011.
     * The chip receives frames but never drives the bus.
     * No ACK bit, no error frames — electrically invisible on the bus.
     */
    esp_err_t ret = mcp2515_set_mode(MCP_MODE_LISTEN_ONLY);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "# Failed to set LISTEN-ONLY mode: %s", esp_err_to_name(ret));
        return ret;
    }
    ESP_LOGI(TAG, "# MCP2515 in LISTEN-ONLY mode — bus is READ-ONLY");

    /* Accept all IDs (no hardware filter) */
    mcp2515_set_filter_accept_all();

    /* CSV header — use printf so no "I (NNN) CSV: " prefix appears in file */
    if (s->cfg.csv_header) {
        printf("# CAN Sniffer capture — Renault Clio IV\n");
        printf("# Filter: 0x%03"PRIX32" - 0x%03"PRIX32"\n",
               s->cfg.id_filter_min, s->cfg.id_filter_max);
        printf("timestamp_us,id_hex,dlc,b0,b1,b2,b3,b4,b5,b6,b7\n");
    }

    s->running = true;

    /* RX task: Core 0, highest CAN priority */
    xTaskCreatePinnedToCore(_rx_task, "sniff_rx",
                            TASK_STACK_CAN_RX, s,
                            TASK_PRIO_CAN_RX,  NULL,
                            TASK_CORE_CAN);

    /* Stats task: Core 1, app priority */
    xTaskCreatePinnedToCore(_stats_task, "sniff_stats",
                            TASK_STACK_APP, s,
                            TASK_PRIO_APP,  NULL,
                            TASK_CORE_APP);

    ESP_LOGI(TAG, "# Sniffer started — logging CSV to UART");
    return ESP_OK;
}

void sniffer_print_stats(const sniffer_t *s)
{
    if (!s) return;

    ESP_LOGI(TAG, "# ═══════════════════════════════════════════════════════");
    ESP_LOGI(TAG, "# CAN Bus Statistics  —  %"PRIu64" total frames  |  %u unique IDs",
             s->total_frames, s->id_count);
    if (s->dropped_frames > 0) {
        ESP_LOGW(TAG, "# WARNING: %"PRIu32" frames dropped (ring buffer overflow)",
                 s->dropped_frames);
    }
    ESP_LOGI(TAG, "# ───────────────────────────────────────────────────────");

    for (uint16_t i = 0; i < s->id_count; i++) {
        const sniffer_id_stats_t *e = &s->id_stats[i];
        if (!e->valid) continue;

        /* Period stats */
        uint32_t mean_us = (e->period_count > 0)
                           ? (uint32_t)(e->period_sum_us / e->period_count)
                           : 0;

        /* Period jitter: max deviation from mean */
        uint32_t jitter_us = 0;
        if (mean_us > 0) {
            uint32_t dev_min = (mean_us > e->min_period_us)
                               ? (mean_us - e->min_period_us) : 0;
            uint32_t dev_max = (e->max_period_us > mean_us)
                               ? (e->max_period_us - mean_us) : 0;
            jitter_us = (dev_min > dev_max) ? dev_min : dev_max;
        }

        ESP_LOGI(TAG, "# ID 0x%03"PRIX32" | %"PRIu64" frames | "
                 "period=%"PRIu32"µs ±%"PRIu32"µs  [%"PRIu32"–%"PRIu32"µs]",
                 e->can_id, e->frame_count,
                 mean_us, jitter_us,
                 e->min_period_us, e->max_period_us);

        /* Per-byte summary */
        for (uint8_t b = 0; b < SNIFFER_BYTE_HISTORY; b++) {
            if (!e->byte_seen[b]) continue;

            uint8_t range = e->byte_max[b] - e->byte_min[b];

            char annotation[48] = "";
            if (e->counter_candidate[b] && e->frame_count > 4) {
                snprintf(annotation, sizeof(annotation),
                         " ← rolling counter (%+d/frame)",
                         e->counter_delta[b]);
            } else if (range == 0) {
                snprintf(annotation, sizeof(annotation), " ← constant 0x%02X",
                         e->byte_min[b]);
            } else if (range >= 0x80) {
                snprintf(annotation, sizeof(annotation), " ← full/wide range");
            }

            ESP_LOGI(TAG, "#   Byte %u: range [0x%02X–0x%02X]%s",
                     b, e->byte_min[b], e->byte_max[b], annotation);
        }
    }
    ESP_LOGI(TAG, "# ═══════════════════════════════════════════════════════");
}