/**
 * @file error_handler.c
 * @brief CAN Error Handler — bus-off recovery, ECU timeout, TX overflow
 *
 * All three subsystems run from a single 10 ms tick task on Core 0.
 *
 * Tick task flow (every ERR_TICK_MS):
 *   1. Read MCP2515 EFLG + TEC/REC
 *   2. If bus-off detected → start recovery sequence (state machine)
 *   3. If in recovery wait → check if wait period has elapsed → restore NORMAL
 *   4. If bus healthy → warn on high TEC/REC
 *   5. If ECU check interval elapsed → scan for timed-out ECUs
 *   6. Dequeue and transmit one pending TX frame (if bus not in bus-off)
 */

#include "error_handler.h"
#include "mcp2515.h"
#include "mcp2515_regs.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <inttypes.h>
#include <string.h>

static const char *TAG = "ERR_HANDLER";

/* ── MCP2515 EFLG bit masks (from mcp2515_regs.h) ──────────────────────── */
/* If your mcp2515_regs.h uses different names, adjust these defines.       */
#ifndef MCP_EFLG_TXBO
#  define MCP_EFLG_TXBO   (1u << 5)   /* Bus-Off state                    */
#endif
#ifndef MCP_EFLG_TXEP
#  define MCP_EFLG_TXEP   (1u << 6)   /* TX Error-Passive (TEC ≥ 128)     */
#endif
#ifndef MCP_EFLG_RXEP
#  define MCP_EFLG_RXEP   (1u << 7)   /* RX Error-Passive (REC ≥ 128)     */
#endif

/* ── Internal helpers ───────────────────────────────────────────────────── */

static inline uint32_t _now_ms(void)
{
    return (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
}

static void _fire(err_handler_t *e, err_event_t event, uint32_t can_id)
{
    e->total_errors++;
    if (e->callback) {
        e->callback(event, can_id, e->callback_ctx);
    }
}

/* ── Subsystem 1: Bus-Off Recovery ──────────────────────────────────────── */

/**
 * Called every tick when EFLG.TXBO is set and we're not already recovering.
 */
static void _busoff_start_recovery(err_handler_t *e)
{
    if (e->bus_faulted) return;

    e->bus_off           = true;
    e->busoff_count++;
    e->recovery_start_ms = _now_ms();

    ESP_LOGE(TAG, "BUS-OFF detected! (event #%u / max %u)",
             e->busoff_count, ERR_BUSOFF_MAX_RETRIES);

    if (e->busoff_count >= ERR_BUSOFF_MAX_RETRIES) {
        e->bus_faulted = true;
        ESP_LOGE(TAG, "BUS PERMANENTLY FAULTED — %u bus-off events. "
                 "Check wiring, termination, and node count.",
                 e->busoff_count);
        _fire(e, ERR_EVENT_BUSOFF_PERMANENT, 0);
        return;
    }

    _fire(e, ERR_EVENT_BUSOFF_DETECTED, 0);

    /*
     * Step 1: Enter CONFIG mode.
     * This halts TX/RX and resets TEC/REC to 0 inside the MCP2515.
     * The bus itself recovers after 128 recessive bits (handled by hardware
     * once we return to NORMAL mode).
     */
    esp_err_t err = mcp2515_set_mode(MCP_MODE_CONFIG);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to enter CONFIG during bus-off recovery (%d)", err);
    } else {
        ESP_LOGW(TAG, "Recovery: CONFIG mode entered, waiting %u ms ...",
                 ERR_BUSOFF_RECOVERY_MS);
    }
}

/**
 * Called every tick while bus_off == true.
 * Returns to NORMAL once the recovery wait has elapsed.
 */
static void _busoff_tick_recovery(err_handler_t *e)
{
    if (e->bus_faulted) return;

    uint32_t elapsed = _now_ms() - e->recovery_start_ms;
    if (elapsed < ERR_BUSOFF_RECOVERY_MS) return;

    /* Wait period over — attempt return to NORMAL */
    esp_err_t err = mcp2515_set_mode(MCP_MODE_NORMAL);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Recovery: failed to re-enter NORMAL mode (%d) — retrying", err);
        /* Reset timer for another wait cycle */
        e->recovery_start_ms = _now_ms();
        return;
    }

    e->bus_off = false;
    ESP_LOGI(TAG, "Recovery: NORMAL mode restored after %"PRIu32" ms (total bus-off events: %u)",
             elapsed, e->busoff_count);

    _fire(e, ERR_EVENT_BUSOFF_RECOVERED, 0);
}

/* ── Subsystem 2: ECU Timeout ────────────────────────────────────────────── */

static void _ecu_timeout_tick(err_handler_t *e)
{
    uint32_t now = _now_ms();

    if ((now - e->last_ecu_check_ms) < ERR_ECU_CHECK_INTERVAL_MS) return;
    e->last_ecu_check_ms = now;

    for (uint8_t i = 0; i < e->ecu_track_count; i++) {
        err_ecu_track_t *t = &e->ecu_track[i];
        if (!t->enabled) continue;

        /*
         * Skip ECUs that have never been seen (last_seen_ms == 0).
         * They are "not yet active" rather than "timed out" — a meaningful
         * distinction for optional ECUs like GPL or Transmission.
         */
        if (t->last_seen_ms == 0) continue;

        uint32_t silent_ms = now - t->last_seen_ms;

        if (!t->timed_out && silent_ms > ERR_ECU_TIMEOUT_MS) {
            t->timed_out = true;
            e->ecu_timeout_count++;

            /* Mark inactive in the registry too */
            for (uint8_t j = 0; j < e->registry->count; j++) {
                if (e->registry->ecus[j].response_id == t->response_id) {
                    e->registry->ecus[j].active = false;
                    ESP_LOGW(TAG, "ECU TIMEOUT: %s (0x%03"PRIX32") — silent for %"PRIu32" ms",
                             e->registry->ecus[j].name,
                             t->response_id,
                             silent_ms);
                    break;
                }
            }
            _fire(e, ERR_EVENT_ECU_TIMEOUT, t->response_id);
        }
    }
}

/* ── Subsystem 3: TX Overflow Queue ─────────────────────────────────────── */

/**
 * Dequeue one frame and transmit it.
 * Called every tick when the bus is healthy and the queue is non-empty.
 * One frame per tick = 10 ms spacing max — avoids hammering a congested bus.
 */
static void _tx_queue_tick(err_handler_t *e)
{
    if (e->bus_off || e->bus_faulted) return;

    err_tx_queue_t *q = &e->tx_queue;
    if (q->count == 0) return;

    can_frame_t *f = &q->frames[q->head];
    esp_err_t    err = mcp2515_transmit_frame(f, ERR_TICK_MS);

    if (err == ESP_OK) {
        q->head  = (q->head + 1) % ERR_TX_QUEUE_DEPTH;
        q->count--;
    } else {
        /*
         * TX failed (e.g. all TX buffers busy).
         * Leave frame in queue — will retry next tick.
         * Don't log here to avoid flooding; the bus-off detector will
         * catch sustained TX failures.
         */
    }
}

/* ── Main tick task ─────────────────────────────────────────────────────── */

static void _err_tick_task(void *arg)
{
    err_handler_t *e = (err_handler_t *)arg;

    ESP_LOGI(TAG, "Error handler tick task started (period=%u ms, core=%d)",
             ERR_TICK_MS, xPortGetCoreID());

    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(ERR_TICK_MS));

        /* ── If permanently faulted, just sit and log periodically ──────── */
        if (e->bus_faulted) {
            /* Log every 10 s so the developer knows the state */
            static uint32_t fault_log_count = 0;
            if (++fault_log_count % (10000 / ERR_TICK_MS) == 0) {
                ESP_LOGE(TAG, "BUS PERMANENTLY FAULTED — system halted. "
                         "Reset to recover.");
            }
            continue;
        }

        /* ── If in recovery, tick the recovery state machine ────────────── */
        if (e->bus_off) {
            _busoff_tick_recovery(e);
            /* Don't check anything else while recovering */
            continue;
        }

        /* ── Read MCP2515 error registers ──────────────────────────────── */
        uint8_t eflg = 0, tec = 0, rec = 0;
        eflg = mcp2515_read_reg(MCP2515_REG_EFLG);
        tec  = mcp2515_read_reg(MCP2515_REG_TEC);
        rec  = mcp2515_read_reg(MCP2515_REG_REC);
        e->tec = tec;
        e->rec = rec;

        /* ── Check for bus-off ──────────────────────────────────────────── */
        if (eflg & MCP_EFLG_TXBO) {
            _busoff_start_recovery(e);
            continue;
        }

        /* ── Warn on high error counters ────────────────────────────────── */
        if (tec > ERR_COUNT_WARN_THRESHOLD) {
            ESP_LOGW(TAG, "TEC=%u (approaching error-passive threshold)", tec);
            _fire(e, ERR_EVENT_TEC_HIGH, 0);
        }
        if (rec > ERR_COUNT_WARN_THRESHOLD) {
            ESP_LOGW(TAG, "REC=%u (approaching error-passive threshold)", rec);
            _fire(e, ERR_EVENT_REC_HIGH, 0);
        }

        /* ── ECU timeout check ──────────────────────────────────────────── */
        _ecu_timeout_tick(e);

        /* ── TX queue drain ─────────────────────────────────────────────── */
        _tx_queue_tick(e);
    }
}

/* ── Public API ─────────────────────────────────────────────────────────── */

void err_handler_init(err_handler_t  *e,
                      ecu_registry_t *registry,
                      err_callback_t  callback,
                      void           *ctx)
{
    if (!e || !registry) return;
    memset(e, 0, sizeof(*e));

    e->registry     = registry;
    e->callback     = callback;
    e->callback_ctx = ctx;

    /*
     * Seed the ECU timeout tracking table from the registry.
     * Every ECU with a non-zero response_id gets a tracking entry.
     * last_seen_ms = 0 means "never seen" (not a timeout).
     */
    for (uint8_t i = 0; i < registry->count && i < ECU_MAX_ENTRIES; i++) {
        if (registry->ecus[i].response_id != 0) {
            e->ecu_track[e->ecu_track_count].response_id  = registry->ecus[i].response_id;
            e->ecu_track[e->ecu_track_count].last_seen_ms = 0;
            e->ecu_track[e->ecu_track_count].timed_out    = false;
            e->ecu_track[e->ecu_track_count].enabled      = true;
            e->ecu_track_count++;
        }
    }

    ESP_LOGI(TAG, "Error handler initialised — tracking %u ECUs", e->ecu_track_count);
    ESP_LOGI(TAG, "  Bus-off max retries : %u", ERR_BUSOFF_MAX_RETRIES);
    ESP_LOGI(TAG, "  Bus-off recovery ms : %u", ERR_BUSOFF_RECOVERY_MS);
    ESP_LOGI(TAG, "  ECU timeout ms      : %u", ERR_ECU_TIMEOUT_MS);
    ESP_LOGI(TAG, "  TX queue depth      : %u", ERR_TX_QUEUE_DEPTH);
}

esp_err_t err_handler_start(err_handler_t *e)
{
    if (!e) return ESP_ERR_INVALID_ARG;

    BaseType_t ret = xTaskCreatePinnedToCore(
        _err_tick_task,
        "err_tick",
        TASK_STACK_CAN_RX,          /* reuse CAN RX stack size from project_config.h */
        (void *)e,
        TASK_PRIO_CAN_RX - 1,      /* prio 14: below CAN RX (15), above app (5) */
        NULL,
        TASK_CORE_CAN               /* Core 0: alongside all CAN I/O tasks */
    );

    if (ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create err_tick task");
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Error handler tick task started (core=%d, prio=%d)",
             TASK_CORE_CAN, TASK_PRIO_CAN_RX - 1);
    return ESP_OK;
}

void err_handler_notify_frame(err_handler_t *e, uint32_t can_id)
{
    if (!e) return;

    uint32_t now = _now_ms();

    for (uint8_t i = 0; i < e->ecu_track_count; i++) {
        err_ecu_track_t *t = &e->ecu_track[i];
        if (!t->enabled || t->response_id != can_id) continue;

        t->last_seen_ms = now;

        /* If this ECU was in timeout, fire recovery event */
        if (t->timed_out) {
            t->timed_out = false;
            e->ecu_recovery_count++;

            /* Mark active in registry */
            for (uint8_t j = 0; j < e->registry->count; j++) {
                if (e->registry->ecus[j].response_id == can_id) {
                    e->registry->ecus[j].active = true;
                    ESP_LOGI(TAG, "ECU RECOVERED: %s (0x%03"PRIX32")",
                             e->registry->ecus[j].name, can_id);
                    break;
                }
            }
            _fire(e, ERR_EVENT_ECU_RECOVERED, can_id);
        }
        break;
    }
}

esp_err_t err_handler_tx_submit(err_handler_t *e, const can_frame_t *frame)
{
    if (!e || !frame) return ESP_ERR_INVALID_ARG;

    err_tx_queue_t *q = &e->tx_queue;
    esp_err_t       result = ESP_OK;

    if (q->count >= ERR_TX_QUEUE_DEPTH) {
        /*
         * Queue full — drop the OLDEST frame (head) to make room.
         * Oldest frame is sacrificed because it's most likely stale.
         */
        q->head = (q->head + 1) % ERR_TX_QUEUE_DEPTH;
        q->count--;
        q->overflow_count++;

        ESP_LOGW(TAG, "TX overflow! Dropped oldest frame. "
                 "Total dropped: %"PRIu32, q->overflow_count);

        _fire(e, ERR_EVENT_TX_OVERFLOW, frame->id);
        result = ESP_ERR_NO_MEM;   /* Tell caller a drop occurred */
    }

    /* Enqueue the new frame at tail */
    q->frames[q->tail] = *frame;
    q->tail  = (q->tail + 1) % ERR_TX_QUEUE_DEPTH;
    q->count++;

    return result;
}

void err_handler_print_status(const err_handler_t *e)
{
    if (!e) return;

    ESP_LOGI(TAG, "═══════════════════════════════════════════════════");
    ESP_LOGI(TAG, "  Error Handler Status");
    ESP_LOGI(TAG, "═══════════════════════════════════════════════════");

    /* Bus state */
    if (e->bus_faulted) {
        ESP_LOGE(TAG, "  BUS STATE      : *** PERMANENT FAULT ***");
    } else if (e->bus_off) {
        ESP_LOGW(TAG, "  BUS STATE      : RECOVERING (bus-off)");
    } else {
        ESP_LOGI(TAG, "  BUS STATE      : NORMAL");
    }

    ESP_LOGI(TAG, "  TEC / REC      : %u / %u", e->tec, e->rec);
    ESP_LOGI(TAG, "  Bus-off events : %u", e->busoff_count);
    ESP_LOGI(TAG, "  Total errors   : %"PRIu32, e->total_errors);

    /* TX queue */
    ESP_LOGI(TAG, "  TX queue depth : %u / %u frames",
             e->tx_queue.count, ERR_TX_QUEUE_DEPTH);
    ESP_LOGI(TAG, "  TX dropped     : %"PRIu32, e->tx_queue.overflow_count);

    /* ECU timeout tracking */
    ESP_LOGI(TAG, "─── ECU Timeout Tracking ───────────────────────────");
    ESP_LOGI(TAG, "  Timeouts       : %"PRIu32, e->ecu_timeout_count);
    ESP_LOGI(TAG, "  Recoveries     : %"PRIu32, e->ecu_recovery_count);

    for (uint8_t i = 0; i < e->ecu_track_count; i++) {
        const err_ecu_track_t *t = &e->ecu_track[i];
        if (!t->enabled) continue;

        const char *state;
        if (t->last_seen_ms == 0) {
            state = "NEVER SEEN";
        } else if (t->timed_out) {
            state = "TIMED OUT";
        } else {
            state = "OK";
        }

        uint32_t age = (t->last_seen_ms > 0)
                       ? (_now_ms() - t->last_seen_ms)
                       : 0;

        ESP_LOGI(TAG, "  [0x%03"PRIX32"] %-10s  last seen: %"PRIu32" ms ago",
                 t->response_id, state, age);
    }
    ESP_LOGI(TAG, "═══════════════════════════════════════════════════");
}

bool err_handler_bus_faulted(const err_handler_t *e)
{
    return e ? e->bus_faulted : true;
}
