/**
 * @file error_handler.h
 * @brief CAN Error Handler — bus-off recovery, ECU timeout, TX overflow
 *
 * Three independent subsystems, all driven by a single periodic tick task
 * running at 10 ms resolution on Core 0 alongside the CAN I/O tasks.
 *
 * ┌─────────────────────────────────────────────────────────────────────┐
 * │  SUBSYSTEM 1 — Bus-Off Recovery                                     │
 * │                                                                     │
 * │  MCP2515 enters bus-off when TEC > 255 (too many TX errors).        │
 * │  Recovery sequence:                                                 │
 * │    1. Detect EFLG register bit 5 (TXBO) set                        │
 * │    2. Switch to CONFIG mode (stops TX/RX, clears error counters)    │
 * │    3. Wait ERR_BUSOFF_RECOVERY_MS (default 500 ms)                  │
 * │    4. Return to NORMAL mode                                         │
 * │    5. Increment busoff_count, log event                             │
 * │  If bus-off recurs ERR_BUSOFF_MAX_RETRIES times without a clean     │
 * │  period, the bus is flagged as permanently faulted.                 │
 * │                                                                     │
 * │  SUBSYSTEM 2 — ECU Timeout                                          │
 * │                                                                     │
 * │  Each ECU entry has a last_seen_ms timestamp (updated by            │
 * │  ecu_manager_update_seen). The error handler checks every           │
 * │  ERR_ECU_CHECK_INTERVAL_MS whether any known ECU has been silent    │
 * │  for longer than ERR_ECU_TIMEOUT_MS. If so, it marks the ECU       │
 * │  inactive and fires the user-supplied timeout callback.             │
 * │                                                                     │
 * │  SUBSYSTEM 3 — TX Overflow                                          │
 * │                                                                     │
 * │  When the TX queue is full and a new frame arrives, the oldest      │
 * │  pending frame is dropped, tx_overflow_count is incremented, and    │
 * │  a warning is logged. The caller gets ESP_ERR_NO_MEM so it can      │
 * │  decide whether to retry or discard.                                │
 * └─────────────────────────────────────────────────────────────────────┘
 *
 * Usage:
 *   err_handler_init(&s_err, &s_ecu_registry);
 *   err_handler_start();                    // spawns tick task
 *
 *   // In can_rx_task, after mcp2515_receive_frame():
 *   err_handler_notify_frame(ecu_resp_id);  // resets that ECU's timer
 *
 *   // Before mcp2515_transmit_frame():
 *   err_handler_tx_submit(&frame);          // returns ESP_ERR_NO_MEM on overflow
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "mcp2515.h"
#include "ecu_manager.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ── Tuneable constants ─────────────────────────────────────────────────── */

/** How long to stay in CONFIG mode during bus-off recovery (ms). */
#ifndef ERR_BUSOFF_RECOVERY_MS
#  define ERR_BUSOFF_RECOVERY_MS        500u
#endif

/** Give up and declare permanent bus fault after this many bus-off events. */
#ifndef ERR_BUSOFF_MAX_RETRIES
#  define ERR_BUSOFF_MAX_RETRIES        5u
#endif

/**
 * Mark an ECU as timed-out if no frame seen for this long (ms).
 * Default 5 s — covers normal 100 ms broadcast ECUs with generous margin.
 */
#ifndef ERR_ECU_TIMEOUT_MS
#  define ERR_ECU_TIMEOUT_MS            5000u
#endif

/** How often to scan for timed-out ECUs (ms). Keep ≥ 100 ms. */
#ifndef ERR_ECU_CHECK_INTERVAL_MS
#  define ERR_ECU_CHECK_INTERVAL_MS     500u
#endif

/** TX overflow ring: max frames queued before oldest is dropped. */
#ifndef ERR_TX_QUEUE_DEPTH
#  define ERR_TX_QUEUE_DEPTH            16u
#endif

/** TEC/REC warning threshold before error-passive state at 128. */
#ifndef ERR_COUNT_WARN_THRESHOLD
#  define ERR_COUNT_WARN_THRESHOLD      96u
#endif

/** Error handler tick task period (ms). All subsystems run on this clock. */
#define ERR_TICK_MS                     10u

/** Task defaults. The app may override these before including this header. */
#ifndef TASK_STACK_CAN_RX
#  define TASK_STACK_CAN_RX             4096
#endif
#ifndef TASK_PRIO_CAN_RX
#  define TASK_PRIO_CAN_RX              15
#endif
#ifndef TASK_CORE_CAN
#  define TASK_CORE_CAN                 0
#endif

/* ── Error event codes ──────────────────────────────────────────────────── */

typedef enum {
    ERR_EVENT_BUSOFF_DETECTED   = 0x01,  /**< MCP2515 entered bus-off       */
    ERR_EVENT_BUSOFF_RECOVERED  = 0x02,  /**< Bus-off recovery succeeded    */
    ERR_EVENT_BUSOFF_PERMANENT  = 0x03,  /**< Retries exhausted, bus dead   */
    ERR_EVENT_ECU_TIMEOUT       = 0x04,  /**< ECU silent > ERR_ECU_TIMEOUT  */
    ERR_EVENT_ECU_RECOVERED     = 0x05,  /**< Timed-out ECU sent a frame    */
    ERR_EVENT_TX_OVERFLOW       = 0x06,  /**< TX queue full, frame dropped  */
    ERR_EVENT_REC_HIGH          = 0x07,  /**< REC > 96 (warning threshold)  */
    ERR_EVENT_TEC_HIGH          = 0x08,  /**< TEC > 96 (warning threshold)  */
} err_event_t;

/* ── Callback ───────────────────────────────────────────────────────────── */

/**
 * @brief User callback fired on any error event.
 *
 * Called from the error handler tick task (Core 0, prio TASK_PRIO_CAN_RX-1).
 * MUST NOT block. MUST NOT call mcp2515_transmit_frame() or re-enter the
 * error handler.
 *
 * @param event    What happened.
 * @param can_id   Relevant CAN ID (ECU response ID for timeout events,
 *                 frame ID for TX overflow, 0 for bus-level events).
 * @param ctx      User pointer supplied at init.
 */
typedef void (*err_callback_t)(err_event_t event, uint32_t can_id, void *ctx);

/* ── Per-ECU tracking ───────────────────────────────────────────────────── */

/**
 * @brief Timeout tracking entry, one per registered ECU.
 */
typedef struct {
    uint32_t  response_id;       /**< ECU response CAN ID                  */
    uint32_t  last_seen_ms;      /**< esp_log_timestamp at last frame       */
    bool      timed_out;         /**< true = currently in timeout state     */
    bool      enabled;           /**< false = slot unused                   */
} err_ecu_track_t;

/* ── TX overflow queue ──────────────────────────────────────────────────── */

/**
 * @brief Simple ring buffer for pending TX frames.
 *
 * err_handler_tx_submit() enqueues. The tick task dequeues and calls
 * mcp2515_transmit_frame(). On overflow the HEAD (oldest) is dropped.
 */
typedef struct {
    can_frame_t  frames[ERR_TX_QUEUE_DEPTH];
    uint8_t      head;           /**< Next frame to transmit               */
    uint8_t      tail;           /**< Next free slot                       */
    uint8_t      count;          /**< Frames currently queued              */
    uint32_t     overflow_count; /**< Cumulative dropped frames            */
} err_tx_queue_t;

/* ── Error handler instance ─────────────────────────────────────────────── */

typedef struct {
    /* References */
    ecu_registry_t  *registry;           /**< Must outlive this struct      */

    /* Bus-off state */
    bool             bus_off;            /**< Currently in bus-off?         */
    bool             bus_faulted;        /**< Permanent fault declared?     */
    uint8_t          busoff_count;       /**< Lifetime bus-off events       */
    uint32_t         recovery_start_ms; /**< Timestamp when recovery began  */

    /* Error counters */
    uint8_t          tec;                /**< Last read TX error counter    */
    uint8_t          rec;                /**< Last read RX error counter    */

    /* ECU timeout tracking */
    err_ecu_track_t  ecu_track[ECU_MAX_ENTRIES];
    uint8_t          ecu_track_count;
    uint32_t         last_ecu_check_ms;

    /* TX overflow queue */
    err_tx_queue_t   tx_queue;

    /* Callback */
    err_callback_t   callback;
    void            *callback_ctx;

    /* Statistics */
    uint32_t         total_errors;
    uint32_t         ecu_timeout_count;
    uint32_t         ecu_recovery_count;

} err_handler_t;

/* ── API ────────────────────────────────────────────────────────────────── */

/**
 * @brief Initialise the error handler.
 *
 * Must be called after ecu_manager_init() — it seeds the ECU timeout
 * tracking table from the registry.
 *
 * @param e         Error handler instance (static storage, zero before call).
 * @param registry  Pointer to the application's ecu_registry_t.
 * @param callback  Optional event callback (NULL to disable).
 * @param ctx       Passed verbatim to callback.
 */
void err_handler_init(err_handler_t    *e,
                      ecu_registry_t   *registry,
                      err_callback_t    callback,
                      void             *ctx);

/**
 * @brief Start the error handler tick task.
 *
 * Spawns a FreeRTOS task pinned to TASK_CORE_CAN at priority
 * (TASK_PRIO_CAN_RX - 1) = 14.  Call once after err_handler_init().
 *
 * @return ESP_OK on success.
 */
esp_err_t err_handler_start(err_handler_t *e);

/**
 * @brief Notify the error handler that a frame was received from an ECU.
 *
 * Call this from the RX task after every successful mcp2515_receive_frame(),
 * BEFORE calling dispatcher_dispatch().  Resets the ECU's timeout timer
 * and clears the timed_out flag if it was set.
 *
 * @param e       Error handler instance.
 * @param can_id  The received frame's CAN ID.
 */
void err_handler_notify_frame(err_handler_t *e, uint32_t can_id);

/**
 * @brief Submit a frame for transmission via the overflow-safe TX queue.
 *
 * Enqueues the frame. If the queue is full, drops the OLDEST pending frame
 * (head), logs a warning, and increments tx_overflow_count.
 *
 * The tick task dequeues and calls mcp2515_transmit_frame() for you.
 * Do NOT call mcp2515_transmit_frame() directly if using this function.
 *
 * @return ESP_OK        Frame queued successfully.
 * @return ESP_ERR_NO_MEM Queue was full; oldest frame was dropped to make room.
 */
esp_err_t err_handler_tx_submit(err_handler_t *e, const can_frame_t *frame);

/**
 * @brief Print a full error report to UART (ESP_LOGI).
 */
void err_handler_print_status(const err_handler_t *e);

/**
 * @brief Return true if the bus is currently in a permanent fault state.
 */
bool err_handler_bus_faulted(const err_handler_t *e);

#ifdef __cplusplus
}
#endif
