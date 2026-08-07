/**
 * @file dispatcher.h
 * @brief CAN Frame Dispatcher — hash-map based O(1) frame routing
 *
 * Architecture:
 *   - Fixed 64-slot open-addressing hash table (no heap allocation)
 *   - Hash: multiplicative hash (id * 2654435761) >> 26
 *   - Linear probing on collision
 *   - One default handler for unregistered IDs
 *   - Thread-safe: caller must hold SPI mutex or call from single task context
 *
 * Usage:
 *   dispatcher_init();
 *   dispatcher_register(0x7E8, engine_handler);
 *   dispatcher_register(0x7E9, bcm_handler);
 *   ...
 *   // In can_rx_task:
 *   dispatcher_dispatch(&frame);
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "mcp2515.h"   /* can_frame_t */

#ifdef __cplusplus
extern "C" {
#endif

/* ── Constants ─────────────────────────────────────────────────────────── */

#define DISPATCHER_TABLE_SIZE   64u   /* must be power of 2 */
#define DISPATCHER_HASH_SHIFT   26u   /* 32 - log2(TABLE_SIZE) */

/* Fibonacci multiplicative hash constant (Knuth) */
#define DISPATCHER_HASH_KNUTH   0x9E3779B9u

/* ── Types ──────────────────────────────────────────────────────────────── */

/**
 * @brief Frame handler callback.
 *
 * Called by dispatcher_dispatch() when a frame with a registered ID arrives.
 * MUST NOT block, MUST NOT re-acquire SPI mutex, MUST NOT call vTaskDelay.
 *
 * @param frame  Pointer to the received CAN frame (read-only).
 * @param ctx    User context pointer supplied at registration.
 */
typedef void (*dispatcher_handler_t)(const can_frame_t *frame, void *ctx);

/**
 * @brief One slot in the dispatch table.
 */
typedef struct {
    uint32_t              can_id;     /**< Registered CAN ID (0 = empty slot) */
    dispatcher_handler_t  handler;    /**< Callback for this ID              */
    void                 *ctx;        /**< User context passed to handler     */
    bool                  occupied;   /**< true when slot is in use           */
} dispatcher_slot_t;

/**
 * @brief Dispatcher instance.
 *
 * Embed one of these per logical bus. Zero-initialise before calling
 * dispatcher_init().
 */
typedef struct {
    dispatcher_slot_t     table[DISPATCHER_TABLE_SIZE];
    dispatcher_handler_t  default_handler;   /**< Called for unknown IDs  */
    void                 *default_ctx;
    uint32_t              dispatch_count;    /**< Total frames dispatched */
    uint32_t              hit_count;         /**< Frames with known ID    */
    uint32_t              miss_count;        /**< Frames with unknown ID  */
    uint32_t              collision_count;   /**< Hash collisions probed  */
} dispatcher_t;

/* ── API ────────────────────────────────────────────────────────────────── */

/**
 * @brief Initialise a dispatcher instance (zeroes all slots).
 */
void dispatcher_init(dispatcher_t *d);

/**
 * @brief Register a handler for a specific CAN ID.
 *
 * @param d        Dispatcher instance.
 * @param can_id   CAN frame ID to match (11-bit standard, 0x000–0x7FF).
 * @param handler  Callback to invoke on match.
 * @param ctx      Arbitrary user pointer forwarded to handler (may be NULL).
 * @return true    Registration succeeded.
 * @return false   Table full (all 64 slots occupied).
 */
bool dispatcher_register(dispatcher_t *d, uint32_t can_id,
                         dispatcher_handler_t handler, void *ctx);

/**
 * @brief Unregister a handler for a specific CAN ID.
 *
 * NOTE: open-addressing tables cannot simply clear a slot — this marks the
 * slot as a tombstone so probing chains remain intact.
 *
 * @return true  Entry found and removed.
 * @return false ID was not registered.
 */
bool dispatcher_unregister(dispatcher_t *d, uint32_t can_id);

/**
 * @brief Dispatch a received frame to its registered handler.
 *
 * Performs one hash lookup + linear probe, then calls the handler.
 * If no handler is registered, calls default_handler (if set).
 * Updates dispatch_count, hit_count, miss_count.
 *
 * @param d      Dispatcher instance.
 * @param frame  Frame to dispatch (must not be NULL).
 */
void dispatcher_dispatch(dispatcher_t *d, const can_frame_t *frame);

/**
 * @brief Set the default handler (called when frame ID has no registration).
 *
 * Pass NULL to disable default handling (unknown frames silently dropped).
 */
void dispatcher_set_default(dispatcher_t *d,
                             dispatcher_handler_t handler, void *ctx);

/**
 * @brief Look up the handler for a CAN ID without dispatching.
 *
 * @return Pointer to slot, or NULL if not found.
 */
const dispatcher_slot_t *dispatcher_lookup(const dispatcher_t *d,
                                            uint32_t can_id);

/**
 * @brief Print dispatcher statistics to UART (ESP_LOGI).
 */
void dispatcher_print_stats(const dispatcher_t *d);

#ifdef __cplusplus
}
#endif