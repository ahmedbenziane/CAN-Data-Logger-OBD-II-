#pragma once
/* ============================================================
 *  ring_buffer.h  —  Lock-free ISR-safe CAN frame ring buffer
 *
 *  Single-producer (ISR/rx_task writer) + single-consumer
 *  (app_task reader). No mutex needed — safe by design.
 *
 *  SIZE must be a power of 2. Using 64 = covers 6.4 seconds
 *  of 10fps traffic before overflow at default Step 3 rate.
 *  At 500kbps max (~5000 frames/sec) this covers 12ms bursts.
 * ============================================================ */

#include <stdint.h>
#include <stdbool.h>
#include "mcp2515.h"   /* can_frame_t */

#define RING_BUF_SIZE  64   /* MUST be power of 2 */

typedef struct {
    can_frame_t       buf[RING_BUF_SIZE];
    volatile uint32_t head;          /* writer index — incremented by rx_task  */
    volatile uint32_t tail;          /* reader index — incremented by app_task  */
    volatile uint32_t overflow_cnt;  /* frames dropped (buffer was full)        */
    volatile uint32_t total_rx;      /* total frames successfully received      */
} can_ring_buf_t;

static inline void ring_buf_init(can_ring_buf_t *rb)
{
    rb->head         = 0;
    rb->tail         = 0;
    rb->overflow_cnt = 0;
    rb->total_rx     = 0;
}

/* Push a frame — returns false (and increments overflow) if full */
static inline bool ring_buf_push(can_ring_buf_t *rb,
                                  const can_frame_t *frame)
{
    uint32_t next = (rb->head + 1) & (RING_BUF_SIZE - 1);
    if (next == rb->tail) {
        rb->overflow_cnt++;
        return false;
    }
    rb->buf[rb->head] = *frame;
    rb->head          = next;
    rb->total_rx++;
    return true;
}

/* Pop a frame — returns false if empty */
static inline bool ring_buf_pop(can_ring_buf_t *rb, can_frame_t *frame)
{
    if (rb->tail == rb->head) return false;
    *frame   = rb->buf[rb->tail];
    rb->tail = (rb->tail + 1) & (RING_BUF_SIZE - 1);
    return true;
}

static inline bool     ring_buf_empty    (const can_ring_buf_t *rb)
    { return rb->tail == rb->head; }

static inline uint32_t ring_buf_used     (const can_ring_buf_t *rb)
    { return (rb->head - rb->tail) & (RING_BUF_SIZE - 1); }

static inline uint32_t ring_buf_overflows(const can_ring_buf_t *rb)
    { return rb->overflow_cnt; }

static inline uint32_t ring_buf_total    (const can_ring_buf_t *rb)
    { return rb->total_rx; }