/**
 * @file dispatcher.c
 * @brief CAN Frame Dispatcher — hash-map based O(1) frame routing
 *
 * Hash scheme:
 *   slot = (can_id * KNUTH_CONSTANT) >> (32 - log2(TABLE_SIZE))
 *   Collision → linear probe (slot + 1) % TABLE_SIZE
 *
 * Tombstone on delete: slot marked occupied=false but can_id retained as
 * sentinel (DISPATCHER_TOMBSTONE) so probing chains are not broken.
 */

#include "dispatcher.h"
#include "esp_log.h"
#include <string.h>

static const char *TAG = "DISPATCHER";

/* Sentinel value for deleted (tombstone) slots */
#define DISPATCHER_TOMBSTONE  0xFFFFFFFFu

/* ── Internal helpers ───────────────────────────────────────────────────── */

/**
 * @brief Compute initial table slot for a CAN ID.
 *
 * Multiplicative Fibonacci hash — distributes IDs uniformly even when they
 * cluster (e.g. 0x7E0–0x7EE are only 15 IDs apart).
 */
static inline uint32_t _hash(uint32_t can_id)
{
    return (can_id * DISPATCHER_HASH_KNUTH) >> DISPATCHER_HASH_SHIFT;
}

/* ── Public API ─────────────────────────────────────────────────────────── */

void dispatcher_init(dispatcher_t *d)
{
    if (!d) return;
    memset(d, 0, sizeof(*d));
    ESP_LOGI(TAG, "Dispatcher initialised — %u slots", DISPATCHER_TABLE_SIZE);
}

bool dispatcher_register(dispatcher_t *d, uint32_t can_id,
                         dispatcher_handler_t handler, void *ctx)
{
    if (!d || !handler) return false;
    if (can_id == 0 || can_id == DISPATCHER_TOMBSTONE) {
        ESP_LOGE(TAG, "register: invalid CAN ID 0x%03"PRIX32, can_id);
        return false;
    }

    uint32_t slot  = _hash(can_id);
    uint32_t probe = 0;

    while (probe < DISPATCHER_TABLE_SIZE) {
        dispatcher_slot_t *s = &d->table[slot];

        /* Slot free or tombstone — claim it */
        if (!s->occupied) {
            s->can_id   = can_id;
            s->handler  = handler;
            s->ctx      = ctx;
            s->occupied = true;
            ESP_LOGI(TAG, "Registered CAN ID 0x%03"PRIX32" → slot %"PRIu32,
                     can_id, slot);
            return true;
        }

        /* Already registered for this ID — update handler */
        if (s->can_id == can_id) {
            s->handler = handler;
            s->ctx     = ctx;
            ESP_LOGW(TAG, "Updated handler for CAN ID 0x%03"PRIX32, can_id);
            return true;
        }

        /* Collision — probe next slot */
        d->collision_count++;
        slot = (slot + 1) % DISPATCHER_TABLE_SIZE;
        probe++;
    }

    ESP_LOGE(TAG, "register: table full — cannot register 0x%03"PRIX32, can_id);
    return false;
}

bool dispatcher_unregister(dispatcher_t *d, uint32_t can_id)
{
    if (!d) return false;

    uint32_t slot  = _hash(can_id);
    uint32_t probe = 0;

    while (probe < DISPATCHER_TABLE_SIZE) {
        dispatcher_slot_t *s = &d->table[slot];

        if (!s->occupied && s->can_id == 0) {
            /* Empty slot — ID definitely not registered */
            return false;
        }

        if (s->occupied && s->can_id == can_id) {
            /* Found — mark as tombstone */
            s->occupied = false;
            s->handler  = NULL;
            s->ctx      = NULL;
            /* Keep can_id as tombstone sentinel so probing chains hold */
            s->can_id   = DISPATCHER_TOMBSTONE;
            ESP_LOGI(TAG, "Unregistered CAN ID 0x%03"PRIX32, can_id);
            return true;
        }

        slot = (slot + 1) % DISPATCHER_TABLE_SIZE;
        probe++;
    }
    return false;
}

void dispatcher_dispatch(dispatcher_t *d, const can_frame_t *frame)
{
    if (!d || !frame) return;

    d->dispatch_count++;

    uint32_t can_id = frame->id;
    uint32_t slot   = _hash(can_id);
    uint32_t probe  = 0;

    while (probe < DISPATCHER_TABLE_SIZE) {
        const dispatcher_slot_t *s = &d->table[slot];

        /* True empty slot — stop probing, ID not registered */
        if (!s->occupied && s->can_id == 0) {
            goto miss;
        }

        /* Active slot matches — dispatch */
        if (s->occupied && s->can_id == can_id) {
            d->hit_count++;
            s->handler(frame, s->ctx);
            return;
        }

        /* Tombstone or different ID — keep probing */
        slot = (slot + 1) % DISPATCHER_TABLE_SIZE;
        probe++;
    }

miss:
    d->miss_count++;
    if (d->default_handler) {
        d->default_handler(frame, d->default_ctx);
    }
}

void dispatcher_set_default(dispatcher_t *d,
                             dispatcher_handler_t handler, void *ctx)
{
    if (!d) return;
    d->default_handler = handler;
    d->default_ctx     = ctx;
}

const dispatcher_slot_t *dispatcher_lookup(const dispatcher_t *d,
                                            uint32_t can_id)
{
    if (!d) return NULL;

    uint32_t slot  = _hash(can_id);
    uint32_t probe = 0;

    while (probe < DISPATCHER_TABLE_SIZE) {
        const dispatcher_slot_t *s = &d->table[slot];

        if (!s->occupied && s->can_id == 0) return NULL;

        if (s->occupied && s->can_id == can_id) return s;

        slot = (slot + 1) % DISPATCHER_TABLE_SIZE;
        probe++;
    }
    return NULL;
}

void dispatcher_print_stats(const dispatcher_t *d)
{
    if (!d) return;
    ESP_LOGI(TAG, "─── Dispatcher Stats ───────────────────────");
    ESP_LOGI(TAG, "  Dispatched : %"PRIu32, d->dispatch_count);
    ESP_LOGI(TAG, "  Hits       : %"PRIu32, d->hit_count);
    ESP_LOGI(TAG, "  Misses     : %"PRIu32, d->miss_count);
    ESP_LOGI(TAG, "  Collisions : %"PRIu32, d->collision_count);

    ESP_LOGI(TAG, "─── Registered IDs ─────────────────────────");
    uint32_t count = 0;
    for (uint32_t i = 0; i < DISPATCHER_TABLE_SIZE; i++) {
        const dispatcher_slot_t *s = &d->table[i];
        if (s->occupied) {
            ESP_LOGI(TAG, "  [%02"PRIu32"] ID=0x%03"PRIX32, i, s->can_id);
            count++;
        }
    }
    ESP_LOGI(TAG, "  Total registered: %"PRIu32"/%u", count, DISPATCHER_TABLE_SIZE);
    ESP_LOGI(TAG, "────────────────────────────────────────────");
}