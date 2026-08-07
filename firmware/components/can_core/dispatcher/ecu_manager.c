/**
 * @file ecu_manager.c
 * @brief ECU Registry — pre-seeded with Renault Clio IV VF14SR6B4FD018433
 *
 * Real data from ELM327 scan of the target vehicle.
 * All 7 ECUs registered in ecu_manager_init().
 */

#include "ecu_manager.h"
#include "esp_log.h"
#include <string.h>
#include <stdio.h>

static const char *TAG = "ECU_MGR";

/* ── Bitmap helpers ─────────────────────────────────────────────────────── */

static inline void _bit_set(uint8_t *bm, uint32_t id)
{
    bm[id >> 3] |= (uint8_t)(1u << (id & 7u));
}

static inline bool _bit_get(const uint8_t *bm, uint32_t id)
{
    return (bm[id >> 3] & (uint8_t)(1u << (id & 7u))) != 0;
}

/* ── Renault Clio IV ECU table ──────────────────────────────────────────── */

/**
 * Pre-seeded ECU descriptors.
 * Source: ELM327 UDS scan, VIN VF14SR6B4FD018433, April 2026.
 *
 * Supplier codes decoded from hex → ASCII (e.g. 474C34 → "GL4").
 * SW versions and diag versions from DID F189 / F18E respectively.
 */
static const ecu_entry_t s_clio_iv_ecus[] = {
    {
        .name        = "Engine ECU",
        .role        = "Powertrain",
        .request_id  = 0x7E0,
        .response_id = 0x7E8,
        .supplier    = "GL4",       /* 474C34 */
        .sw_ver      = "002C",
        .diag_ver    = 84,
        .active      = false
    },
    {
        .name        = "BCM",
        .role        = "Body Control",
        .request_id  = 0x7E1,
        .response_id = 0x7E9,
        .supplier    = "001",       /* 303031 */
        .sw_ver      = "1419",
        .diag_ver    = 5,
        .active      = false
    },
    {
        .name        = "Transmission",
        .role        = "Drivetrain",
        .request_id  = 0x7E2,
        .response_id = 0x7EA,
        .supplier    = "—",
        .sw_ver      = "—",
        .diag_ver    = 0,
        .active      = false
    },
    {
        .name        = "ABS",
        .role        = "Chassis",
        .request_id  = 0x7E3,
        .response_id = 0x7EB,
        .supplier    = "037",       /* 303337 */
        .sw_ver      = "3230",
        .diag_ver    = 9,
        .active      = false
    },
    {
        .name        = "HVAC",
        .role        = "Climate Control",
        .request_id  = 0x7E4,
        .response_id = 0x7EC,
        .supplier    = "042",       /* 303432 */
        .sw_ver      = "0107",
        .diag_ver    = 4,
        .active      = false
    },
    {
        .name        = "SRS / Airbags",
        .role        = "Safety",
        .request_id  = 0x7E5,
        .response_id = 0x7ED,
        .supplier    = "AMR",       /* 414D52 */
        .sw_ver      = "1630",
        .diag_ver    = 4,
        .active      = false
    },
    {
        .name        = "Dashboard",
        .role        = "Instrument Cluster",
        .request_id  = 0x7E6,
        .response_id = 0x7EE,
        .supplier    = "096",       /* 303936 */
        .sw_ver      = "0034",
        .diag_ver    = 18,
        .active      = false
    },
};

#define CLIO_IV_ECU_COUNT  (sizeof(s_clio_iv_ecus) / sizeof(s_clio_iv_ecus[0]))

/* ── Public API ─────────────────────────────────────────────────────────── */

void ecu_manager_init(ecu_registry_t *r)
{
    if (!r) return;
    memset(r, 0, sizeof(*r));

    for (uint32_t i = 0; i < CLIO_IV_ECU_COUNT; i++) {
        if (r->count >= ECU_MAX_ENTRIES) {
            ESP_LOGE(TAG, "init: registry full at entry %"PRIu32, i);
            break;
        }
        r->ecus[r->count++] = s_clio_iv_ecus[i];
    }

    ESP_LOGI(TAG, "ECU registry initialised — %u ECUs loaded (Renault Clio IV)",
             r->count);
}

bool ecu_manager_add(ecu_registry_t *r, const ecu_entry_t *entry)
{
    if (!r || !entry) return false;
    if (r->count >= ECU_MAX_ENTRIES) {
        ESP_LOGE(TAG, "add: registry full");
        return false;
    }
    r->ecus[r->count++] = *entry;
    ESP_LOGI(TAG, "Added ECU: %s (REQ=0x%03"PRIX32" RESP=0x%03"PRIX32")",
             entry->name, entry->request_id, entry->response_id);
    return true;
}

const ecu_entry_t *ecu_manager_find_by_resp(const ecu_registry_t *r,
                                              uint32_t response_id)
{
    if (!r) return NULL;
    for (uint8_t i = 0; i < r->count; i++) {
        if (r->ecus[i].response_id == response_id) {
            return &r->ecus[i];
        }
    }
    return NULL;
}

const ecu_entry_t *ecu_manager_find_by_req(const ecu_registry_t *r,
                                             uint32_t request_id)
{
    if (!r) return NULL;
    for (uint8_t i = 0; i < r->count; i++) {
        if (r->ecus[i].request_id == request_id) {
            return &r->ecus[i];
        }
    }
    return NULL;
}

void ecu_manager_update_seen(ecu_registry_t *r, uint32_t can_id)
{
    if (!r || can_id > CAN_ID_MAX) return;

    r->frame_count++;

    /* Update seen_count only on first observation of this ID */
    if (!_bit_get(r->seen_bitmap, can_id)) {
        _bit_set(r->seen_bitmap, can_id);
        r->seen_count++;
    }

    /* Mark matching ECU active (check both request and response ID) */
    for (uint8_t i = 0; i < r->count; i++) {
        if (r->ecus[i].response_id == can_id ||
            r->ecus[i].request_id  == can_id) {
            r->ecus[i].active = true;
        }
    }
}

bool ecu_manager_id_seen(const ecu_registry_t *r, uint32_t can_id)
{
    if (!r || can_id > CAN_ID_MAX) return false;
    return _bit_get(r->seen_bitmap, can_id);
}

void ecu_manager_print_registry(const ecu_registry_t *r)
{
    if (!r) return;
    ESP_LOGI(TAG, "═══════════════════════════════════════════════════════");
    ESP_LOGI(TAG, "  Renault Clio IV ECU Registry — %u entries", r->count);
    ESP_LOGI(TAG, "═══════════════════════════════════════════════════════");
    ESP_LOGI(TAG, "  %-20s %-18s  REQ    RESP  SUP  SW    DIAG",
             "Name", "Role");
    ESP_LOGI(TAG, "  ──────────────────────────────────────────────────");
    for (uint8_t i = 0; i < r->count; i++) {
        const ecu_entry_t *e = &r->ecus[i];
        ESP_LOGI(TAG, "  %-20s %-18s  0x%03"PRIX32"  0x%03"PRIX32
                 "  %-4s %-4s  v%u",
                 e->name, e->role,
                 e->request_id, e->response_id,
                 e->supplier, e->sw_ver, e->diag_ver);
    }
    ESP_LOGI(TAG, "═══════════════════════════════════════════════════════");
}

void ecu_manager_print_active(const ecu_registry_t *r)
{
    if (!r) return;
    ESP_LOGI(TAG, "─── Active ECUs this session ───────────────────────");
    ESP_LOGI(TAG, "  Total frames seen : %"PRIu32, r->frame_count);
    ESP_LOGI(TAG, "  Unique CAN IDs    : %"PRIu32, r->seen_count);

    bool any_active = false;
    for (uint8_t i = 0; i < r->count; i++) {
        if (r->ecus[i].active) {
            const ecu_entry_t *e = &r->ecus[i];
            ESP_LOGI(TAG, "  ✓ %-20s [%s]  RESP=0x%03"PRIX32,
                     e->name, e->role, e->response_id);
            any_active = true;
        }
    }
    if (!any_active) {
        ESP_LOGI(TAG, "  (no known ECUs seen yet)");
    }

    /* Dump all unique IDs from the bitmap */
    ESP_LOGI(TAG, "─── All unique CAN IDs seen ────────────────────────");
    uint32_t printed = 0;
    char line[80];
    int  pos = 0;

    for (uint32_t id = 0; id <= CAN_ID_MAX; id++) {
        if (_bit_get(r->seen_bitmap, id)) {
            /* Group 8 IDs per log line for readability */
            pos += snprintf(line + pos, sizeof(line) - (size_t)pos,
                            "0x%03"PRIX32" ", id);
            printed++;
            if (printed % 8 == 0) {
                ESP_LOGI(TAG, "  %s", line);
                pos = 0;
                memset(line, 0, sizeof(line));
            }
        }
    }
    if (pos > 0) {
        ESP_LOGI(TAG, "  %s", line);   /* flush remaining */
    }
    ESP_LOGI(TAG, "────────────────────────────────────────────────────");
}