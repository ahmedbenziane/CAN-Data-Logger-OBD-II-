/**
 * @file ecu_manager.h
 * @brief ECU Registry — structured table of known ECUs + active ID bitmap
 *
 * Two responsibilities:
 *
 * 1. STATIC REGISTRY
 *    Pre-seeded with all 7 Renault Clio IV ECUs (real ELM scan data).
 *    Lookup by request ID or response ID.
 *    Used by dispatcher handlers to print ECU name/role on frame receipt.
 *
 * 2. ACTIVE ID BITMAP
 *    2048-bit array (256 bytes) — one bit per CAN ID (0x000–0x7FF).
 *    Call ecu_manager_update_seen(id) on every received frame.
 *    After a sniffer session, ecu_manager_print_active() shows all IDs
 *    observed on the bus — critical for Phase 8 reverse engineering.
 *
 * Memory:
 *   sizeof(ecu_registry_t) ≈ 7 × 64 bytes + 256 bytes bitmap = ~704 bytes
 *   Entirely static — no heap allocation.
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ── Constants ─────────────────────────────────────────────────────────── */

#define ECU_MAX_ENTRIES     16u           /**< Max ECUs in registry         */
#define ECU_NAME_LEN        24u           /**< Max ECU name string length   */
#define ECU_ROLE_LEN        24u           /**< Max ECU role string length   */
#define ECU_SUPPLIER_LEN    8u            /**< Max supplier code length     */
#define ECU_SW_VER_LEN      8u            /**< Max SW version string length */

#define CAN_ID_MAX          0x7FFu        /**< Largest 11-bit standard ID   */
#define SEEN_BITMAP_BYTES   ((CAN_ID_MAX + 1) / 8)  /**< 256 bytes         */

/* ── ECU Descriptor ─────────────────────────────────────────────────────── */

/**
 * @brief Full descriptor for one ECU on the bus.
 *
 * All fields from real Renault Clio IV VIN VF14SR6B4FD018433 ELM scan.
 */
typedef struct {
    char     name[ECU_NAME_LEN];        /**< e.g. "Engine ECU"            */
    char     role[ECU_ROLE_LEN];        /**< e.g. "Powertrain"            */
    uint32_t request_id;                /**< Tester → ECU (e.g. 0x7E0)   */
    uint32_t response_id;               /**< ECU → Tester (e.g. 0x7E8)   */
    char     supplier[ECU_SUPPLIER_LEN];/**< e.g. "GL4"                  */
    char     sw_ver[ECU_SW_VER_LEN];    /**< e.g. "002C"                 */
    uint8_t  diag_ver;                  /**< UDS diagnostic version        */
    bool     active;                    /**< true = seen on bus this session*/
} ecu_entry_t;

/* ── Registry ───────────────────────────────────────────────────────────── */

/**
 * @brief ECU registry instance.
 *
 * Contains the ECU table and the seen-ID bitmap.
 * Zero-initialise, then call ecu_manager_init().
 */
typedef struct {
    ecu_entry_t  ecus[ECU_MAX_ENTRIES]; /**< ECU descriptors               */
    uint8_t      count;                 /**< Number of registered ECUs     */
    uint8_t      seen_bitmap[SEEN_BITMAP_BYTES]; /**< One bit per CAN ID  */
    uint32_t     seen_count;            /**< Unique IDs observed so far    */
    uint32_t     frame_count;           /**< Total frames seen (all IDs)   */
} ecu_registry_t;

/* ── API ────────────────────────────────────────────────────────────────── */

/**
 * @brief Initialise registry and pre-seed with Renault Clio IV ECUs.
 *
 * Always call this first. Existing contents are cleared.
 * After this call, 7 ECUs are registered (Engine, BCM, Transmission,
 * ABS, HVAC, SRS/Airbags, Dashboard).
 */
void ecu_manager_init(ecu_registry_t *r);

/**
 * @brief Add an ECU entry to the registry at runtime.
 *
 * @return true  ECU added.
 * @return false Registry full (ECU_MAX_ENTRIES reached).
 */
bool ecu_manager_add(ecu_registry_t *r, const ecu_entry_t *entry);

/**
 * @brief Find an ECU by its response ID (ECU → tester direction).
 *
 * This is the most common lookup — a frame arrives with ID=0x7E8,
 * we look up which ECU sends on 0x7E8.
 *
 * @return Pointer to entry, or NULL if not found.
 */
const ecu_entry_t *ecu_manager_find_by_resp(const ecu_registry_t *r,
                                              uint32_t response_id);

/**
 * @brief Find an ECU by its request ID (tester → ECU direction).
 *
 * @return Pointer to entry, or NULL if not found.
 */
const ecu_entry_t *ecu_manager_find_by_req(const ecu_registry_t *r,
                                             uint32_t request_id);

/**
 * @brief Record that a CAN ID was observed on the bus.
 *
 * Call this on EVERY received frame, even before dispatching.
 * Sets the appropriate bit in seen_bitmap, increments frame_count,
 * marks matching ECU entry as active.
 *
 * @param can_id  CAN frame ID (must be ≤ CAN_ID_MAX).
 */
void ecu_manager_update_seen(ecu_registry_t *r, uint32_t can_id);

/**
 * @brief Check if a CAN ID has been seen this session.
 *
 * @return true if at least one frame with this ID was received.
 */
bool ecu_manager_id_seen(const ecu_registry_t *r, uint32_t can_id);

/**
 * @brief Print the full ECU registry to UART (ESP_LOGI).
 *
 * Shows all registered ECUs with their IDs, supplier, SW version.
 */
void ecu_manager_print_registry(const ecu_registry_t *r);

/**
 * @brief Print only ECUs and IDs seen active this session.
 *
 * Shows:
 *  - Known ECUs marked active (matched by response_id)
 *  - All unique CAN IDs observed (from seen_bitmap)
 */
void ecu_manager_print_active(const ecu_registry_t *r);

#ifdef __cplusplus
}
#endif