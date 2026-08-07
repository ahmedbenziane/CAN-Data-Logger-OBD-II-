    /**
 * @file decoder_app.h
 * @brief Step 13 — Node A receiver app: signal decoder + UDS test client
 *
 * Renamed from "signal_decoder.h" (which existed in components/can_core/signal)
 * to avoid file/symbol collision.
 *
 * Two responsibilities:
 *   1. Decode Node B's broadcast streams (0x7E8 / 0x0A6 / 0x1A0 / 0x3B0)
 *      and print a live dashboard.
 *   2. Periodically send OBD-II + UDS requests to Node B and report
 *      pass/fail against the Step 13 8-point scorecard. This is what
 *      validates fault-injection criteria (TIMEOUT, SLOW_FC, WRONG_DID).
 *
 * Stays in MCP_MODE_NORMAL so it ACKs Node B's frames (otherwise the
 * Step 12 retransmit storm would kick in — see handoff Section 4).
 */

#pragma once

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Start the decoder + UDS test client on Node A.
 *
 * Spawns:
 *   - RX task        (Core 0, prio TASK_PRIO_CAN_RX)
 *   - Dashboard task (Core 1, prio TASK_PRIO_APP, 500 ms refresh)
 *   - Health task    (Core 1, prio TASK_PRIO_APP, 5 s health report)
 *   - UDS test task  (Core 1, prio TASK_PRIO_APP, 5 s request cycle)
 *
 * Sets MCP2515 to NORMAL mode and accepts all standard IDs.
 */
esp_err_t decoder_app_start(void);

#ifdef __cplusplus
}
#endif