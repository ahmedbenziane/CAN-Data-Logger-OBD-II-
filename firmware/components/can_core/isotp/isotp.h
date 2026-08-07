#pragma once
/* ============================================================
 *  isotp.h  —  Step 5: ISO-TP Transport Layer (ISO 15765-2)
 *
 *  Implements:
 *    - Single Frame    (SF): payload ≤ 7 bytes
 *    - First Frame     (FF): start of payload > 7 bytes
 *    - Consecutive Frame (CF): continuation of multi-frame
 *    - Flow Control    (FC): receiver → sender pacing
 *
 *  Design:
 *    - One isotp_ctx_t per channel (e.g., one per ECU address pair)
 *    - Caller provides TX function; library handles segmentation
 *    - Non-blocking: call isotp_process_frame() from RX path
 *    - Timeouts enforced: N_Bs (FC wait), N_Cr (CF gap)
 *
 *  ISO 15765-2 timing parameters (default, adjustable):
 *    N_As / N_Ar  = 25ms   (CAN frame TX timeout)
 *    N_Bs         = 75ms   (wait for FC from receiver)
 *    N_Br         = 0ms    (time before sending FC)
 *    N_Cs         = 0ms    (time between consecutive frames)
 *    N_Cr         = 150ms  (time to receive next CF)
 * ============================================================ */

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "esp_err.h"
#include "mcp2515.h"

/* ── Error codes ──────────────────────────────────────────── */
#define ISOTP_OK               ESP_OK
#define ISOTP_ERR_TIMEOUT      ESP_ERR_TIMEOUT
#define ISOTP_ERR_OVERFLOW     ESP_ERR_NO_MEM
#define ISOTP_ERR_INVALID      ESP_ERR_INVALID_ARG
#define ISOTP_ERR_BUSY         ESP_ERR_INVALID_STATE
#define ISOTP_ERR_ABORTED      ESP_FAIL

/* ── Frame type nibbles (high nibble of data[0]) ─────────── */
#define ISOTP_FT_SF   0x00   /* Single Frame      */
#define ISOTP_FT_FF   0x10   /* First Frame       */
#define ISOTP_FT_CF   0x20   /* Consecutive Frame */
#define ISOTP_FT_FC   0x30   /* Flow Control      */

/* ── Flow Control status byte ────────────────────────────── */
#define ISOTP_FC_CTS  0x30   /* Continue To Send  */
#define ISOTP_FC_WAIT 0x31   /* Wait              */
#define ISOTP_FC_OVFL 0x32   /* Overflow / abort  */

/* ── Max payload (ISO 15765-2 classical CAN) ─────────────── */
#define ISOTP_MAX_PAYLOAD   4095   /* 12-bit length field */

/* ── Timeout constants (ms) ──────────────────────────────── */
#define ISOTP_N_Bs_MS   2000   /* max wait for FC after FF (was 75ms, real FC arrives in <5ms on bench) */
#define ISOTP_N_Cr_MS   150    /* max wait for next CF     */
#define ISOTP_N_As_MS   25     /* frame TX timeout         */

/* ── RX state machine ─────────────────────────────────────── */
typedef enum {
    ISOTP_RX_IDLE = 0,
    ISOTP_RX_RECEIVING,        /* collecting consecutive frames */
} isotp_rx_state_t;

/* ── TX state machine ─────────────────────────────────────── */
typedef enum {
    ISOTP_TX_IDLE = 0,
    ISOTP_TX_WAIT_FC,          /* sent FF, waiting for FC */
    ISOTP_TX_SENDING,          /* sending consecutive frames */
} isotp_tx_state_t;

/* ── Callback: called when a complete message is received ─── */
typedef void (*isotp_rx_cb_t)(uint16_t src_id,
                               const uint8_t *payload,
                               size_t length,
                               void *user_data);

/* ── TX function pointer: must send one CAN frame ────────── */
typedef esp_err_t (*isotp_tx_fn_t)(const can_frame_t *frame);

/* ── Channel context ──────────────────────────────────────── */
typedef struct {
    /* Addressing */
    uint16_t    tx_id;          /* CAN ID we transmit on         */
    uint16_t    rx_id;          /* CAN ID we listen for          */

    /* TX state */
    isotp_tx_state_t  tx_state;
    const uint8_t    *tx_buf;       /* pointer to caller's buffer    */
    size_t            tx_len;       /* total bytes to send           */
    size_t            tx_offset;    /* bytes sent so far             */
    uint8_t           tx_sn;        /* sequence number (0-15)        */
    uint8_t           fc_block_size;/* from FC: frames per block     */
    uint8_t           fc_stmin;     /* from FC: min inter-frame time */
    uint8_t           fc_frames_rem;/* countdown within block        */
    int64_t           tx_deadline;  /* N_Bs deadline (us)            */

    /* RX state */
    isotp_rx_state_t  rx_state;
    uint8_t           rx_buf[ISOTP_MAX_PAYLOAD];
    size_t            rx_expected;  /* total bytes in message        */
    size_t            rx_received;  /* bytes collected so far        */
    uint8_t           rx_next_sn;   /* expected CF sequence number   */
    int64_t           rx_deadline;  /* N_Cr deadline (us)            */

    /* Hooks */
    isotp_tx_fn_t     tx_fn;        /* function to send a CAN frame  */
    isotp_rx_cb_t     rx_cb;        /* called on complete RX message */
    void             *user_data;    /* passed to rx_cb               */

    /* Stats */
    uint32_t          rx_count;
    uint32_t          tx_count;
    uint32_t          rx_errors;
    uint32_t          tx_errors;
    uint32_t          rx_timeouts;
} isotp_ctx_t;

/* ── Public API ───────────────────────────────────────────── */

/**
 * @brief  Initialise an ISO-TP channel context.
 *
 * @param ctx       Context to initialise (caller-allocated)
 * @param tx_id     CAN ID to use for transmitting frames
 * @param rx_id     CAN ID to expect incoming frames on
 * @param tx_fn     Function that sends one CAN frame
 * @param rx_cb     Called when a complete payload is received
 * @param user_data Opaque pointer passed to rx_cb
 */
void isotp_init(isotp_ctx_t  *ctx,
                uint16_t      tx_id,
                uint16_t      rx_id,
                isotp_tx_fn_t tx_fn,
                isotp_rx_cb_t rx_cb,
                void         *user_data);

/**
 * @brief  Send a payload over ISO-TP.
 *         For payloads ≤7 bytes: sends a Single Frame immediately.
 *         For payloads >7 bytes: sends FF, waits for FC, sends CFs.
 *
 * @param ctx    Channel context
 * @param data   Data to send
 * @param len    Length (1 to ISOTP_MAX_PAYLOAD)
 * @return ISOTP_OK, ISOTP_ERR_BUSY, ISOTP_ERR_INVALID
 *
 * NOTE: For multi-frame this is non-blocking — call isotp_tick()
 *       periodically to drive the TX state machine.
 */
esp_err_t isotp_send(isotp_ctx_t *ctx, const uint8_t *data, size_t len);

/**
 * @brief  Feed an incoming CAN frame into the ISO-TP engine.
 *         Call this from your RX task for frames matching rx_id.
 *
 * @param ctx    Channel context
 * @param frame  The received CAN frame
 * @return ISOTP_OK, ISOTP_ERR_INVALID, ISOTP_ERR_OVERFLOW
 */
esp_err_t isotp_process_frame(isotp_ctx_t *ctx, const can_frame_t *frame);

/**
 * @brief  Drive TX state machine and check timeouts.
 *         Call from a periodic task (every 1ms or on each loop iteration).
 *
 * @param ctx  Channel context
 */
void isotp_tick(isotp_ctx_t *ctx);

/**
 * @brief  Abort any ongoing TX or RX and reset to IDLE.
 */
void isotp_reset(isotp_ctx_t *ctx);

/**
 * @brief  Return true if TX channel is free to accept a new send.
 */
bool isotp_tx_idle(const isotp_ctx_t *ctx);

/**
 * @brief  Return true if RX channel is currently collecting frames.
 */
bool isotp_rx_active(const isotp_ctx_t *ctx);

/**
 * @brief  Print channel statistics via ESP_LOGI.
 */
void isotp_print_stats(const isotp_ctx_t *ctx, const char *tag);