#pragma once
/* ============================================================
 *  uds.h  —  Step 8: UDS (ISO 14229) Session + DID Read
 *
 *  Implements:
 *    0x10  DiagnosticSessionControl
 *    0x22  ReadDataByIdentifier  (single + multi DID)
 *    0x3E  TesterPresent
 *    0x7F  NegativeResponse  (receive/decode)
 *
 *  Transport: ISO-TP (already built in Step 5/6)
 *  Addressing: physical 0x7E0→0x7E8 (engine ECU)
 *
 *  Real values hardcoded from VF14SR6B4FD018433:
 *    ECU software:  002C
 *    Diag version:  84
 *    Supplier:      GL4
 *    VIN:           VF14SR6B4FD018433
 *    Cal ID:        8201312983237106032R
 * ============================================================ */

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "esp_err.h"
#include "isotp.h"

/* ── UDS Service IDs ─────────────────────────────────────── */
#define UDS_SID_SESSION_CTRL      0x10
#define UDS_SID_READ_DID          0x22
#define UDS_SID_TESTER_PRESENT    0x3E
#define UDS_SID_NEGATIVE_RESP     0x7F

/* ── Positive response = SID + 0x40 ─────────────────────── */
#define UDS_POS_RESP(sid)         ((sid) + 0x40)

/* ── Session types ───────────────────────────────────────── */
#define UDS_SESSION_DEFAULT       0x01
#define UDS_SESSION_EXTENDED      0x03
#define UDS_SESSION_PROGRAMMING   0x02

/* ── Negative Response Codes (NRC) ──────────────────────── */
#define UDS_NRC_GENERAL_REJECT          0x10
#define UDS_NRC_SERVICE_NOT_SUPPORTED   0x11
#define UDS_NRC_SUBFUNCTION_NOT_SUPP    0x12
#define UDS_NRC_INCORRECT_LENGTH        0x13
#define UDS_NRC_CONDITIONS_NOT_MET      0x22
#define UDS_NRC_REQUEST_OUT_OF_RANGE    0x31
#define UDS_NRC_SECURITY_ACCESS_DENIED  0x33
#define UDS_NRC_RESPONSE_PENDING        0x78

/* ── DIDs (Data Identifiers) — standard + Renault ────────── */
#define UDS_DID_VIN                 0xF190  /* 17-byte ASCII VIN          */
#define UDS_DID_ECU_SERIAL          0xF18C  /* ECU serial number          */
#define UDS_DID_SW_VERSION          0xF189  /* Software version           */
#define UDS_DID_SPARE_PART_NUM      0xF187  /* Spare part number          */
#define UDS_DID_SYSTEM_NAME         0xF197  /* System/ECU name            */
#define UDS_DID_CAL_ID              0xF186  /* Calibration ID             */
#define UDS_DID_CAL_VER             0xF18A  /* Calibration verification   */
#define UDS_DID_BOOT_SW_ID          0xF180  /* Boot software ID           */
#define UDS_DID_DIAG_VERSION        0xF18E  /* Diagnostic protocol ver    */
#define UDS_DID_ECU_IDENT           0xF100  /* Full ECU identification    */

/* ── CAN addressing (physical, engine ECU) ───────────────── */
#define UDS_PHYS_REQ_ID   0x7E0   /* tester → engine ECU (physical) */
#define UDS_FUNC_REQ_ID   0x7DF   /* tester → all ECUs (functional) */
#define UDS_RESP_ID       0x7E8   /* engine ECU → tester            */

/* ── Timing ──────────────────────────────────────────────── */
#define UDS_P2_TIMEOUT_MS      500   /* max wait for ECU response    */
#define UDS_P2_EXTENDED_MS    5000   /* after ResponsePending (0x78) */
#define UDS_TESTER_PRESENT_MS 2000   /* send TesterPresent every 2s  */
#define UDS_SESSION_TIMEOUT_MS 5000  /* session expires if no activity */

/* ── DID response ────────────────────────────────────────── */
typedef struct {
    uint16_t did;
    uint8_t  data[64];
    size_t   length;
    bool     valid;
} uds_did_result_t;

/* ── Session state ───────────────────────────────────────── */
typedef enum {
    UDS_SESSION_STATE_CLOSED = 0,
    UDS_SESSION_STATE_DEFAULT,
    UDS_SESSION_STATE_EXTENDED,
    UDS_SESSION_STATE_PROGRAMMING,
} uds_session_state_t;

/* ── UDS client context ──────────────────────────────────── */
typedef struct {
    isotp_ctx_t          isotp;
    uds_session_state_t  session;
    bool                 response_ready;
    uint8_t              response_buf[128];
    size_t               response_len;
    int64_t              last_activity_us;
    uint32_t             req_count;
    uint32_t             ok_count;
    uint32_t             err_count;
    uint32_t             timeout_count;
    uint32_t             nrc_count;
} uds_client_t;

/* ── API ─────────────────────────────────────────────────── */

/**
 * @brief  Initialise UDS client (wraps isotp_init).
 *         Uses physical addressing: tx=0x7E0, rx=0x7E8
 */
void uds_init(uds_client_t *client, isotp_tx_fn_t tx_fn);

/**
 * @brief  Feed incoming CAN frame into UDS engine.
 *         Call from RX task for frames with ID == UDS_RESP_ID.
 */
void uds_process_frame(uds_client_t *client, const can_frame_t *frame);

/**
 * @brief  Open a diagnostic session.
 *
 * @param client   UDS client context
 * @param session  UDS_SESSION_DEFAULT / EXTENDED / PROGRAMMING
 * @return ESP_OK, ESP_ERR_TIMEOUT, ESP_FAIL (NRC received)
 */
esp_err_t uds_open_session(uds_client_t *client, uint8_t session);

/**
 * @brief  Send TesterPresent with suppress bit 0x80.
 *         ECU resets its session timer but sends NO response.
 *         This avoids response collisions with ongoing DID scans
 *         on shared CAN ID 0x7E8.
 */
esp_err_t uds_tester_present(uds_client_t *client);

/**
 * @brief  Flush stale ISO-TP RX state before a new request.
 *         Resets the ISO-TP state machine to IDLE and clears
 *         response_ready. Call before uds_open_session and
 *         uds_read_did to discard any leftover partial frame
 *         (e.g. an OBD broadcast that arrived between requests).
 */
void uds_flush_rx(uds_client_t *client);

/**
 * @brief  Read a single DID from the ECU (Service 0x22).
 *
 * @param client  UDS client context
 * @param did     2-byte Data Identifier (e.g. UDS_DID_VIN)
 * @param result  Output: raw data + length
 * @return ESP_OK, ESP_ERR_TIMEOUT, ESP_FAIL
 */
esp_err_t uds_read_did(uds_client_t    *client,
                        uint16_t         did,
                        uds_did_result_t *result);

/**
 * @brief  Decode and pretty-print a DID result.
 *         Handles VIN (ASCII), version numbers, hex dumps.
 */
void uds_print_did(const uds_did_result_t *result, const char *tag);

/**
 * @brief  Print UDS client statistics.
 */
void uds_print_stats(const uds_client_t *client, const char *tag);

/**
 * @brief  Decode NRC byte to human-readable string.
 */
const char *uds_nrc_string(uint8_t nrc);