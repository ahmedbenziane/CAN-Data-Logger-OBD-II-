/* ============================================================
 *  uds.c  —  Step 8/13: UDS (ISO 14229) Session + DID Read
 *
 *  Root-cause fix applied here:
 *  ISO-TP delivers payload starting at the SID byte (PCI byte
 *  already stripped). So response_buf[0] = SID, [1] = first
 *  parameter. All previous versions checked response_buf[1]
 *  for the SID — off by one — which is why every session open
 *  printed "unexpected response 0x03" (the session subtype
 *  byte sitting at [1], not the 0x50 positive response SID).
 *
 *  Also fixed:
 *  - is_negative: checks [0]==0x7F, NRC at [2] (not [3])
 *  - uds_read_did: data starts at [3] (SID+DIDhi+DIDlo)
 *  - uds_tester_present: suppress bit 0x80, no response wait
 *  - uds_flush_rx: resets ISO-TP state + clears response_ready
 *  - on_uds_response: OBD filter at payload[0] (0x40-0x49)
 * ============================================================ */

#include "uds.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>
#include <stdio.h>

static const char *TAG = "UDS  ";

static inline int64_t now_us(void) { return esp_timer_get_time(); }
static inline int64_t ms_to_us(int ms) { return (int64_t)ms * 1000LL; }

/* ── ISO-TP RX callback ──────────────────────────────────────────────
 * isotp.c calls: ctx->rx_cb(frame->id, &frame->data[1], sf_len, ...)
 * So payload[0] is the first UDS application byte = the SID.
 *
 * Layout for UDS positive responses (single frame):
 *   payload[0] = positive SID  (0x50, 0x62, 0x7E)
 *   payload[1] = first param   (session type, DIDhi, 0x00)
 *   payload[2..] = rest of data
 *
 * Layout for OBD broadcast landing on 0x7E8:
 *   payload[0] = 0x41           (OBD mode 1 positive response)
 *   payload[1] = 0x0C           (PID)
 * Reject these so they don't trigger response_ready.              */
static void on_uds_response(uint16_t src_id,
                             const uint8_t *payload,
                             size_t length,
                             void *user_data)
{
    uds_client_t *c = (uds_client_t *)user_data;
    if (!c || length == 0) return;

    if (payload[0] >= 0x40 && payload[0] <= 0x49) {
        ESP_LOGD(TAG, "RX 0x%03X: ignoring OBD frame (SID=0x%02X)",
                 src_id, payload[0]);
        return;
    }

    size_t copy = (length < sizeof(c->response_buf))
                  ? length : sizeof(c->response_buf) - 1;
    memcpy(c->response_buf, payload, copy);
    c->response_len    = copy;
    c->response_ready  = true;
    c->last_activity_us = now_us();
    ESP_LOGD(TAG, "RX %u bytes from 0x%03X  SID=0x%02X",
             (unsigned)length, src_id, payload[0]);
}

/* ── Wait for response with timeout ──────────────────────────────── */
static esp_err_t wait_response(uds_client_t *c, int timeout_ms)
{
    int64_t deadline = now_us() + ms_to_us(timeout_ms);
    while (!c->response_ready) {
        isotp_tick(&c->isotp);
        if (now_us() > deadline) {
            c->timeout_count++;
            return ESP_ERR_TIMEOUT;
        }
        vTaskDelay(1);
    }
    return ESP_OK;
}

/* ── Send request via ISO-TP ─────────────────────────────────────── */
static esp_err_t send_request(uds_client_t *c,
                               const uint8_t *req, size_t len)
{
    c->response_ready = false;
    c->req_count++;
    esp_err_t ret = isotp_send(&c->isotp, req, len);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "isotp_send failed: %s", esp_err_to_name(ret));
        c->err_count++;
    }
    return ret;
}

/* ── Check for negative response ─────────────────────────────────────
 * NRC frame layout (after ISO-TP strips PCI):
 *   [0] = 0x7F  (NegativeResponse SID)
 *   [1] = requested SID
 *   [2] = NRC byte                                                  */
static bool is_negative(const uds_client_t *c, uint8_t *nrc_out)
{
    if (c->response_len >= 3 &&
        c->response_buf[0] == UDS_SID_NEGATIVE_RESP) {
        if (nrc_out) *nrc_out = c->response_buf[2];
        return true;
    }
    return false;
}

/* ── Flush stale ISO-TP RX state ─────────────────────────────────── */
void uds_flush_rx(uds_client_t *c)
{
    if (!c) return;
    isotp_reset(&c->isotp);
    c->response_ready = false;
    c->response_len   = 0;
}

/* ── Public API ──────────────────────────────────────────────────── */

void uds_init(uds_client_t *client, isotp_tx_fn_t tx_fn)
{
    memset(client, 0, sizeof(*client));
    client->session = UDS_SESSION_STATE_CLOSED;

    isotp_init(&client->isotp,
               UDS_PHYS_REQ_ID,
               UDS_RESP_ID,
               tx_fn,
               on_uds_response,
               client);
    ESP_LOGI(TAG, "UDS client ready  tx=0x%03X rx=0x%03X",
             UDS_PHYS_REQ_ID, UDS_RESP_ID);
}

void uds_process_frame(uds_client_t *client, const can_frame_t *frame)
{
    isotp_process_frame(&client->isotp, frame);
}

esp_err_t uds_open_session(uds_client_t *client, uint8_t session_type)
{
    ESP_LOGI(TAG, "Opening session 0x%02X ...", session_type);

    uds_flush_rx(client);

    uint8_t req[] = { 0x02, UDS_SID_SESSION_CTRL, session_type };
    esp_err_t ret = send_request(client, req, sizeof(req));
    if (ret != ESP_OK) return ret;

    ret = wait_response(client, UDS_P2_TIMEOUT_MS);
    if (ret == ESP_ERR_TIMEOUT) {
        ESP_LOGW(TAG, "Session open timeout");
        return ret;
    }

    uint8_t nrc = 0;
    if (is_negative(client, &nrc)) {
        ESP_LOGW(TAG, "Session NRC: 0x%02X (%s)", nrc, uds_nrc_string(nrc));
        client->nrc_count++;
        return ESP_FAIL;
    }

    /* Positive: response_buf[0] = 0x50 (SID+0x40), [1] = session_type */
    if (client->response_len >= 2 &&
        client->response_buf[0] == UDS_POS_RESP(UDS_SID_SESSION_CTRL)) {
        switch (session_type) {
        case UDS_SESSION_DEFAULT:
            client->session = UDS_SESSION_STATE_DEFAULT; break;
        case UDS_SESSION_EXTENDED:
            client->session = UDS_SESSION_STATE_EXTENDED; break;
        case UDS_SESSION_PROGRAMMING:
            client->session = UDS_SESSION_STATE_PROGRAMMING; break;
        }
        client->ok_count++;
        ESP_LOGI(TAG, "Session 0x%02X opened ✓", session_type);
        return ESP_OK;
    }

    ESP_LOGW(TAG, "Session: unexpected response 0x%02X",
             client->response_len > 0 ? client->response_buf[0] : 0);
    client->err_count++;
    return ESP_FAIL;
}

esp_err_t uds_tester_present(uds_client_t *client)
{
    /* Suppress bit 0x80: ECU resets S3 timer, sends NO response.
     * This eliminates the collision between a TesterPresent response
     * and an ongoing DID scan on shared CAN ID 0x7E8.             */
    uint8_t req[] = { 0x02, UDS_SID_TESTER_PRESENT, 0x80 };
    esp_err_t ret = isotp_send(&client->isotp, req, sizeof(req));
    if (ret != ESP_OK) {
        ESP_LOGD(TAG, "TesterPresent TX failed: %s", esp_err_to_name(ret));
        return ret;
    }
    client->last_activity_us = now_us();
    return ESP_OK;
}

esp_err_t uds_read_did(uds_client_t    *client,
                        uint16_t         did,
                        uds_did_result_t *result)
{
    if (!client || !result) return ESP_ERR_INVALID_ARG;

    uds_flush_rx(client);

    uint8_t req[] = {
        0x03,
        UDS_SID_READ_DID,
        (uint8_t)(did >> 8),
        (uint8_t)(did & 0xFF)
    };

    ESP_LOGD(TAG, "ReadDID 0x%04X ...", did);
    esp_err_t ret = send_request(client, req, sizeof(req));
    if (ret != ESP_OK) {
        result->valid = false;
        return ret;
    }

    int total_timeout = UDS_P2_TIMEOUT_MS;
    do {
        ret = wait_response(client, total_timeout);
        if (ret == ESP_ERR_TIMEOUT) {
            ESP_LOGW(TAG, "ReadDID 0x%04X timeout", did);
            result->valid = false;
            return ret;
        }

        uint8_t nrc = 0;
        if (is_negative(client, &nrc)) {
            if (nrc == UDS_NRC_RESPONSE_PENDING) {
                ESP_LOGD(TAG, "DID 0x%04X: ResponsePending, waiting...", did);
                client->response_ready = false;
                total_timeout = UDS_P2_EXTENDED_MS;
                continue;
            }
            ESP_LOGW(TAG, "DID 0x%04X NRC: 0x%02X (%s)",
                     did, nrc, uds_nrc_string(nrc));
            client->nrc_count++;
            result->valid = false;
            return ESP_FAIL;
        }
        break;
    } while (1);

    /* Positive: response_buf[0]=0x62, [1]=DIDhi, [2]=DIDlo, [3..]=data */
    if (client->response_len < 3 ||
        client->response_buf[0] != UDS_POS_RESP(UDS_SID_READ_DID)) {
        ESP_LOGW(TAG, "DID 0x%04X: bad response SID 0x%02X",
                 did, client->response_len > 0 ? client->response_buf[0] : 0);
        client->err_count++;
        result->valid = false;
        return ESP_FAIL;
    }

    /* Data starts at [3]: skip SID(1) + DIDhi(1) + DIDlo(1) */
    size_t data_len = (client->response_len >= 3)
                      ? (client->response_len - 3) : 0;
    if (data_len > sizeof(result->data)) data_len = sizeof(result->data);

    result->did    = did;
    result->length = data_len;
    result->valid  = true;
    memcpy(result->data, &client->response_buf[3], data_len);

    client->ok_count++;
    ESP_LOGD(TAG, "DID 0x%04X: %u data bytes OK", did, (unsigned)data_len);
    return ESP_OK;
}

void uds_print_did(const uds_did_result_t *result, const char *tag)
{
    if (!result->valid) {
        ESP_LOGI(tag, "DID 0x%04X: INVALID", result->did);
        return;
    }

    switch (result->did) {

    case UDS_DID_VIN:
        ESP_LOGI(tag, "VIN                = %.*s",
                 (int)result->length, (char *)result->data);
        break;

    case UDS_DID_ECU_SERIAL:
    case UDS_DID_SPARE_PART_NUM:
    case UDS_DID_SYSTEM_NAME:
    case UDS_DID_CAL_ID: {
        bool is_ascii = true;
        for (size_t i = 0; i < result->length; i++) {
            uint8_t b = result->data[i];
            if (b != 0 && (b < 0x20 || b > 0x7E)) { is_ascii = false; break; }
        }
        const char *label =
            (result->did == UDS_DID_ECU_SERIAL)    ? "ECU Serial     " :
            (result->did == UDS_DID_SPARE_PART_NUM) ? "Spare Part Num " :
            (result->did == UDS_DID_SYSTEM_NAME)    ? "System Name    " :
                                                      "Calibration ID ";
        if (is_ascii) {
            ESP_LOGI(tag, "%s = %.*s",
                     label, (int)result->length, (char *)result->data);
        } else {
            char hex[128] = {0};
            for (size_t i = 0; i < result->length && i < 20; i++)
                snprintf(hex + i*3, 4, "%02X ", result->data[i]);
            ESP_LOGI(tag, "%s = [%s]", label, hex);
        }
        break;
    }

    case UDS_DID_SW_VERSION:
        ESP_LOGI(tag, "SW Version         = %02X%02X",
                 result->length > 0 ? result->data[0] : 0,
                 result->length > 1 ? result->data[1] : 0);
        break;

    case UDS_DID_DIAG_VERSION:
        ESP_LOGI(tag, "Diag Version       = %u",
                 result->length > 0 ? result->data[0] : 0);
        break;

    default: {
        char hex[128] = {0};
        for (size_t i = 0; i < result->length && i < 20; i++)
            snprintf(hex + i*3, 4, "%02X ", result->data[i]);
        ESP_LOGI(tag, "DID 0x%04X         = [%s] (%u bytes)",
                 result->did, hex, (unsigned)result->length);
        break;
    }
    }
}

void uds_print_stats(const uds_client_t *client, const char *tag)
{
    ESP_LOGI(tag,
             "UDS stats: req=%lu ok=%lu err=%lu tmo=%lu nrc=%lu  session=%s",
             (unsigned long)client->req_count,
             (unsigned long)client->ok_count,
             (unsigned long)client->err_count,
             (unsigned long)client->timeout_count,
             (unsigned long)client->nrc_count,
             client->session == UDS_SESSION_STATE_EXTENDED    ? "EXTENDED" :
             client->session == UDS_SESSION_STATE_DEFAULT     ? "DEFAULT"  :
             client->session == UDS_SESSION_STATE_PROGRAMMING ? "PROGRAMMING" :
                                                                "CLOSED");
}

const char *uds_nrc_string(uint8_t nrc)
{
    switch (nrc) {
    case UDS_NRC_GENERAL_REJECT:         return "generalReject";
    case UDS_NRC_SERVICE_NOT_SUPPORTED:  return "serviceNotSupported";
    case UDS_NRC_SUBFUNCTION_NOT_SUPP:   return "subFunctionNotSupported";
    case UDS_NRC_INCORRECT_LENGTH:       return "incorrectMessageLength";
    case UDS_NRC_CONDITIONS_NOT_MET:     return "conditionsNotCorrect";
    case UDS_NRC_REQUEST_OUT_OF_RANGE:   return "requestOutOfRange";
    case UDS_NRC_SECURITY_ACCESS_DENIED: return "securityAccessDenied";
    case UDS_NRC_RESPONSE_PENDING:       return "responsePending";
    default:                             return "unknown";
    }
}