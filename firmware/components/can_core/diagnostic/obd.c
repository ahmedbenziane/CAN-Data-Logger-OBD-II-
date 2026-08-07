/* ============================================================
 *  obd.c  —  Step 6: OBD-II Request/Response Engine
 * ============================================================ */

#include "obd.h"
#include "esp_log.h"
#include "esp_timer.h"
#include <string.h>

static const char *TAG = "OBD  ";

/* ── Internal: decode a raw response into a signal ───────── */
static esp_err_t decode_response(const uint8_t *buf, size_t len,
                                 uint8_t expected_pid,
                                 obd_signal_t *out)
{
    /* Positive response layout: [0x41, PID, data_A, data_B?] */
    if (len < 3)
    {
        ESP_LOGW(TAG, "Response too short: %u bytes", (unsigned)len);
        return ESP_FAIL;
    }
    if (buf[0] != OBD_POSITIVE_RESPONSE)
    {
        ESP_LOGW(TAG, "Not a positive response: 0x%02X", buf[0]);
        return ESP_FAIL;
    }
    if (buf[1] != expected_pid)
    {
        ESP_LOGW(TAG, "PID mismatch: got 0x%02X expected 0x%02X",
                 buf[1], expected_pid);
        return ESP_FAIL;
    }

    out->pid = expected_pid;
    out->valid = true;
    out->timestamp_us = esp_timer_get_time();

    uint8_t A = (len > 2) ? buf[2] : 0;
    uint8_t B = (len > 3) ? buf[3] : 0;

    switch (expected_pid)
    {

    case OBD_PID_RPM:
        /* Formula: RPM = ((A * 256) + B) / 4 */
        out->value = ((A * 256.0f) + B) / 4.0f;
        snprintf(out->unit, sizeof(out->unit), "rpm");
        break;

    case OBD_PID_SPEED:
        /* Formula: Speed = A  (km/h) */
        out->value = (float)A;
        snprintf(out->unit, sizeof(out->unit), "km/h");
        break;

    case OBD_PID_ENGINE_LOAD:
        /* Formula: Load = A * 100 / 255  (%) */
        out->value = A * 100.0f / 255.0f;
        snprintf(out->unit, sizeof(out->unit), "%%");
        break;

    case OBD_PID_COOLANT_TEMP:
        /* Formula: Temp = A - 40  (°C) */
        out->value = (float)A - 40.0f;
        snprintf(out->unit, sizeof(out->unit), "degC");
        break;

    case OBD_PID_INTAKE_TEMP:
        /* Formula: Temp = A - 40  (°C) */
        out->value = (float)A - 40.0f;
        snprintf(out->unit, sizeof(out->unit), "degC");
        break;

    case OBD_PID_THROTTLE:
        /* Formula: Throttle = A * 100 / 255  (%) */
        out->value = A * 100.0f / 255.0f;
        snprintf(out->unit, sizeof(out->unit), "%%");
        break;

    case OBD_PID_SUPPORTED_01_20:
        /* Returns 4-byte bitmask of supported PIDs 0x01-0x20 */
        out->value = (float)((A << 24) | (B << 16) |
                             ((len > 4 ? buf[4] : 0) << 8) |
                             (len > 5 ? buf[5] : 0));
        snprintf(out->unit, sizeof(out->unit), "mask");
        break;

    default:
        /* Unknown PID — return raw byte A */
        out->value = (float)A;
        snprintf(out->unit, sizeof(out->unit), "raw");
        break;
    }
    return ESP_OK;
}

/* ── ISO-TP RX callback: called when full response arrives ── */
static void on_isotp_rx(uint16_t src_id,
                        const uint8_t *payload,
                        size_t length,
                        void *user_data)
{
    obd_engine_t *eng = (obd_engine_t *)user_data;
    if (!eng || length == 0)
        return;

    /* ISO-TP strips the PCI byte before calling this.
     * Payload layout: [0x41][PID][A][B?] for positive response
     *                 [0x7F][SID][NRC]   for negative response
     * No extra length field — the ISO-TP SF length is already consumed. */
    size_t copy = (length < sizeof(eng->response_buf))
                      ? length
                      : sizeof(eng->response_buf) - 1;
    memcpy(eng->response_buf, payload, copy);
    eng->response_len = copy;
    eng->response_ready = true;

    ESP_LOGD(TAG, "RX response %u bytes from 0x%03X  [%02X %02X %02X ...]",
             (unsigned)length, src_id,
             copy > 0 ? payload[0] : 0,
             copy > 1 ? payload[1] : 0,
             copy > 2 ? payload[2] : 0);
}

/* ── Public API ───────────────────────────────────────────── */

void obd_init(obd_engine_t *eng, isotp_tx_fn_t tx_fn)
{
    memset(eng, 0, sizeof(*eng));
    isotp_init(&eng->isotp,
               OBD_REQ_ID,  /* we transmit on 0x7DF */
               OBD_RESP_ID, /* we expect response on 0x7E8 */
               tx_fn,
               on_isotp_rx,
               eng); /* pass engine as user_data to callback */
    ESP_LOGI(TAG, "OBD engine ready  req=0x%03X resp=0x%03X",
             OBD_REQ_ID, OBD_RESP_ID);
}

void obd_process_frame(obd_engine_t *eng, const can_frame_t *frame)
{
    isotp_process_frame(&eng->isotp, frame);
}

esp_err_t obd_request(obd_engine_t *eng, uint8_t pid, obd_signal_t *out)
{
    if (!eng || !out)
        return ESP_ERR_INVALID_ARG;

    /* Build OBD-II Mode 01 request — ISO 15765-4 requires 8 bytes total.
     * Padding byte is conventionally 0x00 or 0x55 (Renault accepts both,
     * 0x00 matches Service 01 spec). ECUs silently reject under-length
     * requests: they ACK the CAN frame but never reply at the OBD layer.
     * This is the cause of the "TX succeeds, no response" symptom on
     * real Renault Clio IV. */
    uint8_t req[2] = {OBD_SERVICE_01, pid};

    /*uint8_t req[8] = {
        0x02, OBD_SERVICE_01, pid,
        0x00, 0x00, 0x00, 0x00, 0x00};*/

    eng->response_ready = false;
    eng->req_count++;
    /* Reset ISO-TP state before each request.
     * If a previous arbitration loss left tx_state=WAIT_FC,
     * the next send would return ISOTP_ERR_BUSY immediately.
     * isotp_reset() forces tx_state back to IDLE.          */
    isotp_reset(&eng->isotp);

    /* Send via ISO-TP (always SF for a 3-byte request) */
    esp_err_t ret = isotp_send(&eng->isotp, req, sizeof(req));
    /* now 8 not 3 */
    if (ret != ESP_OK)
    {
        ESP_LOGW(TAG, "isotp_send failed PID=0x%02X: %s",
                 pid, esp_err_to_name(ret));
        eng->err_count++;
        return ret;
    }

    /* Wait for response (polling, non-RTOS-blocking).
     * NRC 0x78 (requestCorrectlyReceived-ResponsePending) is common on
     * Renault ECUs: the ECU acknowledges the request, needs more processing
     * time, and will send the real answer later. We detect it, log it, and
     * extend the deadline rather than returning an error. */
    int64_t deadline = esp_timer_get_time() +
                       (int64_t)OBD_RESPONSE_TIMEOUT_MS * 1000LL;

    for (;;)
    {
        if (eng->response_ready)
        {
            eng->response_ready = false; /* consume */

            /* NRC 0x78: response pending — ECU needs more time */
            if (eng->response_len >= 3 &&
                eng->response_buf[0] == 0x7F &&
                eng->response_buf[2] == 0x78)
            {
                ESP_LOGD(TAG, "NRC 0x78 (response pending) PID=0x%02X — extending timeout", pid);
                deadline = esp_timer_get_time() + 1000000LL; /* extra 1 s */
                continue;
            }
            break; /* real response (positive or negative) */
        }

        isotp_tick(&eng->isotp);
        if (esp_timer_get_time() > deadline)
        {
            ESP_LOGW(TAG, "Timeout PID=0x%02X", pid);
            eng->timeout_count++;
            out->valid = false;
            return ESP_ERR_TIMEOUT;
        }
        vTaskDelay(1); /* yield 1ms */
    }

    /* Decode the response.
     * ISO-TP strips the PCI byte; payload delivered to on_isotp_rx is:
     *   [0x41][PID][A][B?]  — positive response
     *   [0x7F][SID][NRC]    — negative response
     * Pass the buffer directly — no extra length prefix to skip. */
    if (eng->response_len < 2)
    {
        eng->err_count++;
        out->valid = false;
        return ESP_FAIL;
    }

    ret = decode_response(eng->response_buf,
                          eng->response_len,
                          pid, out);
    if (ret == ESP_OK)
    {
        eng->ok_count++;
    }
    else
    {
        eng->err_count++;
        out->valid = false;
    }
    return ret;
}

void obd_print_signal(const obd_signal_t *sig, const char *tag)
{
    if (!sig->valid)
    {
        ESP_LOGI(tag, "PID=0x%02X  INVALID", sig->pid);
        return;
    }
    /* Format depends on PID */
    switch (sig->pid)
    {
    case OBD_PID_RPM:
        ESP_LOGI(tag, "RPM          = %.0f %s", sig->value, sig->unit);
        break;
    case OBD_PID_SPEED:
        ESP_LOGI(tag, "Speed        = %.0f %s", sig->value, sig->unit);
        break;
    case OBD_PID_ENGINE_LOAD:
        ESP_LOGI(tag, "Engine Load  = %.1f %s", sig->value, sig->unit);
        break;
    case OBD_PID_COOLANT_TEMP:
        ESP_LOGI(tag, "Coolant Temp = %.0f %s", sig->value, sig->unit);
        break;
    case OBD_PID_INTAKE_TEMP:
        ESP_LOGI(tag, "Intake Temp  = %.0f %s", sig->value, sig->unit);
        break;
    case OBD_PID_THROTTLE:
        ESP_LOGI(tag, "Throttle     = %.1f %s", sig->value, sig->unit);
        break;
    default:
        ESP_LOGI(tag, "PID=0x%02X     = %.2f %s",
                 sig->pid, sig->value, sig->unit);
        break;
    }
}

void obd_print_stats(const obd_engine_t *eng, const char *tag)
{
    ESP_LOGI(tag,
             "OBD stats: req=%lu ok=%lu err=%lu tmo=%lu",
             (unsigned long)eng->req_count,
             (unsigned long)eng->ok_count,
             (unsigned long)eng->err_count,
             (unsigned long)eng->timeout_count);
}