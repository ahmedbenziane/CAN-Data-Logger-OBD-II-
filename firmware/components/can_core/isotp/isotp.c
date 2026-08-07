/* ============================================================
 *  isotp.c  —  Step 5: ISO-TP Transport Layer (ISO 15765-2)
 *
 *  Frame format recap (classical CAN, 8-byte data field):
 *
 *  Single Frame (SF):
 *    [0]: 0x0N  (N = payload length, 1-7)
 *    [1..N]: payload
 *
 *  First Frame (FF):
 *    [0]: 0x1H  (H = high nibble of 12-bit length)
 *    [1]: 0xLL  (low byte of 12-bit length)
 *    [2..7]: first 6 bytes of payload
 *
 *  Consecutive Frame (CF):
 *    [0]: 0x2S  (S = sequence number 1-15, wraps to 0)
 *    [1..7]: up to 7 bytes of payload
 *
 *  Flow Control (FC):
 *    [0]: 0x3F  (F = 0=CTS, 1=Wait, 2=Overflow)
 *    [1]: Block Size (0 = send all)
 *    [2]: STmin (0-127=ms, 0xF1-0xF9=100-900µs)
 * ============================================================ */

#include "isotp.h"
#include "esp_log.h"
#include "esp_timer.h"
#include <string.h>

static const char *TAG = "ISOTP";

/* ── Internal helpers ─────────────────────────────────────── */

static inline int64_t now_us(void) { return esp_timer_get_time(); }
static inline int64_t ms_to_us(int ms) { return (int64_t)ms * 1000LL; }

/* STmin: convert FC byte to microseconds */
static uint32_t stmin_to_us(uint8_t stmin)
{
    if (stmin == 0)
        return 0;
    if (stmin <= 0x7F)
        return (uint32_t)stmin * 1000UL; /* 1-127ms */
    if (stmin >= 0xF1 && stmin <= 0xF9)
        return (uint32_t)(stmin - 0xF0) * 100UL; /* 100-900µs */
    return 0;
}

/* Build and send a Flow Control frame */
static esp_err_t send_fc(isotp_ctx_t *ctx, uint8_t fs, uint8_t bs, uint8_t stmin)
{
    can_frame_t fc = {
        .id = ctx->tx_id,
        .dlc = 8,
        .data = {(uint8_t)(ISOTP_FT_FC | (fs & 0x0F)), bs, stmin,
                 0xAA, 0xAA, 0xAA, 0xAA, 0xAA} /* padding */
    };
    return ctx->tx_fn(&fc);
}

/* ── Public API ───────────────────────────────────────────── */

void isotp_init(isotp_ctx_t *ctx,
                uint16_t tx_id,
                uint16_t rx_id,
                isotp_tx_fn_t tx_fn,
                isotp_rx_cb_t rx_cb,
                void *user_data)
{
    memset(ctx, 0, sizeof(*ctx));
    ctx->tx_id = tx_id;
    ctx->rx_id = rx_id;
    ctx->tx_fn = tx_fn;
    ctx->rx_cb = rx_cb;
    ctx->user_data = user_data;
    ctx->tx_state = ISOTP_TX_IDLE;
    ctx->rx_state = ISOTP_RX_IDLE;
    ESP_LOGI(TAG, "init  tx=0x%03X rx=0x%03X", (unsigned)tx_id, (unsigned)rx_id);
}

void isotp_reset(isotp_ctx_t *ctx)
{
    ctx->tx_state = ISOTP_TX_IDLE;
    ctx->rx_state = ISOTP_RX_IDLE;
    ctx->tx_buf = NULL;
    ctx->tx_len = 0;
    ctx->tx_offset = 0;
    ctx->rx_expected = 0;
    ctx->rx_received = 0;
}

bool isotp_tx_idle(const isotp_ctx_t *ctx) { return ctx->tx_state == ISOTP_TX_IDLE; }
bool isotp_rx_active(const isotp_ctx_t *ctx) { return ctx->rx_state == ISOTP_RX_RECEIVING; }

/* ─────────────────────────────────────────────────────────── *
 *  SEND
 * ─────────────────────────────────────────────────────────── */
esp_err_t isotp_send(isotp_ctx_t *ctx, const uint8_t *data, size_t len)
{
    if (!ctx || !data || len == 0 || len > ISOTP_MAX_PAYLOAD)
        return ISOTP_ERR_INVALID;
    if (ctx->tx_state != ISOTP_TX_IDLE)
        return ISOTP_ERR_BUSY;

    /* ── Single Frame (≤7 bytes) ── */
    if (len <= 7)
    {
        can_frame_t sf = {.id = ctx->tx_id, .dlc = 8};
        memset(sf.data, 0x55, 8); /* pad */
        sf.data[0] = (uint8_t)(ISOTP_FT_SF | len);
        memcpy(&sf.data[1], data, len);

        esp_err_t ret = ctx->tx_fn(&sf);
        if (ret == ESP_OK)
        {
            ctx->tx_count++;
            ESP_LOGD(TAG, "TX SF  len=%u  id=0x%03X", (unsigned)len, (unsigned)ctx->tx_id);
        }
        else
        {
            ctx->tx_errors++;
            ESP_LOGW(TAG, "TX SF failed: %s", esp_err_to_name(ret));
        }
        return ret;
    }

    /* ── Multi-frame: send First Frame ── */
    can_frame_t ff = {.id = ctx->tx_id, .dlc = 8};
    ff.data[0] = (uint8_t)(ISOTP_FT_FF | ((len >> 8) & 0x0F));
    ff.data[1] = (uint8_t)(len & 0xFF);
    memcpy(&ff.data[2], data, 6);

    esp_err_t ret = ctx->tx_fn(&ff);
    if (ret != ESP_OK)
    {
        ctx->tx_errors++;
        ESP_LOGW(TAG, "TX FF failed: %s", esp_err_to_name(ret));
        return ret;
    }

    /* Set up TX state — wait for FC */
    ctx->tx_buf = data;
    ctx->tx_len = len;
    ctx->tx_offset = 6; /* 6 bytes went in FF */
    ctx->tx_sn = 1;     /* CF sequence starts at 1 */
    ctx->tx_state = ISOTP_TX_WAIT_FC;
    ctx->tx_deadline = now_us() + ms_to_us(ISOTP_N_Bs_MS);

    ESP_LOGD(TAG, "TX FF  len=%u  id=0x%03X  waiting FC", (unsigned)len, (unsigned)ctx->tx_id);
    return ISOTP_OK;
}

/* ─────────────────────────────────────────────────────────── *
 *  PROCESS INCOMING FRAME
 * ─────────────────────────────────────────────────────────── */
esp_err_t isotp_process_frame(isotp_ctx_t *ctx, const can_frame_t *frame)
{
    if (!ctx || !frame)
        return ISOTP_ERR_INVALID;
    if (frame->dlc == 0)
        return ISOTP_ERR_INVALID;

    uint8_t pci_type = frame->data[0] & 0xF0;

    switch (pci_type)
    {

    /* ── Single Frame received ── */
    case ISOTP_FT_SF:
    {
        uint8_t sf_len = frame->data[0] & 0x0F;
        if (sf_len == 0 || sf_len > 7 || sf_len > (frame->dlc - 1))
        {
            ESP_LOGW(TAG, "RX SF bad length %u", sf_len);
            ctx->rx_errors++;
            return ISOTP_ERR_INVALID;
        }
        /* Reset any ongoing RX */
        ctx->rx_state = ISOTP_RX_IDLE;
        ctx->rx_count++;
        ESP_LOGD(TAG, "RX SF  len=%u  from=0x%03X", sf_len, (unsigned)frame->id);

        if (ctx->rx_cb)
            ctx->rx_cb(frame->id, &frame->data[1], sf_len, ctx->user_data);
        return ISOTP_OK;
    }

    /* ── First Frame received ── */
    case ISOTP_FT_FF:
    {
        uint16_t ff_len = (uint16_t)((frame->data[0] & 0x0F) << 8) | frame->data[1];
        if (ff_len < 8 || ff_len > ISOTP_MAX_PAYLOAD)
        {
            ESP_LOGW(TAG, "RX FF bad length %u", ff_len);
            ctx->rx_errors++;
            send_fc(ctx, 0x02, 0, 0); /* overflow */
            return ISOTP_ERR_OVERFLOW;
        }

        /* Copy the first 6 bytes that came in the FF */
        ctx->rx_expected = ff_len;
        ctx->rx_received = 6;
        memcpy(ctx->rx_buf, &frame->data[2], 6);
        ctx->rx_next_sn = 1;
        ctx->rx_state = ISOTP_RX_RECEIVING;
        ctx->rx_deadline = now_us() + ms_to_us(ISOTP_N_Cr_MS);

        /* Send Flow Control: CTS, block_size=0 (send all), STmin=0 */
        esp_err_t ret = send_fc(ctx, 0x00, 0x00, 0x00);
        if (ret != ESP_OK)
        {
            ESP_LOGW(TAG, "FC send failed");
            ctx->rx_errors++;
        }
        ESP_LOGD(TAG, "RX FF  total=%u  from=0x%03X  FC sent", ff_len, (unsigned)frame->id);
        return ISOTP_OK;
    }

    /* ── Consecutive Frame received ── */
    case ISOTP_FT_CF:
    {
        if (ctx->rx_state != ISOTP_RX_RECEIVING)
        {
            ESP_LOGW(TAG, "RX CF unexpected (not receiving)");
            ctx->rx_errors++;
            return ISOTP_ERR_INVALID;
        }

        uint8_t sn = frame->data[0] & 0x0F;
        if (sn != ctx->rx_next_sn)
        {
            ESP_LOGW(TAG, "RX CF SN mismatch: got %u expected %u",
                     sn, ctx->rx_next_sn);
            ctx->rx_errors++;
            ctx->rx_state = ISOTP_RX_IDLE;
            return ISOTP_ERR_ABORTED;
        }

        /* How many bytes fit from this CF? */
        size_t remaining = ctx->rx_expected - ctx->rx_received;
        size_t copy_len = (remaining > 7) ? 7 : remaining;

        memcpy(&ctx->rx_buf[ctx->rx_received], &frame->data[1], copy_len);
        ctx->rx_received += copy_len;
        ctx->rx_next_sn = (ctx->rx_next_sn + 1) & 0x0F;
        ctx->rx_deadline = now_us() + ms_to_us(ISOTP_N_Cr_MS);

        ESP_LOGD(TAG, "RX CF  sn=%u  rx=%u/%u",
                 sn, (unsigned)ctx->rx_received, (unsigned)ctx->rx_expected);

        /* Check if message is complete */
        if (ctx->rx_received >= ctx->rx_expected)
        {
            ctx->rx_state = ISOTP_RX_IDLE;
            ctx->rx_count++;
            ESP_LOGD(TAG, "RX complete  len=%u  from=0x%03X",
                     (unsigned)ctx->rx_expected, (unsigned)frame->id);
            if (ctx->rx_cb)
                ctx->rx_cb(frame->id, ctx->rx_buf,
                           ctx->rx_expected, ctx->user_data);
        }
        return ISOTP_OK;
    }

    /* ── Flow Control received (we are the sender) ── */
    case ISOTP_FT_FC:
    {
        if (ctx->tx_state != ISOTP_TX_WAIT_FC)
        {
            ESP_LOGD(TAG, "RX FC ignored (not waiting)");
            return ISOTP_OK;
        }

        uint8_t fs = frame->data[0] & 0x0F;
        uint8_t bs = frame->data[1];
        uint8_t stmin = frame->data[2];

        if (fs == 0x00)
        { /* CTS — Continue To Send */
            ctx->fc_block_size = bs;
            ctx->fc_stmin = stmin;
            ctx->fc_frames_rem = (bs == 0) ? 0xFF : bs; /* 0 = unlimited */
            ctx->tx_state = ISOTP_TX_SENDING;
            ctx->tx_deadline = now_us() + ms_to_us(ISOTP_N_As_MS);
            ESP_LOGD(TAG, "RX FC CTS  bs=%u stmin=%u", bs, stmin);
        }
        else if (fs == 0x01)
        { /* Wait */
            ctx->tx_deadline = now_us() + ms_to_us(ISOTP_N_Bs_MS);
            ESP_LOGD(TAG, "RX FC WAIT — extending deadline");
        }
        else
        { /* Overflow or unknown */
            ESP_LOGW(TAG, "RX FC OVFL/ABORT  fs=%u", fs);
            ctx->tx_errors++;
            ctx->tx_state = ISOTP_TX_IDLE;
        }
        return ISOTP_OK;
    }

    default:
        ESP_LOGW(TAG, "RX unknown PCI type 0x%02X", pci_type);
        ctx->rx_errors++;
        return ISOTP_ERR_INVALID;
    }
}

/* ─────────────────────────────────────────────────────────── *
 *  TICK  — drive TX state machine + timeout checks
 * ─────────────────────────────────────────────────────────── */
void isotp_tick(isotp_ctx_t *ctx)
{
    if (!ctx)
        return;
    int64_t now = now_us();

    /* ── TX timeout checks ── */
    if (ctx->tx_state == ISOTP_TX_WAIT_FC)
    {
        if (now > ctx->tx_deadline)
        {
            ESP_LOGW(TAG, "TX timeout waiting FC  id=0x%03X", (unsigned)ctx->tx_id);
            ctx->tx_errors++;
            ctx->rx_timeouts++;
            ctx->tx_state = ISOTP_TX_IDLE;
        }
        return;
    }

    /* ── RX timeout check ── */
    if (ctx->rx_state == ISOTP_RX_RECEIVING)
    {
        if (now > ctx->rx_deadline)
        {
            ESP_LOGW(TAG, "RX timeout waiting CF  received=%u/%u",
                     (unsigned)ctx->rx_received,
                     (unsigned)ctx->rx_expected);
            ctx->rx_errors++;
            ctx->rx_timeouts++;
            ctx->rx_state = ISOTP_RX_IDLE;
        }
    }

    /* ── TX sending: push next consecutive frame ── */
    if (ctx->tx_state != ISOTP_TX_SENDING)
        return;
    if (ctx->tx_offset >= ctx->tx_len)
    {
        /* All bytes sent */
        ctx->tx_count++;
        ctx->tx_state = ISOTP_TX_IDLE;
        ESP_LOGD(TAG, "TX complete  len=%u  id=0x%03X",
                 (unsigned)ctx->tx_len, (unsigned)ctx->tx_id);
        return;
    }

    /* STmin inter-frame delay */
    if (now < ctx->tx_deadline)
        return;

    /* Build next CF */
    size_t remaining = ctx->tx_len - ctx->tx_offset;
    size_t copy_len = (remaining > 7) ? 7 : remaining;

    can_frame_t cf = {.id = ctx->tx_id, .dlc = 8};
    memset(cf.data, 0x55, 8);
    cf.data[0] = (uint8_t)(ISOTP_FT_CF | (ctx->tx_sn & 0x0F));
    memcpy(&cf.data[1], &ctx->tx_buf[ctx->tx_offset], copy_len);

    esp_err_t ret = ctx->tx_fn(&cf);
    if (ret != ESP_OK)
    {
        ESP_LOGW(TAG, "TX CF failed: %s", esp_err_to_name(ret));
        ctx->tx_errors++;
        ctx->tx_state = ISOTP_TX_IDLE;
        return;
    }

    ctx->tx_offset += copy_len;
    ctx->tx_sn = (ctx->tx_sn + 1) & 0x0F;
    ctx->tx_deadline = now + (int64_t)stmin_to_us(ctx->fc_stmin);

    ESP_LOGD(TAG, "TX CF  sn=%u  sent=%u/%u",
             (ctx->tx_sn - 1) & 0x0F,
             (unsigned)ctx->tx_offset,
             (unsigned)ctx->tx_len);

    /* Block size tracking */
    if (ctx->fc_block_size != 0)
    {
        ctx->fc_frames_rem--;
        if (ctx->fc_frames_rem == 0)
        {
            /* Block done — wait for next FC */
            ctx->tx_state = ISOTP_TX_WAIT_FC;
            ctx->tx_deadline = now + ms_to_us(ISOTP_N_Bs_MS);
            ESP_LOGD(TAG, "TX block done — waiting next FC");
        }
    }
}

/* ─────────────────────────────────────────────────────────── *
 *  STATS
 * ─────────────────────────────────────────────────────────── */
void isotp_print_stats(const isotp_ctx_t *ctx, const char *tag)
{
    ESP_LOGI(tag,
             "ISOTP stats [tx=0x%03X rx=0x%03X]  "
             "tx_ok=%lu tx_err=%lu  rx_ok=%lu rx_err=%lu rx_tmo=%lu",
             (unsigned)ctx->tx_id, (unsigned)ctx->rx_id,
             (unsigned long)ctx->tx_count,
             (unsigned long)ctx->tx_errors,
             (unsigned long)ctx->rx_count,
             (unsigned long)ctx->rx_errors,
             (unsigned long)ctx->rx_timeouts);
}
