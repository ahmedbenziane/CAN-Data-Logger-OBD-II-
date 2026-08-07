/* ============================================================
 *  mcp2515.c  —  Step 4: Hardware filters + ring buffer
 *
 *  New in Step 4:
 *    mcp2515_set_filter()        — configure RX acceptance filter
 *    mcp2515_set_rx_mode()       — set RXBn accept-all vs filter
 *    mcp2515_get_overflow_count()— ring buffer overflow stats
 *    Ring buffer integrated into receive path
 * ============================================================ */

#include "mcp2515.h"
#include "mcp2515_regs.h"
#include "hal_config.h"
#include "ring_buffer.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "driver/gpio.h"
#include "freertos/semphr.h"

static const char *TAG = "MCP2515";

static spi_device_handle_t s_dev          = NULL;
static mcp2515_config_t    s_cfg;
static bool                s_initialized  = false;
static TaskHandle_t        s_rx_task      = NULL;
static SemaphoreHandle_t   s_spi_mutex    = NULL;  /* protects all SPI access */

/* Ring buffer — shared between rx_task (writer) and app (reader) */
static can_ring_buf_t      s_rx_ring;

/* ============================================================
 *  SPI primitive
 * ============================================================ */

static esp_err_t spi_xfer(const uint8_t *tx, uint8_t *rx, size_t len)
{
    if (!s_dev) return ESP_ERR_INVALID_STATE;
    if (!s_spi_mutex) return ESP_ERR_INVALID_STATE;

    spi_transaction_t t = {
        .length    = len * 8,
        .tx_buffer = tx,
        .rx_buffer = rx,
    };

    /* Take mutex — blocks until SPI bus is free */
    xSemaphoreTake(s_spi_mutex, portMAX_DELAY);
    esp_err_t ret = spi_device_transmit(s_dev, &t);
    xSemaphoreGive(s_spi_mutex);
    return ret;
}

/* ============================================================
 *  Register primitives (Step 1 — unchanged)
 * ============================================================ */

uint8_t mcp2515_read_reg(uint8_t addr)
{
    uint8_t tx[3] = { MCP2515_CMD_READ, addr, 0x00 };
    uint8_t rx[3] = { 0 };
    if (spi_xfer(tx, rx, 3) != ESP_OK) return 0xFF;
    return rx[2];
}

void mcp2515_write_reg(uint8_t addr, uint8_t val)
{
    uint8_t tx[3] = { MCP2515_CMD_WRITE, addr, val };
    spi_xfer(tx, NULL, 3);
}

void mcp2515_bit_modify(uint8_t addr, uint8_t mask, uint8_t val)
{
    uint8_t tx[4] = { MCP2515_CMD_BIT_MODIFY, addr, mask, val };
    spi_xfer(tx, NULL, 4);
}

uint8_t mcp2515_read_status(void)
{
    uint8_t tx[2] = { MCP2515_CMD_READ_STATUS, 0x00 };
    uint8_t rx[2] = { 0 };
    if (spi_xfer(tx, rx, 2) != ESP_OK) return 0xFF;
    return rx[1];
}

uint8_t mcp2515_read_errors(void)
{
    return mcp2515_read_reg(MCP2515_REG_TEC);
}

uint8_t mcp2515_read_eflg(void)
{
    return mcp2515_read_reg(MCP2515_REG_EFLG);
}

/* ============================================================
 *  Reset / Mode / Bitrate / Self-test (Step 1 — unchanged)
 * ============================================================ */

esp_err_t mcp2515_reset(void)
{
    uint8_t cmd = MCP2515_CMD_RESET;
    esp_err_t ret = spi_xfer(&cmd, NULL, 1);
    vTaskDelay(pdMS_TO_TICKS(10));
    return ret;
}

esp_err_t mcp2515_set_mode(mcp_mode_t mode)
{
    mcp2515_bit_modify(MCP2515_REG_CANCTRL, 0xE0, (uint8_t)mode);
    for (int i = 0; i < 20; i++) {
        if ((mcp2515_read_reg(MCP2515_REG_CANSTAT) & 0xE0) == (uint8_t)mode)
            return ESP_OK;
        vTaskDelay(pdMS_TO_TICKS(1));
    }
    ESP_LOGE(TAG, "set_mode timeout");
    return ESP_ERR_TIMEOUT;
}

esp_err_t mcp2515_set_bitrate(uint32_t bps, uint8_t osc_mhz)
{
    esp_err_t ret = mcp2515_set_mode(MCP_MODE_CONFIG);
    if (ret != ESP_OK) return ret;
    if (bps == 500000 && osc_mhz == 8) {
        mcp2515_write_reg(MCP2515_REG_CNF1, 0x00);
        mcp2515_write_reg(MCP2515_REG_CNF2, 0x90);
        mcp2515_write_reg(MCP2515_REG_CNF3, 0x02);
        return ESP_OK;
    }
    if (bps == 500000 && osc_mhz == 16) {
        mcp2515_write_reg(MCP2515_REG_CNF1, 0x00);
        mcp2515_write_reg(MCP2515_REG_CNF2, 0xF0);
        mcp2515_write_reg(MCP2515_REG_CNF3, 0x86);
        return ESP_OK;
    }
    if (bps == 250000 && osc_mhz == 8) {
        mcp2515_write_reg(MCP2515_REG_CNF1, 0x00);
        mcp2515_write_reg(MCP2515_REG_CNF2, 0xB1);
        mcp2515_write_reg(MCP2515_REG_CNF3, 0x05);
        return ESP_OK;
    }
    if (bps == 125000 && osc_mhz == 8) {
        mcp2515_write_reg(MCP2515_REG_CNF1, 0x01);
        mcp2515_write_reg(MCP2515_REG_CNF2, 0xB1);
        mcp2515_write_reg(MCP2515_REG_CNF3, 0x05);
        return ESP_OK;
    }
    ESP_LOGE(TAG, "set_bitrate: unsupported %lubps@%dMHz",
             (unsigned long)bps, osc_mhz);
    return ESP_ERR_NOT_SUPPORTED;
}

static void report(bool pass, const char *name, int *p, int *f)
{
    if (pass) { (*p)++; ESP_LOGI(TAG, "  [PASS] %s", name); }
    else       { (*f)++; ESP_LOGE(TAG, "  [FAIL] %s", name); }
}

esp_err_t mcp2515_self_test(void)
{
    int passed = 0, failed = 0;
    esp_err_t ret;
    ESP_LOGI(TAG, "--- Self-Test Start ---");

    ret = mcp2515_reset();
    uint8_t cs = mcp2515_read_reg(MCP2515_REG_CANSTAT);
    report(ret == ESP_OK && (cs & 0xE0) == (uint8_t)MCP_MODE_CONFIG,
           "T1: CANSTAT=0x80 -> Config mode", &passed, &failed);

    mcp2515_write_reg(MCP2515_REG_CNF1, 0xA5);
    report(mcp2515_read_reg(MCP2515_REG_CNF1) == 0xA5,
           "T2: CNF1 write/readback 0xA5", &passed, &failed);

    mcp2515_write_reg(MCP2515_REG_CNF2, 0x5A);
    report(mcp2515_read_reg(MCP2515_REG_CNF2) == 0x5A,
           "T3: CNF2 write/readback 0x5A", &passed, &failed);

    mcp2515_write_reg(MCP2515_REG_CANINTE, 0x00);
    mcp2515_bit_modify(MCP2515_REG_CANINTE, 0x03, 0x03);
    uint8_t inte = mcp2515_read_reg(MCP2515_REG_CANINTE);
    report((inte & 0x03) == 0x03,
           "T4: BIT_MODIFY CANINTE[1:0] set", &passed, &failed);
    mcp2515_write_reg(MCP2515_REG_CANINTE, 0x00);

    ret = mcp2515_set_bitrate(s_cfg.bitrate_bps, s_cfg.osc_mhz);
    report(ret == ESP_OK, "T5: Bitrate configured", &passed, &failed);

    ret = mcp2515_set_mode(MCP_MODE_LOOPBACK);
    report(ret == ESP_OK, "T6: Loopback mode entered", &passed, &failed);

    ESP_LOGI(TAG, "--- Self-Test Done: %d passed, %d failed ---",
             passed, failed);
    return (failed == 0) ? ESP_OK : ESP_FAIL;
}

/* ============================================================
 *  Step 4: Hardware Filter API
 *
 *  MCP2515 has 6 acceptance filters and 2 masks:
 *    Filters 0,1  + Mask 0  →  RXB0
 *    Filters 2,3,4,5 + Mask 1  →  RXB1
 *
 *  For 11-bit standard IDs:
 *    FILTERnSIDH = ID[10:3]
 *    FILTERnSIDL = ID[2:0] << 5   (bits 7:5)
 *
 *  A frame is accepted if: (received_id & mask) == (filter & mask)
 *  With mask=0x7FF (exact match): only frames with that exact ID pass.
 *  With mask=0x000 (accept all):  every frame passes regardless of ID.
 * ============================================================ */

/* Register address tables for filters 0-5 */
static const uint8_t FILTER_SIDH[6] = {
    0x00, 0x04, 0x08, 0x10, 0x14, 0x18
};
static const uint8_t FILTER_SIDL[6] = {
    0x01, 0x05, 0x09, 0x11, 0x15, 0x19
};

/* Mask register addresses: index 0 → RXB0, index 1 → RXB1 */
static const uint8_t MASK_SIDH[2] = { MCP2515_REG_RXM0SIDH,
                                       MCP2515_REG_RXM1SIDH };
static const uint8_t MASK_SIDL[2] = { MCP2515_REG_RXM0SIDL,
                                       MCP2515_REG_RXM1SIDL };

esp_err_t mcp2515_set_filter(uint8_t filter_n, const can_filter_t *f)
{
    if (filter_n > 5 || !f) return ESP_ERR_INVALID_ARG;

    /* Must be in CONFIG mode to change filters */
    mcp_mode_t saved = (mcp_mode_t)(
        mcp2515_read_reg(MCP2515_REG_CANSTAT) & 0xE0);
    if (saved != MCP_MODE_CONFIG) {
        esp_err_t ret = mcp2515_set_mode(MCP_MODE_CONFIG);
        if (ret != ESP_OK) return ret;
    }

    /* Write filter ID */
    mcp2515_write_reg(FILTER_SIDH[filter_n],
                      (uint8_t)(f->id >> 3));
    mcp2515_write_reg(FILTER_SIDL[filter_n],
                      (uint8_t)((f->id & 0x07) << 5));

    /* Write corresponding mask (filter 0,1 → mask 0; filter 2-5 → mask 1) */
    uint8_t mask_n = (filter_n <= 1) ? 0 : 1;
    mcp2515_write_reg(MASK_SIDH[mask_n],
                      (uint8_t)(f->mask >> 3));
    mcp2515_write_reg(MASK_SIDL[mask_n],
                      (uint8_t)((f->mask & 0x07) << 5));

    ESP_LOGD(TAG, "filter[%d] id=0x%03lX mask=0x%03lX",
             filter_n, (unsigned long)f->id, (unsigned long)f->mask);

    /* Restore previous mode */
    if (saved != MCP_MODE_CONFIG)
        mcp2515_set_mode(saved);

    return ESP_OK;
}

esp_err_t mcp2515_set_rx_mode(uint8_t rxb, can_rx_mode_t mode)
{
    if (rxb > 1) return ESP_ERR_INVALID_ARG;
    uint8_t ctrl_reg = (rxb == 0) ? MCP2515_REG_RXB0CTRL : MCP2515_REG_RXB1CTRL;

    // Save current mode
    uint8_t canstat = mcp2515_read_reg(MCP2515_REG_CANSTAT) & 0xE0;
    if (canstat != MCP_MODE_CONFIG) {
        mcp2515_set_mode(MCP_MODE_CONFIG);
    }

    mcp2515_bit_modify(ctrl_reg, 0x60, (uint8_t)mode);

    if (canstat != MCP_MODE_CONFIG) {
        mcp2515_set_mode(canstat);   // restore
    }
    return ESP_OK;
}

esp_err_t mcp2515_set_filter_accept_all(void)
{
    esp_err_t ret = mcp2515_set_rx_mode(0, RX_MODE_ACCEPT_ALL);
    if (ret != ESP_OK) return ret;

    return mcp2515_set_rx_mode(1, RX_MODE_ACCEPT_ALL);
}

uint32_t mcp2515_get_overflow_count(void)
{
    return ring_buf_overflows(&s_rx_ring);
}

/* ============================================================
 *  ISR (Step 3 — unchanged)
 * ============================================================ */

static void IRAM_ATTR mcp2515_isr_handler(void *arg)
{
    (void)arg;
    BaseType_t woken = pdFALSE;
    if (s_rx_task)
        vTaskNotifyGiveFromISR(s_rx_task, &woken);
    portYIELD_FROM_ISR(woken);
}

void mcp2515_register_rx_task(TaskHandle_t task_handle)
{
    s_rx_task = task_handle;
}

/* ============================================================
 *  Init (Step 4: configurable filter setup)
 * ============================================================ */

esp_err_t mcp2515_init(const mcp2515_config_t *cfg)
{
    if (!cfg) return ESP_ERR_INVALID_ARG;
    s_cfg = *cfg;
    ring_buf_init(&s_rx_ring);

    if (!s_initialized) {
        spi_bus_config_t bus_cfg = {
            .mosi_io_num     = CONFIG_SPI_MOSI_PIN_DEFAULT,
            .miso_io_num     = CONFIG_SPI_MISO_PIN_DEFAULT,
            .sclk_io_num     = CONFIG_SPI_SCK_PIN_DEFAULT,
            .quadwp_io_num   = -1,
            .quadhd_io_num   = -1,
            .max_transfer_sz = 64,
        };
        esp_err_t ret = spi_bus_initialize(cfg->spi_host, &bus_cfg,
                                           SPI_DMA_CH_DEFAULT);
        if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) return ret;

        spi_device_interface_config_t dev_cfg = {
            .clock_speed_hz = CONFIG_SPI_CLOCK_HZ_DEFAULT,
            .mode           = 0,
            .spics_io_num   = cfg->cs_pin,
            .queue_size     = 8,
        };
        ret = spi_bus_add_device(cfg->spi_host, &dev_cfg, &s_dev);
        if (ret != ESP_OK) return ret;

        /* Create SPI mutex BEFORE marking initialized */
        s_spi_mutex = xSemaphoreCreateMutex();
        if (!s_spi_mutex) return ESP_ERR_NO_MEM;

        s_initialized = true;
    }

    esp_err_t ret = mcp2515_reset();
    if (ret != ESP_OK) return ret;

    ret = mcp2515_set_bitrate(cfg->bitrate_bps, cfg->osc_mhz);
    if (ret != ESP_OK) return ret;

    /* Default: accept-all on both RX buffers (override with set_filter) */
    mcp2515_write_reg(MCP2515_REG_RXM0SIDH, 0x00);
    mcp2515_write_reg(MCP2515_REG_RXM0SIDL, 0x00);
    mcp2515_write_reg(MCP2515_REG_RXM1SIDH, 0x00);
    mcp2515_write_reg(MCP2515_REG_RXM1SIDL, 0x00);
    /* RXB0CTRL: accept-all (bits 6:5 = 11), BUKT=1 (bit 2 = 1).
     * BUKT=1: if RXB0 is full when a new frame arrives, the frame rolls
     * over into RXB1 (using RXB0's acceptance filters).  This prevents
     * overflow loss during high-load sniffing when the ISR has not yet
     * serviced RXB0.  Each frame still lands in exactly ONE buffer. */
    mcp2515_write_reg(MCP2515_REG_RXB0CTRL, 0x64);  /* accept-all, BUKT=1 */
    mcp2515_write_reg(MCP2515_REG_RXB1CTRL, 0x60);  /* accept-all */

    mcp2515_write_reg(MCP2515_REG_CANINTE,
                      MCP2515_INT_RX0IF | MCP2515_INT_RX1IF);

    gpio_config_t io = {
        .pin_bit_mask = (1ULL << cfg->int_pin),
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_NEGEDGE,
    };
    gpio_config(&io);
    gpio_install_isr_service(0);
    gpio_isr_handler_add(cfg->int_pin, mcp2515_isr_handler, NULL);
    ESP_LOGI(TAG, "INT GPIO%d ISR installed", cfg->int_pin);

    return mcp2515_set_mode(cfg->initial_mode);
}

/* ============================================================
 *  TX Frame (Step 3 — unchanged)
 * ============================================================ */

esp_err_t mcp2515_transmit_frame(const can_frame_t *frame,
                                  uint32_t timeout_ms)
{
    if (!frame || frame->dlc > 8) return ESP_ERR_INVALID_ARG;

    /* ── TX buffer register base addresses ────────────────────────────
     * MCP2515 has 3 TX buffers. Rotating between them prevents TXB0
     * contention when multiple tasks (4 broadcast streams + UDS client)
     * call transmit_frame concurrently. Without rotation, TXB0 stays
     * perpetually busy under high frame rates, causing all other tasks
     * to spin-wait and eventually timeout.
     *
     * Round-robin: pick the first free buffer, prefer the next one in
     * sequence to spread load. Under normal 500kbps CAN, each 8-byte
     * frame takes ~144 µs; at 100fps combined rate TXB0 alone would be
     * busy ~1.4% of the time — but with 4 tasks firing simultaneously
     * they all land on TXB0 in the same 1ms tick.
     */
    static const uint8_t  ctrl_regs[3] = {
        MCP2515_REG_TXB0CTRL, MCP2515_REG_TXB1CTRL, MCP2515_REG_TXB2CTRL
    };
    static const uint8_t  sidh_regs[3] = {
        MCP2515_REG_TXB0SIDH, MCP2515_REG_TXB1SIDH, MCP2515_REG_TXB2SIDH
    };
    static const uint8_t  sidl_regs[3] = {
        MCP2515_REG_TXB0SIDL, MCP2515_REG_TXB1SIDL, MCP2515_REG_TXB2SIDL
    };
    static const uint8_t  dlc_regs[3]  = {
        MCP2515_REG_TXB0DLC,  MCP2515_REG_TXB1DLC,  MCP2515_REG_TXB2DLC
    };
    static const uint8_t  d0_regs[3]   = {
        MCP2515_REG_TXB0D0,   MCP2515_REG_TXB1D0,   MCP2515_REG_TXB2D0
    };
    static const uint8_t  rts_cmds[3]  = {
        MCP2515_CMD_RTS_TXB0, MCP2515_CMD_RTS_TXB1, MCP2515_CMD_RTS_TXB2
    };

    /* Find a free TX buffer, starting from the next one after last use */
    static volatile uint8_t s_last_txb = 0;
    int64_t deadline = esp_timer_get_time() + (timeout_ms * 1000LL);
    uint8_t txb = 0xFF;

    while (1) {
        /* Try each buffer once per loop */
        for (uint8_t i = 0; i < 3; i++) {
            uint8_t idx = (s_last_txb + 1 + i) % 3;
            if (!(mcp2515_read_reg(ctrl_regs[idx]) & MCP2515_TXREQ)) {
                txb = idx;
                s_last_txb = idx;
                break;
            }
        }
        if (txb != 0xFF) break;

        if (esp_timer_get_time() > deadline) {
            ESP_LOGE(TAG, "tx: all TXB busy timeout");
            return ESP_ERR_TIMEOUT;
        }
        vTaskDelay(1);
    }

    /* Load frame into selected TX buffer */
    mcp2515_write_reg(sidh_regs[txb], (uint8_t)(frame->id >> 3));
    mcp2515_write_reg(sidl_regs[txb], (uint8_t)((frame->id & 0x07) << 5));
    mcp2515_write_reg(MCP2515_REG_TXB0EID8, 0x00);  /* standard frame */
    mcp2515_write_reg(MCP2515_REG_TXB0EID0, 0x00);
    mcp2515_write_reg(dlc_regs[txb],  frame->dlc);
    for (uint8_t i = 0; i < frame->dlc; i++)
        mcp2515_write_reg(d0_regs[txb] + i, frame->data[i]);

    /* Request to send */
    uint8_t rts = rts_cmds[txb];
    spi_xfer(&rts, NULL, 1);

    /* Wait for completion.
     * MLOA (arbitration loss) is NOT an error — the hardware keeps TXREQ
     * set and retries automatically once the bus is free.  Only TXERR
     * (form/CRC/ACK error) is a real failure that must abort the frame. */
    deadline = esp_timer_get_time() + (timeout_ms * 1000LL);
    while (1) {
        uint8_t ctrl = mcp2515_read_reg(ctrl_regs[txb]);
        if (!(ctrl & MCP2515_TXREQ)) return ESP_OK;
        if (ctrl & MCP2515_TXERR) {
            ESP_LOGE(TAG, "tx: bus error ctrl=0x%02X (TXB%u)", ctrl, txb);
            mcp2515_bit_modify(ctrl_regs[txb], MCP2515_TXREQ, 0);
            return ESP_FAIL;
        }
        /* MLOA set: lost arbitration, hardware will retry — just wait */
        if (esp_timer_get_time() > deadline) {
            ESP_LOGE(TAG, "tx: send timeout (TXB%u)", txb);
            mcp2515_bit_modify(ctrl_regs[txb], MCP2515_TXREQ, 0);
            return ESP_ERR_TIMEOUT;
        }
        vTaskDelay(1);
    }
}

/* ============================================================
 *  RX Frame — Step 4: reads into ring buffer
 *
 *  Two-stage receive:
 *    Stage 1 (rx_task):   mcp2515_receive_frame() reads from
 *                         MCP2515 hardware into the ring buffer
 *    Stage 2 (app_task):  app reads from ring buffer at its pace
 *
 *  This decouples the hardware read timing from application logic.
 * ============================================================ */

bool mcp2515_frame_available(void)
{
    /*
     * MUST read CANINTF directly — NOT mcp2515_read_status() (0xA0).
     *
     * READ_STATUS bits 0-1 (RXB0/RXB1 full) are only cleared by the
     * dedicated READ_RX_BUFFER SPI command (0x90/0x94).
     * Since we use manual register reads, those bits NEVER clear
     * → infinite drain loop reading the same frame forever.
     *
     * CANINTF.RX0IF/RX1IF are cleared by BIT_MODIFY in receive_frame(),
     * so this check is consistent with the clear path.
     */
    uint8_t intf = mcp2515_read_reg(MCP2515_REG_CANINTF);
    return (intf & (MCP2515_INT_RX0IF | MCP2515_INT_RX1IF)) != 0;
}

void mcp2515_clear_interrupts(void)
{
    mcp2515_write_reg(MCP2515_REG_CANINTF, 0x00);
}

esp_err_t mcp2515_receive_frame(can_frame_t *frame)
{
    if (!frame) return ESP_ERR_INVALID_ARG;

    uint8_t intf = mcp2515_read_reg(MCP2515_REG_CANINTF);
    uint8_t sidh_reg, sidl_reg, dlc_reg, d0_reg, clr_mask;

    if (intf & MCP2515_INT_RX0IF) {
        sidh_reg = MCP2515_REG_RXB0SIDH;
        sidl_reg = MCP2515_REG_RXB0SIDL;
        dlc_reg  = MCP2515_REG_RXB0DLC;
        d0_reg   = MCP2515_REG_RXB0D0;
        clr_mask = MCP2515_INT_RX0IF;
    } else if (intf & MCP2515_INT_RX1IF) {
        sidh_reg = MCP2515_REG_RXB1SIDH;
        sidl_reg = MCP2515_REG_RXB1SIDL;
        dlc_reg  = MCP2515_REG_RXB1DLC;
        d0_reg   = MCP2515_REG_RXB1D0;
        clr_mask = MCP2515_INT_RX1IF;
    } else {
        return ESP_ERR_NOT_FOUND;
    }

    frame->timestamp_us = esp_timer_get_time();
    uint8_t sidh = mcp2515_read_reg(sidh_reg);
    uint8_t sidl = mcp2515_read_reg(sidl_reg);
    frame->id    = ((uint32_t)sidh << 3) | ((sidl >> 5) & 0x07);
    frame->dlc   = mcp2515_read_reg(dlc_reg) & 0x0F;
    if (frame->dlc > 8) frame->dlc = 8;
    for (uint8_t i = 0; i < frame->dlc; i++)
        frame->data[i] = mcp2515_read_reg(d0_reg + i);
    for (uint8_t i = frame->dlc; i < 8; i++)
        frame->data[i] = 0x00;

    /*
     * Release the RX buffer BEFORE pushing to ring — minimise HW latency.
     *
     * Use write_reg to zero only the relevant bit — BIT_MODIFY on CANINTF
     * can silently fail on some MCP2515 silicon revisions while in
     * LISTEN-ONLY mode because the chip may ignore BIT_MODIFY when it
     * detects the bus as active. A full write_reg is always safe.
     *
     * We read CANINTF fresh, clear only clr_mask, write back.
     * This avoids accidentally clearing ERRIF or other flags.
     */
    uint8_t intf_now = mcp2515_read_reg(MCP2515_REG_CANINTF);
    mcp2515_write_reg(MCP2515_REG_CANINTF, intf_now & ~clr_mask);

    /* Push into ring buffer — returns false if full (overflow counted) */
    ring_buf_push(&s_rx_ring, frame);

    return ESP_OK;
}

/* Read next frame from the ring buffer (called by app_task) */
esp_err_t mcp2515_pop_frame(can_frame_t *frame)
{
    if (!frame) return ESP_ERR_INVALID_ARG;
    return ring_buf_pop(&s_rx_ring, frame) ? ESP_OK : ESP_ERR_NOT_FOUND;
}

bool mcp2515_ring_has_frame(void)
{
    return !ring_buf_empty(&s_rx_ring);
}