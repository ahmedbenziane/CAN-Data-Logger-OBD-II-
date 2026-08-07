#pragma once
/* ============================================================
 *  mcp2515.h  —  Step 4: Hardware filters + filter config API
 *  Adds: mcp2515_set_filter(), mcp2515_set_rx_mode()
 *  Unchanged: all Step 1 + Step 3 API
 * ============================================================ */

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "hal_config.h"

/* ── Operating modes ───────────────────────────────────────── */
typedef enum {
    MCP_MODE_NORMAL      = 0x00,
    MCP_MODE_SLEEP       = 0x20,
    MCP_MODE_LOOPBACK    = 0x40,
    MCP_MODE_LISTEN_ONLY = 0x60,
    MCP_MODE_CONFIG      = 0x80,
} mcp_mode_t;

/* ── CAN Frame ─────────────────────────────────────────────── */
typedef struct {
    uint32_t id;
    uint8_t  dlc;
    uint8_t  data[8];
    int64_t  timestamp_us;
} can_frame_t;

/* ── Hardware Filter Config ────────────────────────────────── */
typedef struct {
    uint32_t id;     /* CAN ID to match                          */
    uint32_t mask;   /* 1 = bit must match, 0 = don't care       */
                     /* 0x7FF = exact 11-bit match               */
                     /* 0x000 = accept all IDs                   */
} can_filter_t;

/* ── RX Buffer Mode ────────────────────────────────────────── */
typedef enum {
    RX_MODE_FILTER   = 0x00,  /* use acceptance filters (default) */
    RX_MODE_ACCEPT_ALL = 0x60,/* bypass filters — accept any frame */
} can_rx_mode_t;

/* ── Init config ───────────────────────────────────────────── */
typedef struct {
    spi_host_device_t spi_host;
    gpio_num_t        cs_pin;
    gpio_num_t        int_pin;
    uint32_t          bitrate_bps;
    uint8_t           osc_mhz;
    mcp_mode_t        initial_mode;
} mcp2515_config_t;

/* ── Step 1 + 3 API (unchanged) ────────────────────────────── */
esp_err_t mcp2515_init          (const mcp2515_config_t *cfg);
esp_err_t mcp2515_reset         (void);
esp_err_t mcp2515_set_mode      (mcp_mode_t mode);
esp_err_t mcp2515_set_bitrate   (uint32_t bps, uint8_t osc_mhz);
esp_err_t mcp2515_self_test     (void);
uint8_t   mcp2515_read_reg      (uint8_t addr);
void      mcp2515_write_reg     (uint8_t addr, uint8_t val);
void      mcp2515_bit_modify    (uint8_t addr, uint8_t mask, uint8_t val);
uint8_t   mcp2515_read_status   (void);
uint8_t   mcp2515_read_errors   (void);
uint8_t   mcp2515_read_eflg     (void);  /* EFLG: bit6=RXBO overflow, bit7=bus-off */
esp_err_t mcp2515_transmit_frame(const can_frame_t *frame, uint32_t timeout_ms);
esp_err_t mcp2515_receive_frame (can_frame_t *frame);
bool      mcp2515_frame_available(void);
void      mcp2515_clear_interrupts(void);
void      mcp2515_register_rx_task(TaskHandle_t task_handle);

/* ── Step 4 API — Hardware Filters ─────────────────────────── */

/**
 * @brief  Configure one acceptance filter (0-5) and its mask.
 *         Filters 0-1 apply to RXB0, filters 2-5 apply to RXB1.
 *         Must be called while MCP2515 is in CONFIG mode.
 *
 * @param  filter_n   Filter index 0-5
 * @param  f          Filter ID and mask
 */
esp_err_t mcp2515_set_filter(uint8_t filter_n, const can_filter_t *f);

/**
 * @brief  Set RX buffer acceptance mode.
 *         RX_MODE_ACCEPT_ALL: bypass all filters (used for sniffing)
 *         RX_MODE_FILTER:     use configured filters (normal operation)
 *
 * @param  rxb    RX buffer index: 0 or 1
 * @param  mode   RX_MODE_FILTER or RX_MODE_ACCEPT_ALL
 */
esp_err_t mcp2515_set_rx_mode(uint8_t rxb, can_rx_mode_t mode);

/**
 * @brief  Configure both RX buffers to accept every standard CAN ID.
 *         This is a convenience wrapper used by passive sniffing mode.
 */
esp_err_t mcp2515_set_filter_accept_all(void);

/**
 * @brief  Read overflow counters from the ring buffer.
 *         Returns number of frames dropped since last call.
 */
uint32_t mcp2515_get_overflow_count(void);

/* ── Ring buffer read API (used by app_task) ────────────────── */

/**
 * @brief  Pop the next frame from the ring buffer.
 *         Called by the application task (not the rx task).
 * @return ESP_OK if frame returned, ESP_ERR_NOT_FOUND if empty.
 */
esp_err_t mcp2515_pop_frame(can_frame_t *frame);

/**
 * @brief  Returns true if the ring buffer has at least one frame.
 */
bool mcp2515_ring_has_frame(void);
