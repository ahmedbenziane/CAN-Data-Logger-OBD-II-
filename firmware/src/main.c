/**
 * @file main.c
 * @brief Step 14 — Real Car (OBD-II Polling) + Bench Test entry point
 *
 * ── Mode selection ─────────────────────────────────────────────────────
 * Controlled entirely from platformio.ini. Never edit this file to
 * switch modes. Two environments exist:
 *
 *   [env:node_a]                     [env:node_a_bench]
 *   build_flags += -DREAL_CAR_MODE   (REAL_CAR_MODE absent)
 *
 * REAL_CAR_MODE absent  → bench mode (NORMAL, UDS test ON, 60 s auto-stop)
 * REAL_CAR_MODE defined → car mode   (NORMAL, OBD-II poller ON, run forever)
 *
 * Both modes use MCP_MODE_NORMAL. Car mode is NOT LISTEN_ONLY — the OBD-II
 * poller MUST transmit query frames to wake the Renault diagnostic CAN bus
 * and receive responses. LISTEN_ONLY silenced the bus on previous attempts.
 *
 * ── Node roles ─────────────────────────────────────────────────────────
 *   NODE_RECEIVER    (Node A) → decoder + SD logger + GPS + shutdown button
 *                                + UDS client (bench) OR OBD engine (car)
 *   NODE_TRANSMITTER (Node B) → multi-ECU bench simulator (unchanged)
 *
 * ── Flash commands ─────────────────────────────────────────────────────
 *   Car:     pio run --target upload --environment node_a
 *   Bench:   pio run --target upload --environment node_a_bench
 *   Node B:  pio run --target buildfs  --environment node_b
 *            pio run --target uploadfs --environment node_b
 *            pio run --target upload   --environment node_b
 */

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#include "project_config.h"
#include "mcp2515.h"

#if NODE_ROLE == NODE_TRANSMITTER
#include "ecu_sim_app.h"
#endif

#if NODE_ROLE == NODE_RECEIVER
#include "decoder_app.h"
#include "sd_logger.h"
#include "shutdown_button.h"
/* #include "gps_app.h"   // uncomment once NEO-6M is wired */
#endif

#ifndef NODE_NAME
#define NODE_NAME "Node?"
#endif
static const char *TAG = NODE_NAME;

#define SPIFFS_SYSTEM_JSON "/spiffs/system.json"

#if NODE_ROLE == NODE_TRANSMITTER
static ecu_sim_t s_sim;
#endif

void app_main(void)
{
    /* ── Boot banner ────────────────────────────────────────────────── */
    ESP_LOGI(TAG, "╔══════════════════════════════════════════╗");
#ifdef REAL_CAR_MODE
    ESP_LOGI(TAG, "║  Step 14 — REAL CAR  (OBD-II POLLING)    ║");
#else
    ESP_LOGI(TAG, "║  Step 14 — BENCH     (NORMAL mode)       ║");
#endif
    ESP_LOGI(TAG, "║  Node: %-34s ║", NODE_NAME);
    ESP_LOGI(TAG, "╚══════════════════════════════════════════╝");

    /* MCP2515 crystal stabilisation (500 ms for cold POWERON_RESET). */
    vTaskDelay(pdMS_TO_TICKS(500));

#if NODE_ROLE == NODE_RECEIVER
    /* SD logger first — initialises SPI3_HOST (VSPI) while it is free.
     * SD card  → SPI3_HOST  GPIO 13/27/14/26
     * MCP2515  → SPI2_HOST  GPIO 23/19/18/5  (separate bus)             */
    sd_logger_init();

    /* Shutdown button — GPIO33 jumper-to-GND (no LED, no resistor). */
    shutdown_button_init();
#endif

    /* MCP2515 in CONFIG mode here; decoder_app_start() promotes to NORMAL */
    mcp2515_config_t cfg = {
        .spi_host = MCP2515_SPI_HOST,
        .cs_pin = SPI_CS_PIN,
        .int_pin = MCP2515_INT_PIN,
        .osc_mhz = MCP2515_OSC_MHZ,
        .bitrate_bps = CAN_BITRATE,
        .initial_mode = MCP_MODE_CONFIG,
    };

    esp_err_t init_ret = ESP_FAIL;
    for (int attempt = 1; attempt <= 5; attempt++)
    {
        init_ret = mcp2515_init(&cfg);
        if (init_ret == ESP_OK)
            break;
        ESP_LOGW(TAG, "mcp2515_init attempt %d/5: %s — retrying in 500 ms",
                 attempt, esp_err_to_name(init_ret));
        vTaskDelay(pdMS_TO_TICKS(500));
    }
    if (init_ret != ESP_OK)
    {
        ESP_LOGE(TAG, "mcp2515_init failed after 5 attempts — check SPI wiring");
        for (;;)
            vTaskDelay(pdMS_TO_TICKS(1000));
    }
    ESP_LOGI(TAG, "MCP2515 init OK");

/* ── Node B: bench ECU simulator ───────────────────────────────────── */
#if NODE_ROLE == NODE_TRANSMITTER
    ecu_sim_init(&s_sim);
    ecu_sim_load_config(&s_sim, SPIFFS_SYSTEM_JSON);
    ESP_ERROR_CHECK(ecu_sim_start(&s_sim));
    ESP_LOGI(TAG, "Node B — multi-ECU simulator running");
    ESP_LOGI(TAG, "Streams: 0x7E8/100ms  0x0A6/10ms  0x1A0/10ms  0x3B0/20ms");
    for (;;)
        vTaskDelay(pdMS_TO_TICKS(10000));

/* ── Node A: decoder + logger ──────────────────────────────────────── */
#else
    /* Extra 1 s delay before decoder starts:
     *   Bench: lets Node B start broadcasting before Node A listens.
     *   Car:   harmless — the bus is already live before ESP32 boots. */
    vTaskDelay(pdMS_TO_TICKS(1000));

    /* decoder_app_start() reads REAL_CAR_MODE internally:
     *   defined   → NORMAL + dispatcher(0x7E8 → OBD engine) + obd_poller_task
     *   undefined → NORMAL + dispatcher(0x7E8/0xA6/0x1A0/0x3B0) + uds_test_task */
    ESP_ERROR_CHECK(decoder_app_start());

#ifdef REAL_CAR_MODE
    ESP_LOGI(TAG, "Node A — OBD-II active poller (car mode)");
    ESP_LOGI(TAG, "Wiring: OBD pin6=CAN-H  pin14=CAN-L  pin4=GND(ESP32)");
    ESP_LOGI(TAG, "Power:  USB power bank recommended over laptop USB");
    ESP_LOGI(TAG, "Stop:   touch GPIO%d jumper to GND before ignition off",
             SHUTDOWN_BUTTON_GPIO);
    for (;;)
        vTaskDelay(pdMS_TO_TICKS(10000));

#else
    /* Bench: auto-shutdown after 60 s. */
    ESP_LOGI(TAG, "Node A — decoder + UDS test client (bench mode)");
    ESP_LOGI(TAG, "Auto-shutdown in 60 s");
    ESP_LOGI(TAG, "Wait for '✓ SD card safely unmounted' before removing card");
    vTaskDelay(pdMS_TO_TICKS(60000));
    ESP_LOGI(TAG, "Auto-shutdown: flushing SD...");
    sd_logger_deinit();
    ESP_LOGI(TAG, "✓ SD card safe to remove. Halted.");
    for (;;)
        vTaskDelay(pdMS_TO_TICKS(1000));
#endif

#endif /* NODE_ROLE */
}