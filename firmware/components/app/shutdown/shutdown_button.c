/* ============================================================
 *  shutdown_button.c  —  Graceful SD shutdown via jumper wire
 *
 *  See shutdown_button.h for wiring.
 *
 *  Status is signalled to the dashboard via printf JSON lines.
 *  No LED, no GPIO output — only the input pin is used.
 * ============================================================ */

#include "shutdown_button.h"
#include "sd_logger.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"
#include <stdio.h>

static const char *TAG = "SHUTDOWN";

#define DEBOUNCE_TICKS 5 /* 5 × 10 ms = 50 ms */

static inline uint32_t _now_ms(void)
{
    return (uint32_t)(esp_timer_get_time() / 1000);
}

/* Emit a banner state to the dashboard. The dashboard's message
 * handler renders the state as a coloured fullscreen overlay.   */
static void _emit_state(const char *state)
{
    printf("{\"t\":%lu,\"type\":\"shutdown\",\"state\":\"%s\"}\n",
           (unsigned long)_now_ms(), state);
}

static void _button_task(void *arg)
{
    uint32_t low_count = 0;

    for (;;)
    {
        int lvl = gpio_get_level(SHUTDOWN_BUTTON_GPIO);

        if (lvl == 0)
        {
            if (++low_count >= DEBOUNCE_TICKS)
            {
                ESP_LOGW(TAG, "Shutdown button pressed — flushing SD...");
                _emit_state("flushing");

                /* Give the dashboard one frame to render the banner
                 * before sd_logger_deinit blocks on file I/O.       */
                vTaskDelay(pdMS_TO_TICKS(50));

                sd_logger_deinit();

                ESP_LOGI(TAG, "✓ Safe to power off");
                _emit_state("safe");

                /* Re-emit "safe" periodically so a late-connecting
                 * dashboard (or a refresh) still sees the banner.   */
                for (;;)
                {
                    vTaskDelay(pdMS_TO_TICKS(2000));
                    _emit_state("safe");
                }
            }
        }
        else
        {
            low_count = 0;
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

esp_err_t shutdown_button_init(void)
{
    ESP_LOGI(TAG, "Init: button=GPIO%d (internal pull-up, jumper-to-GND)",
             SHUTDOWN_BUTTON_GPIO);

    /* Button input with internal pull-up. GPIO33 idles HIGH.
     * Touching a jumper from this pin to any GND pin reads LOW.
     * After 50 ms of debounced LOW, the shutdown sequence fires. */
    gpio_config_t btn_cfg = {
        .pin_bit_mask = (1ULL << SHUTDOWN_BUTTON_GPIO),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&btn_cfg);

    xTaskCreatePinnedToCore(_button_task, "shutdown_btn", 4096,
                            NULL, 1, NULL, 1);

    ESP_LOGI(TAG, "✓ Shutdown button armed (touch jumper to GND to flush)");
    return ESP_OK;
}