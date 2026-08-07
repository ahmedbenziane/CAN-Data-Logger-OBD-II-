#pragma once
/* ============================================================
 *  shutdown_button.h  —  Graceful SD shutdown via jumper wire
 *
 *  Bare-wire version: no LED, no button component, no resistors.
 *  Just one jumper wire.
 *
 *  Hardware:
 *    GPIO33  ← jumper wire; touch loose end to a GND pin to "press"
 *              Internal pull-up enabled. Idles HIGH.
 *              50 ms debounce in software handles bouncy contact.
 *
 *  Status feedback:
 *    No LED. Status comes from:
 *      1. Serial monitor (ESP_LOGW / ESP_LOGI lines)
 *      2. Dashboard banner via {"type":"shutdown",...} JSON line
 *
 *  Behaviour:
 *    1. Button is debounced (50 ms LOW hold required).
 *    2. On press:
 *         - Dashboard banner → "FLUSHING SD — DO NOT POWER OFF"
 *         - sd_logger_deinit() called → flush + unmount
 *         - Dashboard banner → "SAFE TO POWER OFF"
 *         - System spins forever
 *    3. Accidental press = halt logging. Power-cycle to restart.
 *
 *  Alternative pins if GPIO33 conflicts: GPIO32 also has an
 *  internal pull-up. Avoid GPIO34/35/36/39 (no pull-up).
 *  Avoid GPIO0/15 (strapping pins).
 * ============================================================ */

#include "esp_err.h"

#define SHUTDOWN_BUTTON_GPIO 33

esp_err_t shutdown_button_init(void);