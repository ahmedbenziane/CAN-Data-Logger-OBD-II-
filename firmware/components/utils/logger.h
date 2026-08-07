#pragma once
/* ============================================================
 *  logger.h
 *  Structured logging helpers for CAN stack.
 *  Wraps ESP_LOG macros with consistent formatting.
 * ============================================================ */

#include "esp_log.h"
#include "../can_core/frame/can_frame.h"

/* Print a CAN frame to serial in a consistent format:
 * [RX] ID=0x100 DLC=8 [01 02 03 04 05 06 07 08] ts=123456us
 */
static inline void log_can_frame(const char *tag, const char *dir,
                                  const can_frame_t *f)
{
    ESP_LOGI(tag,
        "[%s] ID=0x%03lX DLC=%d [%02X %02X %02X %02X %02X %02X %02X %02X] ts=%lldus",
        dir,
        (unsigned long)f->id,
        f->dlc,
        f->data[0], f->data[1], f->data[2], f->data[3],
        f->data[4], f->data[5], f->data[6], f->data[7],
        f->timestamp_us
    );
}