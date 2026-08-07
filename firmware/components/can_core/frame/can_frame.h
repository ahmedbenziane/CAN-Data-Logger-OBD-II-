#pragma once
/* ============================================================
 *  can_frame.h — Step 0 Placeholder
 *  Implemented in: Step 3
 *
 *  Defines the canonical raw CAN frame type used by all layers.
 *  Every layer above L1 passes can_frame_t — never raw bytes.
 * ============================================================ */

#include <stdint.h>

/* CAN frame flags */
#define CAN_FLAG_EXTENDED   (1 << 0)   /* 29-bit extended ID */
#define CAN_FLAG_RTR        (1 << 1)   /* Remote Transmission Request */
#define CAN_FLAG_ERROR      (1 << 2)   /* Error frame */

typedef struct {
    /* cppcheck-suppress unusedStructMember */
    uint32_t id;            /* 11-bit standard or 29-bit extended CAN ID */
    uint8_t  dlc;           /* Data Length Code: 0–8 bytes               */
    uint8_t  data[8];       /* Payload                                    */
    uint8_t  flags;         /* CAN_FLAG_* bitmask                        */
    int64_t  timestamp_us;  /* esp_timer_get_time() at reception          */
    uint8_t  channel;       /* Reserved: future multi-bus support         */
} can_frame_t;
