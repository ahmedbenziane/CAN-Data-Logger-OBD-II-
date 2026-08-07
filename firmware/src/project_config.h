#pragma once

/* ============================================================
 *  project_config.h
 *  Central configuration header — included by every component.
 *  All values come from platformio.ini build_flags.
 *  NEVER hardcode pins or roles anywhere else.
 * ============================================================ */

#include <stdint.h>
#include <stdbool.h>
#include "driver/spi_master.h"
#include "driver/uart.h"

/* ── Node Role ────────────────────────────────────────────── */
#define NODE_TRANSMITTER 0 /* Node A: diagnostic tester     */
#define NODE_RECEIVER 1    /* Node B: ECU simulator/sniffer */

/* NODE_ROLE is injected by platformio.ini -DNODE_ROLE=...     */
#ifndef NODE_ROLE
#error "NODE_ROLE not defined. Use -DNODE_ROLE in platformio.ini"
#endif

/* ── SPI / MCP2515 Hardware ───────────────────────────────── */
/* All pins injected by platformio.ini                         */
#ifndef SPI_MOSI_PIN
#define SPI_MOSI_PIN 23
#endif
#ifndef SPI_MISO_PIN
#define SPI_MISO_PIN 19
#endif
#ifndef SPI_SCK_PIN
#define SPI_SCK_PIN 18
#endif
#ifndef SPI_CS_PIN
#define SPI_CS_PIN 5
#endif
#ifndef MCP2515_INT_PIN
#define MCP2515_INT_PIN 4
#endif

/* SPI clock — keep at or below 8 MHz for reliable MCP2515 comms */
#define SPI_CLOCK_HZ (8 * 1000 * 1000)
#if defined(HSPI_HOST)
#define MCP2515_SPI_HOST HSPI_HOST
#elif defined(SPI2_HOST)
#define MCP2515_SPI_HOST SPI2_HOST
#else
#error "No compatible SPI host found for MCP2515"
#endif
#define SPI_DMA_CHANNEL 1

/* ── CAN Bus Parameters ───────────────────────────────────── */
#ifndef CAN_BITRATE
#define CAN_BITRATE 500000 /* 500 kbps — ISO 15765-4     */
#endif
#ifndef MCP2515_OSC_MHZ
#define MCP2515_OSC_MHZ 8 /* Crystal MHz on MCP2515 PCB */
#endif

/* ── Ring Buffer ──────────────────────────────────────────── */
#define CAN_RX_RING_SIZE 256 /* Must be power of 2          */
#define CAN_TX_QUEUE_SIZE 16

/* ── FreeRTOS Task Config ─────────────────────────────────── */
#define TASK_STACK_CAN_RX 4096
#define TASK_STACK_CAN_TX 2048
#define TASK_STACK_APP 8192

#define TASK_PRIO_CAN_RX 15 /* Highest — ISR-driven        */
#define TASK_PRIO_CAN_TX 14
#define TASK_PRIO_APP 5

#define TASK_CORE_CAN 0 /* Core 0: all CAN I/O         */
#define TASK_CORE_APP 1 /* Core 1: processing + app    */

/* ── Logging ──────────────────────────────────────────────── */
#define CAN_LOG_UART_NUM UART_NUM_0
#define CAN_LOG_BAUD 115200
