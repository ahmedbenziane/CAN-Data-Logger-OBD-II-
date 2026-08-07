#pragma once
/* ============================================================
 *  hal_config.h
 *  Bridges platformio.ini build_flags pin defines into the
 *  HAL component. Used ONLY inside components/hal/.
 *
 *  Upper layers use project_config.h instead.
 * ============================================================ */

/* SPI pin defaults — overridden by platformio.ini build_flags */
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
#ifndef MCP2515_OSC_MHZ
#define MCP2515_OSC_MHZ 8
#endif
#ifndef CAN_BITRATE
#define CAN_BITRATE 500000
#endif

/* Expose as names used inside mcp2515.c */
#define CONFIG_SPI_MOSI_PIN_DEFAULT SPI_MOSI_PIN
#define CONFIG_SPI_MISO_PIN_DEFAULT SPI_MISO_PIN
#define CONFIG_SPI_SCK_PIN_DEFAULT SPI_SCK_PIN
#define CONFIG_SPI_CLOCK_HZ_DEFAULT (8 * 1000 * 1000)

#if defined(HSPI_HOST)
#define SPI_HOST_DEFAULT HSPI_HOST
#define MCP2515_SPI_HOST HSPI_HOST
#elif defined(SPI2_HOST)
#define SPI_HOST_DEFAULT SPI2_HOST
#define MCP2515_SPI_HOST SPI2_HOST
#else
#error "No compatible SPI host found for MCP2515 HAL"
#endif
#define SPI_DMA_CH_DEFAULT SPI_DMA_CH_AUTO
