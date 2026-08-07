#ifndef MCP2515_REGS_H
#define MCP2515_REGS_H

/* ── SPI Commands ──────────────────────────────────────────── */
#define MCP2515_CMD_RESET        0xC0
#define MCP2515_CMD_READ         0x03
#define MCP2515_CMD_WRITE        0x02
#define MCP2515_CMD_BIT_MODIFY   0x05
#define MCP2515_CMD_READ_STATUS  0xA0
#define MCP2515_CMD_RX_STATUS    0xB0
#define MCP2515_CMD_RTS_TXB0     0x81
#define MCP2515_CMD_RTS_TXB1     0x82
#define MCP2515_CMD_RTS_TXB2     0x84

/* ── Control / Status ──────────────────────────────────────── */
#define MCP2515_REG_CANSTAT      0x0E
#define MCP2515_REG_CANCTRL      0x0F

/* ── Bit Timing ────────────────────────────────────────────── */
#define MCP2515_REG_CNF3         0x28
#define MCP2515_REG_CNF2         0x29
#define MCP2515_REG_CNF1         0x2A

/* ── Interrupts ────────────────────────────────────────────── */
#define MCP2515_REG_CANINTE      0x2B
#define MCP2515_REG_CANINTF      0x2C

#define MCP2515_INT_RX0IF        0x01
#define MCP2515_INT_RX1IF        0x02
#define MCP2515_INT_TX0IF        0x04
#define MCP2515_INT_TX1IF        0x08
#define MCP2515_INT_TX2IF        0x10
#define MCP2515_INT_ERRIF        0x20
#define MCP2515_INT_WAKIF        0x40
#define MCP2515_INT_MERRF        0x80

/* ── Error Registers ───────────────────────────────────────── */
#define MCP2515_REG_EFLG         0x2D
#define MCP2515_REG_TEC          0x1C
#define MCP2515_REG_REC          0x1D

/* ── RX Acceptance Masks ───────────────────────────────────── */
#define MCP2515_REG_RXM0SIDH     0x20
#define MCP2515_REG_RXM0SIDL     0x21
#define MCP2515_REG_RXM1SIDH     0x24
#define MCP2515_REG_RXM1SIDL     0x25

/* ── RX Buffer 0 ───────────────────────────────────────────── */
#define MCP2515_REG_RXB0CTRL     0x60
#define MCP2515_REG_RXB0SIDH     0x61
#define MCP2515_REG_RXB0SIDL     0x62
#define MCP2515_REG_RXB0EID8     0x63
#define MCP2515_REG_RXB0EID0     0x64
#define MCP2515_REG_RXB0DLC      0x65
#define MCP2515_REG_RXB0D0       0x66

/* ── RX Buffer 1 ───────────────────────────────────────────── */
#define MCP2515_REG_RXB1CTRL     0x70
#define MCP2515_REG_RXB1SIDH     0x71
#define MCP2515_REG_RXB1SIDL     0x72
#define MCP2515_REG_RXB1EID8     0x73
#define MCP2515_REG_RXB1EID0     0x74
#define MCP2515_REG_RXB1DLC      0x75
#define MCP2515_REG_RXB1D0       0x76

/* ── TX Buffer 0 ───────────────────────────────────────────── */
#define MCP2515_REG_TXB0CTRL     0x30
#define MCP2515_REG_TXB0SIDH     0x31
#define MCP2515_REG_TXB0SIDL     0x32
#define MCP2515_REG_TXB0EID8     0x33
#define MCP2515_REG_TXB0EID0     0x34
#define MCP2515_REG_TXB0DLC      0x35
#define MCP2515_REG_TXB0D0       0x36

/* TXBnCTRL bits */
#define MCP2515_TXREQ            0x08   /* TX Request — frame pending  */
#define MCP2515_TXERR            0x10   /* TX Error flag               */
#define MCP2515_MLOA             0x20   /* Lost arbitration            */
#define MCP2515_ABTF             0x40   /* Aborted TX                  */

/* TX Buffer 1 + 2 (for future steps) */
#define MCP2515_REG_TXB1CTRL     0x40
#define MCP2515_REG_TXB1SIDH     0x41
#define MCP2515_REG_TXB1SIDL     0x42
#define MCP2515_REG_TXB1EID8     0x43
#define MCP2515_REG_TXB1EID0     0x44
#define MCP2515_REG_TXB1DLC      0x45
#define MCP2515_REG_TXB1D0       0x46

#define MCP2515_REG_TXB2CTRL     0x50
#define MCP2515_REG_TXB2SIDH     0x51
#define MCP2515_REG_TXB2SIDL     0x52
#define MCP2515_REG_TXB2EID8     0x53
#define MCP2515_REG_TXB2EID0     0x54
#define MCP2515_REG_TXB2DLC      0x55
#define MCP2515_REG_TXB2D0       0x56

/* RX filters (used in Step 4) */
#define MCP2515_REG_RXF0SIDH     0x00
#define MCP2515_REG_RXF0SIDL     0x01
#define MCP2515_REG_RXF1SIDH     0x04
#define MCP2515_REG_RXF1SIDL     0x05
#define MCP2515_REG_RXF2SIDH     0x08
#define MCP2515_REG_RXF2SIDL     0x09

#endif /* MCP2515_REGS_H */