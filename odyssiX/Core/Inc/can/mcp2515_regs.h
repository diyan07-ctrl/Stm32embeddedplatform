/**
 * @file    mcp2515_regs.h
 * @brief   MCP2515 CAN Controller register and command definitions.
 *
 * Reference: Microchip MCP2515 datasheet DS20001801K
 * All addresses, bit masks, and SPI commands required for full driver operation.
 */

#ifndef MCP2515_REGS_H
#define MCP2515_REGS_H

#ifdef __cplusplus
extern "C" {
#endif

/* ── SPI Instructions ─────────────────────────────────────────────────────── */
#define MCP2515_CMD_RESET           0xC0u
#define MCP2515_CMD_READ            0x03u
#define MCP2515_CMD_WRITE           0x02u
#define MCP2515_CMD_READ_STATUS     0xA0u
#define MCP2515_CMD_RX_STATUS       0xB0u
#define MCP2515_CMD_BIT_MODIFY      0x05u

/* Read RX Buffer: instruction[1:0] selects buffer & start point */
#define MCP2515_CMD_READ_RXB0_SIDH  0x90u   /* RXB0 starting at SIDH    */
#define MCP2515_CMD_READ_RXB0_D0    0x92u   /* RXB0 starting at data[0] */
#define MCP2515_CMD_READ_RXB1_SIDH  0x94u   /* RXB1 starting at SIDH    */
#define MCP2515_CMD_READ_RXB1_D0    0x96u   /* RXB1 starting at data[0] */

/* Load TX Buffer: instruction[2:0] selects buffer & start point */
#define MCP2515_CMD_LOAD_TXB0_SIDH  0x40u
#define MCP2515_CMD_LOAD_TXB0_D0    0x41u
#define MCP2515_CMD_LOAD_TXB1_SIDH  0x42u
#define MCP2515_CMD_LOAD_TXB1_D0    0x43u
#define MCP2515_CMD_LOAD_TXB2_SIDH  0x44u
#define MCP2515_CMD_LOAD_TXB2_D0    0x45u

/* Request to Send */
#define MCP2515_CMD_RTS_TX0         0x81u
#define MCP2515_CMD_RTS_TX1         0x82u
#define MCP2515_CMD_RTS_TX2         0x84u
#define MCP2515_CMD_RTS_ALL         0x87u

/* ── Configuration Registers ──────────────────────────────────────────────── */
#define MCP2515_REG_CANSTAT         0x0Eu
#define MCP2515_REG_CANCTRL         0x0Fu

#define MCP2515_REG_CNF3            0x28u
#define MCP2515_REG_CNF2            0x29u
#define MCP2515_REG_CNF1            0x2Au

/* ── CANCTRL bits ─────────────────────────────────────────────────────────── */
#define MCP2515_CANCTRL_REQOP_MASK  0xE0u
#define MCP2515_CANCTRL_REQOP_NORMAL    0x00u
#define MCP2515_CANCTRL_REQOP_SLEEP     0x20u
#define MCP2515_CANCTRL_REQOP_LOOPBACK  0x40u
#define MCP2515_CANCTRL_REQOP_LISTENONLY 0x60u
#define MCP2515_CANCTRL_REQOP_CONFIG    0x80u
#define MCP2515_CANCTRL_CLKEN       0x04u
#define MCP2515_CANCTRL_CLKPRE_MASK 0x03u

/* ── CANSTAT bits ─────────────────────────────────────────────────────────── */
#define MCP2515_CANSTAT_OPMOD_MASK  0xE0u

/* ── Interrupt Registers ──────────────────────────────────────────────────── */
#define MCP2515_REG_CANINTE         0x2Bu
#define MCP2515_REG_CANINTF         0x2Cu

/* CANINTE / CANINTF bits */
#define MCP2515_INT_RX0             0x01u
#define MCP2515_INT_RX1             0x02u
#define MCP2515_INT_TX0             0x04u
#define MCP2515_INT_TX1             0x08u
#define MCP2515_INT_TX2             0x10u
#define MCP2515_INT_ERR             0x20u
#define MCP2515_INT_WAK             0x40u
#define MCP2515_INT_MERR            0x80u

/* ── Error Registers ──────────────────────────────────────────────────────── */
#define MCP2515_REG_EFLG            0x2Du
#define MCP2515_REG_TEC             0x1Cu
#define MCP2515_REG_REC             0x1Du

/* ── TX Buffer Registers ──────────────────────────────────────────────────── */
#define MCP2515_REG_TXB0CTRL        0x30u
#define MCP2515_REG_TXB0SIDH        0x31u
#define MCP2515_REG_TXB0SIDL        0x32u
#define MCP2515_REG_TXB0EID8        0x33u
#define MCP2515_REG_TXB0EID0        0x34u
#define MCP2515_REG_TXB0DLC         0x35u
#define MCP2515_REG_TXB0D0          0x36u

#define MCP2515_REG_TXB1CTRL        0x40u
#define MCP2515_REG_TXB1SIDH        0x41u

#define MCP2515_REG_TXB2CTRL        0x50u
#define MCP2515_REG_TXB2SIDH        0x51u

/* TXBnCTRL bits */
#define MCP2515_TXBCTRL_TXREQ       0x08u
#define MCP2515_TXBCTRL_ABTF        0x40u
#define MCP2515_TXBCTRL_MLOA        0x20u
#define MCP2515_TXBCTRL_TXERR       0x10u

/* ── RX Buffer Registers ──────────────────────────────────────────────────── */
#define MCP2515_REG_RXB0CTRL        0x60u
#define MCP2515_REG_RXB0SIDH        0x61u
#define MCP2515_REG_RXB0SIDL        0x62u
#define MCP2515_REG_RXB0EID8        0x63u
#define MCP2515_REG_RXB0EID0        0x64u
#define MCP2515_REG_RXB0DLC         0x65u
#define MCP2515_REG_RXB0D0          0x66u

#define MCP2515_REG_RXB1CTRL        0x70u
#define MCP2515_REG_RXB1SIDH        0x71u
#define MCP2515_REG_RXB1SIDL        0x72u
#define MCP2515_REG_RXB1EID8        0x73u
#define MCP2515_REG_RXB1EID0        0x74u
#define MCP2515_REG_RXB1DLC         0x75u
#define MCP2515_REG_RXB1D0          0x76u

/* RXBnCTRL bits */
#define MCP2515_RXBCTRL_RXM_MASK    0x60u
#define MCP2515_RXBCTRL_RXM_ANY     0x60u   /* Accept all messages       */
#define MCP2515_RXBCTRL_RXM_STD     0x00u   /* Match filters only        */
#define MCP2515_RXBCTRL_BUKT        0x04u   /* Rollover to RXB1          */

/* ── Filter and Mask Registers ────────────────────────────────────────────── */
#define MCP2515_REG_RXF0SIDH        0x00u
#define MCP2515_REG_RXF0SIDL        0x01u
#define MCP2515_REG_RXF1SIDH        0x04u
#define MCP2515_REG_RXF1SIDL        0x05u
#define MCP2515_REG_RXF2SIDH        0x08u
#define MCP2515_REG_RXF2SIDL        0x09u
#define MCP2515_REG_RXF3SIDH        0x10u
#define MCP2515_REG_RXF3SIDL        0x11u
#define MCP2515_REG_RXF4SIDH        0x14u
#define MCP2515_REG_RXF4SIDL        0x15u
#define MCP2515_REG_RXF5SIDH        0x18u
#define MCP2515_REG_RXF5SIDL        0x19u

#define MCP2515_REG_RXM0SIDH        0x20u
#define MCP2515_REG_RXM0SIDL        0x21u
#define MCP2515_REG_RXM1SIDH        0x24u
#define MCP2515_REG_RXM1SIDL        0x25u

/* ── READ STATUS response bits ────────────────────────────────────────────── */
#define MCP2515_STAT_RX0IF          0x01u
#define MCP2515_STAT_RX1IF          0x02u
#define MCP2515_STAT_TX0REQ         0x04u
#define MCP2515_STAT_TX0IF          0x08u
#define MCP2515_STAT_TX1REQ         0x10u
#define MCP2515_STAT_TX1IF          0x20u
#define MCP2515_STAT_TX2REQ         0x40u
#define MCP2515_STAT_TX2IF          0x80u

#ifdef __cplusplus
}
#endif

#endif /* MCP2515_REGS_H */
