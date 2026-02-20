/**
 * @file    transport.h
 * @brief   micro-ROS custom transport over MCP2515 CAN bus.
 *
 * Implements the four transport callbacks required by Micro XRCE-DDS:
 *   open, close, write, read
 *
 * CAN framing:
 *   TX CAN ID (STM32 → Agent): configurable, default 0x200
 *   RX CAN ID (Agent → STM32): configurable, default 0x100
 *   Each CAN frame carries up to 8 bytes of raw XRCE-DDS stream.
 *   Fragmentation/reassembly is handled transparently.
 *
 * Requires: MICRO_ROS_ENABLED defined to compile.
 */

#ifndef ROS_TRANSPORT_H
#define ROS_TRANSPORT_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

/* ── CAN IDs for micro-ROS transport ──────────────────────────────────────── */
#ifndef UROS_CAN_TX_ID
#define UROS_CAN_TX_ID      0x200u  /**< STM32 → Agent */
#endif

#ifndef UROS_CAN_RX_ID
#define UROS_CAN_RX_ID      0x100u  /**< Agent → STM32 */
#endif

#ifdef MICRO_ROS_ENABLED

#include <uxr/client/transport.h>

/**
 * @brief  Open the CAN transport.
 *
 * MCP2515 is already initialized by app_tasks_init().
 * This just verifies readiness.
 */
bool transport_open(struct uxrCustomTransport *transport);

/**
 * @brief  Close the CAN transport.
 */
bool transport_close(struct uxrCustomTransport *transport);

/**
 * @brief  Write bytes to the CAN bus.
 *
 * Fragments the buffer into 8-byte CAN frames with UROS_CAN_TX_ID
 * and transmits via MCP2515.
 *
 * @param  transport  XRCE-DDS transport handle.
 * @param  buf        Data to send.
 * @param  len        Number of bytes.
 * @param  err        Output error code (0 = OK).
 * @retval Number of bytes sent.
 */
size_t transport_write(struct uxrCustomTransport *transport,
                       const uint8_t *buf, size_t len, uint8_t *err);

/**
 * @brief  Read bytes from the CAN bus.
 *
 * Reads CAN frames from the RX ring buffer, filters by UROS_CAN_RX_ID,
 * and copies payload bytes into buf.
 *
 * @param  transport  XRCE-DDS transport handle.
 * @param  buf        Output buffer.
 * @param  len        Maximum bytes to read.
 * @param  timeout    Timeout in milliseconds.
 * @param  err        Output error code (0 = OK).
 * @retval Number of bytes read.
 */
size_t transport_read(struct uxrCustomTransport *transport,
                      uint8_t *buf, size_t len, int timeout, uint8_t *err);

/**
 * @brief  Register the custom transport with micro-ROS.
 *
 * Calls rmw_uros_set_custom_transport() with the above callbacks.
 * Must be called before rcl_init().
 */
void transport_setup(void);

#else /* MICRO_ROS_ENABLED not defined */

/**
 * @brief  Stub: does nothing when micro-ROS is disabled.
 */
static inline void transport_setup(void) { (void)0; }

#endif /* MICRO_ROS_ENABLED */

#ifdef __cplusplus
}
#endif

#endif /* ROS_TRANSPORT_H */
