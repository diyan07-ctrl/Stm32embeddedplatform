/**
 * @file    mcp2515.h
 * @brief   MCP2515 CAN controller driver over SPI with DMA.
 *
 * Hardware mapping (frozen .ioc):
 *   SPI1  — Master, Mode 0, prescaler overridden to 16 → 5.25 MHz
 *   PA4   — CS (active low, push-pull, init HIGH)
 *   PB0   — INT (EXTI0, falling edge)
 *   DMA2_Stream0 — SPI1 RX
 *   DMA2_Stream3 — SPI1 TX
 *
 * Oscillator configuration:
 *   Default: 8 MHz crystal on MCP2515 module.
 *   Change MCP2515_OSC_HZ if your module uses 16 MHz.
 *
 * Usage:
 *   1. Call mcp2515_init() after MX_SPI1_Init() and osKernelInitialize().
 *   2. Start the CAN RX task via app_tasks_init().
 *   3. Received frames accumulate in the shared ring buffer.
 *   4. Use mcp2515_send() to transmit frames.
 */

#ifndef MCP2515_H
#define MCP2515_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include "can/can_buffer.h"

/* ── Oscillator frequency on the MCP2515 module ───────────────────────────── */
#ifndef MCP2515_OSC_HZ
#define MCP2515_OSC_HZ      8000000UL   /* 8 MHz — change to 16000000 if needed */
#endif

/* ── CAN Bus speed ────────────────────────────────────────────────────────── */
#define MCP2515_CAN_SPEED_500KBPS   500000UL

/* ── Error codes ──────────────────────────────────────────────────────────── */
typedef enum {
    MCP2515_OK = 0,
    MCP2515_ERR_SPI,            /**< SPI transfer failed or timed out     */
    MCP2515_ERR_CONFIG,         /**< Failed to enter configuration mode   */
    MCP2515_ERR_VERIFY,         /**< Register read-back mismatch          */
    MCP2515_ERR_TX_BUSY,        /**< All TX buffers occupied              */
    MCP2515_ERR_TIMEOUT,        /**< Operation timed out                  */
} mcp2515_err_t;

/**
 * @brief  Initialize the MCP2515.
 *
 * Performs hardware reset, configures bit timing for 500 kbps,
 * sets acceptance filters to receive all standard frames,
 * enables RX interrupts, and transitions to normal mode.
 *
 * @retval MCP2515_OK      Initialization succeeded.
 * @retval MCP2515_ERR_*   Specific error code.
 *
 * @pre    MX_SPI1_Init(), MX_DMA_Init(), MX_GPIO_Init() completed.
 * @pre    FreeRTOS kernel initialized (for semaphore creation).
 */
mcp2515_err_t mcp2515_init(void);

/**
 * @brief  Transmit a CAN frame via the MCP2515.
 *
 * Loads the first available TX buffer and requests transmission.
 * Blocks the calling task until the SPI DMA transfer completes.
 *
 * @param  frame  Frame to transmit.
 * @retval MCP2515_OK         Transmission started.
 * @retval MCP2515_ERR_TX_BUSY All TX buffers occupied.
 * @retval MCP2515_ERR_SPI    SPI error.
 */
mcp2515_err_t mcp2515_send(const can_frame_t *frame);

/**
 * @brief  Read all pending RX frames from MCP2515 into the ring buffer.
 *
 * Called by the CAN RX task when notified by EXTI0.
 * Reads both RXB0 and RXB1 if populated, pushes to shared buffer.
 *
 * @param  buf  Destination ring buffer.
 * @retval Number of frames read (0, 1, or 2).
 */
uint32_t mcp2515_read_all_rx(can_buffer_t *buf);

/**
 * @brief  Read the CANINTF register (interrupt flags).
 */
uint8_t mcp2515_read_interrupts(void);

/**
 * @brief  Read the EFLG register (error flags).
 */
uint8_t mcp2515_read_errors(void);

/**
 * @brief  Get the shared CAN RX ring buffer.
 *
 * All received CAN frames are pushed here by the CAN RX task.
 * The micro-ROS transport reads from this buffer.
 */
can_buffer_t *mcp2515_get_rx_buffer(void);

/**
 * @brief  Notify the CAN RX task that an interrupt occurred.
 *
 * Call this from HAL_GPIO_EXTI_Callback when GPIO_PIN_0 fires.
 * Safe to call from ISR context.
 */
void mcp2515_notify_isr(void);

/**
 * @brief  CAN RX task entry point.
 *
 * Waits for EXTI0 notifications, reads MCP2515 via DMA SPI,
 * and pushes received frames into the ring buffer.
 * Created by app_tasks_init() with high priority.
 *
 * @param  argument  Unused (FreeRTOS convention).
 */
void mcp2515_rx_task(void *argument);

/**
 * @brief  Get the RX ring buffer overflow count.
 *
 * Non-zero means frames were lost. May trigger safety.
 * @retval Number of frames lost to overflow since init.
 */
uint32_t mcp2515_get_overflow_count(void);

/**
 * @brief  SPI DMA transfer complete callback.
 *
 * Must be called from HAL_SPI_TxRxCpltCallback for hspi1.
 * Signals the internal semaphore to unblock the waiting task.
 */
void mcp2515_spi_dma_complete(void);

#ifdef __cplusplus
}
#endif

#endif /* MCP2515_H */
