/**
 * @file    bno085_driver.h
 * @brief   BNO085 IMU driver — I2C DMA, SHTP/SH-2 protocol.
 *
 * Hardware mapping (frozen):
 *   I2C1     — 400 kHz fast mode
 *   PB8      — SCL
 *   PB9      — SDA
 *   PA1      — INT (EXTI1, falling edge, data-ready)
 *   DMA1_Stream0 — I2C1 RX
 *
 * Pipeline:
 *   EXTI1 → notify IMUTask → I2C DMA read → DMA ISR notify →
 *   parse SH-2 rotation vector → timestamp → CAN 0x101
 *
 * I2C address: 0x4A (7-bit), 0x94 (8-bit write)
 */

#ifndef BNO085_DRIVER_H
#define BNO085_DRIVER_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

/* ── BNO085 I2C address ──────────────────────────────────────────────── */
#define BNO085_I2C_ADDR             0x4Au
#define BNO085_I2C_ADDR_8BIT        (BNO085_I2C_ADDR << 1u)

/* ── Quaternion output (Q14 fixed-point) ─────────────────────────────── */
typedef struct {
    int16_t qi;         /**< Quaternion i (Q14) */
    int16_t qj;         /**< Quaternion j (Q14) */
    int16_t qk;         /**< Quaternion k (Q14) */
    int16_t qr;         /**< Quaternion real (Q14) */
    uint8_t accuracy;   /**< Accuracy estimate (0-3) */
    uint8_t valid;      /**< 1 if data was parsed successfully */
} bno085_quaternion_t;

/* ── Error codes ─────────────────────────────────────────────────────── */
typedef enum {
    BNO085_OK = 0,
    BNO085_ERR_I2C,
    BNO085_ERR_DMA,
    BNO085_ERR_TIMEOUT,
    BNO085_ERR_BAD_HEADER,
    BNO085_ERR_NO_REPORT,
    BNO085_ERR_CONFIG,
} bno085_err_t;

/**
 * @brief  Initialize BNO085 hardware and configure rotation vector @ 100 Hz.
 *
 * Configures PA1 as EXTI1 falling edge input.
 * Sends SHTP Set Feature Command for rotation vector (10 ms interval).
 * Must be called from task context (uses vTaskDelay).
 *
 * @retval BNO085_OK on success.
 */
bno085_err_t bno085_init(void);

/**
 * @brief  Read one SHTP packet from BNO085 via I2C DMA.
 *
 * Reads 4-byte header, then remaining payload.
 * Blocks calling task until DMA completes (via semaphore).
 *
 * @retval BNO085_OK on success.
 */
bno085_err_t bno085_read_packet(void);

/**
 * @brief  Parse the last received SHTP packet for rotation vector data.
 *
 * @param  q  Output quaternion. Valid only if return is BNO085_OK.
 * @retval BNO085_OK if rotation vector found and parsed.
 * @retval BNO085_ERR_NO_REPORT if packet does not contain rotation vector.
 */
bno085_err_t bno085_parse_rotation_vector(bno085_quaternion_t *q);

/**
 * @brief  Called from EXTI1 ISR when BNO085 INT fires.
 *
 * Notifies the IMU task handle via task notification.
 * Must be called from ISR context.
 */
void bno085_notify_int_isr(void);

/**
 * @brief  Called from HAL_I2C_MasterRxCpltCallback.
 *
 * Signals the internal semaphore to unblock the reading task.
 */
void bno085_i2c_rx_complete(void);

/**
 * @brief  Called from HAL_I2C_ErrorCallback.
 *
 * Signals error to the reading task.
 */
void bno085_i2c_error(void);

/**
 * @brief  Set the IMU task handle for EXTI notifications.
 * @param  handle  FreeRTOS task handle of IMUTask.
 */
void bno085_set_task_handle(void *handle);

#ifdef __cplusplus
}
#endif

#endif /* BNO085_DRIVER_H */
