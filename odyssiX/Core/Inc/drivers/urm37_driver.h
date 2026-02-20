/**
 * @file    urm37_driver.h
 * @brief   URM37 V5.0 ultrasonic driver — UART DMA RX.
 *
 * Hardware mapping (frozen):
 *   USART2    — 9600 baud, 8N1
 *   PA2       — TX (trigger command)
 *   PA3       — RX (distance response)
 *   DMA1_Stream5 — USART2 RX
 *
 * Protocol (serial triggered mode):
 *   TX: 0x22 0x00 0x00 0x22  (trigger measurement)
 *   RX: 0x22 HH LL SUM       (distance = HH*256+LL cm, SUM = 0x22+HH+LL)
 *   If no obstacle: HH=0xFF, LL=0xFF
 *
 * Pipeline:
 *   UltrasonicTask sends trigger → UART DMA RX 4 bytes →
 *   DMA ISR notify → parse → validate → timestamp → safety check → CAN 0x102
 */

#ifndef URM37_DRIVER_H
#define URM37_DRIVER_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

/* ── Measurement result ──────────────────────────────────────────────── */
typedef struct {
    uint16_t distance_cm;   /**< Distance in centimeters (0 if invalid)  */
    uint8_t  valid;         /**< 1 = valid measurement, 0 = error/OOR   */
} urm37_measurement_t;

/* ── Error codes ─────────────────────────────────────────────────────── */
typedef enum {
    URM37_OK = 0,
    URM37_ERR_UART,
    URM37_ERR_DMA,
    URM37_ERR_TIMEOUT,
    URM37_ERR_CHECKSUM,
    URM37_ERR_NO_ECHO,
} urm37_err_t;

/**
 * @brief  Initialize URM37 driver.
 *
 * DMA1_Stream5 for USART2 RX is configured by CubeMX (HAL_UART_MspInit).
 * Creates internal synchronization semaphore.
 * Must be called from task context.
 */
urm37_err_t urm37_init(void);

/**
 * @brief  Trigger a single distance measurement.
 *
 * Sends the 4-byte trigger command via UART TX (polling).
 * Starts DMA RX for 4-byte response.
 * Blocks until DMA completes or timeout.
 *
 * @param  result  Output measurement.
 * @retval URM37_OK on successful measurement.
 */
urm37_err_t urm37_measure(urm37_measurement_t *result);

/**
 * @brief  Called from HAL_UART_RxCpltCallback for USART2.
 *
 * Signals the internal semaphore to unblock urm37_measure().
 */
void urm37_uart_rx_complete(void);

/**
 * @brief  Called from HAL_UART_ErrorCallback for USART2.
 *
 * Signals error to the driver.
 */
void urm37_uart_error(void);

#ifdef __cplusplus
}
#endif

#endif /* URM37_DRIVER_H */
