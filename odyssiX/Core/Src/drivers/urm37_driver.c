/**
 * @file    urm37_driver.c
 * @brief   URM37 V5.0 ultrasonic driver — UART DMA RX.
 *
 * DMA1_Stream5 is configured by CubeMX (HAL_UART_MspInit).
 * NVIC for DMA1_Stream5 is configured in MX_DMA_Init.
 *
 * Trigger via polling TX (4 bytes, infrequent).
 * Response via DMA RX (4 bytes per measurement).
 */

#include "drivers/urm37_driver.h"
#include "config/board_config.h"
#include "main.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include <string.h>

/* ── External HAL handles ────────────────────────────────────────────── */
extern UART_HandleTypeDef huart2;

/* ── UART RX DMA synchronization ────────────────────────────────────── */
static SemaphoreHandle_t uart_dma_sem;
static StaticSemaphore_t uart_dma_sem_buf;

/* ── Error flag ──────────────────────────────────────────────────────── */
static volatile uint8_t  uart_error_flag;

/* ── DMA-safe buffers (static, NOT on stack) ─────────────────────────── */
static uint8_t           rx_buf[4];

/* ── URM37 trigger command ───────────────────────────────────────────── */
static const uint8_t     trigger_cmd[4] = { 0x22u, 0x00u, 0x00u, 0x22u };

/* ── Timeout ─────────────────────────────────────────────────────────── */
#define URM37_DMA_TIMEOUT_MS        100u
#define URM37_TX_TIMEOUT_MS         20u

/* ── Public API ──────────────────────────────────────────────────────── */

urm37_err_t urm37_init(void)
{
    uart_dma_sem = xSemaphoreCreateBinaryStatic(&uart_dma_sem_buf);
    uart_error_flag = 0u;

    return URM37_OK;
}

urm37_err_t urm37_measure(urm37_measurement_t *result)
{
    result->distance_cm = 0u;
    result->valid       = 0u;
    uart_error_flag     = 0u;

    /* Start DMA RX for 4-byte response BEFORE sending trigger
     * (to avoid missing fast responses) */
    HAL_StatusTypeDef status = HAL_UART_Receive_DMA(&huart2, rx_buf, 4u);
    if (status != HAL_OK) {
        return URM37_ERR_DMA;
    }

    /* Send trigger command via polling TX */
    status = HAL_UART_Transmit(&huart2, (uint8_t *)trigger_cmd, 4u,
                                URM37_TX_TIMEOUT_MS);
    if (status != HAL_OK) {
        HAL_UART_DMAStop(&huart2);
        return URM37_ERR_UART;
    }

    /* Wait for DMA RX completion */
    if (xSemaphoreTake(uart_dma_sem, pdMS_TO_TICKS(URM37_DMA_TIMEOUT_MS)) != pdTRUE) {
        HAL_UART_DMAStop(&huart2);
        return URM37_ERR_TIMEOUT;
    }

    if (uart_error_flag) {
        uart_error_flag = 0u;
        return URM37_ERR_UART;
    }

    /* Validate response frame */
    if (rx_buf[0] != 0x22u) {
        return URM37_ERR_CHECKSUM;
    }

    /* Checksum: (header + HH + LL) & 0xFF */
    uint8_t expected_sum = (uint8_t)(rx_buf[0] + rx_buf[1] + rx_buf[2]);
    if (rx_buf[3] != expected_sum) {
        return URM37_ERR_CHECKSUM;
    }

    /* Check for no-echo (0xFFFF) */
    if (rx_buf[1] == 0xFFu && rx_buf[2] == 0xFFu) {
        return URM37_ERR_NO_ECHO;
    }

    /* Parse distance */
    uint16_t dist = ((uint16_t)rx_buf[1] << 8u) | (uint16_t)rx_buf[2];

    /* Validate range: 5–500 cm */
    if (dist < 5u || dist > 500u) {
        result->distance_cm = dist;
        result->valid       = 0u;
        return URM37_OK; /* Measurement succeeded but out of valid range */
    }

    result->distance_cm = dist;
    result->valid       = 1u;
    return URM37_OK;
}

void urm37_uart_rx_complete(void)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xSemaphoreGiveFromISR(uart_dma_sem, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void urm37_uart_error(void)
{
    uart_error_flag = 1u;

    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xSemaphoreGiveFromISR(uart_dma_sem, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}
