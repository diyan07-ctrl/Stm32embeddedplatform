/**
 * @file    board_config.h
 * @brief   Board-level pin and peripheral assignment freeze.
 *
 * BFMC 2026 — odyssiX NUCLEO-F401RE
 * ALL pin mappings are frozen. Do NOT modify.
 */

#ifndef BOARD_CONFIG_H
#define BOARD_CONFIG_H

#ifdef __cplusplus
extern "C" {
#endif

/* ── SPI1 — MCP2515 CAN Controller ──────────────────────────────────────── */
#define MCP2515_SPI_INSTANCE        SPI1
#define MCP2515_CS_PORT             GPIOA
#define MCP2515_CS_PIN              GPIO_PIN_4
#define MCP2515_INT_PORT            GPIOB
#define MCP2515_INT_PIN             GPIO_PIN_0
#define MCP2515_INT_EXTI_IRQn       EXTI0_IRQn

/* ── I2C1 — BNO085 IMU ──────────────────────────────────────────────────── */
#define BNO085_I2C_INSTANCE         I2C1
#define BNO085_I2C_ADDR_7BIT        0x4Au
#define BNO085_I2C_ADDR_WRITE       (BNO085_I2C_ADDR_7BIT << 1u)
#define BNO085_INT_PORT             GPIOA
#define BNO085_INT_PIN              GPIO_PIN_1
#define BNO085_INT_EXTI_IRQn        EXTI1_IRQn

/* ── USART2 — URM37 Ultrasonic ──────────────────────────────────────────── */
#define URM37_UART_INSTANCE         USART2
#define URM37_BAUD                  9600u
#define URM37_TX_PORT               GPIOA
#define URM37_TX_PIN                GPIO_PIN_2
#define URM37_RX_PORT               GPIOA
#define URM37_RX_PIN                GPIO_PIN_3

/* ── TIM4 — PWM Output ──────────────────────────────────────────────────── */
#define PWM_TIM_INSTANCE            TIM4
#define PWM_ESC_CHANNEL             TIM_CHANNEL_1   /* PB6 */
#define PWM_SERVO_CHANNEL           TIM_CHANNEL_2   /* PB7 */

/* ── TIM5 — Timestamp Authority ──────────────────────────────────────────── */
#define TIMESTAMP_TIM_INSTANCE      TIM5

/* ── DMA Assignments ─────────────────────────────────────────────────────── */
/* DMA1_Stream0 — I2C1 RX (BNO085)       */
/* DMA1_Stream5 — USART2 RX (URM37)      */
/* DMA2_Stream0 — SPI1 RX (MCP2515)      */
/* DMA2_Stream3 — SPI1 TX (MCP2515)      */

/* ── NVIC Priority (all peripherals) ────────────────────────────────────── */
#define PERIPH_IRQ_PRIORITY         5u

#ifdef __cplusplus
}
#endif

#endif /* BOARD_CONFIG_H */
