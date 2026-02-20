/**
 * @file    pwm_driver.h
 * @brief   TIM4 PWM output driver — direct CCR register writes.
 *
 * TIM4 configuration (frozen .ioc):
 *   Prescaler: 83  →  1 MHz timer clock
 *   Period:    19999  →  50 Hz PWM
 *   CH1 (PB6): BLDC ESC  →  TIM4->CCR1
 *   CH2 (PB7): Servo     →  TIM4->CCR2
 *   Both channels: PWM1 mode, polarity HIGH
 *
 * 1 CCR count = 1 us.
 * Valid pulse range: 1000–2000 us (clamped internally).
 */

#ifndef PWM_DRIVER_H
#define PWM_DRIVER_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#define PWM_MIN_US  1000u
#define PWM_MAX_US  2000u

/**
 * @brief  Start TIM4 PWM on CH1 and CH2.
 *
 * Must be called once after MX_TIM4_Init() to enable PWM output.
 * Sets initial values to ESC neutral (1485) and servo neutral (1500).
 */
void pwm_driver_start(void);

/**
 * @brief  Set BLDC ESC pulse width.
 *
 * Writes directly to TIM4->CCR1. Clamped to [1000, 2000].
 *
 * @param  pulse_us  Pulse width in microseconds.
 */
void pwm_driver_set_esc(uint16_t pulse_us);

/**
 * @brief  Set servo pulse width.
 *
 * Writes directly to TIM4->CCR2. Clamped to [1000, 2000].
 *
 * @param  pulse_us  Pulse width in microseconds.
 */
void pwm_driver_set_servo(uint16_t pulse_us);

/**
 * @brief  Set both outputs to safe neutral values.
 *
 * ESC → 1485 us, Servo → 1500 us.
 */
void pwm_driver_safe_neutral(void);

#ifdef __cplusplus
}
#endif

#endif /* PWM_DRIVER_H */
