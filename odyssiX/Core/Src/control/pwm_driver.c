/**
 * @file    pwm_driver.c
 * @brief   TIM4 PWM output — direct CCR register writes.
 *
 * TIM4 is configured by Cube-generated MX_TIM4_Init().
 * This module starts PWM output and provides pulse-width setters.
 * 1 CCR count = 1 us (timer clock = 1 MHz with prescaler 83).
 */

#include "control/pwm_driver.h"
#include "control/esc_conditioner.h"
#include "control/servo_conditioner.h"
#include "stm32f4xx_hal.h"

extern TIM_HandleTypeDef htim4;

static inline uint16_t clamp_pulse(uint16_t pulse)
{
    if (pulse < PWM_MIN_US) return PWM_MIN_US;
    if (pulse > PWM_MAX_US) return PWM_MAX_US;
    return pulse;
}

void pwm_driver_start(void)
{
    /* Set initial safe values before starting */
    TIM4->CCR1 = ESC_NEUTRAL_US;
    TIM4->CCR2 = SERVO_NEUTRAL_US;

    /* Start PWM output on both channels */
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
}

void pwm_driver_set_esc(uint16_t pulse_us)
{
    TIM4->CCR1 = clamp_pulse(pulse_us);
}

void pwm_driver_set_servo(uint16_t pulse_us)
{
    TIM4->CCR2 = clamp_pulse(pulse_us);
}

void pwm_driver_safe_neutral(void)
{
    TIM4->CCR1 = ESC_NEUTRAL_US;
    TIM4->CCR2 = SERVO_NEUTRAL_US;
}
