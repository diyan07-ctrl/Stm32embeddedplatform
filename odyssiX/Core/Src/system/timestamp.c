/**
 * @file    timestamp.c
 * @brief   TIM5 microsecond timestamp — 32-bit free-running at 1 MHz.
 *
 * TIM5 is configured by CubeMX (MX_TIM5_Init) with PSC=83, ARR=0xFFFFFFFF.
 * MX_TIM5_Init starts the timer via HAL_TIM_Base_Start.
 *
 * timestamp_init() is kept for safety: ensures TIM5 is running even if
 * CubeMX configuration is accidentally changed.
 *
 * TIM5 is a 32-bit timer on STM32F401RE (APB1, 84 MHz after prescaler doubling).
 */

#include "system/timestamp.h"
#include "stm32f4xx_hal.h"

void timestamp_init(void)
{
    /* TIM5 clock and base config already done by MX_TIM5_Init().
     * Reinforce with direct register writes for safety. */

    __HAL_RCC_TIM5_CLK_ENABLE();

    /* Verify configuration — write only if needed */
    if (TIM5->PSC != 83u || TIM5->ARR != 0xFFFFFFFFu) {
        TIM5->CR1  = 0u;
        TIM5->PSC  = 83u;
        TIM5->ARR  = 0xFFFFFFFFu;
        TIM5->CNT  = 0u;
        TIM5->DIER = 0u;
        TIM5->EGR  = TIM_EGR_UG;
        TIM5->SR   = 0u;
    }

    /* Ensure timer is running */
    TIM5->CR1 |= TIM_CR1_CEN;
}

uint32_t timestamp_now_us(void)
{
    return TIM5->CNT;
}
