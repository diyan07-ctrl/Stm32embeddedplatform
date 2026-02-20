/**
 * @file    watchdog.c
 * @brief   Independent watchdog (IWDG) — ENABLED for competition.
 *
 * IWDG is configured in CubeMX (.ioc) but MX_IWDG_Init() is intentionally
 * NOT called from main(). This module owns the IWDG lifecycle so the
 * countdown starts only AFTER all modules are initialized.
 *
 * IWDG configuration:
 *   LSI ≈ 32 kHz
 *   Prescaler = 128
 *   Reload = 249
 *   Timeout ≈ (128 * 250) / 32000 = ~1.0 second
 *
 * Refresh ONLY from ControlTask.
 * Never from ISR. Never from MicroRosTask.
 * If ControlTask stalls → MCU reset.
 */

#include "system/watchdog.h"
#include "stm32f4xx_hal.h"

static IWDG_HandleTypeDef hiwdg;

bool watchdog_init(void)
{
    hiwdg.Instance       = IWDG;
    hiwdg.Init.Prescaler = IWDG_PRESCALER_128;
    hiwdg.Init.Reload    = 249u;

    if (HAL_IWDG_Init(&hiwdg) != HAL_OK) {
        return false;
    }

    return true;
}

void watchdog_kick(void)
{
    HAL_IWDG_Refresh(&hiwdg);
}
