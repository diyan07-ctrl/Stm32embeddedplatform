/**
 * @file    watchdog.h
 * @brief   Independent watchdog (IWDG) stub.
 *
 * Provides the interface for IWDG initialization and refresh.
 * Currently a stub — enable by uncommenting the body in watchdog.c
 * and enabling HAL_IWDG_MODULE_ENABLED in stm32f4xx_hal_conf.h.
 */

#ifndef WATCHDOG_H
#define WATCHDOG_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>

/**
 * @brief  Initialize the IWDG with a ~1 second timeout.
 * @retval true   IWDG started (or stub, always true).
 */
bool watchdog_init(void);

/**
 * @brief  Refresh (kick) the watchdog.
 *
 * Must be called periodically from the main control loop
 * to prevent system reset.
 */
void watchdog_kick(void);

#ifdef __cplusplus
}
#endif

#endif /* WATCHDOG_H */
