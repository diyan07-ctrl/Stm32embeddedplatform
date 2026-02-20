/**
 * @file    safety_manager.h
 * @brief   Centralized safety state manager.
 *
 * Emergency sources (bitmask):
 *   - Ultrasonic threshold breach
 *   - CAN 0x401 emergency stop
 *   - Control command timeout (100 ms)
 *   - Heartbeat timeout (500 ms)
 *   - CAN buffer overflow
 *
 * Emergency flag is LATCHED. Cleared only by explicit safe-state validation.
 * All timeout checks use TIM5 timestamps (wrap-safe unsigned subtraction).
 */

#ifndef SAFETY_MANAGER_H
#define SAFETY_MANAGER_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

/* ── Emergency source bitmask ─────────────────────────────────────────── */
#define SAFETY_SRC_ULTRASONIC       (1u << 0)
#define SAFETY_SRC_CAN_ESTOP        (1u << 1)
#define SAFETY_SRC_CTRL_TIMEOUT     (1u << 2)
#define SAFETY_SRC_HB_TIMEOUT       (1u << 3)
#define SAFETY_SRC_CAN_OVERFLOW     (1u << 4)

/**
 * @brief  Initialize safety manager. All sources clear.
 */
void safety_manager_init(void);

/**
 * @brief  Set one or more emergency sources (OR into latch).
 * @param  sources  Bitmask of SAFETY_SRC_* flags.
 *
 * Protected by FreeRTOS critical section.
 */
void safety_manager_set_emergency(uint8_t sources);

/**
 * @brief  Clear one or more emergency sources.
 * @param  sources  Bitmask of SAFETY_SRC_* flags to clear.
 *
 * Emergency is cleared ONLY when all sources are 0.
 * Protected by FreeRTOS critical section.
 */
void safety_manager_clear_source(uint8_t sources);

/**
 * @brief  Record that a valid control command was received.
 *
 * Resets the control command timeout timer.
 */
void safety_manager_touch_control(void);

/**
 * @brief  Record that a valid heartbeat-relevant CAN frame was received.
 *
 * Resets the heartbeat timeout timer (any RX from Jetson counts).
 */
void safety_manager_touch_heartbeat(void);

/**
 * @brief  Evaluate all timeout sources.
 *
 * Call periodically (e.g., from ControlTask at 500 Hz).
 * Sets SAFETY_SRC_CTRL_TIMEOUT if no command for 100 ms.
 * Sets SAFETY_SRC_HB_TIMEOUT if no Jetson CAN for 500 ms.
 */
void safety_manager_check_timeouts(void);

/**
 * @brief  Query overall emergency state.
 * @retval true   Emergency is active (one or more sources set).
 * @retval false  System is nominal.
 */
bool safety_manager_is_emergency(void);

/**
 * @brief  Get the active emergency source bitmask.
 * @retval Bitmask of SAFETY_SRC_* flags currently set.
 */
uint8_t safety_manager_get_sources(void);

#ifdef __cplusplus
}
#endif

#endif /* SAFETY_MANAGER_H */
