/**
 * @file    safety_manager.c
 * @brief   Centralized safety state manager — latched emergency flag.
 *
 * All timing uses TIM5 via timestamp_now_us().
 * Concurrency: FreeRTOS critical sections for shared state.
 */

#include "control/safety_manager.h"
#include "system/timestamp.h"
#include "can/can_protocol.h"
#include "FreeRTOS.h"
#include "task.h"

/* ── Internal state ───────────────────────────────────────────────────── */
static uint8_t  emergency_sources;
static uint32_t last_control_us;
static uint32_t last_heartbeat_us;
static uint8_t  control_received;   /* Set once first command arrives */
static uint8_t  heartbeat_received; /* Set once first Jetson frame arrives */

void safety_manager_init(void)
{
    emergency_sources   = 0u;
    last_control_us     = timestamp_now_us();
    last_heartbeat_us   = timestamp_now_us();
    control_received    = 0u;
    heartbeat_received  = 0u;
}

void safety_manager_set_emergency(uint8_t sources)
{
    taskENTER_CRITICAL();
    emergency_sources |= sources;
    taskEXIT_CRITICAL();
}

void safety_manager_clear_source(uint8_t sources)
{
    taskENTER_CRITICAL();
    emergency_sources &= ~sources;
    taskEXIT_CRITICAL();
}

void safety_manager_touch_control(void)
{
    taskENTER_CRITICAL();
    last_control_us  = timestamp_now_us();
    control_received = 1u;
    /* Clear control timeout if it was set */
    emergency_sources &= ~((uint8_t)SAFETY_SRC_CTRL_TIMEOUT);
    taskEXIT_CRITICAL();
}

void safety_manager_touch_heartbeat(void)
{
    taskENTER_CRITICAL();
    last_heartbeat_us  = timestamp_now_us();
    heartbeat_received = 1u;
    /* Clear heartbeat timeout if it was set */
    emergency_sources &= ~((uint8_t)SAFETY_SRC_HB_TIMEOUT);
    taskEXIT_CRITICAL();
}

void safety_manager_check_timeouts(void)
{
    uint32_t now = timestamp_now_us();

    taskENTER_CRITICAL();

    /* Control command timeout: only after first command received */
    if (control_received) {
        if ((now - last_control_us) >= CONTROL_TIMEOUT_US) {
            emergency_sources |= SAFETY_SRC_CTRL_TIMEOUT;
        }
    }

    /* Heartbeat timeout: only after first Jetson frame received */
    if (heartbeat_received) {
        if ((now - last_heartbeat_us) >= HEARTBEAT_TIMEOUT_US) {
            emergency_sources |= SAFETY_SRC_HB_TIMEOUT;
        }
    }

    taskEXIT_CRITICAL();
}

bool safety_manager_is_emergency(void)
{
    uint8_t src;
    taskENTER_CRITICAL();
    src = emergency_sources;
    taskEXIT_CRITICAL();
    return src != 0u;
}

uint8_t safety_manager_get_sources(void)
{
    uint8_t src;
    taskENTER_CRITICAL();
    src = emergency_sources;
    taskEXIT_CRITICAL();
    return src;
}
