/**
 * @file    atomic_guard.c
 * @brief   Double-buffered atomic shared control structure.
 *
 * Guarantees tear-free reads/writes between producer (CAN dispatcher)
 * and consumer (500 Hz control task) using FreeRTOS critical sections.
 *
 * Timestamp uses TIM5 (NOT HAL_GetTick).
 */

#include "control/atomic_guard.h"
#include "system/timestamp.h"
#include "FreeRTOS.h"
#include "task.h"
#include <string.h>

/** Double buffer: producer writes to buf[1 - active], consumer reads buf[active] */
static control_cmd_t guard_buf[2];

/** Index of the buffer the consumer reads from */
static volatile uint8_t active_idx;

void atomic_guard_init(void)
{
    memset(guard_buf, 0, sizeof(guard_buf));
    active_idx = 0u;
}

void atomic_guard_write(float speed, float steering)
{
    /* Write to the INACTIVE buffer (no contention with consumer) */
    uint8_t write_idx = 1u - active_idx;

    guard_buf[write_idx].speed        = speed;
    guard_buf[write_idx].steering     = steering;
    guard_buf[write_idx].timestamp_us = timestamp_now_us();
    guard_buf[write_idx].valid        = 1u;

    /* Atomically swap the active index */
    taskENTER_CRITICAL();
    active_idx = write_idx;
    taskEXIT_CRITICAL();
}

bool atomic_guard_read(control_cmd_t *out)
{
    taskENTER_CRITICAL();
    *out = guard_buf[active_idx];
    taskEXIT_CRITICAL();

    return (out->valid != 0u);
}
