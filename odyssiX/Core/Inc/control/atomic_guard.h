/**
 * @file    atomic_guard.h
 * @brief   Double-buffered atomic shared control structure.
 *
 * Provides tear-free transfer of control commands between:
 *   Producer: micro-ROS CAN dispatcher (MicroRosTask context)
 *   Consumer: 500 Hz control task
 *
 * Uses FreeRTOS critical sections for the index swap / copy.
 * volatile alone is NOT sufficient for multi-word coherence.
 *
 * Timestamps use TIM5 microseconds (NOT HAL_GetTick).
 */

#ifndef ATOMIC_GUARD_H
#define ATOMIC_GUARD_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

/** Control command payload */
typedef struct {
    float    speed;         /**< Normalized speed  [-1.0, +1.0]    */
    float    steering;      /**< Normalized steering [-1.0, +1.0]  */
    uint32_t timestamp_us;  /**< TIM5 timestamp at write time      */
    uint8_t  valid;         /**< Non-zero once first command arrives */
} control_cmd_t;

/**
 * @brief  Initialize the double-buffer guard.
 *
 * Sets both buffers to neutral (speed=0, steering=0, valid=0).
 * Must be called before any read/write.
 */
void atomic_guard_init(void);

/**
 * @brief  Write a new control command (producer side).
 *
 * Writes to the inactive buffer, then atomically swaps the active index
 * inside a FreeRTOS critical section.
 *
 * @param  speed     Normalized speed [-1.0, +1.0].
 * @param  steering  Normalized steering [-1.0, +1.0].
 *
 * @note   May be called from any task context. NOT safe from ISR.
 */
void atomic_guard_write(float speed, float steering);

/**
 * @brief  Read the latest control command (consumer side).
 *
 * Copies the active buffer contents inside a FreeRTOS critical section.
 *
 * @param  out  Output: snapshot of the latest command.
 * @retval true   A valid command was returned.
 * @retval false  No command has been written yet (out is zeroed).
 */
bool atomic_guard_read(control_cmd_t *out);

#ifdef __cplusplus
}
#endif

#endif /* ATOMIC_GUARD_H */
