/**
 * @file    timestamp.h
 * @brief   TIM5 microsecond timestamp authority.
 *
 * TIM5 is a 32-bit free-running counter at 1 MHz.
 * Never reset. Used for ALL timing in the system.
 * HAL_GetTick() SHALL NOT be used for safety or sensor timing.
 *
 * Wrap-safe: unsigned subtraction handles 32-bit overflow.
 * Full wrap period: ~4295 seconds (~71.6 minutes).
 */

#ifndef TIMESTAMP_H
#define TIMESTAMP_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/**
 * @brief  Initialize TIM5 as 32-bit free-running 1 MHz counter.
 *
 * Must be called once before any timestamp usage.
 * Clock: APB1 timer clock = 84 MHz, prescaler = 83 → 1 MHz.
 */
void timestamp_init(void);

/**
 * @brief  Read current TIM5 counter value.
 * @retval Microseconds since TIM5 start (wraps at 2^32).
 */
uint32_t timestamp_now_us(void);

/**
 * @brief  Compute elapsed time since a previous timestamp.
 *
 * Uses unsigned subtraction — wrap-safe as long as elapsed < 2^31 µs (~35 min).
 *
 * @param  since  Previous timestamp from timestamp_now_us().
 * @retval Elapsed microseconds.
 */
static inline uint32_t timestamp_elapsed_us(uint32_t since)
{
    return timestamp_now_us() - since;
}

/**
 * @brief  Check if a timeout has expired.
 *
 * @param  start_us     Timestamp when the period started.
 * @param  timeout_us   Timeout duration in microseconds.
 * @retval 1 if timed out, 0 if not.
 */
static inline uint8_t timestamp_expired(uint32_t start_us, uint32_t timeout_us)
{
    return (timestamp_now_us() - start_us) >= timeout_us ? 1u : 0u;
}

#ifdef __cplusplus
}
#endif

#endif /* TIMESTAMP_H */
