/**
 * @file    esc_conditioner.h
 * @brief   BLDC ESC signal conditioner for QUICRUN FUSION SE.
 *
 * Deadband specification (FROZEN):
 *   Neutral:       1485 us
 *   Forward start: 1443 us
 *   Reverse start: 1527 us
 *   Full range:    1000–2000 us
 *
 * Features:
 *   - Asymmetric deadband mapping (1500 us is NOT neutral)
 *   - Neutral crossing protection (must pass through neutral)
 *   - Rate limiter (configurable us/tick)
 *   - Timeout safety (returns to neutral if stale)
 */

#ifndef ESC_CONDITIONER_H
#define ESC_CONDITIONER_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* ── QUICRUN FUSION SE deadband constants ─────────────────────────────────── */
#define ESC_NEUTRAL_US          1485u   /**< True ESC neutral pulse         */
#define ESC_FWD_START_US        1443u   /**< Forward engagement threshold   */
#define ESC_REV_START_US        1527u   /**< Reverse engagement threshold   */
#define ESC_MIN_US              1000u   /**< Absolute minimum pulse         */
#define ESC_MAX_US              2000u   /**< Absolute maximum pulse         */

/* ── Tunable parameters ───────────────────────────────────────────────────── */
#define ESC_RATE_LIMIT_US       10u     /**< Max change per control tick (us)*/
#define ESC_TIMEOUT_MS          250u    /**< Command timeout (ms)           */
#define ESC_NEUTRAL_DWELL_TICKS 5u      /**< Ticks at neutral before direction change */

/**
 * @brief  Initialize ESC conditioner state.
 *
 * Sets output to ESC_NEUTRAL_US with no active direction.
 */
void esc_conditioner_init(void);

/**
 * @brief  Compute the conditioned ESC pulse width.
 *
 * Maps normalized speed [-1.0, +1.0] through the asymmetric deadband,
 * applies rate limiting and neutral crossing protection.
 *
 * @param  speed_normalized  Requested speed: -1.0 = full reverse,
 *                           0.0 = neutral, +1.0 = full forward.
 * @param  timestamp_ms      Current time from HAL_GetTick().
 * @retval Pulse width in microseconds [1000, 2000].
 */
uint16_t esc_conditioner_update(float speed_normalized, uint32_t timestamp_ms);

/**
 * @brief  Force ESC to neutral immediately (safety).
 */
void esc_conditioner_force_neutral(void);

#ifdef __cplusplus
}
#endif

#endif /* ESC_CONDITIONER_H */
