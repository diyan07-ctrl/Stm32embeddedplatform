/**
 * @file    servo_conditioner.h
 * @brief   Servo signal conditioner — linear mapping.
 *
 * Specification:
 *   Range:   1000–2000 us
 *   Neutral: 1500 us
 *   Mapping: linear from normalized [-1.0, +1.0]
 */

#ifndef SERVO_CONDITIONER_H
#define SERVO_CONDITIONER_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#define SERVO_MIN_US        1000u
#define SERVO_MAX_US        2000u
#define SERVO_NEUTRAL_US    1500u

/**
 * @brief  Compute servo pulse from normalized steering.
 *
 * @param  steering_normalized  -1.0 = full left, 0.0 = center, +1.0 = full right.
 * @retval Pulse width in microseconds [1000, 2000].
 */
uint16_t servo_conditioner_update(float steering_normalized);

#ifdef __cplusplus
}
#endif

#endif /* SERVO_CONDITIONER_H */
