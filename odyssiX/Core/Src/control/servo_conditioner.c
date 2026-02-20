/**
 * @file    servo_conditioner.c
 * @brief   Servo signal conditioner — linear mapping with clamping.
 */

#include "control/servo_conditioner.h"

uint16_t servo_conditioner_update(float steering_normalized)
{
    /* Clamp input */
    if (steering_normalized > 1.0f)  steering_normalized = 1.0f;
    if (steering_normalized < -1.0f) steering_normalized = -1.0f;

    /* Linear map: -1.0 → 1000, 0.0 → 1500, +1.0 → 2000 */
    float pulse_f = (float)SERVO_NEUTRAL_US + (steering_normalized * 500.0f);

    int32_t pulse = (int32_t)(pulse_f + 0.5f); /* Round to nearest */

    if (pulse < (int32_t)SERVO_MIN_US) pulse = (int32_t)SERVO_MIN_US;
    if (pulse > (int32_t)SERVO_MAX_US) pulse = (int32_t)SERVO_MAX_US;

    return (uint16_t)pulse;
}
