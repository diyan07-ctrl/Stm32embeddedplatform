/**
 * @file    esc_conditioner.c
 * @brief   BLDC ESC signal conditioner for QUICRUN FUSION SE.
 *
 * Implements:
 *   - Asymmetric deadband mapping (neutral at 1485 us, NOT 1500)
 *   - Rate limiter (configurable us/tick)
 *   - Neutral crossing protection (forward→reverse must transit neutral)
 *   - Timeout safety (reverts to neutral if command goes stale)
 */

#include "control/esc_conditioner.h"
#include "stm32f4xx_hal.h"

/* ── Internal state ───────────────────────────────────────────────────────── */
typedef enum {
    ESC_DIR_NEUTRAL = 0,
    ESC_DIR_FORWARD,
    ESC_DIR_REVERSE,
} esc_direction_t;

static uint16_t         current_pulse;
static esc_direction_t  current_dir;
static uint32_t         last_update_ms;
static uint32_t         neutral_dwell_count;

/* ── Helpers ──────────────────────────────────────────────────────────────── */

static inline int32_t clamp_i32(int32_t val, int32_t lo, int32_t hi)
{
    if (val < lo) return lo;
    if (val > hi) return hi;
    return val;
}

static inline float clamp_f(float val, float lo, float hi)
{
    if (val < lo) return lo;
    if (val > hi) return hi;
    return val;
}

/**
 * @brief  Map normalized speed to pulse width using asymmetric deadband.
 *
 * Forward: speed (0.0, 1.0] → [ESC_FWD_START_US, ESC_MIN_US]
 *   Note: forward goes BELOW neutral (1443 → 1000)
 * Reverse: speed [-1.0, 0.0) → [ESC_REV_START_US, ESC_MAX_US]
 *   Note: reverse goes ABOVE neutral (1527 → 2000)
 * Zero: → ESC_NEUTRAL_US
 */
static uint16_t map_speed_to_pulse(float speed)
{
    speed = clamp_f(speed, -1.0f, 1.0f);

    if (speed > 0.001f) {
        /* Forward: map (0, 1] → [ESC_FWD_START_US .. ESC_MIN_US] (decreasing) */
        float range = (float)(ESC_FWD_START_US - ESC_MIN_US);
        int32_t pulse = (int32_t)(ESC_FWD_START_US - (speed * range));
        return (uint16_t)clamp_i32(pulse, ESC_MIN_US, ESC_FWD_START_US);
    } else if (speed < -0.001f) {
        /* Reverse: map [-1, 0) → [ESC_MAX_US .. ESC_REV_START_US] (decreasing toward neutral) */
        float abs_speed = -speed;
        float range = (float)(ESC_MAX_US - ESC_REV_START_US);
        int32_t pulse = (int32_t)(ESC_REV_START_US + (abs_speed * range));
        return (uint16_t)clamp_i32(pulse, ESC_REV_START_US, ESC_MAX_US);
    } else {
        return ESC_NEUTRAL_US;
    }
}

/**
 * @brief  Determine requested direction from speed.
 */
static esc_direction_t speed_to_direction(float speed)
{
    if (speed > 0.001f)  return ESC_DIR_FORWARD;
    if (speed < -0.001f) return ESC_DIR_REVERSE;
    return ESC_DIR_NEUTRAL;
}

/* ── Public API ───────────────────────────────────────────────────────────── */

void esc_conditioner_init(void)
{
    current_pulse       = ESC_NEUTRAL_US;
    current_dir         = ESC_DIR_NEUTRAL;
    last_update_ms      = 0u;
    neutral_dwell_count = 0u;
}

uint16_t esc_conditioner_update(float speed_normalized, uint32_t timestamp_ms)
{
    /* ── Timeout safety: if no update for too long, force neutral ─────── */
    if (last_update_ms != 0u &&
        (timestamp_ms - last_update_ms) > ESC_TIMEOUT_MS) {
        esc_conditioner_force_neutral();
        return current_pulse;
    }
    last_update_ms = timestamp_ms;

    /* ── Determine target pulse and requested direction ───────────────── */
    esc_direction_t requested_dir = speed_to_direction(speed_normalized);
    uint16_t target_pulse = map_speed_to_pulse(speed_normalized);

    /* ── Neutral crossing protection ──────────────────────────────────── */
    if (current_dir != ESC_DIR_NEUTRAL && requested_dir != ESC_DIR_NEUTRAL) {
        if (current_dir != requested_dir) {
            /* Direction reversal requested: force through neutral first */
            target_pulse = ESC_NEUTRAL_US;
            neutral_dwell_count = 0u;
        }
    }

    /* If we're at neutral and want to change direction, enforce dwell time */
    if (current_dir == ESC_DIR_NEUTRAL && requested_dir != ESC_DIR_NEUTRAL) {
        if (neutral_dwell_count < ESC_NEUTRAL_DWELL_TICKS) {
            neutral_dwell_count++;
            target_pulse = ESC_NEUTRAL_US;
        }
    }

    /* ── Rate limiter ─────────────────────────────────────────────────── */
    int32_t delta = (int32_t)target_pulse - (int32_t)current_pulse;

    if (delta > (int32_t)ESC_RATE_LIMIT_US) {
        delta = (int32_t)ESC_RATE_LIMIT_US;
    } else if (delta < -(int32_t)ESC_RATE_LIMIT_US) {
        delta = -(int32_t)ESC_RATE_LIMIT_US;
    }

    int32_t new_pulse = (int32_t)current_pulse + delta;
    current_pulse = (uint16_t)clamp_i32(new_pulse, ESC_MIN_US, ESC_MAX_US);

    /* ── Update current direction based on actual output ──────────────── */
    if (current_pulse < ESC_FWD_START_US) {
        current_dir = ESC_DIR_FORWARD;
        neutral_dwell_count = 0u;
    } else if (current_pulse > ESC_REV_START_US) {
        current_dir = ESC_DIR_REVERSE;
        neutral_dwell_count = 0u;
    } else {
        /* In the deadband region: consider neutral */
        current_dir = ESC_DIR_NEUTRAL;
    }

    return current_pulse;
}

void esc_conditioner_force_neutral(void)
{
    current_pulse       = ESC_NEUTRAL_US;
    current_dir         = ESC_DIR_NEUTRAL;
    neutral_dwell_count = 0u;
}
