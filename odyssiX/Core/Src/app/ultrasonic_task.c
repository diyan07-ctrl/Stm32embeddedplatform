/**
 * @file    ultrasonic_task.c
 * @brief   Ultrasonic task — URM37 periodic measurement @ 20 Hz.
 *
 * Pipeline:
 *   vTaskDelayUntil (50 ms) → urm37_measure() → timestamp →
 *   safety check (≤15 cm) → CAN frame 0x102 → MCP2515 TX
 *
 * Stack: 512 words
 * Priority: osPriorityAboveNormal
 */

#include "drivers/urm37_driver.h"
#include "can/mcp2515.h"
#include "can/can_protocol.h"
#include "control/safety_manager.h"
#include "system/timestamp.h"

#include "FreeRTOS.h"
#include "task.h"
#include <string.h>

/* ── Ultrasonic task period: 50 ms = 20 Hz ────────────────────────────── */
#define ULTRA_TASK_PERIOD_MS    50u

/* ── Internal: pack ultrasonic data into CAN frame 0x102 ─────────────── */
static void pack_ultra_can_frame(can_frame_t *frame,
                                  uint16_t dist_cm,
                                  uint8_t  status,
                                  uint32_t ts_us)
{
    frame->id  = CAN_ID_ULTRASONIC_FRONT;
    frame->dlc = CAN_ULTRA_DLC;

    /* Distance (big-endian uint16) */
    frame->data[0] = (uint8_t)(dist_cm >> 8u);
    frame->data[1] = (uint8_t)(dist_cm & 0xFFu);

    /* Status */
    frame->data[2] = status;
    frame->data[3] = 0x00u; /* Reserved */

    /* Timestamp (big-endian uint32) */
    frame->data[4] = (uint8_t)(ts_us >> 24u);
    frame->data[5] = (uint8_t)(ts_us >> 16u);
    frame->data[6] = (uint8_t)(ts_us >> 8u);
    frame->data[7] = (uint8_t)(ts_us & 0xFFu);
}

/* ── Task entry point ────────────────────────────────────────────────── */
void ultrasonic_task_entry(void *argument)
{
    (void)argument;

    /* Initialize URM37 driver (semaphore creation) */
    urm37_init();

    TickType_t last_wake = xTaskGetTickCount();

    for (;;) {
        /* Periodic 20 Hz measurement */
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(ULTRA_TASK_PERIOD_MS));

        /* Trigger measurement and wait for DMA response */
        urm37_measurement_t meas;
        urm37_err_t err = urm37_measure(&meas);

        /* Timestamp the measurement */
        uint32_t now_us = timestamp_now_us();

        /* Determine status and build CAN frame */
        can_frame_t frame;
        uint8_t status;

        if (err != URM37_OK) {
            /* Measurement error — report distance 0 with error status */
            pack_ultra_can_frame(&frame, 0u, CAN_ULTRA_STATUS_ERROR, now_us);
        } else if (!meas.valid) {
            /* Out of valid range */
            pack_ultra_can_frame(&frame, meas.distance_cm,
                                  CAN_ULTRA_STATUS_OOR, now_us);
            status = CAN_ULTRA_STATUS_OOR;
        } else {
            /* Valid measurement */
            pack_ultra_can_frame(&frame, meas.distance_cm,
                                  CAN_ULTRA_STATUS_VALID, now_us);

            /* Safety check: emergency if obstacle ≤ 15 cm */
            if (meas.distance_cm <= ULTRA_EMERGENCY_THRESHOLD_CM) {
                safety_manager_set_emergency(SAFETY_SRC_ULTRASONIC);
            }
        }

        (void)status; /* suppress unused warning in error path */

        /* Send CAN frame 0x102 */
        mcp2515_send(&frame);
    }
}
