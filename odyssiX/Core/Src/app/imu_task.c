/**
 * @file    imu_task.c
 * @brief   IMU task — event-driven BNO085 rotation vector @ 100 Hz.
 *
 * Pipeline:
 *   EXTI1 (BNO085 INT) → task notification → I2C DMA read →
 *   parse rotation vector → timestamp → CAN frame 0x101 → MCP2515 TX
 *
 * Stack: 1024 words (CubeMX-created IMUTask)
 * Priority: osPriorityHigh
 */

#include "drivers/bno085_driver.h"
#include "can/mcp2515.h"
#include "can/can_protocol.h"
#include "system/timestamp.h"
#include "control/safety_manager.h"

#include "FreeRTOS.h"
#include "task.h"
#include <string.h>

/* ── Latest IMU data (protected by critical section) ──────────────────── */
static bno085_quaternion_t latest_quat;
static uint32_t            latest_timestamp_us;

/* ── Internal: pack quaternion into CAN frame 0x101 ──────────────────── */
static void pack_imu_can_frame(can_frame_t *frame, const bno085_quaternion_t *q)
{
    frame->id  = CAN_ID_IMU_DATA;
    frame->dlc = CAN_IMU_DLC;

    /* Q14 big-endian: qi, qj, qk, qr */
    frame->data[0] = (uint8_t)((uint16_t)q->qi >> 8u);
    frame->data[1] = (uint8_t)((uint16_t)q->qi & 0xFFu);
    frame->data[2] = (uint8_t)((uint16_t)q->qj >> 8u);
    frame->data[3] = (uint8_t)((uint16_t)q->qj & 0xFFu);
    frame->data[4] = (uint8_t)((uint16_t)q->qk >> 8u);
    frame->data[5] = (uint8_t)((uint16_t)q->qk & 0xFFu);
    frame->data[6] = (uint8_t)((uint16_t)q->qr >> 8u);
    frame->data[7] = (uint8_t)((uint16_t)q->qr & 0xFFu);
}

/* ── Public: get latest IMU data (thread-safe) ────────────────────────── */
void imu_task_get_latest(bno085_quaternion_t *q, uint32_t *ts_us)
{
    taskENTER_CRITICAL();
    *q     = latest_quat;
    *ts_us = latest_timestamp_us;
    taskEXIT_CRITICAL();
}

/* ── Task entry point ────────────────────────────────────────────────── */
void imu_task_entry(void *argument)
{
    (void)argument;

    /* Register this task for EXTI1 notifications */
    bno085_set_task_handle(xTaskGetCurrentTaskHandle());

    /* Initialize BNO085 (waits for boot, configures rotation vector @ 100 Hz) */
    bno085_err_t err = bno085_init();
    (void)err; /* Continue even if init fails — task retries on each notification */

    memset(&latest_quat, 0, sizeof(latest_quat));
    latest_timestamp_us = 0u;

    for (;;) {
        /* Wait for EXTI1 notification (BNO085 data-ready) */
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        /* Read SHTP packet via I2C DMA */
        if (bno085_read_packet() != BNO085_OK) {
            continue;
        }

        /* Parse rotation vector */
        bno085_quaternion_t q;
        if (bno085_parse_rotation_vector(&q) != BNO085_OK) {
            continue; /* Not a rotation vector report */
        }

        /* Timestamp the measurement */
        uint32_t now_us = timestamp_now_us();

        /* Store latest data (critical section for coherent read) */
        taskENTER_CRITICAL();
        latest_quat         = q;
        latest_timestamp_us = now_us;
        taskEXIT_CRITICAL();

        /* Pack and send CAN frame 0x101 */
        can_frame_t frame;
        pack_imu_can_frame(&frame, &q);
        mcp2515_send(&frame);
    }
}
