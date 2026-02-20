/**
 * @file    micro_ros_task.c
 * @brief   micro-ROS CAN dispatcher task + heartbeat transmitter.
 *
 * Pipeline:
 *   Loop: micro_ros_node_spin() → dispatch CAN frames by ID
 *   Periodic: heartbeat CAN frame 0x7FF every 200 ms
 *
 * Stack: 512 words
 * Priority: osPriorityAboveNormal
 */

#include "ros/micro_ros_node.h"
#include "can/mcp2515.h"
#include "can/can_protocol.h"
#include "control/safety_manager.h"
#include "system/timestamp.h"

#include "FreeRTOS.h"
#include "task.h"
#include <string.h>

/* ── Heartbeat interval: 200 ms = 5 Hz ──────────────────────────────── */
#define HEARTBEAT_INTERVAL_MS   200u

/* ── Spin delay between iterations ───────────────────────────────────── */
#define SPIN_DELAY_MS           10u

/* ── Internal: pack heartbeat CAN frame 0x7FF ────────────────────────── */
static void send_heartbeat(uint8_t state, uint32_t uptime_s)
{
    can_frame_t frame;
    frame.id  = CAN_ID_HEARTBEAT;
    frame.dlc = CAN_HB_DLC;

    frame.data[0] = state;
    frame.data[1] = safety_manager_get_sources();

    /* Uptime in seconds (big-endian uint32) */
    frame.data[2] = (uint8_t)(uptime_s >> 24u);
    frame.data[3] = (uint8_t)(uptime_s >> 16u);
    frame.data[4] = (uint8_t)(uptime_s >> 8u);
    frame.data[5] = (uint8_t)(uptime_s & 0xFFu);

    frame.data[6] = 0x00u; /* Reserved */
    frame.data[7] = 0x00u; /* Reserved */

    mcp2515_send(&frame);
}

/* ── Task entry point ────────────────────────────────────────────────── */
void micro_ros_task_entry(void *argument)
{
    (void)argument;

    /* Initialize CAN command dispatcher */
    micro_ros_node_init();

    /* Send initial INIT heartbeat */
    send_heartbeat(CAN_HB_STATE_INIT, 0u);

    uint32_t start_us       = timestamp_now_us();
    uint32_t last_hb_us     = start_us;

    for (;;) {
        /* Dispatch all pending CAN RX frames */
        micro_ros_node_spin();

        /* Periodic heartbeat every 200 ms */
        uint32_t now_us = timestamp_now_us();
        if ((now_us - last_hb_us) >= (HEARTBEAT_INTERVAL_MS * 1000u)) {
            last_hb_us = now_us;

            /* Compute uptime in seconds */
            uint32_t uptime_s = (now_us - start_us) / 1000000u;

            /* Determine state */
            uint8_t state;
            if (safety_manager_is_emergency()) {
                state = CAN_HB_STATE_EMERGENCY;
            } else {
                state = CAN_HB_STATE_RUNNING;
            }

            send_heartbeat(state, uptime_s);
        }

        /* Yield to other tasks */
        vTaskDelay(pdMS_TO_TICKS(SPIN_DELAY_MS));
    }
}
