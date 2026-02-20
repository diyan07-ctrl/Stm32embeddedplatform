/**
 * @file    micro_ros_node.c
 * @brief   micro-ROS CAN command dispatcher — frozen ID map.
 *
 * This module bridges CAN frames from the Jetson to the STM32 control system.
 * Replaces the standard XRCE-DDS transport with direct CAN frame parsing.
 *
 * Subscriptions (Jetson → STM32):
 *   /control/cmd       ← CAN 0x201 (speed + steering, Q15 int16)
 *   /clock_sync        ← CAN 0x301 (Jetson ROS timestamp)
 *   /safety/emergency_stop ← CAN 0x401 (stop/clear flag)
 */

#include "ros/micro_ros_node.h"
#include "can/can_buffer.h"
#include "can/can_protocol.h"
#include "can/mcp2515.h"
#include "control/atomic_guard.h"
#include "control/safety_manager.h"

/* ── Internal: parse control command (0x201) ─────────────────────────── */
static void handle_control_cmd(const can_frame_t *frame)
{
    if (frame->dlc < CAN_CTRL_DLC) {
        return; /* Malformed frame */
    }

    /* Parse Q15 fixed-point speed and steering (big-endian int16) */
    int16_t speed_q15 = (int16_t)(((uint16_t)frame->data[0] << 8u) |
                                   (uint16_t)frame->data[1]);
    int16_t steer_q15 = (int16_t)(((uint16_t)frame->data[2] << 8u) |
                                   (uint16_t)frame->data[3]);

    /* Convert Q15 to float: divide by 32767 */
    float speed    = (float)speed_q15 / CAN_Q15_SCALE;
    float steering = (float)steer_q15 / CAN_Q15_SCALE;

    /* Clamp to [-1.0, +1.0] */
    if (speed > 1.0f)    speed = 1.0f;
    if (speed < -1.0f)   speed = -1.0f;
    if (steering > 1.0f) steering = 1.0f;
    if (steering < -1.0f) steering = -1.0f;

    /* Write to atomic double-buffer (consumed by ControlTask) */
    atomic_guard_write(speed, steering);

    /* Update safety timestamps */
    safety_manager_touch_control();
    safety_manager_touch_heartbeat();
}

/* ── Internal: parse clock sync (0x301) ──────────────────────────────── */
static void handle_clock_sync(const can_frame_t *frame)
{
    (void)frame;

    /* Clock sync from Jetson.
     * The Jetson is the time authority — STM32 does not adjust its clock.
     * We just record that we heard from the Jetson (heartbeat). */
    safety_manager_touch_heartbeat();
}

/* ── Internal: parse emergency stop (0x401) ──────────────────────────── */
static void handle_emergency_stop(const can_frame_t *frame)
{
    if (frame->dlc < CAN_ESTOP_DLC) {
        return; /* Malformed */
    }

    if (frame->data[0] == CAN_ESTOP_ACTIVATE) {
        safety_manager_set_emergency(SAFETY_SRC_CAN_ESTOP);
    } else if (frame->data[0] == CAN_ESTOP_CLEAR) {
        safety_manager_clear_source(SAFETY_SRC_CAN_ESTOP);
    }

    safety_manager_touch_heartbeat();
}

/* ── Public API ──────────────────────────────────────────────────────── */

void micro_ros_node_init(void)
{
    /* No dynamic allocation. All state is in the modules we depend on. */
}

uint32_t micro_ros_node_spin(void)
{
    can_buffer_t *rx_buf = mcp2515_get_rx_buffer();
    can_frame_t frame;
    uint32_t count = 0u;

    while (can_buffer_pop(rx_buf, &frame)) {
        switch (frame.id) {
        case CAN_ID_CONTROL_CMD:
            handle_control_cmd(&frame);
            break;

        case CAN_ID_CLOCK_SYNC:
            handle_clock_sync(&frame);
            break;

        case CAN_ID_EMERGENCY_STOP:
            handle_emergency_stop(&frame);
            break;

        default:
            /* Should not happen with hardware filters active.
             * Silently discard. Do NOT add new ID handling here. */
            break;
        }
        count++;
    }

    return count;
}
