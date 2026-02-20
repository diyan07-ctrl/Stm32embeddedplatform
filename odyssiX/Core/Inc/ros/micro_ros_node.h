/**
 * @file    micro_ros_node.h
 * @brief   micro-ROS CAN command dispatcher.
 *
 * Processes incoming CAN frames from the MCP2515 RX ring buffer.
 * Maps frozen CAN IDs to the control system:
 *
 *   0x201 → /control/cmd       → atomic_guard_write()
 *   0x301 → /clock_sync        → clock sync handler
 *   0x401 → /safety/emergency_stop → safety_manager
 *
 * No transport IDs. No XRCE-DDS framing. Raw CAN frames only.
 * When MICRO_ROS_ENABLED is defined, uses rclc executor patterns.
 */

#ifndef MICRO_ROS_NODE_H
#define MICRO_ROS_NODE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/**
 * @brief  Initialize the CAN command dispatcher.
 *
 * No dynamic allocation. All state is static.
 */
void micro_ros_node_init(void);

/**
 * @brief  Process all pending CAN RX frames.
 *
 * Pops frames from the MCP2515 RX ring buffer.
 * Dispatches based on CAN ID:
 *   0x201 → parse speed/steering → atomic_guard_write() + safety_manager touch
 *   0x301 → clock sync (updates heartbeat timestamp)
 *   0x401 → emergency stop → safety_manager set/clear
 *
 * Returns the number of frames processed.
 * O(1) per frame. No blocking.
 *
 * @retval Number of frames dispatched.
 */
uint32_t micro_ros_node_spin(void);

#ifdef __cplusplus
}
#endif

#endif /* MICRO_ROS_NODE_H */
