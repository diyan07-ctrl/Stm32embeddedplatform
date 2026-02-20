/**
 * @file    ros_node.h
 * @brief   micro-ROS node, executor, and subscription management.
 *
 * Sets up:
 *   - rclc allocator, support, node
 *   - Subscription to /cmd_vel (geometry_msgs/msg/Twist)
 *   - Executor with subscription handle
 *
 * On each spin, the subscription callback writes speed and steering
 * to the atomic guard for the 500 Hz control task to consume.
 *
 * Requires: MICRO_ROS_ENABLED defined to compile.
 */

#ifndef ROS_NODE_H
#define ROS_NODE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>

#ifdef MICRO_ROS_ENABLED

/**
 * @brief  Initialize the micro-ROS node and executor.
 *
 * Creates node "odyssix_mcu" in namespace "", subscribes to /cmd_vel.
 * Must be called after transport_setup() and osKernelStart().
 *
 * @retval true   Node initialized successfully.
 * @retval false  Initialization failed.
 */
bool ros_node_init(void);

/**
 * @brief  Spin the executor once.
 *
 * Polls for incoming messages and dispatches callbacks.
 * Call this in a loop from the micro-ROS task.
 *
 * @param  timeout_ms  Maximum time to wait for data.
 */
void ros_node_spin_once(uint32_t timeout_ms);

/**
 * @brief  Clean up the micro-ROS node and executor.
 */
void ros_node_fini(void);

#else /* MICRO_ROS_ENABLED not defined */

static inline bool ros_node_init(void) { return false; }
static inline void ros_node_spin_once(uint32_t timeout_ms) { (void)timeout_ms; }
static inline void ros_node_fini(void) { (void)0; }

#endif /* MICRO_ROS_ENABLED */

#ifdef __cplusplus
}
#endif

#endif /* ROS_NODE_H */
