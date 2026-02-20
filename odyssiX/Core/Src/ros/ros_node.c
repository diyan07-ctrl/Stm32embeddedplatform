/**
 * @file    ros_node.c
 * @brief   micro-ROS node, executor, and /cmd_vel subscription.
 *
 * Creates node "odyssix_mcu", subscribes to /cmd_vel (Twist).
 * Callback maps linear.x → speed, angular.z → steering,
 * and writes to the atomic guard for the control task.
 *
 * Only compiled when MICRO_ROS_ENABLED is defined.
 */

#ifdef MICRO_ROS_ENABLED

#include "ros/ros_node.h"
#include "ros/transport.h"
#include "control/atomic_guard.h"

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <geometry_msgs/msg/twist.h>

#include <string.h>

/* ── micro-ROS objects (static lifetime) ──────────────────────────────────── */
static rcl_allocator_t         allocator;
static rclc_support_t          support;
static rcl_node_t              node;
static rcl_subscription_t      cmd_vel_sub;
static rclc_executor_t         executor;
static geometry_msgs__msg__Twist cmd_vel_msg;

/* ── Subscription callback ────────────────────────────────────────────────── */

static void cmd_vel_callback(const void *msg_in)
{
    const geometry_msgs__msg__Twist *twist =
        (const geometry_msgs__msg__Twist *)msg_in;

    float speed    = (float)twist->linear.x;   /* Normalized [-1.0, 1.0] */
    float steering = (float)twist->angular.z;  /* Normalized [-1.0, 1.0] */

    atomic_guard_write(speed, steering);
}

/* ── Public API ───────────────────────────────────────────────────────────── */

bool ros_node_init(void)
{
    allocator = rcl_get_default_allocator();

    /* Initialize support (connects to agent via custom transport) */
    rcl_ret_t ret = rclc_support_init(&support, 0, NULL, &allocator);
    if (ret != RCL_RET_OK) return false;

    /* Create node */
    ret = rclc_node_init_default(&node, "odyssix_mcu", "", &support);
    if (ret != RCL_RET_OK) return false;

    /* Create subscription to /cmd_vel */
    ret = rclc_subscription_init_default(
        &cmd_vel_sub,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        "cmd_vel"
    );
    if (ret != RCL_RET_OK) return false;

    /* Initialize executor with 1 subscription handle */
    ret = rclc_executor_init(&executor, &support.context, 1, &allocator);
    if (ret != RCL_RET_OK) return false;

    ret = rclc_executor_add_subscription(
        &executor,
        &cmd_vel_sub,
        &cmd_vel_msg,
        &cmd_vel_callback,
        ON_NEW_DATA
    );
    if (ret != RCL_RET_OK) return false;

    return true;
}

void ros_node_spin_once(uint32_t timeout_ms)
{
    rclc_executor_spin_some(&executor, RCL_MS_TO_NS(timeout_ms));
}

void ros_node_fini(void)
{
    rclc_executor_fini(&executor);
    rcl_subscription_fini(&cmd_vel_sub, &node);
    rcl_node_fini(&node);
    rclc_support_fini(&support);
}

#endif /* MICRO_ROS_ENABLED */
