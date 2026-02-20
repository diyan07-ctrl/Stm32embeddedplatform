/**
 * @file    can_protocol.h
 * @brief   Frozen CAN ID map and frame format definitions.
 *
 * BFMC 2026 — odyssiX
 * Standard 11-bit CAN IDs. 500 kbps. No other IDs allowed.
 */

#ifndef CAN_PROTOCOL_H
#define CAN_PROTOCOL_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* ══════════════════════════════════════════════════════════════════════════
 *  FROZEN CAN ID MAP — DO NOT ADD, REMOVE, OR MODIFY
 * ══════════════════════════════════════════════════════════════════════════ */

/* STM32 → Jetson */
#define CAN_ID_IMU_DATA             0x101u  /**< Rotation vector quaternion  */
#define CAN_ID_ULTRASONIC_FRONT     0x102u  /**< Front ultrasonic distance   */

/* Jetson → STM32 */
#define CAN_ID_CONTROL_CMD          0x201u  /**< /control/cmd                */
#define CAN_ID_CLOCK_SYNC           0x301u  /**< /clock_sync                 */
#define CAN_ID_EMERGENCY_STOP       0x401u  /**< /safety/emergency_stop      */

/* STM32 → All */
#define CAN_ID_HEARTBEAT            0x7FFu  /**< Heartbeat                   */

/* ══════════════════════════════════════════════════════════════════════════
 *  MCP2515 HARDWARE RX FILTER ACCEPT LIST
 *  Only these IDs pass through the MCP2515 hardware filter.
 * ══════════════════════════════════════════════════════════════════════════ */
#define CAN_RX_FILTER_0             CAN_ID_CONTROL_CMD      /* 0x201 */
#define CAN_RX_FILTER_1             CAN_ID_CLOCK_SYNC       /* 0x301 */
#define CAN_RX_FILTER_2             CAN_ID_EMERGENCY_STOP   /* 0x401 */

/* ══════════════════════════════════════════════════════════════════════════
 *  FRAME FORMATS (Big-Endian byte order for multi-byte fields)
 * ══════════════════════════════════════════════════════════════════════════ */

/**
 * 0x101 — IMU Data (DLC=8)
 * Byte 0-1: qi  (int16 Q14, big-endian) — quaternion i
 * Byte 2-3: qj  (int16 Q14, big-endian) — quaternion j
 * Byte 4-5: qk  (int16 Q14, big-endian) — quaternion k
 * Byte 6-7: qr  (int16 Q14, big-endian) — quaternion real
 */
#define CAN_IMU_DLC                 8u

/**
 * 0x102 — Ultrasonic Front (DLC=8)
 * Byte 0-1: distance_cm (uint16, big-endian)
 * Byte 2:   status (0=valid, 1=out_of_range, 2=error)
 * Byte 3:   reserved (0x00)
 * Byte 4-7: timestamp_us (uint32, big-endian) — TIM5 timestamp
 */
#define CAN_ULTRA_DLC               8u
#define CAN_ULTRA_STATUS_VALID      0x00u
#define CAN_ULTRA_STATUS_OOR        0x01u
#define CAN_ULTRA_STATUS_ERROR      0x02u

/**
 * 0x201 — Control Command (DLC=4)
 * Byte 0-1: speed    (int16, big-endian) — Q15: [-32768..32767] → [-1.0..+1.0)
 * Byte 2-3: steering (int16, big-endian) — Q15: [-32768..32767] → [-1.0..+1.0)
 */
#define CAN_CTRL_DLC                4u
#define CAN_Q15_SCALE               32767.0f

/**
 * 0x301 — Clock Sync (DLC=8)
 * Byte 0-7: jetson_time_ns (uint64, big-endian) — Jetson ROS time
 */
#define CAN_CLOCK_SYNC_DLC          8u

/**
 * 0x401 — Emergency Stop (DLC=1)
 * Byte 0: 0x01 = STOP, 0x00 = CLEAR
 */
#define CAN_ESTOP_DLC               1u
#define CAN_ESTOP_ACTIVATE          0x01u
#define CAN_ESTOP_CLEAR             0x00u

/**
 * 0x7FF — Heartbeat (DLC=8)
 * Byte 0:   state (0=INIT, 1=RUNNING, 2=EMERGENCY, 3=FAULT)
 * Byte 1:   safety_sources (bitmask of active emergency sources)
 * Byte 2-5: uptime_s (uint32, big-endian)
 * Byte 6-7: reserved
 */
#define CAN_HB_DLC                  8u
#define CAN_HB_STATE_INIT           0x00u
#define CAN_HB_STATE_RUNNING        0x01u
#define CAN_HB_STATE_EMERGENCY      0x02u
#define CAN_HB_STATE_FAULT          0x03u

/* ── Timeout constants (microseconds) ────────────────────────────────────── */
#define CONTROL_TIMEOUT_US          100000u     /* 100 ms */
#define HEARTBEAT_TIMEOUT_US        500000u     /* 500 ms */
#define HEARTBEAT_SEND_INTERVAL_US  200000u     /* 200 ms (5 Hz) */

/* ── Ultrasonic safety thresholds ────────────────────────────────────────── */
#define ULTRA_MIN_RANGE_CM          5u
#define ULTRA_MAX_RANGE_CM          500u
#define ULTRA_EMERGENCY_THRESHOLD_CM 15u

#ifdef __cplusplus
}
#endif

#endif /* CAN_PROTOCOL_H */
