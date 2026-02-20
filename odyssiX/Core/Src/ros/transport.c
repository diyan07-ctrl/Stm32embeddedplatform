/**
 * @file    transport.c
 * @brief   micro-ROS custom transport over MCP2515 CAN bus.
 *
 * Implements XRCE-DDS transport callbacks.
 * TX: fragments byte stream into 8-byte CAN frames.
 * RX: reassembles CAN frames (filtered by UROS_CAN_RX_ID) into byte stream.
 *
 * Only compiled when MICRO_ROS_ENABLED is defined.
 */

#ifdef MICRO_ROS_ENABLED

#include "ros/transport.h"
#include "can/mcp2515.h"
#include "can/can_buffer.h"
#include "stm32f4xx_hal.h"
#include "cmsis_os.h"
#include <string.h>

#include <rmw_microros/rmw_microros.h>

bool transport_open(struct uxrCustomTransport *transport)
{
    (void)transport;
    /* MCP2515 is already initialized by app_modules_init() */
    return true;
}

bool transport_close(struct uxrCustomTransport *transport)
{
    (void)transport;
    return true;
}

size_t transport_write(struct uxrCustomTransport *transport,
                       const uint8_t *buf, size_t len, uint8_t *err)
{
    (void)transport;

    size_t bytes_sent = 0;

    while (bytes_sent < len) {
        can_frame_t frame;
        frame.id = UROS_CAN_TX_ID;

        size_t remaining = len - bytes_sent;
        frame.dlc = (remaining > CAN_FRAME_MAX_DLC)
                   ? CAN_FRAME_MAX_DLC
                   : (uint8_t)remaining;

        memcpy(frame.data, &buf[bytes_sent], frame.dlc);

        mcp2515_err_t mcp_err = mcp2515_send(&frame);
        if (mcp_err != MCP2515_OK) {
            *err = 1u;
            break;
        }

        bytes_sent += frame.dlc;
    }

    *err = 0u;
    return bytes_sent;
}

size_t transport_read(struct uxrCustomTransport *transport,
                      uint8_t *buf, size_t len, int timeout, uint8_t *err)
{
    (void)transport;

    can_buffer_t *rx_buf = mcp2515_get_rx_buffer();
    size_t bytes_read = 0;
    uint32_t start = HAL_GetTick();

    while (bytes_read < len) {
        can_frame_t frame;

        if (can_buffer_pop(rx_buf, &frame)) {
            /* Filter: only accept frames with our RX ID */
            if (frame.id == UROS_CAN_RX_ID) {
                size_t to_copy = frame.dlc;
                if (bytes_read + to_copy > len) {
                    to_copy = len - bytes_read;
                }
                memcpy(&buf[bytes_read], frame.data, to_copy);
                bytes_read += to_copy;
            }
            /* Frames with other IDs are silently discarded */
        } else {
            /* Buffer empty — check timeout */
            if ((int)(HAL_GetTick() - start) >= timeout) {
                break;
            }
            osDelay(1);
        }
    }

    *err = 0u;
    return bytes_read;
}

void transport_setup(void)
{
    rmw_uros_set_custom_transport(
        true,                /* framing enabled */
        NULL,                /* args (unused)   */
        transport_open,
        transport_close,
        transport_write,
        transport_read
    );
}

#endif /* MICRO_ROS_ENABLED */
