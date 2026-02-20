/**
 * @file    can_buffer.h
 * @brief   ISR-safe CAN frame ring buffer.
 *
 * Lock-free single-producer / single-consumer ring buffer for CAN frames.
 * Producer: CAN RX task (from MCP2515 read).
 * Consumer: micro-ROS transport read function.
 *
 * Thread safety:
 *   - One producer and one consumer may operate concurrently without locking.
 *   - Multiple producers require external synchronization.
 */

#ifndef CAN_BUFFER_H
#define CAN_BUFFER_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

/** Maximum CAN data bytes per frame (standard CAN 2.0) */
#define CAN_FRAME_MAX_DLC   8u

/** Ring buffer depth — must be power of 2 */
#define CAN_BUFFER_SIZE     32u

/** CAN frame representation */
typedef struct {
    uint32_t id;                    /**< Standard or extended CAN ID        */
    uint8_t  dlc;                   /**< Data length code (0–8)             */
    uint8_t  data[CAN_FRAME_MAX_DLC]; /**< Payload bytes                   */
} can_frame_t;

/** Ring buffer control structure */
typedef struct {
    can_frame_t frames[CAN_BUFFER_SIZE];
    volatile uint32_t head;         /**< Next write position (producer)     */
    volatile uint32_t tail;         /**< Next read position (consumer)      */
} can_buffer_t;

/**
 * @brief  Initialize ring buffer to empty state.
 * @param  buf  Pointer to buffer instance.
 */
void can_buffer_init(can_buffer_t *buf);

/**
 * @brief  Push a CAN frame into the buffer.
 * @param  buf    Pointer to buffer instance.
 * @param  frame  Frame to copy in.
 * @retval true   Frame stored successfully.
 * @retval false  Buffer full — frame discarded.
 */
bool can_buffer_push(can_buffer_t *buf, const can_frame_t *frame);

/**
 * @brief  Pop a CAN frame from the buffer.
 * @param  buf    Pointer to buffer instance.
 * @param  frame  Output: popped frame.
 * @retval true   Frame retrieved.
 * @retval false  Buffer empty.
 */
bool can_buffer_pop(can_buffer_t *buf, can_frame_t *frame);

/**
 * @brief  Return number of frames currently buffered.
 */
uint32_t can_buffer_count(const can_buffer_t *buf);

/**
 * @brief  Check if buffer is empty.
 */
bool can_buffer_is_empty(const can_buffer_t *buf);

#ifdef __cplusplus
}
#endif

#endif /* CAN_BUFFER_H */
