/**
 * @file    can_buffer.c
 * @brief   ISR-safe CAN frame ring buffer implementation.
 *
 * Lock-free SPSC (single-producer, single-consumer) ring buffer.
 * Uses power-of-2 masking for index wrap.
 * No dynamic allocation. No FreeRTOS dependency.
 */

#include "can/can_buffer.h"
#include "stm32f4xx_hal.h"
#include <string.h>

_Static_assert((CAN_BUFFER_SIZE & (CAN_BUFFER_SIZE - 1u)) == 0u,
               "CAN_BUFFER_SIZE must be a power of 2");

#define BUFFER_MASK  (CAN_BUFFER_SIZE - 1u)

void can_buffer_init(can_buffer_t *buf)
{
    buf->head = 0u;
    buf->tail = 0u;
    memset(buf->frames, 0, sizeof(buf->frames));
}

bool can_buffer_push(can_buffer_t *buf, const can_frame_t *frame)
{
    uint32_t next_head = (buf->head + 1u) & BUFFER_MASK;

    if (next_head == buf->tail) {
        return false; /* Full */
    }

    buf->frames[buf->head] = *frame;

    /* Compiler barrier — ensure frame is written before head advances. */
    __DMB();

    buf->head = next_head;
    return true;
}

bool can_buffer_pop(can_buffer_t *buf, can_frame_t *frame)
{
    if (buf->tail == buf->head) {
        return false; /* Empty */
    }

    *frame = buf->frames[buf->tail];

    /* Compiler barrier — ensure frame is read before tail advances. */
    __DMB();

    buf->tail = (buf->tail + 1u) & BUFFER_MASK;
    return true;
}

uint32_t can_buffer_count(const can_buffer_t *buf)
{
    return (buf->head - buf->tail) & BUFFER_MASK;
}

bool can_buffer_is_empty(const can_buffer_t *buf)
{
    return buf->head == buf->tail;
}
