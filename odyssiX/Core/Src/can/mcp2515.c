/**
 * @file    mcp2515.c
 * @brief   MCP2515 CAN controller driver — SPI DMA, FreeRTOS.
 *
 * All SPI transfers use DMA with binary semaphore synchronization.
 * CS pin (PA4) is managed by direct register writes for speed.
 * The RX task waits on a task notification from the EXTI0 callback.
 */

#include "can/mcp2515.h"
#include "can/mcp2515_regs.h"
#include "can/can_protocol.h"
#include "main.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include <string.h>

/* ── External HAL handles (defined in main.c) ─────────────────────────────── */
extern SPI_HandleTypeDef hspi1;

/* ── CS Pin Control (PA4) ─────────────────────────────────────────────────── */
#define CS_LOW()   (GPIOA->BSRR = (uint32_t)GPIO_PIN_4 << 16u)
#define CS_HIGH()  (GPIOA->BSRR = GPIO_PIN_4)

/* ── SPI DMA synchronization ──────────────────────────────────────────────── */
static SemaphoreHandle_t     spi_dma_sem;
static StaticSemaphore_t     spi_dma_sem_buf;

/* ── SPI mutex (prevents concurrent SPI access from multiple tasks) ───────── */
static SemaphoreHandle_t     spi_mutex;
static StaticSemaphore_t     spi_mutex_buf;

/* ── CAN RX task notification handle ──────────────────────────────────────── */
static TaskHandle_t          can_rx_task_handle;

/* ── Shared RX ring buffer ────────────────────────────────────────────────── */
static can_buffer_t          rx_buffer;

/* ── Overflow counter ────────────────────────────────────────────────────── */
static volatile uint32_t     rx_overflow_count;

/* ── DMA transfer buffers (must not be on stack) ──────────────────────────── */
#define SPI_BUF_SIZE  16u
static uint8_t               spi_tx_buf[SPI_BUF_SIZE];
static uint8_t               spi_rx_buf[SPI_BUF_SIZE];

/* ── SPI transfer timeout ─────────────────────────────────────────────────── */
#define SPI_DMA_TIMEOUT_MS   50u

/* ── Bit timing configurations ────────────────────────────────────────────── */
#if (MCP2515_OSC_HZ == 8000000UL)
/*
 * 8 MHz oscillator, 500 kbps
 * BRP=0 → TQ = 250 ns, 8 TQ/bit
 * SyncSeg=1, PropSeg=2, PS1=3, PS2=2 → sample point 75%
 */
#define MCP2515_CNF1_VAL    0x00u   /* SJW=1TQ, BRP=0 */
#define MCP2515_CNF2_VAL    0x91u   /* BTLMODE=1, SAM=0, PHSEG1=2(3TQ), PRSEG=1(2TQ) */
#define MCP2515_CNF3_VAL    0x01u   /* PHSEG2=1(2TQ) */

#elif (MCP2515_OSC_HZ == 16000000UL)
/*
 * 16 MHz oscillator, 500 kbps
 * BRP=0 → TQ = 125 ns, 16 TQ/bit
 * SyncSeg=1, PropSeg=3, PS1=8, PS2=4 → sample point 75%
 */
#define MCP2515_CNF1_VAL    0x00u
#define MCP2515_CNF2_VAL    0xBAu   /* BTLMODE=1, SAM=0, PHSEG1=7(8TQ), PRSEG=2(3TQ) */
#define MCP2515_CNF3_VAL    0x03u   /* PHSEG2=3(4TQ) */

#else
#error "MCP2515_OSC_HZ must be 8000000 or 16000000"
#endif

/* ── Internal helpers ─────────────────────────────────────────────────────── */

/**
 * @brief  Perform a DMA SPI transfer (TX+RX simultaneously).
 *
 * Asserts CS, starts DMA, waits on semaphore, deasserts CS.
 * Must be called with spi_mutex held.
 *
 * @param  len  Transfer length (must be <= SPI_BUF_SIZE).
 * @retval HAL_OK on success.
 */
static HAL_StatusTypeDef spi_xfer(uint16_t len)
{
    CS_LOW();

    HAL_StatusTypeDef status = HAL_SPI_TransmitReceive_DMA(&hspi1,
                                                            spi_tx_buf,
                                                            spi_rx_buf,
                                                            len);
    if (status != HAL_OK) {
        CS_HIGH();
        return status;
    }

    /* Wait for DMA complete callback to give the semaphore */
    if (xSemaphoreTake(spi_dma_sem, pdMS_TO_TICKS(SPI_DMA_TIMEOUT_MS)) != pdTRUE) {
        HAL_SPI_Abort(&hspi1);
        CS_HIGH();
        return HAL_TIMEOUT;
    }

    CS_HIGH();
    return HAL_OK;
}

/**
 * @brief  Read a single MCP2515 register.
 */
static mcp2515_err_t reg_read(uint8_t addr, uint8_t *value)
{
    memset(spi_tx_buf, 0, 3);
    spi_tx_buf[0] = MCP2515_CMD_READ;
    spi_tx_buf[1] = addr;

    if (spi_xfer(3) != HAL_OK) return MCP2515_ERR_SPI;

    *value = spi_rx_buf[2];
    return MCP2515_OK;
}

/**
 * @brief  Write a single MCP2515 register.
 */
static mcp2515_err_t reg_write(uint8_t addr, uint8_t value)
{
    spi_tx_buf[0] = MCP2515_CMD_WRITE;
    spi_tx_buf[1] = addr;
    spi_tx_buf[2] = value;

    if (spi_xfer(3) != HAL_OK) return MCP2515_ERR_SPI;

    return MCP2515_OK;
}

/**
 * @brief  Modify specific bits in a MCP2515 register.
 */
static mcp2515_err_t reg_bit_modify(uint8_t addr, uint8_t mask, uint8_t value)
{
    spi_tx_buf[0] = MCP2515_CMD_BIT_MODIFY;
    spi_tx_buf[1] = addr;
    spi_tx_buf[2] = mask;
    spi_tx_buf[3] = value;

    if (spi_xfer(4) != HAL_OK) return MCP2515_ERR_SPI;

    return MCP2515_OK;
}

/**
 * @brief  Send the RESET command.
 */
static mcp2515_err_t chip_reset(void)
{
    spi_tx_buf[0] = MCP2515_CMD_RESET;

    if (spi_xfer(1) != HAL_OK) return MCP2515_ERR_SPI;

    /* MCP2515 needs ~2 ms after reset */
    vTaskDelay(pdMS_TO_TICKS(5));
    return MCP2515_OK;
}

/**
 * @brief  Wait for MCP2515 to enter a specific operating mode.
 */
static mcp2515_err_t wait_for_mode(uint8_t expected_mode, uint32_t timeout_ms)
{
    uint32_t start = HAL_GetTick();
    uint8_t  canstat;

    while ((HAL_GetTick() - start) < timeout_ms) {
        mcp2515_err_t err = reg_read(MCP2515_REG_CANSTAT, &canstat);
        if (err != MCP2515_OK) return err;

        if ((canstat & MCP2515_CANSTAT_OPMOD_MASK) == expected_mode) {
            return MCP2515_OK;
        }
        vTaskDelay(1);
    }

    return MCP2515_ERR_TIMEOUT;
}

/**
 * @brief  Read one RX buffer (RXB0 or RXB1) from MCP2515.
 *
 * Uses the READ RX BUFFER instruction for efficiency.
 * Reads SIDH + SIDL + EID8 + EID0 + DLC + 8 data bytes = 13 bytes.
 */
static mcp2515_err_t read_rx_buffer(uint8_t rxb_cmd, can_frame_t *frame)
{
    memset(spi_tx_buf, 0, 14);
    spi_tx_buf[0] = rxb_cmd;

    if (spi_xfer(14) != HAL_OK) return MCP2515_ERR_SPI;

    /* Parse: spi_rx_buf[1]=SIDH, [2]=SIDL, [3]=EID8, [4]=EID0, [5]=DLC, [6..13]=data */
    uint16_t sidh = spi_rx_buf[1];
    uint16_t sidl = spi_rx_buf[2];

    /* Standard ID: SIDH[7:0] + SIDL[7:5] */
    frame->id = (uint32_t)((sidh << 3u) | (sidl >> 5u));

    frame->dlc = spi_rx_buf[5] & 0x0Fu;
    if (frame->dlc > CAN_FRAME_MAX_DLC) {
        frame->dlc = CAN_FRAME_MAX_DLC;
    }

    memcpy(frame->data, &spi_rx_buf[6], frame->dlc);

    return MCP2515_OK;
}

/* ── Public API ───────────────────────────────────────────────────────────── */

mcp2515_err_t mcp2515_init(void)
{
    /* Create synchronization primitives */
    spi_dma_sem = xSemaphoreCreateBinaryStatic(&spi_dma_sem_buf);
    spi_mutex   = xSemaphoreCreateMutexStatic(&spi_mutex_buf);

    xSemaphoreTake(spi_mutex, portMAX_DELAY);

    /* Hardware reset */
    mcp2515_err_t err = chip_reset();
    if (err != MCP2515_OK) { xSemaphoreGive(spi_mutex); return err; }

    /* Verify we're in configuration mode after reset */
    err = wait_for_mode(MCP2515_CANCTRL_REQOP_CONFIG, 100);
    if (err != MCP2515_OK) { xSemaphoreGive(spi_mutex); return MCP2515_ERR_CONFIG; }

    /* Configure bit timing for 500 kbps */
    err = reg_write(MCP2515_REG_CNF1, MCP2515_CNF1_VAL);
    if (err != MCP2515_OK) { xSemaphoreGive(spi_mutex); return err; }

    err = reg_write(MCP2515_REG_CNF2, MCP2515_CNF2_VAL);
    if (err != MCP2515_OK) { xSemaphoreGive(spi_mutex); return err; }

    err = reg_write(MCP2515_REG_CNF3, MCP2515_CNF3_VAL);
    if (err != MCP2515_OK) { xSemaphoreGive(spi_mutex); return err; }

    /* Verify bit timing */
    uint8_t readback;
    err = reg_read(MCP2515_REG_CNF1, &readback);
    if (err != MCP2515_OK || readback != MCP2515_CNF1_VAL) {
        xSemaphoreGive(spi_mutex);
        return MCP2515_ERR_VERIFY;
    }

    /* Configure RX buffers: use hardware filters, enable rollover */
    err = reg_write(MCP2515_REG_RXB0CTRL,
                    MCP2515_RXBCTRL_RXM_STD | MCP2515_RXBCTRL_BUKT);
    if (err != MCP2515_OK) { xSemaphoreGive(spi_mutex); return err; }

    err = reg_write(MCP2515_REG_RXB1CTRL, MCP2515_RXBCTRL_RXM_STD);
    if (err != MCP2515_OK) { xSemaphoreGive(spi_mutex); return err; }

    /* ── Hardware RX filters: accept ONLY 0x201, 0x301, 0x401 ────────── */
    /* Mask 0 (for RXB0, filters 0-1): match all 11 bits of standard ID */
    err = reg_write(MCP2515_REG_RXM0SIDH, 0xFFu);
    if (err != MCP2515_OK) { xSemaphoreGive(spi_mutex); return err; }
    err = reg_write(MCP2515_REG_RXM0SIDL, 0xE0u);
    if (err != MCP2515_OK) { xSemaphoreGive(spi_mutex); return err; }

    /* Mask 1 (for RXB1, filters 2-5): match all 11 bits of standard ID */
    err = reg_write(MCP2515_REG_RXM1SIDH, 0xFFu);
    if (err != MCP2515_OK) { xSemaphoreGive(spi_mutex); return err; }
    err = reg_write(MCP2515_REG_RXM1SIDL, 0xE0u);
    if (err != MCP2515_OK) { xSemaphoreGive(spi_mutex); return err; }

    /* Filter 0 (RXB0): accept 0x201 = /control/cmd */
    err = reg_write(MCP2515_REG_RXF0SIDH,
                    (uint8_t)(CAN_RX_FILTER_0 >> 3u));
    if (err != MCP2515_OK) { xSemaphoreGive(spi_mutex); return err; }
    err = reg_write(MCP2515_REG_RXF0SIDL,
                    (uint8_t)((CAN_RX_FILTER_0 & 0x07u) << 5u));
    if (err != MCP2515_OK) { xSemaphoreGive(spi_mutex); return err; }

    /* Filter 1 (RXB0): accept 0x301 = /clock_sync */
    err = reg_write(MCP2515_REG_RXF1SIDH,
                    (uint8_t)(CAN_RX_FILTER_1 >> 3u));
    if (err != MCP2515_OK) { xSemaphoreGive(spi_mutex); return err; }
    err = reg_write(MCP2515_REG_RXF1SIDL,
                    (uint8_t)((CAN_RX_FILTER_1 & 0x07u) << 5u));
    if (err != MCP2515_OK) { xSemaphoreGive(spi_mutex); return err; }

    /* Filter 2 (RXB1): accept 0x401 = /safety/emergency_stop */
    err = reg_write(MCP2515_REG_RXF2SIDH,
                    (uint8_t)(CAN_RX_FILTER_2 >> 3u));
    if (err != MCP2515_OK) { xSemaphoreGive(spi_mutex); return err; }
    err = reg_write(MCP2515_REG_RXF2SIDL,
                    (uint8_t)((CAN_RX_FILTER_2 & 0x07u) << 5u));
    if (err != MCP2515_OK) { xSemaphoreGive(spi_mutex); return err; }

    /* Filters 3-5 (RXB1): duplicate 0x401 for all remaining filter slots */
    err = reg_write(MCP2515_REG_RXF3SIDH,
                    (uint8_t)(CAN_RX_FILTER_2 >> 3u));
    if (err != MCP2515_OK) { xSemaphoreGive(spi_mutex); return err; }
    err = reg_write(MCP2515_REG_RXF3SIDL,
                    (uint8_t)((CAN_RX_FILTER_2 & 0x07u) << 5u));
    if (err != MCP2515_OK) { xSemaphoreGive(spi_mutex); return err; }

    err = reg_write(MCP2515_REG_RXF4SIDH,
                    (uint8_t)(CAN_RX_FILTER_2 >> 3u));
    if (err != MCP2515_OK) { xSemaphoreGive(spi_mutex); return err; }
    err = reg_write(MCP2515_REG_RXF4SIDL,
                    (uint8_t)((CAN_RX_FILTER_2 & 0x07u) << 5u));
    if (err != MCP2515_OK) { xSemaphoreGive(spi_mutex); return err; }

    err = reg_write(MCP2515_REG_RXF5SIDH,
                    (uint8_t)(CAN_RX_FILTER_2 >> 3u));
    if (err != MCP2515_OK) { xSemaphoreGive(spi_mutex); return err; }
    err = reg_write(MCP2515_REG_RXF5SIDL,
                    (uint8_t)((CAN_RX_FILTER_2 & 0x07u) << 5u));
    if (err != MCP2515_OK) { xSemaphoreGive(spi_mutex); return err; }

    /* Enable RX interrupts (RX0IE + RX1IE) */
    err = reg_write(MCP2515_REG_CANINTE,
                    MCP2515_INT_RX0 | MCP2515_INT_RX1);
    if (err != MCP2515_OK) { xSemaphoreGive(spi_mutex); return err; }

    /* Clear all interrupt flags */
    err = reg_write(MCP2515_REG_CANINTF, 0x00u);
    if (err != MCP2515_OK) { xSemaphoreGive(spi_mutex); return err; }

    /* Initialize RX ring buffer and overflow counter */
    can_buffer_init(&rx_buffer);
    rx_overflow_count = 0u;

    /* Switch to normal mode */
    err = reg_bit_modify(MCP2515_REG_CANCTRL,
                         MCP2515_CANCTRL_REQOP_MASK,
                         MCP2515_CANCTRL_REQOP_NORMAL);
    if (err != MCP2515_OK) { xSemaphoreGive(spi_mutex); return err; }

    err = wait_for_mode(MCP2515_CANCTRL_REQOP_NORMAL, 100);
    if (err != MCP2515_OK) { xSemaphoreGive(spi_mutex); return MCP2515_ERR_CONFIG; }

    xSemaphoreGive(spi_mutex);
    return MCP2515_OK;
}

mcp2515_err_t mcp2515_send(const can_frame_t *frame)
{
    xSemaphoreTake(spi_mutex, portMAX_DELAY);

    /* Read status to find a free TX buffer */
    memset(spi_tx_buf, 0, 3);
    spi_tx_buf[0] = MCP2515_CMD_READ_STATUS;

    if (spi_xfer(3) != HAL_OK) {
        xSemaphoreGive(spi_mutex);
        return MCP2515_ERR_SPI;
    }

    uint8_t status = spi_rx_buf[1];

    /* Select first available TX buffer */
    uint8_t load_cmd;
    uint8_t rts_cmd;

    if (!(status & MCP2515_STAT_TX0REQ)) {
        load_cmd = MCP2515_CMD_LOAD_TXB0_SIDH;
        rts_cmd  = MCP2515_CMD_RTS_TX0;
    } else if (!(status & MCP2515_STAT_TX1REQ)) {
        load_cmd = MCP2515_CMD_LOAD_TXB1_SIDH;
        rts_cmd  = MCP2515_CMD_RTS_TX1;
    } else if (!(status & MCP2515_STAT_TX2REQ)) {
        load_cmd = MCP2515_CMD_LOAD_TXB2_SIDH;
        rts_cmd  = MCP2515_CMD_RTS_TX2;
    } else {
        xSemaphoreGive(spi_mutex);
        return MCP2515_ERR_TX_BUSY;
    }

    /* Build the TX buffer load: cmd + SIDH + SIDL + EID8 + EID0 + DLC + data */
    uint8_t dlc = frame->dlc;
    if (dlc > CAN_FRAME_MAX_DLC) dlc = CAN_FRAME_MAX_DLC;

    memset(spi_tx_buf, 0, SPI_BUF_SIZE);
    spi_tx_buf[0] = load_cmd;
    spi_tx_buf[1] = (uint8_t)(frame->id >> 3u);       /* SIDH */
    spi_tx_buf[2] = (uint8_t)((frame->id & 0x07u) << 5u); /* SIDL */
    spi_tx_buf[3] = 0x00u;                              /* EID8 */
    spi_tx_buf[4] = 0x00u;                              /* EID0 */
    spi_tx_buf[5] = dlc;                                /* DLC  */
    memcpy(&spi_tx_buf[6], frame->data, dlc);

    if (spi_xfer((uint16_t)(6u + dlc)) != HAL_OK) {
        xSemaphoreGive(spi_mutex);
        return MCP2515_ERR_SPI;
    }

    /* Request to send */
    spi_tx_buf[0] = rts_cmd;
    if (spi_xfer(1) != HAL_OK) {
        xSemaphoreGive(spi_mutex);
        return MCP2515_ERR_SPI;
    }

    xSemaphoreGive(spi_mutex);
    return MCP2515_OK;
}

uint32_t mcp2515_read_all_rx(can_buffer_t *buf)
{
    uint32_t count = 0;

    xSemaphoreTake(spi_mutex, portMAX_DELAY);

    /* Read interrupt flags */
    uint8_t intf;
    if (reg_read(MCP2515_REG_CANINTF, &intf) != MCP2515_OK) {
        xSemaphoreGive(spi_mutex);
        return 0;
    }

    /* RXB0 has data */
    if (intf & MCP2515_INT_RX0) {
        can_frame_t frame;
        if (read_rx_buffer(MCP2515_CMD_READ_RXB0_SIDH, &frame) == MCP2515_OK) {
            if (!can_buffer_push(buf, &frame)) {
                rx_overflow_count++;
            }
            count++;
        }
        reg_bit_modify(MCP2515_REG_CANINTF, MCP2515_INT_RX0, 0x00u);
    }

    /* RXB1 has data */
    if (intf & MCP2515_INT_RX1) {
        can_frame_t frame;
        if (read_rx_buffer(MCP2515_CMD_READ_RXB1_SIDH, &frame) == MCP2515_OK) {
            if (!can_buffer_push(buf, &frame)) {
                rx_overflow_count++;
            }
            count++;
        }
        reg_bit_modify(MCP2515_REG_CANINTF, MCP2515_INT_RX1, 0x00u);
    }

    /* Clear error flags if any */
    if (intf & (MCP2515_INT_ERR | MCP2515_INT_MERR)) {
        reg_bit_modify(MCP2515_REG_CANINTF,
                       MCP2515_INT_ERR | MCP2515_INT_MERR, 0x00u);
    }

    xSemaphoreGive(spi_mutex);
    return count;
}

uint8_t mcp2515_read_interrupts(void)
{
    uint8_t val = 0;
    xSemaphoreTake(spi_mutex, portMAX_DELAY);
    reg_read(MCP2515_REG_CANINTF, &val);
    xSemaphoreGive(spi_mutex);
    return val;
}

uint8_t mcp2515_read_errors(void)
{
    uint8_t val = 0;
    xSemaphoreTake(spi_mutex, portMAX_DELAY);
    reg_read(MCP2515_REG_EFLG, &val);
    xSemaphoreGive(spi_mutex);
    return val;
}

can_buffer_t *mcp2515_get_rx_buffer(void)
{
    return &rx_buffer;
}

uint32_t mcp2515_get_overflow_count(void)
{
    return rx_overflow_count;
}

void mcp2515_notify_isr(void)
{
    if (can_rx_task_handle != NULL) {
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        vTaskNotifyGiveFromISR(can_rx_task_handle, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}

void mcp2515_rx_task(void *argument)
{
    (void)argument;

    /* Store own handle for ISR notification */
    can_rx_task_handle = xTaskGetCurrentTaskHandle();

    for (;;) {
        /* Block until EXTI0 ISR gives notification */
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        /* Read all pending frames from MCP2515 */
        mcp2515_read_all_rx(&rx_buffer);
    }
}

void mcp2515_spi_dma_complete(void)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xSemaphoreGiveFromISR(spi_dma_sem, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}
