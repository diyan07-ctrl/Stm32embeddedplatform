/**
 * @file    bno085_driver.c
 * @brief   BNO085 IMU driver — SHTP over I2C with DMA.
 *
 * Implements the Hillcrest SHTP protocol for reading SH-2 rotation vector
 * reports from the BNO085 at 100 Hz.
 *
 * I2C DMA synchronization: binary semaphore (static).
 * EXTI notification: task notification to IMUTask.
 */

#include "drivers/bno085_driver.h"
#include "config/board_config.h"
#include "main.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include <string.h>

/* ── External HAL handles ────────────────────────────────────────────── */
extern I2C_HandleTypeDef hi2c1;

/* ── SHTP Protocol Constants ─────────────────────────────────────────── */
#define SHTP_HEADER_SIZE            4u
#define SHTP_MAX_PAYLOAD            128u
#define SHTP_MAX_PACKET             (SHTP_HEADER_SIZE + SHTP_MAX_PAYLOAD)

#define SHTP_CHAN_COMMAND            0u
#define SHTP_CHAN_EXECUTABLE         1u
#define SHTP_CHAN_CONTROL            2u
#define SHTP_CHAN_INPUT_REPORT       3u

/* ── SH-2 Report IDs ────────────────────────────────────────────────── */
#define SH2_ROTATION_VECTOR         0x05u
#define SH2_SET_FEATURE_CMD         0xFDu
#define SH2_PRODUCT_ID_REQ          0xF9u

/* ── Rotation vector report interval: 10 ms = 100 Hz ────────────────── */
#define RV_REPORT_INTERVAL_US       10000u

/* ── I2C DMA synchronization ─────────────────────────────────────────── */
static SemaphoreHandle_t    i2c_dma_sem;
static StaticSemaphore_t    i2c_dma_sem_buf;

/* ── Error flag ──────────────────────────────────────────────────────── */
static volatile uint8_t     i2c_error_flag;

/* ── IMU task handle for EXTI notification ───────────────────────────── */
static TaskHandle_t         imu_task_handle;

/* ── SHTP packet buffer (static, NOT on stack — DMA safe) ────────────── */
static uint8_t              shtp_rx_buf[SHTP_MAX_PACKET];
static uint16_t             shtp_rx_len;

/* ── SHTP TX buffer for commands ─────────────────────────────────────── */
static uint8_t              shtp_tx_buf[32];

/* ── SHTP channel sequence numbers ───────────────────────────────────── */
static uint8_t              shtp_seq[6];

/* ── I2C DMA timeout ─────────────────────────────────────────────────── */
#define I2C_DMA_TIMEOUT_MS          50u

/* ── Internal: wait for I2C DMA completion ───────────────────────────── */
static bno085_err_t wait_i2c_dma(void)
{
    if (xSemaphoreTake(i2c_dma_sem, pdMS_TO_TICKS(I2C_DMA_TIMEOUT_MS)) != pdTRUE) {
        HAL_I2C_Master_Abort_IT(&hi2c1, BNO085_I2C_ADDR_8BIT);
        return BNO085_ERR_TIMEOUT;
    }
    if (i2c_error_flag) {
        i2c_error_flag = 0u;
        return BNO085_ERR_I2C;
    }
    return BNO085_OK;
}

/* ── Internal: read N bytes from BNO085 via I2C DMA ──────────────────── */
static bno085_err_t i2c_read_dma(uint8_t *buf, uint16_t len)
{
    i2c_error_flag = 0u;

    HAL_StatusTypeDef status = HAL_I2C_Master_Receive_DMA(&hi2c1,
                                                           BNO085_I2C_ADDR_8BIT,
                                                           buf, len);
    if (status != HAL_OK) {
        return BNO085_ERR_I2C;
    }

    return wait_i2c_dma();
}

/* ── Internal: write N bytes to BNO085 via I2C (polling for commands) ── */
static bno085_err_t i2c_write_blocking(const uint8_t *buf, uint16_t len)
{
    HAL_StatusTypeDef status = HAL_I2C_Master_Transmit(&hi2c1,
                                                        BNO085_I2C_ADDR_8BIT,
                                                        (uint8_t *)buf, len,
                                                        100u);
    if (status != HAL_OK) {
        return BNO085_ERR_I2C;
    }
    return BNO085_OK;
}

/* ── Internal: send SHTP packet to BNO085 ────────────────────────────── */
static bno085_err_t shtp_send(uint8_t channel, const uint8_t *payload, uint16_t payload_len)
{
    uint16_t total_len = SHTP_HEADER_SIZE + payload_len;

    if (total_len > sizeof(shtp_tx_buf)) {
        return BNO085_ERR_CONFIG;
    }

    /* SHTP header */
    shtp_tx_buf[0] = (uint8_t)(total_len & 0xFFu);
    shtp_tx_buf[1] = (uint8_t)((total_len >> 8u) & 0x7Fu);
    shtp_tx_buf[2] = channel;
    shtp_tx_buf[3] = shtp_seq[channel]++;

    /* Payload */
    memcpy(&shtp_tx_buf[SHTP_HEADER_SIZE], payload, payload_len);

    return i2c_write_blocking(shtp_tx_buf, total_len);
}

/* ── Internal: configure PA1 as EXTI1 falling edge ───────────────────── */
static void bno085_gpio_init(void)
{
    __HAL_RCC_GPIOA_CLK_ENABLE();

    GPIO_InitTypeDef gpio = {0};
    gpio.Pin  = BNO085_INT_PIN;
    gpio.Mode = GPIO_MODE_IT_FALLING;
    gpio.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(BNO085_INT_PORT, &gpio);

    HAL_NVIC_SetPriority(BNO085_INT_EXTI_IRQn, PERIPH_IRQ_PRIORITY, 0);
    HAL_NVIC_EnableIRQ(BNO085_INT_EXTI_IRQn);
}

/* ── Internal: drain any pending SHTP advertisement packets ──────────── */
static void bno085_drain_startup(void)
{
    /* After reset, BNO085 sends several advertisement packets.
     * Read and discard them (up to 10 packets, 5 ms delay each). */
    for (uint8_t i = 0; i < 10u; i++) {
        /* Check if INT is low (data available) */
        if (HAL_GPIO_ReadPin(BNO085_INT_PORT, BNO085_INT_PIN) == GPIO_PIN_RESET) {
            bno085_read_packet();
        } else {
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(5));
    }
}

/* ── Public API ──────────────────────────────────────────────────────── */

bno085_err_t bno085_init(void)
{
    /* Create I2C DMA semaphore */
    i2c_dma_sem = xSemaphoreCreateBinaryStatic(&i2c_dma_sem_buf);
    i2c_error_flag = 0u;
    memset(shtp_seq, 0, sizeof(shtp_seq));

    /* Configure PA1 EXTI */
    bno085_gpio_init();

    /* Wait for BNO085 to boot after power-on (~100 ms) */
    vTaskDelay(pdMS_TO_TICKS(120));

    /* Drain startup advertisement packets */
    bno085_drain_startup();

    /* Send Set Feature Command: enable Rotation Vector at 100 Hz */
    uint8_t set_feature[17];
    memset(set_feature, 0, sizeof(set_feature));
    set_feature[0]  = SH2_SET_FEATURE_CMD;
    set_feature[1]  = SH2_ROTATION_VECTOR;
    /* Feature flags = 0 */
    /* Change sensitivity = 0 */
    /* Report interval in microseconds (little-endian uint32) */
    set_feature[5]  = (uint8_t)(RV_REPORT_INTERVAL_US & 0xFFu);
    set_feature[6]  = (uint8_t)((RV_REPORT_INTERVAL_US >> 8u) & 0xFFu);
    set_feature[7]  = (uint8_t)((RV_REPORT_INTERVAL_US >> 16u) & 0xFFu);
    set_feature[8]  = (uint8_t)((RV_REPORT_INTERVAL_US >> 24u) & 0xFFu);
    /* Batch interval = 0 */
    /* Sensor specific = 0 */

    bno085_err_t err = shtp_send(SHTP_CHAN_CONTROL, set_feature, 17u);
    if (err != BNO085_OK) {
        return err;
    }

    /* Small delay for sensor to process command */
    vTaskDelay(pdMS_TO_TICKS(50));

    /* Drain any response packets */
    bno085_drain_startup();

    return BNO085_OK;
}

bno085_err_t bno085_read_packet(void)
{
    bno085_err_t err;

    /* Step 1: Read SHTP header (4 bytes) via I2C DMA */
    err = i2c_read_dma(shtp_rx_buf, SHTP_HEADER_SIZE);
    if (err != BNO085_OK) {
        shtp_rx_len = 0u;
        return err;
    }

    /* Parse header length (little-endian, bit 15 = continuation) */
    uint16_t pkt_len = (uint16_t)shtp_rx_buf[0] |
                       (uint16_t)((shtp_rx_buf[1] & 0x7Fu) << 8u);

    if (pkt_len == 0u || pkt_len == 0x7FFFu) {
        /* No data or invalid */
        shtp_rx_len = 0u;
        return BNO085_ERR_BAD_HEADER;
    }

    /* Clamp to buffer size */
    if (pkt_len > SHTP_MAX_PACKET) {
        pkt_len = SHTP_MAX_PACKET;
    }

    /* Step 2: Read remaining payload */
    uint16_t remaining = pkt_len - SHTP_HEADER_SIZE;
    if (remaining > 0u) {
        err = i2c_read_dma(&shtp_rx_buf[SHTP_HEADER_SIZE], remaining);
        if (err != BNO085_OK) {
            shtp_rx_len = 0u;
            return err;
        }
    }

    shtp_rx_len = pkt_len;
    return BNO085_OK;
}

bno085_err_t bno085_parse_rotation_vector(bno085_quaternion_t *q)
{
    q->valid = 0u;

    if (shtp_rx_len < SHTP_HEADER_SIZE + 14u) {
        return BNO085_ERR_NO_REPORT;
    }

    /* Check this is an input report (channel 3) */
    uint8_t channel = shtp_rx_buf[2];
    if (channel != SHTP_CHAN_INPUT_REPORT) {
        return BNO085_ERR_NO_REPORT;
    }

    /* Scan payload for rotation vector report (ID = 0x05) */
    uint8_t *payload = &shtp_rx_buf[SHTP_HEADER_SIZE];
    uint16_t payload_len = shtp_rx_len - SHTP_HEADER_SIZE;

    /* The first byte after optional timestamp is the report ID.
     * BNO085 may prepend a timestamp reference (report ID 0xFB, 5 bytes).
     * Scan for 0x05. */
    uint16_t offset = 0u;

    while (offset < payload_len) {
        uint8_t report_id = payload[offset];

        if (report_id == SH2_ROTATION_VECTOR) {
            /* Rotation Vector report format:
             * [0]    Report ID (0x05)
             * [1]    Sequence number
             * [2]    Status (bits 1:0 = accuracy)
             * [3]    Delay
             * [4-5]  i (int16 Q14, little-endian)
             * [6-7]  j (int16 Q14, little-endian)
             * [8-9]  k (int16 Q14, little-endian)
             * [10-11] real (int16 Q14, little-endian)
             * [12-13] accuracy (uint16 Q12, little-endian) - optional
             */
            if ((offset + 12u) > payload_len) {
                return BNO085_ERR_NO_REPORT;
            }

            q->qi = (int16_t)((uint16_t)payload[offset + 4] |
                              ((uint16_t)payload[offset + 5] << 8u));
            q->qj = (int16_t)((uint16_t)payload[offset + 6] |
                              ((uint16_t)payload[offset + 7] << 8u));
            q->qk = (int16_t)((uint16_t)payload[offset + 8] |
                              ((uint16_t)payload[offset + 9] << 8u));
            q->qr = (int16_t)((uint16_t)payload[offset + 10] |
                              ((uint16_t)payload[offset + 11] << 8u));

            q->accuracy = payload[offset + 2] & 0x03u;
            q->valid = 1u;
            return BNO085_OK;
        }

        /* Skip over known report types to find rotation vector */
        if (report_id == 0xFBu) {
            /* Timestamp reference: 5 bytes */
            offset += 5u;
        } else if (report_id == 0xFAu) {
            /* Base timestamp: 5 bytes */
            offset += 5u;
        } else {
            /* Unknown report — skip to next byte and try again.
             * In practice, rotation vector is 14 bytes. */
            offset += 1u;
        }
    }

    return BNO085_ERR_NO_REPORT;
}

void bno085_set_task_handle(void *handle)
{
    imu_task_handle = (TaskHandle_t)handle;
}

void bno085_notify_int_isr(void)
{
    if (imu_task_handle != NULL) {
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        vTaskNotifyGiveFromISR(imu_task_handle, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}

void bno085_i2c_rx_complete(void)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xSemaphoreGiveFromISR(i2c_dma_sem, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void bno085_i2c_error(void)
{
    i2c_error_flag = 1u;

    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xSemaphoreGiveFromISR(i2c_dma_sem, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}
