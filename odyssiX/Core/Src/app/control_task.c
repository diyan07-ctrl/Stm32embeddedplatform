/**
 * @file    control_task.c
 * @brief   Application hub: module init, task creation, 500 Hz control loop.
 *
 * This file provides:
 *   - app_modules_init():     Initialize all application modules
 *   - app_tasks_create():     Create all FreeRTOS tasks
 *   - app_default_task_run(): micro-ROS task entry (called from defaultTask)
 *
 * Control task pipeline (500 Hz, Priority 3):
 *   atomic_guard_read → safety check → esc_conditioner → servo_conditioner → pwm_driver
 *
 * Safety integration:
 *   - Emergency flag → force BLDC=1485, Servo=1500
 *   - Timeout checks every tick via safety_manager
 *   - CAN overflow monitoring
 *   - Watchdog refresh ONLY here
 *
 * HAL callbacks:
 *   HAL_GPIO_EXTI_Callback     → MCP2515 (PB0), BNO085 (PA1)
 *   HAL_SPI_TxRxCpltCallback   → MCP2515 SPI DMA
 *   HAL_I2C_MasterRxCpltCallback → BNO085 I2C DMA
 *   HAL_I2C_ErrorCallback      → BNO085 I2C error
 *   HAL_UART_RxCpltCallback    → URM37 UART DMA
 *   HAL_UART_ErrorCallback     → URM37 UART error
 */

#include "app/app_tasks.h"
#include "can/mcp2515.h"
#include "can/can_buffer.h"
#include "can/can_protocol.h"
#include "control/atomic_guard.h"
#include "control/esc_conditioner.h"
#include "control/servo_conditioner.h"
#include "control/pwm_driver.h"
#include "control/safety_manager.h"
#include "drivers/bno085_driver.h"
#include "drivers/urm37_driver.h"
#include "system/timestamp.h"
#include "system/watchdog.h"

#include "main.h"
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"
#include "stm32f4xx_hal.h"

/* ── External HAL handles ─────────────────────────────────────────────────── */
extern SPI_HandleTypeDef  hspi1;
extern TIM_HandleTypeDef  htim4;
extern I2C_HandleTypeDef  hi2c1;
extern UART_HandleTypeDef huart2;

/* ── Control task: 500 Hz ─────────────────────────────────────────────────── */
#define CONTROL_TASK_PERIOD_MS   2u   /* 500 Hz = 2 ms period */

static StaticTask_t      control_task_tcb;
static StackType_t       control_task_stack[512];
static osThreadId_t      control_task_handle;

static const osThreadAttr_t control_task_attr = {
    .name       = "controlTask",
    .cb_mem     = &control_task_tcb,
    .cb_size    = sizeof(control_task_tcb),
    .stack_mem  = &control_task_stack[0],
    .stack_size = sizeof(control_task_stack),
    .priority   = (osPriority_t) osPriorityHigh,
};

/* ── CAN RX task ──────────────────────────────────────────────────────────── */
static StaticTask_t      can_rx_task_tcb;
static StackType_t       can_rx_task_stack[512];
static osThreadId_t      can_rx_task_handle;

static const osThreadAttr_t can_rx_task_attr = {
    .name       = "canRxTask",
    .cb_mem     = &can_rx_task_tcb,
    .cb_size    = sizeof(can_rx_task_tcb),
    .stack_mem  = &can_rx_task_stack[0],
    .stack_size = sizeof(can_rx_task_stack),
    .priority   = (osPriority_t) osPriorityHigh,
};

/* ── Ultrasonic task ──────────────────────────────────────────────────────── */
static StaticTask_t      ultra_task_tcb;
static StackType_t       ultra_task_stack[512];
static osThreadId_t      ultra_task_handle;

static const osThreadAttr_t ultra_task_attr = {
    .name       = "ultraTask",
    .cb_mem     = &ultra_task_tcb,
    .cb_size    = sizeof(ultra_task_tcb),
    .stack_mem  = &ultra_task_stack[0],
    .stack_size = sizeof(ultra_task_stack),
    .priority   = (osPriority_t) osPriorityAboveNormal,
};

/* ── micro-ROS (CAN dispatcher) task ──────────────────────────────────────── */
static StaticTask_t      uros_task_tcb;
static StackType_t       uros_task_stack[512];
static osThreadId_t      uros_task_handle;

static const osThreadAttr_t uros_task_attr = {
    .name       = "urosTask",
    .cb_mem     = &uros_task_tcb,
    .cb_size    = sizeof(uros_task_tcb),
    .stack_mem  = &uros_task_stack[0],
    .stack_size = sizeof(uros_task_stack),
    .priority   = (osPriority_t) osPriorityAboveNormal,
};

/* ── External task entry points (defined in Core/Src/app) ─────────────────── */
extern void imu_task_entry(void *argument);
extern void ultrasonic_task_entry(void *argument);
extern void micro_ros_task_entry(void *argument);

/* ── Previous overflow count for safety monitoring ────────────────────────── */
static uint32_t prev_overflow_count;

/* ══════════════════════════════════════════════════════════════════════════
 *  500 Hz CONTROL LOOP
 * ══════════════════════════════════════════════════════════════════════════ */

static void control_task_entry(void *argument)
{
    (void)argument;

    TickType_t last_wake = xTaskGetTickCount();

    for (;;) {
        /* ── Check safety timeouts ──────────────────────────────────────── */
        safety_manager_check_timeouts();

        /* ── Check CAN overflow ─────────────────────────────────────────── */
        uint32_t ovf = mcp2515_get_overflow_count();
        if (ovf > prev_overflow_count) {
            safety_manager_set_emergency(SAFETY_SRC_CAN_OVERFLOW);
            prev_overflow_count = ovf;
        }

        /* ── Read control command from atomic double-buffer ─────────────── */
        control_cmd_t cmd;
        bool have_cmd = atomic_guard_read(&cmd);

        /* ── Apply safety override ──────────────────────────────────────── */
        if (safety_manager_is_emergency()) {
            /* Emergency: force safe neutral on BOTH outputs */
            pwm_driver_safe_neutral();
            esc_conditioner_force_neutral();
        } else if (have_cmd) {
            /* Normal operation: condition and apply */
            uint32_t now_us = timestamp_now_us();

            /* Convert TIM5 timestamp to milliseconds for ESC conditioner
             * (ESC timeout is specified in ms) */
            uint32_t now_ms = now_us / 1000u;

            uint16_t esc_pulse   = esc_conditioner_update(cmd.speed, now_ms);
            uint16_t servo_pulse = servo_conditioner_update(cmd.steering);

            pwm_driver_set_esc(esc_pulse);
            pwm_driver_set_servo(servo_pulse);
        } else {
            /* No valid command yet — hold at neutral */
            pwm_driver_safe_neutral();
            esc_conditioner_force_neutral();
        }

        /* ── Kick watchdog (ONLY here, nowhere else) ────────────────────── */
        watchdog_kick();

        /* ── Precise 2 ms period ────────────────────────────────────────── */
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(CONTROL_TASK_PERIOD_MS));
    }
}

/* ══════════════════════════════════════════════════════════════════════════
 *  MODULE INITIALIZATION
 * ══════════════════════════════════════════════════════════════════════════ */

void app_modules_init(void)
{
    /* ── TIM5 timestamp authority (must be first) ───────────────────────── */
    timestamp_init();

    /* ── Override SPI1 prescaler: Cube may set /2, MCP2515 needs ≤10 MHz ─ */
    hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16; /* 84/16 = 5.25 MHz */
    HAL_SPI_Init(&hspi1);

    /* ── Initialize MCP2515 CAN controller (with hardware RX filters) ──── */
    mcp2515_init();

    /* ── Initialize control modules ─────────────────────────────────────── */
    atomic_guard_init();
    esc_conditioner_init();
    safety_manager_init();

    /* ── Start PWM output (sets neutral values) ─────────────────────────── */
    pwm_driver_start();

    /* ── Initialize overflow tracking ───────────────────────────────────── */
    prev_overflow_count = 0u;

    /* ── Initialize hardware watchdog (~1 second timeout) ───────────────── */
    watchdog_init();
}

/* ══════════════════════════════════════════════════════════════════════════
 *  TASK CREATION
 * ══════════════════════════════════════════════════════════════════════════ */

void app_tasks_create(void)
{
    /* CAN RX task — highest priority, wakes on EXTI0 */
    can_rx_task_handle = osThreadNew(mcp2515_rx_task, NULL, &can_rx_task_attr);

    /* 500 Hz control task — highest priority (deterministic) */
    control_task_handle = osThreadNew(control_task_entry, NULL, &control_task_attr);

    /* Ultrasonic task — above normal priority */
    ultra_task_handle = osThreadNew(ultrasonic_task_entry, NULL, &ultra_task_attr);

    /* micro-ROS (CAN dispatcher) task — above normal priority */
    uros_task_handle = osThreadNew(micro_ros_task_entry, NULL, &uros_task_attr);
}

/* ══════════════════════════════════════════════════════════════════════════
 *  DEFAULT TASK ENTRY (CubeMX-generated defaultTask runs IMU)
 * ══════════════════════════════════════════════════════════════════════════ */

void app_default_task_run(void)
{
    /* The defaultTask now runs the IMU task.
     * This avoids creating yet another task and reuses the CubeMX-created one.
     * IMUTask (created by CubeMX with 1024-word stack) is available but
     * we route through defaultTask for cleaner integration. */

    /* Minimal idle loop — actual work is done in the dedicated tasks */
    for (;;) {
        osDelay(1000);
    }
}

/* ══════════════════════════════════════════════════════════════════════════
 *  HAL CALLBACKS — ISR Context, Notify Only
 * ══════════════════════════════════════════════════════════════════════════ */

/**
 * @brief  EXTI callback — dispatches GPIO interrupts.
 *
 * PIN_0 (PB0) = MCP2515 INT → CAN RX task
 * PIN_1 (PA1) = BNO085 INT  → IMU task
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == GPIO_PIN_0) {
        mcp2515_notify_isr();
    }
    if (GPIO_Pin == GPIO_PIN_1) {
        bno085_notify_int_isr();
    }
}

/**
 * @brief  SPI DMA transfer complete callback.
 */
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
    if (hspi->Instance == SPI1) {
        mcp2515_spi_dma_complete();
    }
}

/**
 * @brief  I2C master RX DMA complete callback.
 */
void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    if (hi2c->Instance == I2C1) {
        bno085_i2c_rx_complete();
    }
}

/**
 * @brief  I2C error callback.
 */
void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c)
{
    if (hi2c->Instance == I2C1) {
        bno085_i2c_error();
    }
}

/**
 * @brief  UART RX DMA complete callback.
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2) {
        urm37_uart_rx_complete();
    }
}

/**
 * @brief  UART error callback.
 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2) {
        urm37_uart_error();
    }
}
