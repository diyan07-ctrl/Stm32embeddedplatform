/**
 * @file    app_tasks.h
 * @brief   Application task creation and initialization.
 *
 * Creates:
 *   - CAN RX Task      (osPriorityHigh)        — reads MCP2515 on EXTI0
 *   - Control Task      (osPriorityHigh)        — 500 Hz ESC/servo loop
 *   - Ultrasonic Task   (osPriorityAboveNormal) — URM37 @ 20 Hz
 *   - micro-ROS Task    (osPriorityAboveNormal) — CAN command dispatcher
 *
 * IMUTask is created by CubeMX (osPriorityHigh, 1024-word static stack).
 * defaultTask is created by CubeMX (osPriorityNormal).
 */

#ifndef APP_TASKS_H
#define APP_TASKS_H

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief  Initialize all application modules.
 *
 * Call from main.c USER CODE BEGIN 2 (after peripheral init,
 * before osKernelInitialize).
 *
 * Initializes (in order):
 *   1. TIM5 timestamp authority
 *   2. MCP2515 CAN controller (with hardware RX filters)
 *   3. Atomic guard (double-buffer)
 *   4. ESC conditioner
 *   5. Safety manager
 *   6. PWM driver (starts TIM4)
 *   7. Hardware watchdog (IWDG, ~1s timeout)
 *
 * @pre  All MX_*_Init() functions have been called.
 */
void app_modules_init(void);

/**
 * @brief  Create application FreeRTOS tasks.
 *
 * Call from main.c USER CODE BEGIN RTOS_THREADS (after osKernelInitialize).
 *
 * Creates (all static allocation):
 *   - CAN RX task      (512 words stack)
 *   - Control task      (512 words stack)
 *   - Ultrasonic task   (512 words stack)
 *   - micro-ROS task    (512 words stack)
 */
void app_tasks_create(void);

/**
 * @brief  Entry point for defaultTask (CubeMX-created).
 *
 * Call from StartDefaultTask (USER CODE BEGIN 5).
 * Runs a minimal idle loop — actual work is in dedicated tasks.
 */
void app_default_task_run(void);

#ifdef __cplusplus
}
#endif

#endif /* APP_TASKS_H */
