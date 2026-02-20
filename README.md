# STM32 Embedded Firmware — BFMC 2026 Autonomous Vehicle Controller

**Node Name:** `odyssix_mcu`
**Target MCU:** STM32F401RE (ARM Cortex-M4, 84 MHz)
**RTOS:** FreeRTOS (detected — 5 concurrent tasks)
**Communication:** MCP2515 CAN @ 500 kbps
**Source Language:** C++

---

## Table of Contents

1. [Executive Summary](#1-executive-summary)
2. [Hardware Overview](#2-hardware-overview)
3. [Peripheral Configuration](#3-peripheral-configuration)
4. [Firmware Architecture](#4-firmware-architecture)
5. [micro-ROS Integration](#5-micro-ros-integration)
6. [Control Flow](#6-control-flow)
7. [Safety System](#7-safety-system)
8. [Real-Time Timing Analysis](#8-real-time-timing-analysis)
9. [Memory Usage](#9-memory-usage)
10. [Build Instructions](#10-build-instructions)
11. [Known Limitations](#11-known-limitations)
12. [Design Decisions](#12-design-decisions)

---

## 1. Executive Summary

This firmware runs on an STM32F401RE microcontroller and serves as the deterministic real-time hardware abstraction and safety enforcement layer for a BFMC 2026 autonomous vehicle. The Jetson companion computer handles perception, planning, and localization and sends drive commands over a 500 kbps CAN bus. The STM32 receives these commands, applies conditioning and safety constraints, and drives actuator hardware directly via hardware PWM timers.

**RTOS:** FreeRTOS is confirmed present. Five tasks are created at startup with assigned stack sizes and FreeRTOS priority levels. The system is not bare-metal.

**Determinism guarantees:**

- The ControlTask runs at exactly 500 Hz (2 ms period) using `osDelayUntil` against a FreeRTOS tick reference. Jitter target is documented as `< 50 µs`.
- All safety-critical timestamps use TIM5, a 32-bit hardware counter running at 1 MHz (1 µs resolution), never `HAL_GetTick()`.
- PWM register writes occur exclusively inside ControlTask. No ISR writes directly to CCR registers.
- The IWDG watchdog is refreshed exclusively inside ControlTask, enforcing that a ControlTask stall triggers a hardware reset within approximately 1 second.

**Real-time characteristics:**

- Control loop: 500 Hz, 2 ms period
- IMU sampling: 100 Hz, event-driven on hardware interrupt from BNO085
- Ultrasonic sampling: 20 Hz, 50 ms period
- CAN RX dispatch: event-driven on MCP2515 INT pin

**micro-ROS is conditionally compiled.** The macro `MICRO_ROS_ENABLED` gates all micro-ROS code. When disabled, CAN remains the sole communication channel. When enabled, a custom XRCE-DDS transport over CAN (separate ID space from sensor traffic) carries the ROS2 graph connection.

---

## 2. Hardware Overview

### 2.1 Target Board and MCU

| Parameter | Value | Source |
|-----------|-------|--------|
| MCU | STM32F401RE | Derived from SYSCLK=84 MHz (F401RE maximum) and HSE=8 MHz in `system_config.hpp` |
| Core | ARM Cortex-M4 with FPU | F401RE silicon |
| SYSCLK | 84 MHz | `system_config.hpp`: `SYSTEM_CLOCK_MHZ = 84` |
| HSE | 8 MHz | `system_config.hpp`: `HSE_VALUE_MHZ = 8` |
| APB1 bus | 42 MHz | `system_config.hpp`: `APB1_CLOCK_MHZ = 42` |
| APB2 bus | 84 MHz | `system_config.hpp`: `APB2_CLOCK_MHZ = 84` |
| Flash | 512 KB | STM32F401RE datasheet |
| RAM | 96 KB | STM32F401RE datasheet |
| FPU | Enabled | Cortex-M4 FPU present; micro-ROS library built for ARMHF |

**Note:** The `.ioc` file was not present in the analyzed directory. Board identity and PLL configuration are derived from constants in `system_config.hpp`, not from CubeMX project settings.

### 2.2 Clock Derivation

```
HSE = 8 MHz
PLL input = 8 MHz
SYSCLK = 84 MHz   (PLL configured by CubeMX — exact M/N/P not determinable from source)

APB1 prescaler = /2  → APB1 = 42 MHz
APB2 prescaler = /1  → APB2 = 84 MHz

APB1 timer clock = APB1 × 2 = 84 MHz   (when APB1 prescaler != 1)
APB2 timer clock = APB2 × 1 = 84 MHz

TIM4 (APB1 timer): timer clock = 84 MHz
TIM5 (APB1 timer): timer clock = 84 MHz
SPI1  (APB2):      peripheral clock = 84 MHz
```

### 2.3 Pin Assignment Table

Extracted from `system_config.hpp`, driver headers, and peripheral configuration constants. The `.ioc` file was not present; AF assignments are inferred from STM32F401 datasheet correlation with the used peripheral instances.

| Pin | Peripheral | Alt Function | Direction | Connected Hardware | Purpose |
|-----|-----------|-------------|-----------|-------------------|---------|
| PB6 | TIM4_CH1 | AF2 | Output | QUICRUN FUSION SE ESC | Drive PWM, 50 Hz |
| PB7 | TIM4_CH2 | AF2 | Output | RC Steering Servo | Steering PWM, 50 Hz |
| PA4 | SPI1_NSS (GPIO) | GPIO_Output | Output | MCP2515 /CS | SPI chip select, active low |
| PA5 | SPI1_SCK | AF5 | Output | MCP2515 SCK | SPI clock |
| PA6 | SPI1_MISO | AF5 | Input | MCP2515 SDO | SPI data from MCP2515 |
| PA7 | SPI1_MOSI | AF5 | Output | MCP2515 SDI | SPI data to MCP2515 |
| PB0 | EXTI0 | GPIO_Input | Input | MCP2515 INT | CAN RX interrupt, falling edge |
| PB8 | I2C1_SCL | AF4 | Output (OD) | BNO085 SCL | I2C clock, 400 kHz |
| PB9 | I2C1_SDA | AF4 | Bidir (OD) | BNO085 SDA | I2C data |
| PC4 | EXTI4 | GPIO_Input | Input | BNO085 INT | IMU data-ready interrupt |
| PA2 | USART2_TX | AF7 | Output | URM37 UART RX | Ultrasonic command TX |
| PA3 | USART2_RX | AF7 | Input | URM37 UART TX | Ultrasonic response RX |

**Note:** Pin assignments for TIM5 (internal timestamp counter) involve no external GPIO. TIM5 counter is read directly from `TIM5->CNT` in software.

---

## 3. Peripheral Configuration

### 3.1 TIM4 — PWM Output (ESC + Servo)

| Parameter | Value | Formula |
|-----------|-------|---------|
| Timer instance | TIM4 | APB1 timer |
| Timer clock | 84 MHz | APB1 × 2 (APB1 prescaler = /2) |
| Prescaler (PSC) | 83 | Reload value; effective divisor = PSC + 1 = 84 |
| Timer tick frequency | 1 MHz | 84 MHz / 84 = 1,000,000 Hz |
| Auto-Reload Register (ARR) | 19999 | Reload value |
| PWM period | 20,000 µs | (ARR + 1) / tick_freq = 20000 / 1,000,000 |
| PWM frequency | 50 Hz | 1 / 20 ms |
| Resolution | 1 µs | 1 / tick_freq |
| CH1 (PB6) purpose | ESC drive signal | |
| CH2 (PB7) purpose | Steering servo | |
| ESC neutral CCR1 | 1485 | 1485 µs pulse width |
| ESC forward start CCR1 | 1443 | 1443 µs (ESC-specific deadband) |
| ESC reverse start CCR1 | 1527 | 1527 µs (ESC-specific deadband) |
| ESC full range CCR1 | [1000, 2000] | Hardware clamp enforced in `pwm_driver_set_esc()` |
| Servo neutral CCR2 | 1500 | 1500 µs pulse width |
| Servo full range CCR2 | [1000, 2000] | Hardware clamp enforced in `pwm_driver_set_servo()` |
| Mode | PWM Mode 1 | Output high while counter < CCR |
| CCR write method | Direct `TIM4->CCR1/2` | No HAL wrapper used |

```
PWM frequency formula:
f_PWM = f_timer / ((PSC + 1) × (ARR + 1))
f_PWM = 84,000,000 / (84 × 20,000) = 50 Hz
```

### 3.2 TIM5 — Microsecond Timestamp Counter

| Parameter | Value | Formula |
|-----------|-------|---------|
| Timer instance | TIM5 | APB1 timer, 32-bit |
| Timer clock | 84 MHz | APB1 × 2 |
| Prescaler (PSC) | 83 | Effective divisor = 84 |
| Tick frequency | 1 MHz | 84 MHz / 84 = 1 µs per tick |
| Auto-Reload Register (ARR) | 0xFFFFFFFF | Full 32-bit range |
| Wrap period | ~4295 seconds | ~71.6 minutes |
| Read method | `TIM5->CNT` direct | Single 32-bit read, no synchronization needed |
| Wrap-safe arithmetic | Required | All elapsed-time calculations use unsigned subtraction |
| Started by | `timestamp_init()` → `HAL_TIM_Base_Start()` | No interrupt used |

### 3.3 USART2 — URM37 Ultrasonic Sensor

| Parameter | Value |
|-----------|-------|
| Baud rate | 9600 |
| Word length | 8 bits |
| Parity | None |
| Stop bits | Not determinable from source (HAL default = 1) |
| Flow control | None |
| RX mode | DMA (4-byte circular receive into fixed buffer) |
| TX mode | Blocking `HAL_UART_Transmit()`, 20 ms timeout |
| DMA channel | Not determinable from source (requires .ioc) |
| Interrupt source | `HAL_UART_RxCpltCallback` → `urm37_dma_rx_complete_isr()` |

### 3.4 I2C1 — BNO085 IMU

| Parameter | Value |
|-----------|-------|
| Clock speed | 400 kHz (Fast Mode) |
| Addressing | 7-bit |
| BNO085 address | 0x4A |
| Transfer mode | DMA (HAL_I2C_Master_Receive_DMA) |
| Read sequence | 4-byte SHTP header, then full payload (two separate DMA transactions) |
| Interrupt source | `HAL_I2C_MasterRxCpltCallback` → `bno085_i2c_rx_complete_isr()` |
| Error callback | `HAL_I2C_ErrorCallback` → `bno085_i2c_error_isr()` |
| Pull-up requirement | External (not determinable from source) |

### 3.5 SPI1 — MCP2515 CAN Controller

| Parameter | Value | Formula |
|-----------|-------|---------|
| SPI instance | SPI1 | APB2 peripheral |
| APB2 clock | 84 MHz | |
| Prescaler | /16 | Set in CubeMX |
| SPI clock | 5.25 MHz | 84 MHz / 16 |
| Mode | Mode 0 (CPOL=0, CPHA=0) | MCP2515 requirement |
| Data frame | 8-bit | |
| DMA streams | DMA2_Stream0 (RX), DMA2_Stream3 (TX) | `mcp2515.hpp` |
| CS pin | PA4, GPIO output | Direct BSRR write for speed |
| INT pin | PB0, EXTI0 falling edge | `mcp2515_notify_isr()` |
| Synchronization | `spi_dma_sem` (binary semaphore) + `spi_mutex` (mutex) | |

### 3.6 MCP2515 CAN Bit Timing (500 kbps, 8 MHz oscillator)

| Register | Value | Decoded |
|----------|-------|---------|
| CNF1 | 0x00 | BRP=0 (effective=1), SJW=1 TQ |
| CNF2 | 0x91 | BTLMODE=1, SAM=0, PHSEG1=3 TQ, PRSEG=2 TQ |
| CNF3 | 0x01 | PHSEG2=2 TQ |

```
Bit timing formula:
TQ = 2 × BRP / Fosc = 2 × 1 / 8,000,000 = 250 ns
Bit time = (1 + PRSEG + PHSEG1 + PHSEG2) × TQ
         = (1 + 2 + 3 + 2) × 250 ns = 8 × 250 ns = 2000 ns
Bit rate = 1 / 2000 ns = 500,000 bps = 500 kbps ✓
Sample point = (1 + PRSEG + PHSEG1) / total TQ = 6/8 = 75%
```

### 3.7 CAN RX Filter Configuration

| Buffer | Mask | Filter 0 | Filter 1 | Accepts |
|--------|------|----------|----------|---------|
| RXB0 | 0x6FF | 0x201 | 0x301 | 0x201 (/control/cmd), 0x301 (/clock_sync) |
| RXB1 | 0x7FF | 0x401 | — | 0x401 exclusively (/safety/emergency_stop) |

Rollover enabled: if RXB0 is full, overflow frames are stored in RXB1.

### 3.8 Independent Watchdog (IWDG)

| Parameter | Value | Formula |
|-----------|-------|---------|
| Clock source | LSI (~32 kHz) | Independent of main clocks |
| Prescaler | 128 | IWDG_PRESCALER_128 |
| Reload value | 249 | IWDG_RELOAD_VAL |
| Timeout | ~1.0 second | (249 + 1) × 128 / 32,000 = 1.0 s |
| Refresh location | ControlTask only | `watchdog_kick()` at bottom of 500 Hz loop |
| Stop-on-debug | Not determinable from source | DBGMCU_APB1_FZ not confirmed |

Once `watchdog_init()` is called, the IWDG cannot be disabled by software.

### 3.9 NVIC Interrupt Priorities

| Interrupt | Source | Callback | Notes |
|-----------|--------|----------|-------|
| EXTI0 | PB0 — MCP2515 INT | `mcp2515_notify_isr()` | Wakes CAN RX task via semaphore |
| EXTI4 | PC4 — BNO085 INT | `bno085_notify_int_isr()` | Notifies IMU task |
| DMA2_Stream0 | SPI1 RX complete | `mcp2515_spi_dma_complete()` | Signals SPI semaphore |
| DMA2_Stream3 | SPI1 TX complete | `mcp2515_spi_dma_complete()` | Same semaphore |
| I2C1_EV | I2C RX complete | `bno085_i2c_rx_complete_isr()` | Two-stage (header, payload) |
| I2C1_ER | I2C error | `bno085_i2c_error_isr()` | Abort and reset |
| USART2 | UART RX/error | `urm37_dma_rx_complete_isr()` | 4-byte DMA complete |

Exact NVIC priority values are not determinable from source (requires `.ioc` or `stm32f4xx_hal_msp.c`, neither of which was present).

---

## 4. Firmware Architecture

### 4.1 Folder Structure

```
Core/
├── app/
│   ├── main.cpp              Entry point: module init, task creation, HAL callbacks
│   ├── control_task.cpp      500 Hz deterministic control loop
│   ├── imu_task.cpp          Event-driven IMU acquisition and publish
│   ├── micro_ros_task.cpp    micro-ROS executor spin loop (conditional)
│   └── ultrasonic_task.cpp   20 Hz ultrasonic measurement and CAN publish
│
├── Inc/
│   ├── config/
│   │   ├── system_config.hpp All hardware constants (frozen for BFMC 2026)
│   │   └── can_ids.hpp       CAN ID map (frozen for BFMC 2026)
│   ├── control/
│   │   ├── control_buffer.hpp Double-buffered lock-free command store
│   │   ├── pwm_driver.hpp    TIM4 CCR abstraction
│   │   └── safety_manager.hpp Safety FSM, timeout tracking
│   ├── drivers/
│   │   ├── bno085_driver.hpp BNO085 SHTP/SH-2 I2C DMA driver
│   │   ├── can_driver.hpp    CAN frame packing and RX dispatch
│   │   ├── mcp2515.hpp       MCP2515 SPI DMA driver
│   │   └── urm37_driver.hpp  URM37 UART DMA driver
│   ├── ros/
│   │   ├── micro_ros_node.hpp Node, executor, pub/sub handles
│   │   └── micro_ros_transport.hpp CAN-based XRCE-DDS transport
│   └── system/
│       ├── atomic_guard.hpp  FreeRTOS critical section RAII
│       ├── timestamp.hpp     TIM5 1 µs counter
│       └── watchdog.hpp      IWDG driver
│
└── Src/
    ├── control/
    │   ├── pwm_driver.cpp
    │   └── safety_manager.cpp
    ├── drivers/
    │   ├── bno085_driver.cpp
    │   ├── can_driver.cpp
    │   ├── mcp2515.cpp       ~1542 lines, full MCP2515 protocol implementation
    │   └── urm37_driver.cpp
    ├── ros/
    │   ├── micro_ros_node.cpp
    │   └── micro_ros_transport.cpp
    └── system/
        ├── atomic_guard.cpp
        ├── timestamp.cpp
        └── watchdog.cpp
```

### 4.2 Layered Architecture

```
+----------------------------------------------------------+
|                    Application Layer                     |
|  control_task.cpp  imu_task.cpp  ultrasonic_task.cpp     |
|  micro_ros_task.cpp  main.cpp                            |
+----------------------------------------------------------+
|                    Middleware Layer                      |
|  micro_ros_node.cpp   micro_ros_transport.cpp            |
|  can_driver.cpp  (frame dispatch and packing)            |
+----------------------------------------------------------+
|                    Driver Layer                          |
|  mcp2515.cpp  bno085_driver.cpp  urm37_driver.cpp        |
|  pwm_driver.cpp  safety_manager.cpp                     |
+----------------------------------------------------------+
|                    System Layer                          |
|  timestamp.cpp  watchdog.cpp  atomic_guard.cpp           |
+----------------------------------------------------------+
|                    HAL Layer                             |
|  STM32F4xx_HAL_Driver (SPI, I2C, UART, TIM, IWDG)       |
+----------------------------------------------------------+
|                    Hardware                              |
|  STM32F401RE, MCP2515, BNO085, URM37, TIM4, TIM5        |
+----------------------------------------------------------+
```

### 4.3 FreeRTOS Task Table

| Task | Priority (FreeRTOS) | Stack (words) | Stack (bytes) | Scheduling | Period |
|------|--------------------|----|---|-----------|--------|
| CAN RX | 4 — osPriorityHigh | 512 | 2048 | Event-driven on PB0 EXTI0 | On demand |
| Control | 3 — osPriorityAboveNormal | 512 | 2048 | `osDelayUntil` | 2 ms (500 Hz) |
| IMU | 3 — osPriorityAboveNormal | 1024 | 4096 | Event-driven on PC4 EXTI4 | On demand |
| Ultrasonic | 2 — osPriorityNormal | 512 | 2048 | `osDelay` | 50 ms (20 Hz) |
| micro-ROS | 2 — osPriorityNormal | 1024 | 4096 | Spin loop | Continuous |

### 4.4 DMA Usage Summary

| Stream | Peripheral | Direction | Used By |
|--------|-----------|-----------|---------|
| DMA2_Stream0 | SPI1 | RX | MCP2515 SPI receive |
| DMA2_Stream3 | SPI1 | TX | MCP2515 SPI transmit |
| I2C1 DMA (stream not determinable) | I2C1 | RX | BNO085 header and payload reads |
| USART2 DMA (stream not determinable) | USART2 | RX | URM37 4-byte response |

All DMA transfers are interrupt-notified. No polling of DMA complete flags occurs at the application level.

---

## 5. micro-ROS Integration

**Compile guard:** All micro-ROS code is wrapped in `#ifdef MICRO_ROS_ENABLED`. When this macro is undefined, the `micro_ros_task.cpp` falls back to an idle loop and no micro-ROS entities are created.

### 5.1 Transport

| Property | Value |
|----------|-------|
| Transport type |
| TX CAN ID | 0x200 (STM32 → Jetson agent) |
| RX CAN ID | 0x100 (Jetson agent → STM32) |
| Frame size | 8 bytes per CAN frame |
| Fragmentation | `transport_write()` splits byte stream into 8-byte CAN frames |
| Reassembly | `transport_read()` pops from 32-entry ring buffer, reassembles stream |
| Ring buffer size | 32 entries × 8 bytes = 256 bytes maximum in-flight RX |
| Thread safety | `__DMB()` memory barriers on head/tail pointer updates |
| Registered via | `rmw_uros_set_custom_transport()` |

**Note:** This CAN ID space (0x100/0x200) is entirely separate from the sensor CAN traffic (0x101–0x7FF / 0x201–0x401). Both share the same MCP2515 hardware. The MCP2515 RX filters as configured do **not** accept 0x100. The micro-ROS RX frames at ID 0x100 would be dropped by hardware filters in the current configuration unless additional filter slots are configured. This is a potential integration issue (see Section 11).

### 5.2 Node Configuration

| Property | Value |
|----------|-------|
| Node name | `odyssix_mcu` |
| Namespace | Default (empty) |
| Executor type | `rclc_executor` |
| Executor handles | 2 (subscription handles only) |
| Memory model | Static — no dynamic allocation after `ros_node_init()` |
| Allocator | Default RCL allocator (static pool) |

### 5.3 Subscriptions

| Topic | Message Type | Fields Used | Callback |
|-------|-------------|-------------|----------|
| `/control/cmd` | `geometry_msgs/msg/Twist` | `linear.x` (speed), `angular.z` (steering) | `cmd_vel_callback()` → `control_buffer.write()` + `safety_notify_control_received()` |
| `/safety/emergency_stop` | `std_msgs/msg/Bool` | `data` | `estop_callback()` → `safety_set_emergency()` or `safety_clear_source()` |

### 5.4 Publishers

| Topic | Message Type | Publish Trigger | Data Source |
|-------|-------------|-----------------|-------------|
| `/imu/data` | `sensor_msgs/msg/Imu` | Called from `imu_task.cpp` | BNO085 Q14 quaternion converted to double (`/ 16384.0`) |
| `/ultrasonic/front` | `sensor_msgs/msg/Range` | Called from `ultrasonic_task.cpp` | URM37 cm → meters (`/ 100.0`); `-1.0` if invalid |

**Range message constants (set once at init):**

| Field | Value |
|-------|-------|
| `radiation_type` | ULTRASOUND |
| `field_of_view` | 0.26 rad (~14.9°) |
| `min_range` | 0.05 m |
| `max_range` | 5.00 m |

### 5.5 Memory and Reconnection

Static message objects (`cmd_vel_msg`, `estop_msg`, `imu_msg`, `range_msg`) are declared at file scope. Zero heap allocation occurs after node initialization. Session timeout handling and reconnection are not explicitly implemented in `micro_ros_task.cpp`; if `ros_node_init()` fails, the task falls into an idle loop. No retry loop or session re-establishment is implemented.

---

## 6. Control Flow

### 6.1 Boot Sequence

```
Reset
  |
  v
HAL_Init()               -- SysTick at 1 kHz, NVIC priority grouping
SystemClock_Config()     -- HSE → PLL → 84 MHz SYSCLK
MX_GPIO_Init()
MX_SPI1_Init()
MX_I2C1_Init()
MX_USART2_UART_Init()
MX_TIM4_Init()           -- PWM timer (50 Hz)
MX_TIM5_Init()           -- Timestamp counter (1 MHz)
MX_IWDG_Init()           -- IWDG armed; must be kicked within ~1 s
  |
  v
app_modules_init()
  |-- timestamp_init()           -- Start TIM5 counter
  |-- control_buffer_init()      -- Zero both double-buffer slots
  |-- safety_init()              -- Clear all emergency sources
  |-- mcp2515_init()             -- Reset, configure, verify, enter normal mode
  |-- pwm_driver_start()         -- Set ESC=1485, Servo=1500; start TIM4 PWM
  |-- watchdog_init()            -- Arm IWDG (point of no return)
  |
  v
app_tasks_create()
  |-- Create CAN RX task  (priority 4)
  |-- Create Control task (priority 3)
  |-- Create IMU task     (priority 3)
  |-- Create Ultrasonic task (priority 2)
  |-- Create micro-ROS task  (priority 2) [if MICRO_ROS_ENABLED]
  |
  v
osKernelStart()          -- FreeRTOS scheduler begins; main() does not return
```

### 6.2 Main Loop Structure (ControlTask — 500 Hz)

```
ControlTask
  |
  v
osDelayUntil(&tick_ref, 2)        -- 2 ms period; tick_ref updated each cycle
  |
  v
safety_check_timeouts()           -- Update CONTROL_TIMEOUT and HEARTBEAT_TIMEOUT
  |
  v
[safety_is_emergency()?]
  |-- YES --> pwm_driver_safe_neutral()   -- ESC=1485, Servo=1500
  |           continue to next cycle
  |
  v
control_buffer.read(cmd)          -- Atomic read of latest command
  |
  v
ESC Conditioner
  |-- Map cmd.speed [-1.0, +1.0] to pulse_us
  |-- Apply asymmetric deadband:
  |     speed > 0: map [0,1] → [1443, 1000]
  |     speed < 0: map [0,-1] → [1527, 2000]
  |     |speed| < threshold: neutral=1485
  |-- Apply rate limiter: delta <= 10 µs/tick
  |-- Apply neutral crossing: 5-tick dwell if direction reversal detected
  |-- Apply timeout: if TIM5 elapsed since last valid cmd > 250 ms → neutral
  |
  v
Servo Conditioner
  |-- Linear map cmd.steering [-1.0, +1.0] → [1000, 2000]
  |-- Clamp to [1000, 2000]
  |
  v
pwm_driver_set_esc(esc_us)        -- TIM4->CCR1 = esc_us
pwm_driver_set_servo(servo_us)    -- TIM4->CCR2 = servo_us
  |
  v
Heartbeat (every 200 ms = every 100th tick at 500 Hz)
  |-- can_driver_send_heartbeat()  -- CAN ID 0x7FF: uptime + status
  |-- HAL_GPIO_TogglePin(LED)      -- 5 Hz LED blink
  |
  v
watchdog_kick()                   -- Refresh IWDG
```

### 6.3 CAN Command Reception to Actuator Write

```
MCP2515 INT (PB0) falls
  |
  v
EXTI0_IRQHandler
  └─> mcp2515_notify_isr()        -- Releases binary semaphore
        |
        v
CAN RX Task (priority 4, unblocks)
  └─> mcp2515_read_all_rx()
        |-- Read CANINTF
        |-- For each pending RX buffer:
        |     Read frame via SPI DMA → wait semaphore → call dispatch callback
        |
        v
      can_driver_dispatch_rx(frame)
        |
        [ID == 0x201] ──> handle_control_cmd()
                            |-- unpack_f32_le(payload+0) → speed
                            |-- unpack_f32_le(payload+4) → steering
                            |-- control_buffer.write(speed, steering, TIM5->CNT)
                            |-- safety_notify_control_received()
                            |
                            (Next ControlTask cycle reads buffer and writes CCR)
                            |
                            v
                          TIM4->CCR1 = esc_us
                          TIM4->CCR2 = servo_us
```

### 6.4 IMU Data Flow (BNO085 → CAN + micro-ROS)

```
BNO085 asserts INT (PC4 falls)
  |
  v
EXTI4_IRQHandler
  └─> bno085_notify_int_isr()      -- Task notification to IMU task

IMU Task wakes (priority 3)
  └─> bno085_read_rotation_vector()
        |-- HAL_I2C_Master_Receive_DMA (4-byte SHTP header)
        |-- Wait DMA semaphore
        |-- HAL_I2C_Master_Receive_DMA (full payload)
        |-- Wait DMA semaphore
        |-- Parse SHTP channel, length
        |-- Parse SH-2 rotation vector report (ID 0x05)
        |-- Extract I, J, K, Real (int16, Q14)
        |-- Timestamp = TIM5->CNT
        |
        v
      can_driver_send_imu()         -- CAN ID 0x101 (4x int16 Q14 + timestamp)
        |
        v
      [MICRO_ROS_ENABLED]
      ros_node_publish_imu()
        |-- Convert Q14 to double: val / 16384.0
        |-- Fill sensor_msgs/Imu orientation quaternion
        |-- rcl_publish(&imu_pub, &imu_msg)
```

---

## 7. Safety System

### 7.1 Safety Sources (Bitmask)

| Bit | Source Constant | Trigger Condition | Timeout Value |
|-----|----------------|------------------|---------------|
| Bit 0 | `SAFETY_SRC_ULTRASONIC` | Distance < 20 cm | N/A (immediate) |
| Bit 1 | `SAFETY_SRC_JETSON_ESTOP` | CAN 0x401 with `data=true` | N/A (explicit message) |
| Bit 2 | `SAFETY_SRC_CONTROL_TIMEOUT` | No CAN 0x201 for > 100 ms | 100 ms (TIM5-measured) |
| Bit 3 | `SAFETY_SRC_HEARTBEAT_TIMEOUT` | No CAN 0x301 for > 500 ms | 500 ms (TIM5-measured) |

### 7.2 State Machine

```
                        +------------------+
                        |   NORMAL STATE   |
                        |  (no bits set)   |
                        +------------------+
                         |       |       |       |
              Ultrasonic |  Jetson|  Ctrl |  HB   |
              < 20 cm    |  ESTOP | TIMED | TIMED |
                         v       v   OUT  v  OUT  v
                   +---------------------------------------+
                   |           EMERGENCY STATE             |
                   |  (one or more source bits latched)    |
                   |                                       |
                   |  pwm_driver_safe_neutral() called     |
                   |  every ControlTask cycle              |
                   +---------------------------------------+
                         |
              [source cleared]
              safety_clear_source(src)
                         |
              [all bits == 0] ───> NORMAL STATE
```

Emergency state is not latched permanently for individual sources. Each source can be cleared independently via `safety_clear_source()`. The JETSON_ESTOP source is cleared by receiving CAN 0x401 with `data=false`.

### 7.3 Timeout Evaluation

Timeout checks occur inside `safety_check_timeouts()`, called at the top of every ControlTask iteration (500 Hz). Timing uses TIM5 microseconds with wrap-safe unsigned subtraction:

```cpp
// Wrap-safe elapsed check (handles single overflow of uint32_t)
uint32_t elapsed_us = timestamp_now_us() - last_control_us;
if (elapsed_us > CONTROL_TIMEOUT_MS * 1000) {
    safety_set_emergency(SAFETY_SRC_CONTROL_TIMEOUT);
}
```

Timeouts do not engage until the first command is received. `first_control_received` and `first_heartbeat_received` flags gate timeout activation.

### 7.4 ESC-Level Timeout (Separate from Safety FSM)

In addition to the safety FSM, the ESC conditioner in `control_task.cpp` independently enforces a 250 ms command timeout using TIM5. If the last valid command timestamp is older than 250 ms, the conditioner forces ESC to neutral (1485 µs) regardless of the buffer contents. This is a second layer below the safety manager.

### 7.5 Watchdog Behavior

If ControlTask is blocked, starved by higher-priority tasks, or loops indefinitely, `watchdog_kick()` is not reached within 1 second and the IWDG asserts hardware reset. This is the last-resort recovery mechanism. Upon reset, all peripherals and tasks are re-initialized and the vehicle returns to neutral PWM within the boot sequence.

---

## 8. Real-Time Timing Analysis

### 8.1 Task Rates

| Task | Rate | Period | Scheduling Mechanism |
|------|------|--------|---------------------|
| ControlTask | 500 Hz | 2 ms | `osDelayUntil` (absolute, avoids drift) |
| IMUTask | 100 Hz | 10 ms | Event-driven on BNO085 hardware INT |
| UltrasonicTask | 20 Hz | 50 ms | `osDelay` (relative, can drift) |
| CAN RX Task | Asynchronous | N/A | Event-driven on MCP2515 INT |
| micro-ROS Task | Continuous | N/A | Spin loop with no sleep |

### 8.2 PWM Resolution and Update Latency

| Metric | Value |
|--------|-------|
| PWM timer resolution | 1 µs (1 CCR count = 1 µs) |
| PWM period | 20,000 µs (50 Hz) |
| CCR update-to-output latency | 0 – 20,000 µs (update takes effect at next timer overflow) |
| Steering resolution | 1000 steps across 1000–2000 µs range |
| ESC rate limiter | 10 µs/tick → 5,000 µs/second maximum rate of change |

### 8.3 End-to-End Command Latency Chain

```
Jetson publishes CAN frame (0x201)
  + MCP2515 CAN bus propagation      < 10 µs   (500 kbps, 8-byte frame)
  + MCP2515 INT assertion delay      < 5 µs
  + EXTI ISR → semaphore post        < 10 µs
  + CAN RX task preempts             < context switch time (~10 µs FreeRTOS)
  + SPI DMA read (frame retrieval)   ~15 µs    (12 SPI bytes at 5.25 MHz)
  + dispatch + buffer write          < 5 µs
  = Control buffer updated           < ~55 µs total from frame reception

  + ControlTask next wake cycle      0 – 2,000 µs  (worst case: just missed)
  + ESC conditioner computation      < 10 µs
  + TIM4 CCR register write          < 1 µs
  + TIM4 counter to next overflow    0 – 20,000 µs (PWM output latch)

Worst-case end-to-end: ~2,055 µs (command) + up to 20,000 µs (PWM output latch)
Best-case end-to-end: ~55 µs + 0 µs (CCR captured immediately before overflow)
```

### 8.4 IMU Latency Chain

```
BNO085 generates rotation vector
  + BNO085 INT assertion             < 1 µs
  + EXTI4 ISR → task notification    < 10 µs
  + IMU task unblocks                < context switch
  + I2C DMA read header (4 bytes)    ~20 µs at 400 kHz
  + DMA complete ISR → semaphore     < 5 µs
  + I2C DMA read payload (~16 bytes) ~50 µs at 400 kHz
  + Parse + timestamp                < 5 µs
  + CAN TX via MCP2515 SPI DMA       ~30 µs
Target total:                        < 5 ms (documented)
```

---

## 9. Memory Usage

### 9.1 Static Task Stack Budget

| Task | Stack Allocation | Purpose |
|------|-----------------|---------|
| CAN RX | 512 words (2,048 bytes) | MCP2515 SPI buffers on stack |
| Control | 512 words (2,048 bytes) | Conditioner state, local variables |
| IMU | 1,024 words (4,096 bytes) | SHTP buffer, I2C staging |
| Ultrasonic | 512 words (2,048 bytes) | UART receive buffer |
| micro-ROS | 1,024 words (4,096 bytes) | RCL serialization buffers |
| **Total tasks** | **3,584 words (14,336 bytes)** | |

FreeRTOS also allocates internal TCB and stack memory from `configTOTAL_HEAP_SIZE`. The FreeRTOSConfig.h file was not present in the analyzed source tree; heap size is not determinable.

### 9.2 Build Output

No build output (`.map` file, `arm-none-eabi-size` output) was present in the analyzed repository. The following sizes are not determinable from source: `.text`, `.data`, `.bss`, flash usage percentage, RAM usage percentage.

### 9.3 Linker Script

No `.ld` linker script was present in the analyzed directory. Default STM32F401RE memory map applies if CubeIDE-generated linker script is used:

| Region | Base Address | Size |
|--------|-------------|------|
| FLASH | 0x08000000 | 512 KB |
| RAM | 0x20000000 | 96 KB |

Stack size definition: Not determinable without linker script.

### 9.4 micro-ROS Static Allocations

The micro-ROS node uses the default RCL static allocator. Message objects are statically declared at file scope in `micro_ros_node.cpp`. The static memory pool size is not explicitly set (no `MICROROS_ALLOCATOR_STATIC_POOL_SIZE` constant found); the default allocator behavior applies. This means heap may be used during initialization if the static pool is not configured explicitly. Post-init heap usage is zero by design intent.

---

## 10. Build Instructions

### 10.1 Required Toolchain

| Tool | Version | Source |
|------|---------|--------|
| STM32CubeIDE | 1.14.x or later | st.com |
| arm-none-eabi-gcc | 12.3.rel1 (bundled with CubeIDE) | Bundled |
| OpenOCD | Bundled with CubeIDE | Bundled |
| ST-Link firmware | v3.x or later | STM32CubeProgrammer |

### 10.2 Build Procedure

```bash
# Open project in STM32CubeIDE:
# File > Open Projects from File System > select STM32_micro_ros_S/ directory

# Build:
# Project > Build Project (Ctrl+B)
# or from command line if Makefile present:
make -j$(nproc) all
```

If `MICRO_ROS_ENABLED` is required, add the define to the project preprocessor symbols:

```
Project > Properties > C/C++ Build > Settings > MCU GCC Compiler > Preprocessor
Add: MICRO_ROS_ENABLED
```

The `libmicroros.a` prebuilt static library must be present in the project. Its location and linker flags are not determinable from the analyzed source tree (no Makefile or `.cproject` was present).

### 10.3 Flash Procedure

```bash
# Via STM32CubeIDE:
# Run > Run (Ctrl+F11)  -- flash and run
# Run > Debug (F11)     -- flash and halt at main()

# Via command line:
STM32_Programmer_CLI -c port=SWD -w build/STM32_micro_ros_S.elf -v -rst

# Via OpenOCD:
openocd -f interface/stlink.cfg \
        -f target/stm32f4x.cfg \
        -c "program build/STM32_micro_ros_S.elf verify reset exit"
```

### 10.4 USB Device Permissions (Linux)

```bash
# Copy ST-Link udev rules:
sudo cp /usr/local/share/st-stm32cubeide-*/udev/*.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && sudo udevadm trigger
sudo usermod -aG dialout $USER
# Re-login required
```

### 10.5 Watchdog During Debug Sessions

The IWDG will reset the MCU during long debug sessions at breakpoints. To suppress:

Enable `DBG_IWDG_STOP` via STM32CubeIDE:
```
Debugger configuration > SWV > "Stop watchdog counters when halted"
```

Or in CubeMX: `System Core > SYS > Debug > IWDG stopped when core is halted`.

---

## 11. Known Limitations

The following issues are identified strictly from code analysis. No assumptions are made.

### 11.1 micro-ROS CAN Transport — RX Filter Gap

The MCP2515 RX hardware filters are configured to accept CAN IDs: 0x201, 0x301, and 0x401. The micro-ROS transport expects to receive frames at CAN ID 0x100. **CAN ID 0x100 is not included in any hardware filter.** Frames from the Jetson micro-ROS agent at 0x100 will be silently discarded by the MCP2515 hardware. The micro-ROS subscription will never receive data. This is a blocking defect for the micro-ROS transport when `MICRO_ROS_ENABLED` is defined.

### 11.2 micro-ROS Session Recovery

`micro_ros_task.cpp` falls into an idle loop if `ros_node_init()` fails. No retry logic or session re-establishment is implemented. If the Jetson agent is not running at STM32 boot time, or if the session drops, micro-ROS communication will not recover without an MCU reset. The IWDG does not trigger in this case because the micro-ROS task stall does not prevent ControlTask from running and refreshing the watchdog.

### 11.3 UltrasonicTask Drift

`ultrasonic_task.cpp` uses `osDelay(ULTRASONIC_PERIOD_MS)` (relative delay). The actual period is `ULTRASONIC_PERIOD_MS + task_execution_time`. At 20 Hz with a 50 ms target period and non-trivial UART transaction time, the effective rate will be below 20 Hz. `osDelayUntil` should be used for periodic tasks requiring isochronous sampling.

### 11.4 micro-ROS Ring Buffer Thread Safety

`micro_ros_transport.cpp` uses `__DMB()` memory barriers for ring buffer head/tail pointer updates, but no atomic compare-and-swap or critical section protects the push/pop operations. If both CAN RX task (which pushes to the ring buffer) and micro-ROS task (which pops from it) run on the same core under FreeRTOS cooperative scheduling, this may be safe. However, it is not explicitly documented and is a potential race condition under priority inversion scenarios.

### 11.5 Blocking UART TX in URM37 Driver

`urm37_driver.cpp` uses blocking `HAL_UART_Transmit()` with a 20 ms timeout to send the trigger command. This blocks the UltrasonicTask thread for up to 20 ms on a UART error condition. Since UltrasonicTask has normal FreeRTOS priority, this can delay lower-priority tasks.

### 11.6 Missing BNO085 Re-initialization on Persistent I2C Error

`bno085_i2c_error_isr()` aborts the I2C DMA transaction and notifies the task, but no re-initialization of the BNO085 sensor is implemented. If the I2C bus becomes stuck in an error state (e.g., SDA held low by sensor), the IMU task will continue to receive error notifications indefinitely without recovery. Manual power-cycle or bus recovery sequence (9 SCL pulses) is not implemented.

### 11.7 Stack Overflow Risk in Control and CAN RX Tasks

ControlTask and CAN RX Task are both allocated 512 words (2048 bytes). FreeRTOS stack overflow detection (configCHECK_FOR_STACK_OVERFLOW) is not confirmed enabled (FreeRTOSConfig.h not present). The MCP2515 driver operates with local SPI transfer buffers on stack within the CAN RX task. Exact stack usage is not determinable without a runtime high-water mark measurement.

---

## 12. Design Decisions

### 12.1 CAN Bus as Primary Communication Channel

The system uses MCP2515 (external SPI CAN controller) rather than a native CAN peripheral. The STM32F401RE does not include a bxCAN peripheral. The MCP2515 provides 500 kbps CAN 2.0B over SPI, enabling the same physical bus to carry: sensor data (IMU, ultrasonic), actuator commands (from Jetson), safety commands, heartbeat, and the micro-ROS XRCE-DDS stream. A single cable pair connects the STM32 to the Jetson, simplifying the physical interface.

### 12.2 FreeRTOS Over Bare-Metal Cooperative Scheduling

Five independent hardware interfaces (SPI/CAN, I2C/IMU, UART/Ultrasonic, PWM output, micro-ROS serial) each require independent blocking wait points (semaphores on DMA completion, task notifications on hardware interrupts). Implementing this without an RTOS would require a complex state machine spanning all peripherals. FreeRTOS permits each interface to block independently without blocking others.

### 12.3 TIM5 Instead of HAL_GetTick() for Safety Timing

`HAL_GetTick()` returns milliseconds derived from the SysTick 1 kHz interrupt. SysTick can be preempted or delayed by higher-priority interrupts, causing millisecond-level jitter. Safety timeout checks (100 ms control timeout, 500 ms heartbeat timeout, 250 ms ESC timeout) require sub-millisecond accuracy. TIM5 at 1 MHz provides 1 µs resolution with zero ISR dependency, since it is a hardware free-running counter read directly.

### 12.4 Double-Buffered Control Command Store

The control buffer uses a double-buffer pattern with atomic index swap rather than a FreeRTOS queue. A queue would require CAN RX task to block if the queue is full, or ControlTask to block waiting for a new message. The double-buffer pattern decouples the producer (CAN RX, priority 4) from the consumer (ControlTask, priority 3) with no blocking and no data loss: the consumer always reads the most recently written command.

### 12.5 ESC Asymmetric Deadband and Rate Limiter in Firmware

The QUICRUN FUSION SE ESC requires distinct neutral-crossing timing (5-tick dwell) and has separate forward/reverse activation thresholds (1443 µs forward, 1527 µs reverse, 1485 µs neutral). These parameters are enforced in firmware rather than relying on the Jetson to produce correctly conditioned PWM values. This ensures the ESC conditioning is consistent regardless of which Jetson node sends commands and cannot be misconfigured from the ROS side.

### 12.6 IMU Fixed-Point Q14 on CAN

The BNO085 provides quaternion components as Q14 fixed-point integers (int16, scale = 1/16384). Rather than converting to float before CAN transmission, the firmware packs the raw int16 values directly into CAN frames. This eliminates floating-point computation in the ISR context, preserves full sensor precision, and is endian-deterministic (little-endian pack/unpack). The Jetson or the micro-ROS node converts to double (dividing by 16384.0) only when needed.

---

*Document generated from static source analysis of `STM32_micro_ros_S/`. All values are extracted directly from source constants and implementation code. Values marked "Not determinable from current repository contents" require the `.ioc` project file, linker script, `FreeRTOSConfig.h`, or build output to resolve.*
