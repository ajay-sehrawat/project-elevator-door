/**
 * @file config.h
 * @brief System-wide compile-time constants with documented safety rationale.
 *
 * MISRA Rule 5.4 equivalent: Every numeric literal in the application is
 * replaced by a named constant defined here.  The safety rationale for each
 * timing value is documented inline.
 */

#ifndef CONFIG_H
#define CONFIG_H

#include <stdint.h>

/* ---------------------------------------------------------------------------
 * FreeRTOS Task Priorities
 * Higher numeric value = higher priority under FreeRTOS.
 * Safety tasks must always pre-empt control/comms tasks.
 * --------------------------------------------------------------------------- */

/** @brief Safety Monitor — must pre-empt all other tasks to enforce SR-1..SR-7. */
#define PRIORITY_SAFETY_TASK        5u

/** @brief Event Dispatcher — routes payloads before FSM consumes them. */
#define PRIORITY_DISPATCHER_TASK    4u

/** @brief Control/FSM Task — same level as Dispatcher; priority within Core 1. */
#define PRIORITY_CONTROL_TASK       4u

/** @brief HAL Input / UART RX — must not starve but yields to safety tasks. */
#define PRIORITY_HAL_INPUT_TASK     3u

/** @brief UART TX / transmit — backpressure handler; lowest comms priority. */
#define PRIORITY_UART_TX_TASK       2u

/** @brief Logger — never blocks safety or control tasks. */
#define PRIORITY_LOGGER_TASK        1u

/* ---------------------------------------------------------------------------
 * FreeRTOS Core Assignments (Dual-Core ESP32)
 * Core 0: HAL / Comms / Logging — peripheral-facing.
 * Core 1: Safety / Dispatcher / FSM — safety-critical, time-deterministic.
 * --------------------------------------------------------------------------- */
#define CORE_COMMS      0
#define CORE_SAFETY     1

/* ---------------------------------------------------------------------------
 * FreeRTOS Stack Sizes (in WORDS, not bytes)
 * Over-provisioned by 40 % to allow headroom detected by uxTaskGetStackHighWaterMark.
 * --------------------------------------------------------------------------- */
#define STACK_SAFETY_TASK       3072u
#define STACK_DISPATCHER_TASK   3072u
#define STACK_CONTROL_TASK      4096u  /* FSM + NVS calls need more stack */
#define STACK_HAL_INPUT_TASK    3072u
#define STACK_UART_TX_TASK      2048u
#define STACK_LOGGER_TASK       2048u

/* ---------------------------------------------------------------------------
 * Queue Depths
 * Sized to absorb burst traffic without dropping safety events.
 * --------------------------------------------------------------------------- */

/** @brief Central dispatcher queue. Absorbs sensor burst + command overlap. */
#define DISPATCHER_QUEUE_DEPTH  16u

/** @brief UART TX queue. Backpressure drops oldest telemetry, never crashes. */
#define UART_TX_QUEUE_DEPTH     8u

/** @brief Logger queue — lossy; logging must never block safety path. */
#define LOGGER_QUEUE_DEPTH      16u

/* ---------------------------------------------------------------------------
 * Communication & Timing
 * --------------------------------------------------------------------------- */

/**
 * @brief Communication timeout (ms). SR-5: If no valid frame arrives within
 * this window, halt actuators and enter FAULT.
 * Rationale: 100 ms is consistent with IEC 62280 safety integrity level
 * recommendations for low-speed safety networks.
 */
#define COMM_TIMEOUT_MS         100u

/**
 * @brief State report interval (ms). FR-5: Transmit current state at ≤ 200 ms.
 * Nominal 100 ms provides 2× margin.
 */
#define STATE_REPORT_INTERVAL_MS 100u

/**
 * @brief Obstruction reaction deadline (ms) over the UART simulation path.
 * SR-2: Hardware deadline is ≤ 20 ms; UART simulation path budget is ≤ 100 ms.
 */
#define OBSTRUCTION_REACT_MS    100u

/**
 * @brief Motor stall timeout (ms). SR-7: If the door has not reached its
 * physical limit sensor within this window, assume mechanical stall.
 * Derived from: nominal travel time 3 s + 20 % clock-drift margin = 3 600 ms.
 */
#define MOTOR_STALL_TIMEOUT_MS  3600u

/**
 * @brief Clock-drift compensation factor (%).
 * Applied to all timer-based deadlines to absorb ESP32 RTC drift under load.
 */
#define CLOCK_DRIFT_MARGIN_PCT  20u

/* ---------------------------------------------------------------------------
 * Debounce
 * --------------------------------------------------------------------------- */

/**
 * @brief Sensor debounce window (ms). HAL must not dispatch an event until
 * the sensor signal has been stable for this period.
 * Rationale: Mechanical contact bounce on limit switches can last up to 10 ms.
 * 20 ms provides 2× margin (SR-3).
 */
#define SENSOR_DEBOUNCE_MS      20u

/* ---------------------------------------------------------------------------
 * UART Configuration
 * --------------------------------------------------------------------------- */
#define UART_PORT_NUM           UART_NUM_0
#define UART_BAUD_RATE          115200
#define UART_RX_BUF_SIZE        512u
#define UART_TX_BUF_SIZE        512u

/** @brief Maximum length of a single UART frame including null terminator. */
#define UART_FRAME_MAX_LEN      64u

/* ---------------------------------------------------------------------------
 * NVS Namespace & Keys
 * --------------------------------------------------------------------------- */
#define NVS_NAMESPACE           "elev_door"
#define NVS_KEY_FAULT_STATE     "fault_state"
#define NVS_KEY_FAULT_CODE      "fault_code"

/* ---------------------------------------------------------------------------
 * TWDT (Task Watchdog Timer)
 * --------------------------------------------------------------------------- */

/**
 * @brief TWDT feed interval (ms). Safety task must feed the watchdog at this
 * cadence. Must be less than CONFIG_ESP_TASK_WDT_TIMEOUT_S * 1000 / 2.
 */
#define TWDT_FEED_INTERVAL_MS   1000u

/* ---------------------------------------------------------------------------
 * CRC-8 polynomial (Dallas/Maxim).
 * NFR-5: Every UART frame is CRC-8 validated before processing.
 * --------------------------------------------------------------------------- */
#define CRC8_POLYNOMIAL         0x31u
#define CRC8_INITIAL            0xFFu

/* ---------------------------------------------------------------------------
 * Misc
 * --------------------------------------------------------------------------- */

/** @brief Maximum log message length for the logger queue. */
#define LOG_MSG_MAX_LEN         128u

/** @brief Stack high-water-mark report interval (ms). */
#define STACK_REPORT_INTERVAL_MS 5000u

#endif /* CONFIG_H */