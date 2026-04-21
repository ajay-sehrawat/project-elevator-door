/**
 * @file safety_monitor.h
 * @brief Safety Monitor task — highest-priority watchdog over FSM & sensors.
 *
 * Responsibilities:
 *  - Feeds the ESP32 Hardware Task Watchdog Timer (TWDT) every TWDT_FEED_INTERVAL_MS.
 *  - Independently enforces SR-1 (obstruction), SR-3 (SPOF), SR-5 (comms timeout).
 *  - Injects synthetic events (EVT_MOTOR_STALL, EVT_COMM_TIMEOUT, EVT_SPOF_DETECTED)
 *    into the Dispatcher queue so the FSM handles them uniformly.
 *  - Never directly mutates FSM state — it enqueues events. This maintains the
 *    single-writer principle for the FSM mutex.
 */

#ifndef SAFETY_MONITOR_H
#define SAFETY_MONITOR_H

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "esp_err.h"

/**
 * @brief Initialise Safety Monitor (resets internal timestamps).
 */
esp_err_t safety_monitor_init(void);

/**
 * @brief FreeRTOS task entry point for SafetyTask (Core 1, Priority 5).
 * @param pvParameters  Pointer to QueueHandle_t (the dispatcher queue).
 */
void safety_monitor_task(void *pvParameters);

/**
 * @brief Called by the HAL RX task whenever a valid frame arrives.
 * Resets the communication timeout watchdog (SR-5).
 * Re-entrant / interrupt-safe (uses atomic write).
 */
void safety_monitor_reset_comm_watchdog(void);

/**
 * @brief Notify Safety Monitor that a sensor reading arrived (for SPOF cross-check).
 * @param fully_open    Current state of the fully-open sensor.
 * @param fully_closed  Current state of the fully-closed sensor.
 */
void safety_monitor_update_sensors(uint8_t fully_open, uint8_t fully_closed);

#endif /* SAFETY_MONITOR_H */