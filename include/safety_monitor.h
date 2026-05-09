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