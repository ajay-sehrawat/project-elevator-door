/**
 * @file event_logger.h
 * @brief Asynchronous event logger (NFR-4).
 *
 * All state transitions, commands, queue overflows, and fault events are
 * logged via this module.  Logging is fire-and-forget from the caller's
 * perspective — it posts to a low-priority logger queue so it never blocks
 * safety-critical tasks (NFR-6).
 */

#ifndef EVENT_LOGGER_H
#define EVENT_LOGGER_H

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "system_event.h"
#include "door_fsm.h"

/**
 * @brief Initialise logger queue.  Call before creating LoggerTask.
 * @return Handle to logger queue, or NULL on OOM.
 */
QueueHandle_t logger_init(void);

/**
 * @brief FreeRTOS task entry for LoggerTask (Core 0, Priority 1).
 * @param pvParameters Pointer to QueueHandle_t (logger queue).
 */
void logger_task(void *pvParameters);

/**
 * @brief Log a system event asynchronously (non-blocking).
 * Drops silently if logger queue is full (logger must not block safety path).
 */
void logger_log_event(QueueHandle_t logger_q, const system_event_t *evt);

/**
 * @brief Log a state transition.
 */
void logger_log_transition(QueueHandle_t logger_q,
                           fsm_state_t old_state,
                           fsm_state_t new_state,
                           const system_event_t *trigger);

/**
 * @brief Log a queue overflow (NFR-4, NFR-5).
 */
void logger_log_queue_overflow(QueueHandle_t logger_q, const char *queue_name);

#endif /* EVENT_LOGGER_H */