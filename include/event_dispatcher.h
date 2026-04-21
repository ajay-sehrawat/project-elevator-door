/**
 * @file event_dispatcher.h
 * @brief Central event dispatcher — routes typed events to FSM and Safety Monitor.
 *
 * The Dispatcher runs on Core 1 at Priority 4.  It owns the single central
 * FreeRTOS Queue that is the system's sole inter-layer communication channel.
 * Both the HAL (Core 0) and internal producers push events here; the
 * Dispatcher routes them to the correct consumers.
 *
 * Queue overflow policy: if the queue is full, the overflow is logged and the
 * event is DROPPED with a warning (never blocking the producer — NFR-5).
 */

#ifndef EVENT_DISPATCHER_H
#define EVENT_DISPATCHER_H

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "system_event.h"
#include "esp_err.h"

/**
 * @brief Create the central dispatcher queue and return its handle.
 * @return Handle to the newly created queue, or NULL on OOM.
 */
QueueHandle_t dispatcher_create_queue(void);

/**
 * @brief FreeRTOS task entry point for DispatcherTask (Core 1, Priority 4).
 * @param pvParameters  Pointer to QueueHandle_t (the dispatcher queue itself).
 */
void dispatcher_task(void *pvParameters);

/**
 * @brief Enqueue an event from any task/ISR context.
 * Non-blocking (timeout = 0).  Returns pdFALSE on overflow (caller logs).
 */
BaseType_t dispatcher_post_event(QueueHandle_t q, const system_event_t *evt);

/**
 * @brief Enqueue from an ISR context.
 * Calls xQueueSendToBackFromISR; sets *pxHigherPriorityTaskWoken appropriately.
 */
BaseType_t dispatcher_post_event_from_isr(QueueHandle_t q,
                                          const system_event_t *evt,
                                          BaseType_t *pxHigherPriorityTaskWoken);

#endif /* EVENT_DISPATCHER_H */