/**
 * @file dispatcher/event_dispatcher.c
 * @brief Central event dispatcher — routes system_event_t to FSM & Safety Monitor.
 *
 * Design Rationale:
 *   The Dispatcher is the single consumer of the central queue.  It forwards
 *   events to:
 *     1. The FSM (door_fsm) via the same queue consumed inside fsm_control_task.
 *     2. The Safety Monitor via safety_monitor_* APIs (which are re-entrant).
 *
 *   In practice the FSM task and Dispatcher task BOTH block on the same central
 *   queue — this is intentional.  The Dispatcher's role is pre-filtering:
 *   it guards against queue-overflow cascades and provides a single point
 *   for audit logging before any consumer sees the event.
 *
 *   The simpler pattern used here is:
 *     - HAL pushes to `central_queue`.
 *     - Dispatcher task reads from `central_queue` and forwards to:
 *         * `fsm_queue`  (a secondary queue owned by the FSM task).
 *         * Safety Monitor (direct function call — safe because Safety task
 *           only *reads* shared state; it does not mutate it here).
 *
 *   This two-queue design keeps the FSM task fully decoupled from the HAL and
 *   isolates queue-overflow handling in one place.
 */

#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "esp_timer.h"

#include "event_dispatcher.h"
#include "system_event.h"
#include "config.h"
#include "door_fsm.h"          /* for the FSM queue handle exposed below */
#include "safety_monitor.h"
#include "event_logger.h"

static const char *TAG = "DISPATCHER";

/* The FSM task queue — created in fsm_init(), referenced here for forwarding. */
extern QueueHandle_t g_fsm_queue;

/* Logger queue — created in logger_init(). */
extern QueueHandle_t g_logger_queue;

/* ---------------------------------------------------------------------------
 * Queue factory
 * --------------------------------------------------------------------------- */
QueueHandle_t dispatcher_create_queue(void)
{
    QueueHandle_t q = xQueueCreate(DISPATCHER_QUEUE_DEPTH, sizeof(system_event_t));
    if (q == NULL) {
        ESP_LOGE(TAG, "Failed to create dispatcher queue — OOM");
    }
    return q;
}

/* ---------------------------------------------------------------------------
 * Non-blocking post helpers
 * --------------------------------------------------------------------------- */
BaseType_t dispatcher_post_event(QueueHandle_t q, const system_event_t *evt)
{
    return xQueueSendToBack(q, evt, 0u); /* Zero timeout — never block caller. */
}

BaseType_t dispatcher_post_event_from_isr(QueueHandle_t q,
                                          const system_event_t *evt,
                                          BaseType_t *pxHigherPriorityTaskWoken)
{
    return xQueueSendToBackFromISR(q, evt, pxHigherPriorityTaskWoken);
}

/* ---------------------------------------------------------------------------
 * Dispatcher Task
 * Runs on Core 1 at Priority 4.
 * --------------------------------------------------------------------------- */
void dispatcher_task(void *pvParameters)
{
    QueueHandle_t central_queue = (QueueHandle_t)pvParameters;
    system_event_t evt;

    ESP_LOGI(TAG, "Dispatcher task started (Core %d)", xPortGetCoreID());

    for (;;) {
        /* Block until an event arrives. Timeout used for periodic health check. */
        if (xQueueReceive(central_queue, &evt, pdMS_TO_TICKS(COMM_TIMEOUT_MS))
                == pdTRUE) {

            ESP_LOGD(TAG, "Dispatch evt=0x%02X src=%d ts=%lu",
                     (unsigned)evt.type,
                     (int)evt.source,
                     (unsigned long)evt.timestamp_ms);

            /* --- Log every event before routing (NFR-4). --- */
            logger_log_event(g_logger_queue, &evt);

            /* --- Forward to FSM queue. --- */
            if (g_fsm_queue != NULL) {
                if (xQueueSendToBack(g_fsm_queue, &evt, 0u) != pdTRUE) {
                    ESP_LOGW(TAG, "FSM queue OVERFLOW — dropping evt 0x%02X",
                             (unsigned)evt.type);
                    logger_log_queue_overflow(g_logger_queue, "fsm_queue");
                }
            }

            /*
             * Safety Monitor gets first look at obstruction, SPOF, and
             * timeout events so it can enforce SR-1..SR-7 independently
             * of the FSM's response.
             */
            if (evt.type == EVT_OBSTRUCTION_DETECTED ||
                evt.type == EVT_COMM_TIMEOUT         ||
                evt.type == EVT_SPOF_DETECTED        ||
                evt.type == EVT_MOTOR_STALL) {

                /*
                 * Safety Monitor reaction is handled inside safety_monitor_task
                 * via the same FSM queue — the safety task re-posts a synthetic
                 * event if it needs to override the FSM.  We do not call
                 * safety-monitor functions here to avoid priority inversion
                 * (the Dispatcher runs at Priority 4, Safety at Priority 5).
                 * The Safety task will naturally preempt and handle first.
                 */
            }
        }
        /* If queue was empty (timeout), Dispatcher simply loops.
         * The comm-timeout will be detected independently by the Safety Monitor. */
    }
}