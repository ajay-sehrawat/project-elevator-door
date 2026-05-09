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
            }
        }

    }
}