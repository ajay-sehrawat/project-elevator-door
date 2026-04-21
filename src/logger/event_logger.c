/**
 * @file logger/event_logger.c
 * @brief Asynchronous event logger (NFR-4, SRK-5).
 *
 * Logging is always fire-and-forget.  The Logger task runs at Priority 1
 * on Core 0 so it cannot block any safety-critical path.
 *
 * Stack monitoring: every STACK_REPORT_INTERVAL_MS, the Logger prints
 * uxTaskGetStackHighWaterMark() for all registered tasks (NFR-2).
 */

#include <string.h>
#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "esp_timer.h"

#include "event_logger.h"
#include "config.h"
#include "system_event.h"
#include "door_fsm.h"

static const char *TAG = "LOGGER";

/* ---------------------------------------------------------------------------
 * Log entry struct — copied by value through the logger queue.
 * --------------------------------------------------------------------------- */
typedef struct {
    enum { LOG_TYPE_EVENT, LOG_TYPE_TRANSITION, LOG_TYPE_OVERFLOW } kind;
    system_event_t evt;               /* For LOG_TYPE_EVENT / TRANSITION. */
    fsm_state_t    old_state;
    fsm_state_t    new_state;
    char           overflow_name[32]; /* For LOG_TYPE_OVERFLOW. */
} log_entry_t;

static QueueHandle_t s_logger_queue = NULL;

/* Task handles for stack reporting — populated by app_main. */
/* We declare them weak so unit tests can link without them. */
__attribute__((weak)) TaskHandle_t g_task_safety     = NULL;
__attribute__((weak)) TaskHandle_t g_task_dispatcher = NULL;
__attribute__((weak)) TaskHandle_t g_task_control    = NULL;
__attribute__((weak)) TaskHandle_t g_task_hal_rx     = NULL;
__attribute__((weak)) TaskHandle_t g_task_hal_tx     = NULL;
__attribute__((weak)) TaskHandle_t g_task_logger     = NULL;

static const char *state_str(fsm_state_t s)
{
    switch (s) {
        case FSM_STATE_INIT:    return "INIT";
        case FSM_STATE_HOMING:  return "HOMING";
        case FSM_STATE_IDLE:    return "IDLE";
        case FSM_STATE_OPENING: return "OPENING";
        case FSM_STATE_OPEN:    return "OPEN";
        case FSM_STATE_CLOSING: return "CLOSING";
        case FSM_STATE_CLOSED:  return "CLOSED";
        case FSM_STATE_FAULT:   return "FAULT";
        default:                return "?";
    }
}

static const char *event_str(event_type_t t)
{
    switch (t) {
        case EVT_CMD_OPEN:            return "CMD_OPEN";
        case EVT_CMD_CLOSE:           return "CMD_CLOSE";
        case EVT_CMD_EMERGENCY_OPEN:  return "CMD_EMERGENCY_OPEN";
        case EVT_CMD_RESET:           return "CMD_RESET";
        case EVT_SENSOR_FULLY_OPEN:   return "SENSOR_FULLY_OPEN";
        case EVT_SENSOR_FULLY_CLOSED: return "SENSOR_FULLY_CLOSED";
        case EVT_OBSTRUCTION_DETECTED:return "OBSTRUCTION_DETECTED";
        case EVT_OBSTRUCTION_CLEAR:   return "OBSTRUCTION_CLEAR";
        case EVT_COMM_TIMEOUT:        return "COMM_TIMEOUT";
        case EVT_MOTOR_STALL:         return "MOTOR_STALL";
        case EVT_SPOF_DETECTED:       return "SPOF_DETECTED";
        case EVT_HOMING_COMPLETE:     return "HOMING_COMPLETE";
        case EVT_FAULT_PERSIST:       return "FAULT_PERSIST";
        case EVT_HEARTBEAT:           return "HEARTBEAT";
        default:                      return "UNKNOWN";
    }
}

/* ---------------------------------------------------------------------------
 * Public API
 * --------------------------------------------------------------------------- */

QueueHandle_t logger_init(void)
{
    s_logger_queue = xQueueCreate(LOGGER_QUEUE_DEPTH, sizeof(log_entry_t));
    if (s_logger_queue == NULL) {
        ESP_LOGE(TAG, "Failed to create logger queue — OOM");
    }
    return s_logger_queue;
}

void logger_log_event(QueueHandle_t logger_q, const system_event_t *evt)
{
    if (logger_q == NULL || evt == NULL) return;
    log_entry_t entry = { .kind = LOG_TYPE_EVENT, .evt = *evt };
    xQueueSendToBack(logger_q, &entry, 0u); /* Non-blocking — drop if full. */
}

void logger_log_transition(QueueHandle_t logger_q,
                           fsm_state_t old_state,
                           fsm_state_t new_state,
                           const system_event_t *trigger)
{
    if (logger_q == NULL) return;
    log_entry_t entry = {
        .kind      = LOG_TYPE_TRANSITION,
        .old_state = old_state,
        .new_state = new_state,
    };
    if (trigger) {
        entry.evt = *trigger;
    }
    xQueueSendToBack(logger_q, &entry, 0u);
}

void logger_log_queue_overflow(QueueHandle_t logger_q, const char *queue_name)
{
    if (logger_q == NULL) return;
    log_entry_t entry = { .kind = LOG_TYPE_OVERFLOW };
    strncpy(entry.overflow_name, queue_name, sizeof(entry.overflow_name) - 1u);
    xQueueSendToBack(logger_q, &entry, 0u);
}

/* ---------------------------------------------------------------------------
 * Logger Task
 * --------------------------------------------------------------------------- */
void logger_task(void *pvParameters)
{
    QueueHandle_t q = (QueueHandle_t)pvParameters;
    log_entry_t   entry;
    uint32_t      last_stack_report_ms = 0u;

    ESP_LOGI(TAG, "Logger task started (Core %d)", xPortGetCoreID());

    for (;;) {
        /* Drain log queue — yield if empty. */
        while (xQueueReceive(q, &entry, 0u) == pdTRUE) {
            uint32_t ts = entry.evt.timestamp_ms;

            switch (entry.kind) {
            case LOG_TYPE_EVENT:
                ESP_LOGI(TAG, "[EVT  ] ts=%lu  %-22s src=%d payload=%lu",
                         (unsigned long)ts,
                         event_str(entry.evt.type),
                         (int)entry.evt.source,
                         (unsigned long)entry.evt.payload);
                break;

            case LOG_TYPE_TRANSITION:
                ESP_LOGI(TAG, "[TRANS] ts=%lu  %s → %s  (trigger=%s)",
                         (unsigned long)ts,
                         state_str(entry.old_state),
                         state_str(entry.new_state),
                         event_str(entry.evt.type));
                break;

            case LOG_TYPE_OVERFLOW:
                ESP_LOGW(TAG, "[OVERFLOW] Queue '%s' was full — event dropped",
                         entry.overflow_name);
                break;

            default:
                break;
            }
        }

        /* NFR-2: Periodic stack high-water-mark report. */
        uint32_t now_ms = (uint32_t)(esp_timer_get_time() / 1000LL);
        if ((now_ms - last_stack_report_ms) >= STACK_REPORT_INTERVAL_MS) {
            last_stack_report_ms = now_ms;
            ESP_LOGI(TAG, "--- Stack HWM (words) ---");
            if (g_task_safety)     ESP_LOGI(TAG, "  SafetyTask:     %u", (unsigned)uxTaskGetStackHighWaterMark(g_task_safety));
            if (g_task_dispatcher) ESP_LOGI(TAG, "  DispatcherTask: %u", (unsigned)uxTaskGetStackHighWaterMark(g_task_dispatcher));
            if (g_task_control)    ESP_LOGI(TAG, "  ControlTask:    %u", (unsigned)uxTaskGetStackHighWaterMark(g_task_control));
            if (g_task_hal_rx)     ESP_LOGI(TAG, "  HAL_RX:         %u", (unsigned)uxTaskGetStackHighWaterMark(g_task_hal_rx));
            if (g_task_hal_tx)     ESP_LOGI(TAG, "  HAL_TX:         %u", (unsigned)uxTaskGetStackHighWaterMark(g_task_hal_tx));
            if (g_task_logger)     ESP_LOGI(TAG, "  LoggerTask:     %u", (unsigned)uxTaskGetStackHighWaterMark(g_task_logger));
            ESP_LOGI(TAG, "-------------------------");
        }

        vTaskDelay(pdMS_TO_TICKS(10u));
    }
}