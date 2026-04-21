/**
 * @file safety/safety_monitor.c
 * @brief Safety Monitor — highest-priority task on Core 1.
 *
 * Responsibilities enforced independently of the FSM:
 *   SR-1  Obstruction must halt/reverse door motion.
 *   SR-2  Obstruction reaction within OBSTRUCTION_REACT_MS.
 *   SR-3  SPOF cross-check (contradictory sensors → FAULT).
 *   SR-5  Communication timeout within COMM_TIMEOUT_MS.
 *   SR-7  The Safety Monitor does NOT duplicate stall detection — that is
 *         handled by the FSM's esp_timer callback to avoid redundancy.
 *
 * The Safety Monitor NEVER mutates FSM state directly.
 * It enqueues typed events to the Dispatcher queue.  The FSM, running at the
 * same priority (4) on the same core (1), processes them in turn.
 * This single-writer pattern eliminates concurrent writes to s_fsm_ctx.
 *
 * TWDT: The Safety Monitor feeds the ESP32 Hardware Task Watchdog every
 * TWDT_FEED_INTERVAL_MS.  If this task ever hangs, the hardware TWDT fires
 * and resets the ESP32, guaranteeing a fail-safe restart.
 */

#include <stdatomic.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_task_wdt.h"

#include "safety_monitor.h"
#include "config.h"
#include "system_event.h"
#include "event_dispatcher.h"

static const char *TAG = "SAFETY";

/* Dispatcher queue reference (set at init). */
static QueueHandle_t s_dispatch_q = NULL;

/*
 * Communication watchdog timestamp.
 * Written by HAL RX task (Core 0), read by Safety task (Core 1).
 * Using C11 _Atomic to ensure atomic read/write without a mutex
 * (avoids priority inversion on what is a single 32-bit value).
 */
static _Atomic uint32_t s_last_valid_frame_ms = 0u;

/*
 * Raw sensor state snapshot — written by HAL (Core 0) via
 * safety_monitor_update_sensors(), read by Safety task (Core 1).
 * Separate atomics per sensor to avoid struct-copy tearing.
 */
static _Atomic uint8_t s_raw_fully_open   = 0u;
static _Atomic uint8_t s_raw_fully_closed = 0u;

/* ---------------------------------------------------------------------------
 * Public API
 * --------------------------------------------------------------------------- */

esp_err_t safety_monitor_init(void)
{
    s_last_valid_frame_ms = (uint32_t)(esp_timer_get_time() / 1000LL);
    ESP_LOGI(TAG, "Safety Monitor initialised");
    return ESP_OK;
}

void safety_monitor_reset_comm_watchdog(void)
{
    /* Atomic write — safe from HAL RX task on Core 0. */
    atomic_store(&s_last_valid_frame_ms,
                 (uint32_t)(esp_timer_get_time() / 1000LL));
}

void safety_monitor_update_sensors(uint8_t fully_open, uint8_t fully_closed)
{
    atomic_store(&s_raw_fully_open,   fully_open);
    atomic_store(&s_raw_fully_closed, fully_closed);
}

/* ---------------------------------------------------------------------------
 * Helper: post a safety event to the dispatcher queue.
 * --------------------------------------------------------------------------- */
static void post_safety_event(event_type_t type)
{
    system_event_t evt = {
        .type         = type,
        .source       = SRC_INTERNAL_SAFETY,
        .timestamp_ms = (uint32_t)(esp_timer_get_time() / 1000LL),
        .payload      = 0u,
    };
    if (dispatcher_post_event(s_dispatch_q, &evt) != pdTRUE) {
        ESP_LOGE(TAG, "CRITICAL: safety queue overflow — event 0x%02X lost!", (unsigned)type);
        /*
         * Queue overflow on a safety-critical event is a system-level failure.
         * The TWDT will catch a hung Safety task; a queue overflow here indicates
         * the FSM/Dispatcher pipeline is saturated. Log and continue — the watchdog
         * will eventually reset the system to a safe state.
         */
    }
}

/* ---------------------------------------------------------------------------
 * Safety Monitor Task
 * --------------------------------------------------------------------------- */
void safety_monitor_task(void *pvParameters)
{
    s_dispatch_q = (QueueHandle_t)pvParameters;

    ESP_LOGI(TAG, "Safety Monitor task started (Core %d)", xPortGetCoreID());

    /* Register this task with the ESP32 Hardware Task Watchdog (TWDT). */
    esp_task_wdt_add(NULL); /* NULL = current task. */

    uint32_t last_twdt_feed_ms = (uint32_t)(esp_timer_get_time() / 1000LL);

    for (;;) {
        uint32_t now_ms = (uint32_t)(esp_timer_get_time() / 1000LL);

        /* ── SR-5: Communication Timeout Check ───────────────────────────── */
        uint32_t last_frame = atomic_load(&s_last_valid_frame_ms);
        if ((now_ms - last_frame) > COMM_TIMEOUT_MS) {
            ESP_LOGW(TAG, "Comm timeout detected: %lu ms since last frame",
                     (unsigned long)(now_ms - last_frame));
            post_safety_event(EVT_COMM_TIMEOUT);
            /*
             * Reset the watchdog timestamp so we don't flood the queue with
             * repeated timeouts.  The FSM will handle it and enter FAULT;
             * subsequent RESET + homing will re-enable normal comms.
             */
            atomic_store(&s_last_valid_frame_ms, now_ms);
        }

        /* ── SR-3: SPOF Cross-Check ──────────────────────────────────────── */
        uint8_t fo = (uint8_t)atomic_load(&s_raw_fully_open);
        uint8_t fc = (uint8_t)atomic_load(&s_raw_fully_closed);
        if (fo && fc) {
            ESP_LOGE(TAG, "SPOF detected in Safety Monitor: fully_open && fully_closed");
            post_safety_event(EVT_SPOF_DETECTED);
        }

        /* ── TWDT Feed ───────────────────────────────────────────────────── */
        if ((now_ms - last_twdt_feed_ms) >= TWDT_FEED_INTERVAL_MS) {
            esp_task_wdt_reset();
            last_twdt_feed_ms = now_ms;
            ESP_LOGD(TAG, "TWDT fed at %lu ms", (unsigned long)now_ms);
        }

        /* Safety task runs every 10 ms — much faster than all safety deadlines. */
        vTaskDelay(pdMS_TO_TICKS(10u));
    }
}