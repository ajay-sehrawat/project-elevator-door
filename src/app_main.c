/**
 * @file app_main.c
 * @brief System entry point — initialises all subsystems and spawns FreeRTOS tasks.
 *
 * Boot Sequence:
 *   1. nvs_flash_init()        — NVS must be ready before fault_nvs_open().
 *   2. fault_nvs_open()        — Open NVS namespace.
 *   3. fault_nvs_read()        — Check for persisted fault from previous session.
 *   4. logger_init()           — Logger queue must exist before any task logs.
 *   5. fsm_init()              — Creates FSM queue and mutex.
 *   6. safety_monitor_init()   — Resets comm watchdog timestamp.
 *   7. dispatcher_create_queue()
 *   8. hal_init()              — UART driver + TX queue.
 *   9. Task creation (pinned to correct cores as per specification).
 *  10. If persisted fault found: inject EVT_FAULT_PERSIST into FSM queue.
 *
 * Task Watchdog:
 *   esp_task_wdt_init() is called here.  The Safety Monitor task registers
 *   itself with the TWDT (esp_task_wdt_add) inside safety_monitor_task().
 *
 * Power Management:
 *   Active mode is enforced.  See hal_uart_mock.c for the rationale.
 *   pm_config.max_freq_mhz = pm_config.min_freq_mhz = 240 to prevent DVFS
 *   scaling from introducing timing jitter in safety loops.
 */

#include <stdio.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_task_wdt.h"

#include "config.h"
#include "system_event.h"
#include "hal.h"
#include "door_fsm.h"
#include "safety_monitor.h"
#include "event_dispatcher.h"
#include "event_logger.h"
#include "fault_nvs.h"

static const char *TAG = "MAIN";

/* ---------------------------------------------------------------------------
 * Task Handle Storage (exported weak-linked in event_logger.c)
 * --------------------------------------------------------------------------- */
TaskHandle_t g_task_safety     = NULL;
TaskHandle_t g_task_dispatcher = NULL;
TaskHandle_t g_task_control    = NULL;
TaskHandle_t g_task_hal_rx     = NULL;
TaskHandle_t g_task_hal_tx     = NULL;
TaskHandle_t g_task_logger     = NULL;

/* ---------------------------------------------------------------------------
 * FreeRTOS Stack Overflow Hook (NFR-2)
 * Called by FreeRTOS kernel when it detects a task has overflowed its stack.
 * Must be defined in application code.
 * --------------------------------------------------------------------------- */
void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName)
{
    /*
     * Stack overflow is a non-recoverable fault.  Log and halt.
     * The TWDT will fire after TWDT_FEED_INTERVAL_MS and reset the chip.
     */
    ESP_LOGE(TAG, "!!! STACK OVERFLOW in task: %s !!!", pcTaskName);
    (void)xTask;
    /* Spin — let TWDT reset the system. */
    for (;;) {
        vTaskDelay(portMAX_DELAY);
    }
}

/* ---------------------------------------------------------------------------
 * FreeRTOS Malloc Failure Hook (NFR-2)
 * --------------------------------------------------------------------------- */
void vApplicationMallocFailedHook(void)
{
    ESP_LOGE(TAG, "!!! Heap allocation failure — system halt !!!");
    for (;;) {
        vTaskDelay(portMAX_DELAY);
    }
}

/* ---------------------------------------------------------------------------
 * app_main
 * --------------------------------------------------------------------------- */
void app_main(void)
{
    ESP_LOGI(TAG, "=== Elevator Door Safety Controller Booting ===");

    /* ── 1. NVS Flash Init ──────────────────────────────────────────────── */
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        /* NVS partition was truncated or version mismatch — erase and retry. */
        ESP_LOGW(TAG, "NVS partition damaged — erasing");
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    ESP_LOGI(TAG, "NVS flash initialised");

    /* ── 2. Open NVS namespace ──────────────────────────────────────────── */
    ESP_ERROR_CHECK(fault_nvs_open());

    /* ── 3. Check for persisted fault (SR-4, NFR-3) ─────────────────────── */
    fault_code_t boot_fault = FAULT_NONE;
    ESP_ERROR_CHECK(fault_nvs_read(&boot_fault));
    if (boot_fault != FAULT_NONE) {
        ESP_LOGW(TAG, "Persisted fault detected on boot: code=%d", (int)boot_fault);
    }

    /* ── 4. Logger Queue ────────────────────────────────────────────────── */
    QueueHandle_t logger_queue = logger_init();
    if (logger_queue == NULL) {
        ESP_LOGE(TAG, "Logger init failed — cannot continue");
        return;
    }
    /* Make logger queue accessible to FSM and dispatcher. */
    extern QueueHandle_t g_logger_queue;
    g_logger_queue = logger_queue;

    /* ── 5. FSM Init ────────────────────────────────────────────────────── */
    ESP_ERROR_CHECK(fsm_init());

    /* ── 6. Safety Monitor Init ─────────────────────────────────────────── */
    ESP_ERROR_CHECK(safety_monitor_init());

    /* ── 7. Dispatcher Queue ────────────────────────────────────────────── */
    QueueHandle_t dispatcher_queue = dispatcher_create_queue();
    if (dispatcher_queue == NULL) {
        ESP_LOGE(TAG, "Dispatcher queue creation failed — cannot continue");
        return;
    }

    /* ── 8. HAL Init ────────────────────────────────────────────────────── */
    ESP_ERROR_CHECK(hal_init(dispatcher_queue));

    /* ── 9. Hardware Task Watchdog (TWDT) ───────────────────────────────── */
    /*
     * timeout_ms: CONFIG_ESP_TASK_WDT_TIMEOUT_S * 1000 (set in platformio.ini).
     * panic: true — triggers system panic + reboot on watchdog expiry.
     * The Safety task will call esp_task_wdt_add() and esp_task_wdt_reset()
     * to keep the watchdog alive.
     */
    const esp_task_wdt_config_t twdt_cfg = {
        .timeout_ms    = CONFIG_ESP_TASK_WDT_TIMEOUT_S * 1000u,
        .idle_core_mask= 0u,   /* Do not watch idle tasks. */
        .trigger_panic = true,
    };
    ret = esp_task_wdt_init(&twdt_cfg);
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
        /* ESP_ERR_INVALID_STATE means TWDT was already initialised by IDF. */
        ESP_ERROR_CHECK(ret);
    }
    ESP_LOGI(TAG, "TWDT configured (%d s timeout, panic on expiry)",
             CONFIG_ESP_TASK_WDT_TIMEOUT_S);

    /* ── 10. Spawn Tasks ────────────────────────────────────────────────── */

    /*
     * Core 1 — Safety-Critical (Safety > Dispatcher = Control)
     */
    xTaskCreatePinnedToCore(
        safety_monitor_task,
        "SafetyTask",
        STACK_SAFETY_TASK,
        (void *)dispatcher_queue,   /* Safety posts to Dispatcher queue. */
        PRIORITY_SAFETY_TASK,
        &g_task_safety,
        CORE_SAFETY
    );

    xTaskCreatePinnedToCore(
        dispatcher_task,
        "DispatcherTask",
        STACK_DISPATCHER_TASK,
        (void *)dispatcher_queue,
        PRIORITY_DISPATCHER_TASK,
        &g_task_dispatcher,
        CORE_SAFETY
    );

    xTaskCreatePinnedToCore(
        fsm_control_task,
        "ControlTask",
        STACK_CONTROL_TASK,
        (void *)dispatcher_queue,   /* FSM references central queue for stall timer. */
        PRIORITY_CONTROL_TASK,
        &g_task_control,
        CORE_SAFETY
    );

    /*
     * Core 0 — Communications & Peripherals
     */
    xTaskCreatePinnedToCore(
        hal_uart_rx_task,
        "HAL_RX",
        STACK_HAL_INPUT_TASK,
        NULL,                       /* Uses s_dispatcher_queue from hal_init. */
        PRIORITY_HAL_INPUT_TASK,
        &g_task_hal_rx,
        CORE_COMMS
    );

    xTaskCreatePinnedToCore(
        hal_uart_tx_task,
        "HAL_TX",
        STACK_UART_TX_TASK,
        NULL,
        PRIORITY_UART_TX_TASK,
        &g_task_hal_tx,
        CORE_COMMS
    );

    xTaskCreatePinnedToCore(
        logger_task,
        "LoggerTask",
        STACK_LOGGER_TASK,
        (void *)logger_queue,
        PRIORITY_LOGGER_TASK,
        &g_task_logger,
        CORE_COMMS
    );

    ESP_LOGI(TAG, "All tasks spawned");

    /* ── 11. Inject boot-fault event if NVS fault was found (SR-4) ──────── */
    if (boot_fault != FAULT_NONE) {
        /*
         * Small delay to allow FSM task to start and block on its queue
         * before we inject the persisted fault event.
         */
        vTaskDelay(pdMS_TO_TICKS(50u));

        system_event_t fault_evt = {
            .type         = EVT_FAULT_PERSIST,
            .source       = SRC_INTERNAL_SAFETY,
            .timestamp_ms = (uint32_t)(esp_timer_get_time() / 1000LL),
            .payload      = (uint32_t)boot_fault,
        };

        extern QueueHandle_t g_fsm_queue;
        if (xQueueSendToBack(g_fsm_queue, &fault_evt, pdMS_TO_TICKS(100u)) != pdTRUE) {
            ESP_LOGE(TAG, "Failed to inject EVT_FAULT_PERSIST — FSM queue full");
        } else {
            ESP_LOGW(TAG, "EVT_FAULT_PERSIST injected — FSM will boot into FAULT");
        }
    }

    /*
     * app_main returns here.  The FreeRTOS scheduler takes over.
     * All work is done inside the spawned tasks.
     *
     * Power Management: Active mode maintained — see hal_uart_mock.c.
     * To implement light-sleep in future:
     *   esp_pm_config_t pm_cfg = { .max_freq_mhz = 240, .min_freq_mhz = 240,
     *                              .light_sleep_enable = false };
     *   esp_pm_configure(&pm_cfg);
     */
    ESP_LOGI(TAG, "app_main complete — scheduler running");
}