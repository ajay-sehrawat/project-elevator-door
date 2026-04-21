#include <string.h>
#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_timer.h"

#include "door_fsm.h"
#include "hal.h"
#include "config.h"
#include "system_event.h"
#include "fault_nvs.h"
#include "event_logger.h"
#include "event_dispatcher.h"

static const char *TAG = "FSM";             //Unquie file identifier for logging or output to console

/* ---------------------------------------------------------------------------
 * Globals shared with Dispatcher (extern declarations in dispatcher.c)
 * --------------------------------------------------------------------------- */

/* Queue from which the FSM task consumes events. g prefix = globa; */
QueueHandle_t  g_fsm_queue    = NULL;

/** Mutex protecting s_fsm_ctx. */
SemaphoreHandle_t g_fsm_mutex = NULL;

/** Logger queue — set by app_main after logger_init(). */
QueueHandle_t  g_logger_queue = NULL;

/* ---------------------------------------------------------------------------
 * FSM Context (protected by g_fsm_mutex)
 * --------------------------------------------------------------------------- */
typedef struct {
    fsm_state_t      state;
    fault_code_t     fault_code;
    emergency_lock_t emergency_lock;
    uint8_t          obstruction_active; /**< 1 = obstruction not yet cleared. */
    uint8_t          homing_complete;    /**< 1 = homing done this session. //For Claibration Done or not    */
} fsm_ctx_t;

static fsm_ctx_t s_fsm_ctx = {
    .state             = FSM_STATE_INIT,
    .fault_code        = FAULT_NONE,
    .emergency_lock    = EMERGENCY_LOCK_OFF,
    .obstruction_active= 0u,
    .homing_complete   = 0u,
};                                          //Intializing starting context


//Motor Stall Timer, Timer is used to detect if motor runs too long -> something is wrong ( stall ).
static esp_timer_handle_t s_stall_timer = NULL;

/* Forward reference — stall timer callback needs a dispatcher queue handle. */
static QueueHandle_t s_central_queue_ref = NULL;

static void stall_timer_callback(void *arg)
{
    /* Called from esp_timer task — post to FSM queue (thread-safe). */
    system_event_t stall_evt = {
        .type         = EVT_MOTOR_STALL,
        .source       = SRC_INTERNAL_FSM,
        .timestamp_ms = (uint32_t)(esp_timer_get_time() / 1000LL),
        .payload      = 0u,
    };
    if (s_central_queue_ref != NULL) {                      //check queue is intialized before posting
        if (dispatcher_post_event(s_central_queue_ref, &stall_evt) != pdTRUE) { //Sends event to dispatcher queue pdTrue = 1 in FreeRtos
            ESP_LOGE(TAG, "CRITICAL: FSM queue full — stall event lost!");         //If queue full logs 
        }
    }
}

static void stall_timer_start(void)
{
    /* Adjust timeout for clock drift: effective = timeout * (1 + drift%). */
    uint64_t timeout_us = (uint64_t)MOTOR_STALL_TIMEOUT_MS                                      //3.6 sec
                          * (100u + CLOCK_DRIFT_MARGIN_PCT) / 100u * 1000ULL;                   //3.6 * 1.2 as 20% clocl drift
    esp_timer_start_once(s_stall_timer, timeout_us);                          //ESP function start timer after timout call stall_timer_callback function through s_stall_timer
    ESP_LOGD(TAG, "Stall timer started (%lu ms)", (unsigned long)(timeout_us / 1000u));
}

static void stall_timer_cancel(void)            //Cancel timer if running
{
    esp_timer_stop(s_stall_timer); /* Idempotent — safe to call even if not running. */
    ESP_LOGD(TAG, "Stall timer cancelled");
}

/* ---------------------------------------------------------------------------
 * Internal: transition helper
 * Acquires mutex, updates state, logs, releases mutex.
 * HAL actuator calls must happen OUTSIDE this function (see race-condition note).
 * --------------------------------------------------------------------------- */
static void fsm_transition(fsm_state_t new_state, fault_code_t fault_code,
                           const system_event_t *trigger)
{
    //take Lock to save state Lock FSM context before modifying as multiple tasks can access s_fsm_ctx
    xSemaphoreTake(g_fsm_mutex, portMAX_DELAY);
    fsm_state_t old_state = s_fsm_ctx.state;
    s_fsm_ctx.state       = new_state;              //FSM moves to new state
    if (fault_code != FAULT_NONE) {                 //Not every transition is a fault so conditional
        s_fsm_ctx.fault_code = fault_code;
    }
    xSemaphoreGive(g_fsm_mutex);

    if (old_state != new_state) {                   //To avoid unnecessary logging of same state
        logger_log_transition(g_logger_queue, old_state, new_state, trigger);
        ESP_LOGI(TAG, "State: %d → %d (evt=0x%02X)",
                 (int)old_state, (int)new_state, (unsigned)(trigger ? trigger->type : 0));      //Ouputs tranition to console
    }
}

// Internal: enter FAULT state — always calls hal_motor_stop() first (SR-2).
//No matter waht fault system always ends in safe state
static void fsm_enter_fault(fault_code_t code, const system_event_t *trigger)
{
    //Stop actuators immediately (safety-first)
    hal_motor_stop();

    // Transition FSM.
    fsm_transition(FSM_STATE_FAULT, code, trigger);

    /* 3. Persist to NVS (NFR-3). */
    esp_err_t nvs_ret = fault_nvs_write(code);
    if (nvs_ret != ESP_OK) {                        //If write fails output to console
        ESP_LOGE(TAG, "NVS write failed: %s — fault NOT persisted!", esp_err_to_name(nvs_ret));
    }

    /* 4. Notify Supervisor. */
    hal_tx_enqueue("$STATE,STATE=FAULT\n");

    //Cancel any pending stall timer. Already in fault state no need of timer
    stall_timer_cancel();                   

    ESP_LOGE(TAG, "FAULT entered — code=%d", (int)code);
}

// Internal: state name string (for logging), Converts an FSM state (enum value) into a human-readable string
//ESP_LOGI(TAG, "State: %d", state);   Output = State: 3 without this
static const char *state_name(fsm_state_t s)
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
        default:                return "UNKNOWN";
    }
}

/* ---------------------------------------------------------------------------
 * Homing Routine
 * Drives the door to a known physical position before accepting commands (SR-4).
 * In a real system this would command a slow creep toward the closed limit.
 * In the mock, we signal the start and wait for EVT_SENSOR_FULLY_CLOSED.
 * --------------------------------------------------------------------------- */
static void fsm_start_homing(void)
{
    fsm_transition(FSM_STATE_HOMING, FAULT_NONE, NULL);             //Trigger = NULL as internal action of FSM not caused by external event
    hal_tx_enqueue("$STATE,STATE=HOMING\n");
    // Command motor toward closed limit to find physical reference.
    hal_motor_close();
    stall_timer_start(); // Stall guard applies during homing too
    ESP_LOGI(TAG, "Homing initiated — driving to closed limit");
}

/* ---------------------------------------------------------------------------
 * FSM Initialisation
 * --------------------------------------------------------------------------- */
esp_err_t fsm_init(void)
{
    g_fsm_mutex = xSemaphoreCreateMutex();              //Variable declared at start of file so global scope
    if (g_fsm_mutex == NULL) {
        ESP_LOGE(TAG, "Failed to create FSM mutex");
        return ESP_ERR_NO_MEM;
    }

    g_fsm_queue = xQueueCreate(DISPATCHER_QUEUE_DEPTH, sizeof(system_event_t));     //(no of events = 16 , size of each event)
    if (g_fsm_queue == NULL) {
        ESP_LOGE(TAG, "Failed to create FSM queue");
        return ESP_ERR_NO_MEM;
    }

    //Create stall timer (one-shot, not started yet).
    const esp_timer_create_args_t stall_timer_args = {
        .callback        = stall_timer_callback,
        .arg             = NULL,
        .dispatch_method = ESP_TIMER_TASK,
        .name            = "stall_timer",
    };
    //Creates timer instance
    esp_err_t ret = esp_timer_create(&stall_timer_args, &s_stall_timer);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create stall timer: %s", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "FSM initialised");
    return ESP_OK;
}

 * Thread-safe accessors
 //Functions that let other parts of the system read FSM data safely without corrupting it.
 //Thread Safe way as multiple tasks run in parallel like fsm, logger, safety
fsm_state_t fsm_get_state(void)
{
    xSemaphoreTake(g_fsm_mutex, portMAX_DELAY);
    fsm_state_t s = s_fsm_ctx.state;
    xSemaphoreGive(g_fsm_mutex);
    return s;
}

emergency_lock_t fsm_get_emergency_lock(void)
{
    xSemaphoreTake(g_fsm_mutex, portMAX_DELAY);
    emergency_lock_t e = s_fsm_ctx.emergency_lock;
    xSemaphoreGive(g_fsm_mutex);
    return e;
}

fault_code_t fsm_get_fault_code(void)
{
    xSemaphoreTake(g_fsm_mutex, portMAX_DELAY);
    fault_code_t f = s_fsm_ctx.fault_code;
    xSemaphoreGive(g_fsm_mutex);
    return f;
}

// FSM Event Processing — the core switch-case transition table
static void fsm_process_event(const system_event_t *evt)
{
    // Snapshot state under mutex — release before calling HAL.
    xSemaphoreTake(g_fsm_mutex, portMAX_DELAY);
    fsm_state_t       cur_state       = s_fsm_ctx.state;
    emergency_lock_t  emg_lock        = s_fsm_ctx.emergency_lock;
    uint8_t           obs_active      = s_fsm_ctx.obstruction_active;
    xSemaphoreGive(g_fsm_mutex);

    switch (evt->type) {

    /* ======================================================================
     * EMERGENCY OPEN — highest priority; overrides any state (FR-3, SR-6)
     * ====================================================================== */
    case EVT_CMD_EMERGENCY_OPEN:
        stall_timer_cancel();
        hal_motor_stop(); /* Stop first, regardless of current motion. */
        fsm_transition(FSM_STATE_OPENING, FAULT_NONE, evt);

        xSemaphoreTake(g_fsm_mutex, portMAX_DELAY);
        s_fsm_ctx.emergency_lock     = EMERGENCY_LOCK_ON;
        s_fsm_ctx.obstruction_active = 0u;
        xSemaphoreGive(g_fsm_mutex);

        hal_motor_open();
        stall_timer_start();
        hal_tx_enqueue("$STATE,STATE=OPENING\n");
        ESP_LOGW(TAG, "EMERGENCY OPEN — lock engaged");
        break;

    /* ======================================================================
     * RESET — clears FAULT, clears emergency lock, restarts homing (SR-4)
     * ====================================================================== */
    case EVT_CMD_RESET:
        if (cur_state != FSM_STATE_FAULT && emg_lock == EMERGENCY_LOCK_OFF) {
            /* Redundant RESET in non-fault state — ACK, no action (FR-4). */
            ESP_LOGI(TAG, "RESET received in state %s — no action", state_name(cur_state));
            break;
        }
        stall_timer_cancel();
        xSemaphoreTake(g_fsm_mutex, portMAX_DELAY);
        s_fsm_ctx.emergency_lock     = EMERGENCY_LOCK_OFF;
        s_fsm_ctx.obstruction_active = 0u;
        s_fsm_ctx.fault_code         = FAULT_NONE;
        s_fsm_ctx.homing_complete    = 0u;
        xSemaphoreGive(g_fsm_mutex);

        fault_nvs_clear(); /* Safe recovery — clear persisted fault (NFR-3). */
        fsm_start_homing();
        break;

    /* ======================================================================
     * CMD OPEN (FR-1, FR-3)
     * ====================================================================== */
    case EVT_CMD_OPEN:
        /* Reject if in FAULT or emergency lockdown. */
        if (cur_state == FSM_STATE_FAULT) {
            ESP_LOGW(TAG, "CMD_OPEN rejected — in FAULT");
            break;
        }
        if (emg_lock == EMERGENCY_LOCK_ON) {
            /* Emergency lock: ignore OPEN (lock will release on RESET). */
            break;
        }
        /* Reject if obstruction is still active. */
        if (obs_active) {
            ESP_LOGW(TAG, "CMD_OPEN rejected — obstruction active");
            break;
        }
        /* Redundant OPEN (FR-4). */
        if (cur_state == FSM_STATE_OPEN || cur_state == FSM_STATE_OPENING) {
            ESP_LOGD(TAG, "CMD_OPEN — redundant, no state change");
            break;
        }
        /* Reject during HOMING (FR-3). */
        if (cur_state == FSM_STATE_HOMING || cur_state == FSM_STATE_INIT) {
            ESP_LOGW(TAG, "CMD_OPEN rejected — homing not complete");
            break;
        }
        /* Preempt CLOSING (FR-3). */
        if (cur_state == FSM_STATE_CLOSING) {
            stall_timer_cancel();
            hal_motor_stop();
            ESP_LOGI(TAG, "CMD_OPEN preempts CLOSING");
        }
        fsm_transition(FSM_STATE_OPENING, FAULT_NONE, evt);
        hal_motor_open();
        stall_timer_start();
        hal_tx_enqueue("$STATE,STATE=OPENING\n");
        break;

    /* ======================================================================
     * CMD CLOSE (FR-2, FR-3)
     * ====================================================================== */
    case EVT_CMD_CLOSE:
        if (cur_state == FSM_STATE_FAULT) {
            ESP_LOGW(TAG, "CMD_CLOSE rejected — in FAULT");
            break;
        }
        if (emg_lock == EMERGENCY_LOCK_ON) {
            ESP_LOGW(TAG, "CMD_CLOSE rejected — emergency lock active (SR-6)");
            break;
        }
        if (obs_active) {
            ESP_LOGW(TAG, "CMD_CLOSE rejected — obstruction active (SR-1)");
            break;
        }
        /* FR-3: CLOSE rejected during OPENING or HOMING (priority inversion guard). */
        if (cur_state == FSM_STATE_OPENING ||
            cur_state == FSM_STATE_HOMING  ||
            cur_state == FSM_STATE_INIT) {
            ESP_LOGW(TAG, "CMD_CLOSE rejected — door is OPENING/HOMING (FR-3)");
            break;
        }
        /* Redundant CLOSE (FR-4). */
        if (cur_state == FSM_STATE_CLOSED || cur_state == FSM_STATE_CLOSING) {
            ESP_LOGD(TAG, "CMD_CLOSE — redundant, no state change");
            break;
        }
        fsm_transition(FSM_STATE_CLOSING, FAULT_NONE, evt);
        hal_motor_close();
        stall_timer_start();
        hal_tx_enqueue("$STATE,STATE=CLOSING\n");
        break;

    /* ======================================================================
     * SENSOR: Door Fully Open (SR-7 — stall resolution)
     * ====================================================================== */
    case EVT_SENSOR_FULLY_OPEN:
        if (cur_state == FSM_STATE_OPENING) {
            stall_timer_cancel(); /* Physical limit reached — no stall. */
            hal_motor_stop();
            fsm_transition(FSM_STATE_OPEN, FAULT_NONE, evt);
            hal_tx_enqueue("$STATE,STATE=OPEN\n");
        } else {
            ESP_LOGD(TAG, "EVT_SENSOR_FULLY_OPEN in state %s — ignored",
                     state_name(cur_state));
        }
        break;

    /* ======================================================================
     * SENSOR: Door Fully Closed
     * ====================================================================== */
    case EVT_SENSOR_FULLY_CLOSED:
        if (cur_state == FSM_STATE_CLOSING || cur_state == FSM_STATE_HOMING) {
            stall_timer_cancel();
            hal_motor_stop();
            if (cur_state == FSM_STATE_HOMING) {
                /* Homing complete — NVS clean, ready for commands (SR-4). */
                xSemaphoreTake(g_fsm_mutex, portMAX_DELAY);
                s_fsm_ctx.homing_complete = 1u;
                xSemaphoreGive(g_fsm_mutex);
                fsm_transition(FSM_STATE_IDLE, FAULT_NONE, evt);
                hal_tx_enqueue("$STATE,STATE=IDLE\n");
                ESP_LOGI(TAG, "Homing complete — system IDLE");
            } else {
                fsm_transition(FSM_STATE_CLOSED, FAULT_NONE, evt);
                hal_tx_enqueue("$STATE,STATE=CLOSED\n");
            }
        }
        break;

    /* ======================================================================
     * SENSOR: Obstruction Detected (SR-1, SR-2)
     * ====================================================================== */
    case EVT_OBSTRUCTION_DETECTED:
        xSemaphoreTake(g_fsm_mutex, portMAX_DELAY);
        s_fsm_ctx.obstruction_active = 1u;
        xSemaphoreGive(g_fsm_mutex);

        if (cur_state == FSM_STATE_CLOSING) {
            /* Immediate reversal — SR-1 mandates stop then open. */
            stall_timer_cancel();
            hal_motor_stop();
            fsm_transition(FSM_STATE_OPENING, FAULT_NONE, evt);
            hal_motor_open();
            stall_timer_start();
            hal_tx_enqueue("$STATE,STATE=OPENING\n");
            ESP_LOGW(TAG, "Obstruction detected during CLOSING — reversing to OPEN");
        } else {
            ESP_LOGW(TAG, "Obstruction detected in state %s", state_name(cur_state));
        }
        break;

    /* ======================================================================
     * SENSOR: Obstruction Cleared (SR-1)
     * ====================================================================== */
    case EVT_OBSTRUCTION_CLEAR:
        xSemaphoreTake(g_fsm_mutex, portMAX_DELAY);
        s_fsm_ctx.obstruction_active = 0u;
        xSemaphoreGive(g_fsm_mutex);
        ESP_LOGI(TAG, "Obstruction cleared — normal operations may resume");
        break;

    /* ======================================================================
     * FAULT: Motor Stall (SR-7)
     * ====================================================================== */
    case EVT_MOTOR_STALL:
        if (cur_state == FSM_STATE_OPENING || cur_state == FSM_STATE_CLOSING ||
            cur_state == FSM_STATE_HOMING) {
            ESP_LOGE(TAG, "Motor stall detected in state %s", state_name(cur_state));
            fsm_enter_fault(FAULT_MOTOR_STALL, evt);
        }
        break;

    /* ======================================================================
     * FAULT: SPOF — contradictory sensor data (SR-3)
     * ====================================================================== */
    case EVT_SPOF_DETECTED:
        ESP_LOGE(TAG, "SPOF detected — contradictory sensors");
        fsm_enter_fault(FAULT_SPOF, evt);
        break;

    /* ======================================================================
     * FAULT: Communication Timeout (SR-5)
     * ====================================================================== */
    case EVT_COMM_TIMEOUT:
        if (cur_state != FSM_STATE_FAULT) {
            ESP_LOGE(TAG, "Communication timeout — entering FAULT");
            fsm_enter_fault(FAULT_COMM_TIMEOUT, evt);
        }
        break;

    /* ======================================================================
     * Boot fault persisted in NVS (SR-4, NFR-3)
     * ====================================================================== */
    case EVT_FAULT_PERSIST:
        ESP_LOGE(TAG, "NVS fault found on boot — booting into FAULT");
        /* Do not call fsm_enter_fault (which would re-write NVS); just set state. */
        fsm_transition(FSM_STATE_FAULT, FAULT_NVS_BOOT, evt);
        hal_tx_enqueue("$STATE,STATE=FAULT\n");
        break;

    /* ======================================================================
     * Homing complete (EVT_HOMING_COMPLETE injected by homing sub-routine)
     * ====================================================================== */
    case EVT_HOMING_COMPLETE:
        /* Already handled via EVT_SENSOR_FULLY_CLOSED during homing. */
        break;

    /* ======================================================================
     * Heartbeat — used to verify task liveness (NFR-2)
     * ====================================================================== */
    case EVT_HEARTBEAT:
        ESP_LOGD(TAG, "Heartbeat — state=%s", state_name(cur_state));
        break;

    default:
        ESP_LOGW(TAG, "Unhandled event type 0x%02X in state %s",
                 (unsigned)evt->type, state_name(cur_state));
        break;
    }
}

/* ---------------------------------------------------------------------------
 * FSM Control Task Entry Point
 * --------------------------------------------------------------------------- */
void fsm_control_task(void *pvParameters)
{
    s_central_queue_ref = (QueueHandle_t)pvParameters;

    system_event_t evt;
    char state_buf[UART_FRAME_MAX_LEN];
    uint32_t last_report_ms = 0u;

    ESP_LOGI(TAG, "FSM Control task started (Core %d)", xPortGetCoreID());

    /* Begin homing immediately on boot (SR-4). */
    fsm_start_homing();

    for (;;) {
        /* Block on FSM queue with a state-report timeout. */
        if (xQueueReceive(g_fsm_queue, &evt,
                          pdMS_TO_TICKS(STATE_REPORT_INTERVAL_MS)) == pdTRUE) {
            fsm_process_event(&evt);
        }

        /* FR-5: Periodic state reporting every STATE_REPORT_INTERVAL_MS. */
        uint32_t now_ms = (uint32_t)(esp_timer_get_time() / 1000LL);
        if ((now_ms - last_report_ms) >= STATE_REPORT_INTERVAL_MS) {
            last_report_ms = now_ms;
            fsm_state_t cur = fsm_get_state();
            snprintf(state_buf, UART_FRAME_MAX_LEN,
                     "$STATE,STATE=%s,TS=%lu\n",
                     state_name(cur), (unsigned long)now_ms);
            hal_tx_enqueue(state_buf);
        }
    }
}

/* ---------------------------------------------------------------------------
 * Unit Test Instrumentation
 * Only compiled when the test build passes -DFSM_UNIT_TEST.
 * Exposes internal static functions so test_door_fsm.c can drive the FSM
 * directly without a FreeRTOS task queue or UART in the loop.
 * --------------------------------------------------------------------------- */
#ifdef FSM_UNIT_TEST

void fsm_process_event_test(const system_event_t *evt)
{
    fsm_process_event(evt);
}

void fsm_reset_for_test(void)
{
    /* Create primitives on first call (test environment replaces app_main). */
    if (g_fsm_mutex == NULL) {
        g_fsm_mutex = xSemaphoreCreateMutex();
    }
    if (g_fsm_queue == NULL) {
        g_fsm_queue = xQueueCreate(DISPATCHER_QUEUE_DEPTH, sizeof(system_event_t));
    }

    /* Reset all context fields to a clean INIT state. */
    xSemaphoreTake(g_fsm_mutex, portMAX_DELAY);
    s_fsm_ctx.state              = FSM_STATE_INIT;
    s_fsm_ctx.fault_code         = FAULT_NONE;
    s_fsm_ctx.emergency_lock     = EMERGENCY_LOCK_OFF;
    s_fsm_ctx.obstruction_active = 0u;
    s_fsm_ctx.homing_complete    = 0u;
    xSemaphoreGive(g_fsm_mutex);

    /* Replicate real boot: start homing sequence immediately. */
    fsm_start_homing();
}

#endif /* FSM_UNIT_TEST */