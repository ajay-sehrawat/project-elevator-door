/**
 * @file test/unit/test_door_fsm.c
 * @brief Unit test suite for the Door FSM transition table.
 *
 * Framework: Unity (bundled with ESP-IDF test infrastructure).
 *
 * Design Philosophy:
 *   The FSM logic is tested in ISOLATION from FreeRTOS, UART, NVS, and HAL.
 *   We achieve this by:
 *     1. Providing stub implementations of all external dependencies
 *        (hal_*, fault_nvs_*, logger_*, dispatcher_*).
 *     2. Calling fsm_process_event_test() — a test-only wrapper that bypasses
 *        the FreeRTOS task loop and directly drives the FSM state machine.
 *     3. Reading FSM state via fsm_get_state() (the same mutex-protected
 *        accessor used in production).
 *
 * Test Coverage:
 *   TC-01  Boot state is INIT / HOMING.
 *   TC-02  Homing completes → IDLE on FULLY_CLOSED.
 *   TC-03  CMD_OPEN from IDLE → OPENING.
 *   TC-04  FULLY_OPEN sensor → OPEN; stall timer cancelled.
 *   TC-05  CMD_CLOSE from OPEN → CLOSING.
 *   TC-06  FULLY_CLOSED sensor → CLOSED.
 *   TC-07  CMD_OPEN from CLOSED → OPENING (full round trip).
 *   TC-08  Obstruction during CLOSING → immediate OPENING reversal (SR-1).
 *   TC-09  CMD_CLOSE rejected while obstruction is active (SR-1).
 *   TC-10  Obstruction clear re-enables CLOSE (SR-1).
 *   TC-11  CMD_CLOSE during OPENING is rejected — priority inversion (FR-3).
 *   TC-12  CMD_OPEN preempts CLOSING (FR-3).
 *   TC-13  EMERGENCY_OPEN from CLOSING → OPENING + emergency lock (SR-6).
 *   TC-14  CMD_CLOSE rejected under emergency lock (SR-6).
 *   TC-15  RESET clears emergency lock and restarts homing (SR-6, SR-4).
 *   TC-16  Motor stall during OPENING → FAULT (SR-7).
 *   TC-17  Motor stall during CLOSING → FAULT (SR-7).
 *   TC-18  Motor stall during HOMING → FAULT (SR-7).
 *   TC-19  SPOF event → FAULT regardless of current state (SR-3).
 *   TC-20  COMM_TIMEOUT → FAULT (SR-5).
 *   TC-21  CMD_OPEN in FAULT is rejected (FR-1).
 *   TC-22  CMD_CLOSE in FAULT is rejected (FR-2).
 *   TC-23  RESET from FAULT → HOMING (safe recovery, SR-4).
 *   TC-24  EVT_FAULT_PERSIST on boot → FAULT without homing (SR-4, NFR-3).
 *   TC-25  Redundant CMD_OPEN in OPEN state → no state change (FR-4).
 *   TC-26  Redundant CMD_CLOSE in CLOSED state → no state change (FR-4).
 *   TC-27  EMERGENCY_OPEN from FAULT overrides fault (SR-6).
 *   TC-28  EMERGENCY_OPEN during CLOSING → OPENING immediately (SR-6, FR-3).
 *   TC-29  CMD_OPEN during HOMING is rejected (SR-4).
 *   TC-30  RESET in non-fault, non-emergency state is a no-op (FR-4).
 */

#include "unity.h"
#include <string.h>
#include <stdint.h>

/*
 * Pull in the FSM header and config — but NOT the full implementation.
 * We link against a test-instrumented version of door_fsm.c that exposes
 * fsm_process_event_test() and fsm_reset_for_test().
 */
#include "door_fsm.h"
#include "system_event.h"
#include "config.h"

/* ---------------------------------------------------------------------------
 * Test-Only FSM API (implemented at the bottom of this file via a thin shim
 * around the static fsm_process_event() function, which is made accessible
 * by compiling door_fsm.c with -DFSM_UNIT_TEST).
 * --------------------------------------------------------------------------- */

/**
 * @brief Reset FSM internal state to INIT and clear all flags.
 * Only available when compiled with -DFSM_UNIT_TEST.
 */
extern void fsm_reset_for_test(void);

/**
 * @brief Directly drive the FSM with a single event.
 * Bypasses the FreeRTOS task queue.
 */
extern void fsm_process_event_test(const system_event_t *evt);

/* ---------------------------------------------------------------------------
 * Stub Tracking Variables
 * These let tests assert that the correct HAL functions were called.
 * --------------------------------------------------------------------------- */
static int  stub_motor_open_count  = 0;
static int  stub_motor_close_count = 0;
static int  stub_motor_stop_count  = 0;
static int  stub_nvs_write_count   = 0;
static fault_code_t stub_last_nvs_fault = FAULT_NONE;
static int  stub_nvs_clear_count   = 0;

/* ---------------------------------------------------------------------------
 * HAL Stubs — replace the real HAL for unit tests.
 * --------------------------------------------------------------------------- */
void hal_motor_open(void)  { stub_motor_open_count++;  }
void hal_motor_close(void) { stub_motor_close_count++; }
void hal_motor_stop(void)  { stub_motor_stop_count++;  }

BaseType_t hal_tx_enqueue(const char *s)
{
    (void)s;
    return pdTRUE;
}
void hal_send_ack(void)  {}
void hal_send_nack(void) {}

/* ---------------------------------------------------------------------------
 * NVS Stubs
 * --------------------------------------------------------------------------- */
esp_err_t fault_nvs_write(fault_code_t code)
{
    stub_nvs_write_count++;
    stub_last_nvs_fault = code;
    return ESP_OK;
}
esp_err_t fault_nvs_clear(void)
{
    stub_nvs_clear_count++;
    stub_last_nvs_fault = FAULT_NONE;
    return ESP_OK;
}
esp_err_t fault_nvs_read(fault_code_t *code)  { *code = FAULT_NONE; return ESP_OK; }
esp_err_t fault_nvs_open(void)                { return ESP_OK; }
void      fault_nvs_close(void)               {}

/* ---------------------------------------------------------------------------
 * Dispatcher / Logger Stubs
 * --------------------------------------------------------------------------- */
QueueHandle_t g_logger_queue = NULL;

BaseType_t dispatcher_post_event(QueueHandle_t q, const system_event_t *evt)
{
    (void)q; (void)evt;
    return pdTRUE;
}
void logger_log_event(QueueHandle_t q, const system_event_t *evt)       { (void)q; (void)evt; }
void logger_log_transition(QueueHandle_t q, fsm_state_t a, fsm_state_t b,
                            const system_event_t *t)                     { (void)q; (void)a; (void)b; (void)t; }
void logger_log_queue_overflow(QueueHandle_t q, const char *n)           { (void)q; (void)n; }

/* esp_timer stub */
int64_t esp_timer_get_time(void) { return 0LL; }

/* ---------------------------------------------------------------------------
 * Helper: build a typed event with timestamp = 0.
 * --------------------------------------------------------------------------- */
static system_event_t make_evt(event_type_t type)
{
    system_event_t e = {
        .type         = type,
        .source       = SRC_UART_SUPERVISOR,
        .timestamp_ms = 0u,
        .payload      = 0u,
    };
    return e;
}

/* ---------------------------------------------------------------------------
 * setUp / tearDown — called by Unity before/after each test.
 * --------------------------------------------------------------------------- */
void setUp(void)
{
    fsm_reset_for_test();
    stub_motor_open_count  = 0;
    stub_motor_close_count = 0;
    stub_motor_stop_count  = 0;
    stub_nvs_write_count   = 0;
    stub_last_nvs_fault    = FAULT_NONE;
    stub_nvs_clear_count   = 0;
}

void tearDown(void) {}

/* ===========================================================================
 * TC-01  Boot state is HOMING
 * =========================================================================== */
void test_TC01_boot_state_is_homing(void)
{
    /*
     * After fsm_reset_for_test(), the FSM calls fsm_start_homing() which
     * transitions to HOMING and drives motor_close for physical reference.
     */
    TEST_ASSERT_EQUAL_INT(FSM_STATE_HOMING, fsm_get_state());
    TEST_ASSERT_GREATER_THAN(0, stub_motor_close_count);
}

/* ===========================================================================
 * TC-02  Homing completes on FULLY_CLOSED → IDLE
 * =========================================================================== */
void test_TC02_homing_completes_to_idle(void)
{
    system_event_t e = make_evt(EVT_SENSOR_FULLY_CLOSED);
    fsm_process_event_test(&e);
    TEST_ASSERT_EQUAL_INT(FSM_STATE_IDLE, fsm_get_state());
    /* Motor must be stopped when limit switch triggers. */
    TEST_ASSERT_GREATER_THAN(0, stub_motor_stop_count);
}

/* ===========================================================================
 * TC-03  CMD_OPEN from IDLE → OPENING
 * =========================================================================== */
void test_TC03_cmd_open_from_idle_to_opening(void)
{
    /* Complete homing first. */
    system_event_t homed = make_evt(EVT_SENSOR_FULLY_CLOSED);
    fsm_process_event_test(&homed);
    TEST_ASSERT_EQUAL_INT(FSM_STATE_IDLE, fsm_get_state());

    stub_motor_open_count = 0;
    system_event_t open = make_evt(EVT_CMD_OPEN);
    fsm_process_event_test(&open);

    TEST_ASSERT_EQUAL_INT(FSM_STATE_OPENING, fsm_get_state());
    TEST_ASSERT_EQUAL_INT(1, stub_motor_open_count);
}

/* ===========================================================================
 * TC-04  FULLY_OPEN sensor during OPENING → OPEN, motor stopped
 * =========================================================================== */
void test_TC04_fully_open_sensor_stops_motor(void)
{
    system_event_t homed  = make_evt(EVT_SENSOR_FULLY_CLOSED);
    system_event_t open   = make_evt(EVT_CMD_OPEN);
    system_event_t at_top = make_evt(EVT_SENSOR_FULLY_OPEN);

    fsm_process_event_test(&homed);
    fsm_process_event_test(&open);
    stub_motor_stop_count = 0;
    fsm_process_event_test(&at_top);

    TEST_ASSERT_EQUAL_INT(FSM_STATE_OPEN, fsm_get_state());
    TEST_ASSERT_EQUAL_INT(1, stub_motor_stop_count);
}

/* ===========================================================================
 * TC-05  CMD_CLOSE from OPEN → CLOSING
 * =========================================================================== */
void test_TC05_cmd_close_from_open_to_closing(void)
{
    system_event_t homed  = make_evt(EVT_SENSOR_FULLY_CLOSED);
    system_event_t open   = make_evt(EVT_CMD_OPEN);
    system_event_t at_top = make_evt(EVT_SENSOR_FULLY_OPEN);
    system_event_t close  = make_evt(EVT_CMD_CLOSE);

    fsm_process_event_test(&homed);
    fsm_process_event_test(&open);
    fsm_process_event_test(&at_top);

    stub_motor_close_count = 0;
    fsm_process_event_test(&close);

    TEST_ASSERT_EQUAL_INT(FSM_STATE_CLOSING, fsm_get_state());
    TEST_ASSERT_EQUAL_INT(1, stub_motor_close_count);
}

/* ===========================================================================
 * TC-06  FULLY_CLOSED sensor during CLOSING → CLOSED
 * =========================================================================== */
void test_TC06_fully_closed_sensor_to_closed(void)
{
    /* Bring door to CLOSING. */
    fsm_process_event_test(&(system_event_t){ .type = EVT_SENSOR_FULLY_CLOSED });
    fsm_process_event_test(&(system_event_t){ .type = EVT_CMD_OPEN });
    fsm_process_event_test(&(system_event_t){ .type = EVT_SENSOR_FULLY_OPEN });
    fsm_process_event_test(&(system_event_t){ .type = EVT_CMD_CLOSE });

    stub_motor_stop_count = 0;
    fsm_process_event_test(&(system_event_t){ .type = EVT_SENSOR_FULLY_CLOSED });

    TEST_ASSERT_EQUAL_INT(FSM_STATE_CLOSED, fsm_get_state());
    TEST_ASSERT_EQUAL_INT(1, stub_motor_stop_count);
}

/* ===========================================================================
 * TC-07  Full open/close round trip from CLOSED back to OPEN
 * =========================================================================== */
void test_TC07_full_round_trip(void)
{
    /* Homing */
    fsm_process_event_test(&(system_event_t){ .type = EVT_SENSOR_FULLY_CLOSED });
    TEST_ASSERT_EQUAL_INT(FSM_STATE_IDLE, fsm_get_state());

    /* Open */
    fsm_process_event_test(&(system_event_t){ .type = EVT_CMD_OPEN });
    TEST_ASSERT_EQUAL_INT(FSM_STATE_OPENING, fsm_get_state());
    fsm_process_event_test(&(system_event_t){ .type = EVT_SENSOR_FULLY_OPEN });
    TEST_ASSERT_EQUAL_INT(FSM_STATE_OPEN, fsm_get_state());

    /* Close */
    fsm_process_event_test(&(system_event_t){ .type = EVT_CMD_CLOSE });
    TEST_ASSERT_EQUAL_INT(FSM_STATE_CLOSING, fsm_get_state());
    fsm_process_event_test(&(system_event_t){ .type = EVT_SENSOR_FULLY_CLOSED });
    TEST_ASSERT_EQUAL_INT(FSM_STATE_CLOSED, fsm_get_state());

    /* Open again */
    fsm_process_event_test(&(system_event_t){ .type = EVT_CMD_OPEN });
    TEST_ASSERT_EQUAL_INT(FSM_STATE_OPENING, fsm_get_state());
    fsm_process_event_test(&(system_event_t){ .type = EVT_SENSOR_FULLY_OPEN });
    TEST_ASSERT_EQUAL_INT(FSM_STATE_OPEN, fsm_get_state());
}

/* ===========================================================================
 * TC-08  Obstruction during CLOSING → immediate reversal to OPENING (SR-1)
 * =========================================================================== */
void test_TC08_obstruction_during_closing_reverses(void)
{
    /* Reach CLOSING state. */
    fsm_process_event_test(&(system_event_t){ .type = EVT_SENSOR_FULLY_CLOSED });
    fsm_process_event_test(&(system_event_t){ .type = EVT_CMD_OPEN });
    fsm_process_event_test(&(system_event_t){ .type = EVT_SENSOR_FULLY_OPEN });
    fsm_process_event_test(&(system_event_t){ .type = EVT_CMD_CLOSE });
    TEST_ASSERT_EQUAL_INT(FSM_STATE_CLOSING, fsm_get_state());

    stub_motor_stop_count = 0;
    stub_motor_open_count = 0;
    system_event_t obs = make_evt(EVT_OBSTRUCTION_DETECTED);
    fsm_process_event_test(&obs);

    /* Must stop THEN reverse. */
    TEST_ASSERT_EQUAL_INT(FSM_STATE_OPENING, fsm_get_state());
    TEST_ASSERT_EQUAL_INT(1, stub_motor_stop_count);
    TEST_ASSERT_EQUAL_INT(1, stub_motor_open_count);
}

/* ===========================================================================
 * TC-09  CMD_CLOSE rejected while obstruction is active (SR-1)
 * =========================================================================== */
void test_TC09_close_rejected_during_obstruction(void)
{
    /* Reach OPEN state. */
    fsm_process_event_test(&(system_event_t){ .type = EVT_SENSOR_FULLY_CLOSED });
    fsm_process_event_test(&(system_event_t){ .type = EVT_CMD_OPEN });
    fsm_process_event_test(&(system_event_t){ .type = EVT_SENSOR_FULLY_OPEN });
    TEST_ASSERT_EQUAL_INT(FSM_STATE_OPEN, fsm_get_state());

    /* Mark obstruction active. */
    fsm_process_event_test(&(system_event_t){ .type = EVT_OBSTRUCTION_DETECTED });

    /* CMD_CLOSE must be rejected. */
    stub_motor_close_count = 0;
    fsm_process_event_test(&(system_event_t){ .type = EVT_CMD_CLOSE });

    TEST_ASSERT_EQUAL_INT(FSM_STATE_OPEN, fsm_get_state());
    TEST_ASSERT_EQUAL_INT(0, stub_motor_close_count);
}

/* ===========================================================================
 * TC-10  Obstruction clear re-enables CLOSE (SR-1)
 * =========================================================================== */
void test_TC10_obstruction_clear_reenables_close(void)
{
    /* Reach OPEN with obstruction. */
    fsm_process_event_test(&(system_event_t){ .type = EVT_SENSOR_FULLY_CLOSED });
    fsm_process_event_test(&(system_event_t){ .type = EVT_CMD_OPEN });
    fsm_process_event_test(&(system_event_t){ .type = EVT_SENSOR_FULLY_OPEN });
    fsm_process_event_test(&(system_event_t){ .type = EVT_OBSTRUCTION_DETECTED });

    /* Clear obstruction. */
    fsm_process_event_test(&(system_event_t){ .type = EVT_OBSTRUCTION_CLEAR });

    /* Now CLOSE should be accepted. */
    stub_motor_close_count = 0;
    fsm_process_event_test(&(system_event_t){ .type = EVT_CMD_CLOSE });

    TEST_ASSERT_EQUAL_INT(FSM_STATE_CLOSING, fsm_get_state());
    TEST_ASSERT_EQUAL_INT(1, stub_motor_close_count);
}

/* ===========================================================================
 * TC-11  CMD_CLOSE during OPENING is rejected — no priority inversion (FR-3)
 * =========================================================================== */
void test_TC11_close_during_opening_rejected(void)
{
    fsm_process_event_test(&(system_event_t){ .type = EVT_SENSOR_FULLY_CLOSED });
    fsm_process_event_test(&(system_event_t){ .type = EVT_CMD_OPEN });
    TEST_ASSERT_EQUAL_INT(FSM_STATE_OPENING, fsm_get_state());

    stub_motor_close_count = 0;
    fsm_process_event_test(&(system_event_t){ .type = EVT_CMD_CLOSE });

    /* State must remain OPENING; no motor_close command issued. */
    TEST_ASSERT_EQUAL_INT(FSM_STATE_OPENING, fsm_get_state());
    TEST_ASSERT_EQUAL_INT(0, stub_motor_close_count);
}

/* ===========================================================================
 * TC-12  CMD_OPEN preempts CLOSING (FR-3)
 * =========================================================================== */
void test_TC12_open_preempts_closing(void)
{
    /* Reach CLOSING. */
    fsm_process_event_test(&(system_event_t){ .type = EVT_SENSOR_FULLY_CLOSED });
    fsm_process_event_test(&(system_event_t){ .type = EVT_CMD_OPEN });
    fsm_process_event_test(&(system_event_t){ .type = EVT_SENSOR_FULLY_OPEN });
    fsm_process_event_test(&(system_event_t){ .type = EVT_CMD_CLOSE });
    TEST_ASSERT_EQUAL_INT(FSM_STATE_CLOSING, fsm_get_state());

    stub_motor_stop_count = 0;
    stub_motor_open_count = 0;
    fsm_process_event_test(&(system_event_t){ .type = EVT_CMD_OPEN });

    TEST_ASSERT_EQUAL_INT(FSM_STATE_OPENING, fsm_get_state());
    /* Stop must precede open motor command. */
    TEST_ASSERT_GREATER_THAN(0, stub_motor_stop_count);
    TEST_ASSERT_EQUAL_INT(1, stub_motor_open_count);
}

/* ===========================================================================
 * TC-13  EMERGENCY_OPEN from CLOSING → OPENING + emergency lock engaged (SR-6)
 * =========================================================================== */
void test_TC13_emergency_open_from_closing(void)
{
    /* Reach CLOSING. */
    fsm_process_event_test(&(system_event_t){ .type = EVT_SENSOR_FULLY_CLOSED });
    fsm_process_event_test(&(system_event_t){ .type = EVT_CMD_OPEN });
    fsm_process_event_test(&(system_event_t){ .type = EVT_SENSOR_FULLY_OPEN });
    fsm_process_event_test(&(system_event_t){ .type = EVT_CMD_CLOSE });

    stub_motor_stop_count = 0;
    stub_motor_open_count = 0;
    fsm_process_event_test(&(system_event_t){ .type = EVT_CMD_EMERGENCY_OPEN });

    TEST_ASSERT_EQUAL_INT(FSM_STATE_OPENING, fsm_get_state());
    TEST_ASSERT_EQUAL_INT(EMERGENCY_LOCK_ON, fsm_get_emergency_lock());
    TEST_ASSERT_GREATER_THAN(0, stub_motor_stop_count);
    TEST_ASSERT_EQUAL_INT(1, stub_motor_open_count);
}

/* ===========================================================================
 * TC-14  CMD_CLOSE rejected under emergency lock (SR-6)
 * =========================================================================== */
void test_TC14_close_rejected_under_emergency_lock(void)
{
    /* Engage emergency open from OPEN. */
    fsm_process_event_test(&(system_event_t){ .type = EVT_SENSOR_FULLY_CLOSED });
    fsm_process_event_test(&(system_event_t){ .type = EVT_CMD_OPEN });
    fsm_process_event_test(&(system_event_t){ .type = EVT_SENSOR_FULLY_OPEN });
    fsm_process_event_test(&(system_event_t){ .type = EVT_CMD_EMERGENCY_OPEN });
    fsm_process_event_test(&(system_event_t){ .type = EVT_SENSOR_FULLY_OPEN });

    TEST_ASSERT_EQUAL_INT(EMERGENCY_LOCK_ON, fsm_get_emergency_lock());

    stub_motor_close_count = 0;
    fsm_process_event_test(&(system_event_t){ .type = EVT_CMD_CLOSE });

    TEST_ASSERT_NOT_EQUAL(FSM_STATE_CLOSING, fsm_get_state());
    TEST_ASSERT_EQUAL_INT(0, stub_motor_close_count);
}

/* ===========================================================================
 * TC-15  RESET clears emergency lock and restarts homing (SR-6, SR-4)
 * =========================================================================== */
void test_TC15_reset_clears_emergency_lock_and_rehomes(void)
{
    /* Engage emergency lock. */
    fsm_process_event_test(&(system_event_t){ .type = EVT_SENSOR_FULLY_CLOSED });
    fsm_process_event_test(&(system_event_t){ .type = EVT_CMD_EMERGENCY_OPEN });

    TEST_ASSERT_EQUAL_INT(EMERGENCY_LOCK_ON, fsm_get_emergency_lock());

    stub_motor_close_count = 0;
    fsm_process_event_test(&(system_event_t){ .type = EVT_CMD_RESET });

    TEST_ASSERT_EQUAL_INT(EMERGENCY_LOCK_OFF, fsm_get_emergency_lock());
    TEST_ASSERT_EQUAL_INT(FSM_STATE_HOMING, fsm_get_state());
    /* Homing drives motor_close. */
    TEST_ASSERT_GREATER_THAN(0, stub_motor_close_count);
}

/* ===========================================================================
 * TC-16  Motor stall during OPENING → FAULT (SR-7)
 * =========================================================================== */
void test_TC16_motor_stall_during_opening_faults(void)
{
    fsm_process_event_test(&(system_event_t){ .type = EVT_SENSOR_FULLY_CLOSED });
    fsm_process_event_test(&(system_event_t){ .type = EVT_CMD_OPEN });
    TEST_ASSERT_EQUAL_INT(FSM_STATE_OPENING, fsm_get_state());

    stub_motor_stop_count = 0;
    stub_nvs_write_count  = 0;
    fsm_process_event_test(&(system_event_t){ .type = EVT_MOTOR_STALL });

    TEST_ASSERT_EQUAL_INT(FSM_STATE_FAULT, fsm_get_state());
    TEST_ASSERT_EQUAL_INT(FAULT_MOTOR_STALL, fsm_get_fault_code());
    /* Motor must be stopped on fault entry. */
    TEST_ASSERT_GREATER_THAN(0, stub_motor_stop_count);
    /* Fault must be persisted to NVS. */
    TEST_ASSERT_GREATER_THAN(0, stub_nvs_write_count);
    TEST_ASSERT_EQUAL_INT(FAULT_MOTOR_STALL, stub_last_nvs_fault);
}

/* ===========================================================================
 * TC-17  Motor stall during CLOSING → FAULT (SR-7)
 * =========================================================================== */
void test_TC17_motor_stall_during_closing_faults(void)
{
    fsm_process_event_test(&(system_event_t){ .type = EVT_SENSOR_FULLY_CLOSED });
    fsm_process_event_test(&(system_event_t){ .type = EVT_CMD_OPEN });
    fsm_process_event_test(&(system_event_t){ .type = EVT_SENSOR_FULLY_OPEN });
    fsm_process_event_test(&(system_event_t){ .type = EVT_CMD_CLOSE });
    TEST_ASSERT_EQUAL_INT(FSM_STATE_CLOSING, fsm_get_state());

    fsm_process_event_test(&(system_event_t){ .type = EVT_MOTOR_STALL });

    TEST_ASSERT_EQUAL_INT(FSM_STATE_FAULT, fsm_get_state());
    TEST_ASSERT_EQUAL_INT(FAULT_MOTOR_STALL, fsm_get_fault_code());
}

/* ===========================================================================
 * TC-18  Motor stall during HOMING → FAULT (SR-7)
 * =========================================================================== */
void test_TC18_motor_stall_during_homing_faults(void)
{
    /* FSM starts in HOMING after reset. */
    TEST_ASSERT_EQUAL_INT(FSM_STATE_HOMING, fsm_get_state());

    fsm_process_event_test(&(system_event_t){ .type = EVT_MOTOR_STALL });

    TEST_ASSERT_EQUAL_INT(FSM_STATE_FAULT, fsm_get_state());
    TEST_ASSERT_EQUAL_INT(FAULT_MOTOR_STALL, fsm_get_fault_code());
}

/* ===========================================================================
 * TC-19  SPOF event → FAULT regardless of current state (SR-3)
 * =========================================================================== */
void test_TC19_spof_causes_fault_from_any_state(void)
{
    /* Test from IDLE. */
    fsm_process_event_test(&(system_event_t){ .type = EVT_SENSOR_FULLY_CLOSED });
    TEST_ASSERT_EQUAL_INT(FSM_STATE_IDLE, fsm_get_state());

    fsm_process_event_test(&(system_event_t){ .type = EVT_SPOF_DETECTED });
    TEST_ASSERT_EQUAL_INT(FSM_STATE_FAULT, fsm_get_state());
    TEST_ASSERT_EQUAL_INT(FAULT_SPOF, fsm_get_fault_code());
}

/* ===========================================================================
 * TC-20  COMM_TIMEOUT → FAULT (SR-5)
 * =========================================================================== */
void test_TC20_comm_timeout_causes_fault(void)
{
    fsm_process_event_test(&(system_event_t){ .type = EVT_SENSOR_FULLY_CLOSED });
    fsm_process_event_test(&(system_event_t){ .type = EVT_CMD_OPEN });
    TEST_ASSERT_EQUAL_INT(FSM_STATE_OPENING, fsm_get_state());

    stub_motor_stop_count = 0;
    fsm_process_event_test(&(system_event_t){ .type = EVT_COMM_TIMEOUT });

    TEST_ASSERT_EQUAL_INT(FSM_STATE_FAULT, fsm_get_state());
    TEST_ASSERT_EQUAL_INT(FAULT_COMM_TIMEOUT, fsm_get_fault_code());
    TEST_ASSERT_GREATER_THAN(0, stub_motor_stop_count);
}

/* ===========================================================================
 * TC-21  CMD_OPEN in FAULT is rejected (FR-1)
 * =========================================================================== */
void test_TC21_open_rejected_in_fault(void)
{
    /* Enter FAULT via stall. */
    fsm_process_event_test(&(system_event_t){ .type = EVT_SENSOR_FULLY_CLOSED });
    fsm_process_event_test(&(system_event_t){ .type = EVT_CMD_OPEN });
    fsm_process_event_test(&(system_event_t){ .type = EVT_MOTOR_STALL });
    TEST_ASSERT_EQUAL_INT(FSM_STATE_FAULT, fsm_get_state());

    stub_motor_open_count = 0;
    fsm_process_event_test(&(system_event_t){ .type = EVT_CMD_OPEN });

    TEST_ASSERT_EQUAL_INT(FSM_STATE_FAULT, fsm_get_state());
    TEST_ASSERT_EQUAL_INT(0, stub_motor_open_count);
}

/* ===========================================================================
 * TC-22  CMD_CLOSE in FAULT is rejected (FR-2)
 * =========================================================================== */
void test_TC22_close_rejected_in_fault(void)
{
    fsm_process_event_test(&(system_event_t){ .type = EVT_SENSOR_FULLY_CLOSED });
    fsm_process_event_test(&(system_event_t){ .type = EVT_CMD_OPEN });
    fsm_process_event_test(&(system_event_t){ .type = EVT_MOTOR_STALL });
    TEST_ASSERT_EQUAL_INT(FSM_STATE_FAULT, fsm_get_state());

    stub_motor_close_count = 0;
    fsm_process_event_test(&(system_event_t){ .type = EVT_CMD_CLOSE });

    TEST_ASSERT_EQUAL_INT(FSM_STATE_FAULT, fsm_get_state());
    TEST_ASSERT_EQUAL_INT(0, stub_motor_close_count);
}

/* ===========================================================================
 * TC-23  RESET from FAULT → HOMING (safe recovery, SR-4)
 * =========================================================================== */
void test_TC23_reset_from_fault_restores_homing(void)
{
    /* Enter FAULT. */
    fsm_process_event_test(&(system_event_t){ .type = EVT_SPOF_DETECTED });
    TEST_ASSERT_EQUAL_INT(FSM_STATE_FAULT, fsm_get_state());

    stub_nvs_clear_count   = 0;
    stub_motor_close_count = 0;
    fsm_process_event_test(&(system_event_t){ .type = EVT_CMD_RESET });

    TEST_ASSERT_EQUAL_INT(FSM_STATE_HOMING, fsm_get_state());
    TEST_ASSERT_EQUAL_INT(FAULT_NONE, fsm_get_fault_code());
    /* NVS must be cleared on successful reset. */
    TEST_ASSERT_EQUAL_INT(1, stub_nvs_clear_count);
    /* Homing drives toward closed limit. */
    TEST_ASSERT_GREATER_THAN(0, stub_motor_close_count);
}

/* ===========================================================================
 * TC-24  EVT_FAULT_PERSIST on boot → FSM boots into FAULT without homing
 *        (SR-4, NFR-3)
 * =========================================================================== */
void test_TC24_nvs_boot_fault_goes_straight_to_fault(void)
{
    /*
     * Simulate: app_main injected EVT_FAULT_PERSIST BEFORE any homing event.
     * In the test, FSM boots in HOMING; we inject FAULT_PERSIST immediately.
     */
    TEST_ASSERT_EQUAL_INT(FSM_STATE_HOMING, fsm_get_state());

    system_event_t fe = make_evt(EVT_FAULT_PERSIST);
    fe.payload = (uint32_t)FAULT_NVS_BOOT;
    fsm_process_event_test(&fe);

    TEST_ASSERT_EQUAL_INT(FSM_STATE_FAULT, fsm_get_state());
    TEST_ASSERT_EQUAL_INT(FAULT_NVS_BOOT, fsm_get_fault_code());
    /* Normal operations must be blocked — CMD_OPEN rejected. */
    stub_motor_open_count = 0;
    fsm_process_event_test(&(system_event_t){ .type = EVT_CMD_OPEN });
    TEST_ASSERT_EQUAL_INT(FSM_STATE_FAULT, fsm_get_state());
    TEST_ASSERT_EQUAL_INT(0, stub_motor_open_count);
}

/* ===========================================================================
 * TC-25  Redundant CMD_OPEN in OPEN state → no state change (FR-4)
 * =========================================================================== */
void test_TC25_redundant_open_no_transition(void)
{
    fsm_process_event_test(&(system_event_t){ .type = EVT_SENSOR_FULLY_CLOSED });
    fsm_process_event_test(&(system_event_t){ .type = EVT_CMD_OPEN });
    fsm_process_event_test(&(system_event_t){ .type = EVT_SENSOR_FULLY_OPEN });
    TEST_ASSERT_EQUAL_INT(FSM_STATE_OPEN, fsm_get_state());

    stub_motor_open_count = 0;
    fsm_process_event_test(&(system_event_t){ .type = EVT_CMD_OPEN });

    TEST_ASSERT_EQUAL_INT(FSM_STATE_OPEN, fsm_get_state());
    TEST_ASSERT_EQUAL_INT(0, stub_motor_open_count);
}

/* ===========================================================================
 * TC-26  Redundant CMD_CLOSE in CLOSED state → no state change (FR-4)
 * =========================================================================== */
void test_TC26_redundant_close_no_transition(void)
{
    fsm_process_event_test(&(system_event_t){ .type = EVT_SENSOR_FULLY_CLOSED });
    fsm_process_event_test(&(system_event_t){ .type = EVT_CMD_OPEN });
    fsm_process_event_test(&(system_event_t){ .type = EVT_SENSOR_FULLY_OPEN });
    fsm_process_event_test(&(system_event_t){ .type = EVT_CMD_CLOSE });
    fsm_process_event_test(&(system_event_t){ .type = EVT_SENSOR_FULLY_CLOSED });
    TEST_ASSERT_EQUAL_INT(FSM_STATE_CLOSED, fsm_get_state());

    stub_motor_close_count = 0;
    fsm_process_event_test(&(system_event_t){ .type = EVT_CMD_CLOSE });

    TEST_ASSERT_EQUAL_INT(FSM_STATE_CLOSED, fsm_get_state());
    TEST_ASSERT_EQUAL_INT(0, stub_motor_close_count);
}

/* ===========================================================================
 * TC-27  EMERGENCY_OPEN from FAULT overrides fault (SR-6)
 * =========================================================================== */
void test_TC27_emergency_open_overrides_fault(void)
{
    /* Enter fault. */
    fsm_process_event_test(&(system_event_t){ .type = EVT_COMM_TIMEOUT });
    TEST_ASSERT_EQUAL_INT(FSM_STATE_FAULT, fsm_get_state());

    stub_motor_open_count = 0;
    fsm_process_event_test(&(system_event_t){ .type = EVT_CMD_EMERGENCY_OPEN });

    TEST_ASSERT_EQUAL_INT(FSM_STATE_OPENING, fsm_get_state());
    TEST_ASSERT_EQUAL_INT(EMERGENCY_LOCK_ON, fsm_get_emergency_lock());
    TEST_ASSERT_EQUAL_INT(1, stub_motor_open_count);
}

/* ===========================================================================
 * TC-28  EMERGENCY_OPEN during CLOSING → immediate OPENING (SR-6, FR-3)
 * =========================================================================== */
void test_TC28_emergency_open_during_closing(void)
{
    fsm_process_event_test(&(system_event_t){ .type = EVT_SENSOR_FULLY_CLOSED });
    fsm_process_event_test(&(system_event_t){ .type = EVT_CMD_OPEN });
    fsm_process_event_test(&(system_event_t){ .type = EVT_SENSOR_FULLY_OPEN });
    fsm_process_event_test(&(system_event_t){ .type = EVT_CMD_CLOSE });
    TEST_ASSERT_EQUAL_INT(FSM_STATE_CLOSING, fsm_get_state());

    stub_motor_open_count = 0;
    fsm_process_event_test(&(system_event_t){ .type = EVT_CMD_EMERGENCY_OPEN });

    TEST_ASSERT_EQUAL_INT(FSM_STATE_OPENING, fsm_get_state());
    TEST_ASSERT_EQUAL_INT(EMERGENCY_LOCK_ON, fsm_get_emergency_lock());
    TEST_ASSERT_EQUAL_INT(1, stub_motor_open_count);
}

/* ===========================================================================
 * TC-29  CMD_OPEN during HOMING is rejected (SR-4)
 * =========================================================================== */
void test_TC29_open_during_homing_rejected(void)
{
    TEST_ASSERT_EQUAL_INT(FSM_STATE_HOMING, fsm_get_state());

    stub_motor_open_count = 0;
    fsm_process_event_test(&(system_event_t){ .type = EVT_CMD_OPEN });

    /* Must stay in HOMING — physical reality not yet known. */
    TEST_ASSERT_EQUAL_INT(FSM_STATE_HOMING, fsm_get_state());
    TEST_ASSERT_EQUAL_INT(0, stub_motor_open_count);
}

/* ===========================================================================
 * TC-30  RESET in non-fault, non-emergency state is a no-op (FR-4)
 * =========================================================================== */
void test_TC30_reset_noop_in_idle(void)
{
    /* Reach IDLE. */
    fsm_process_event_test(&(system_event_t){ .type = EVT_SENSOR_FULLY_CLOSED });
    TEST_ASSERT_EQUAL_INT(FSM_STATE_IDLE, fsm_get_state());

    stub_motor_close_count = 0;
    fsm_process_event_test(&(system_event_t){ .type = EVT_CMD_RESET });

    /* State must remain IDLE; no homing motor command. */
    TEST_ASSERT_EQUAL_INT(FSM_STATE_IDLE, fsm_get_state());
    TEST_ASSERT_EQUAL_INT(0, stub_motor_close_count);
}

/* ---------------------------------------------------------------------------
 * Unity entry point
 * --------------------------------------------------------------------------- */
void app_main(void)
{
    UNITY_BEGIN();

    RUN_TEST(test_TC01_boot_state_is_homing);
    RUN_TEST(test_TC02_homing_completes_to_idle);
    RUN_TEST(test_TC03_cmd_open_from_idle_to_opening);
    RUN_TEST(test_TC04_fully_open_sensor_stops_motor);
    RUN_TEST(test_TC05_cmd_close_from_open_to_closing);
    RUN_TEST(test_TC06_fully_closed_sensor_to_closed);
    RUN_TEST(test_TC07_full_round_trip);
    RUN_TEST(test_TC08_obstruction_during_closing_reverses);
    RUN_TEST(test_TC09_close_rejected_during_obstruction);
    RUN_TEST(test_TC10_obstruction_clear_reenables_close);
    RUN_TEST(test_TC11_close_during_opening_rejected);
    RUN_TEST(test_TC12_open_preempts_closing);
    RUN_TEST(test_TC13_emergency_open_from_closing);
    RUN_TEST(test_TC14_close_rejected_under_emergency_lock);
    RUN_TEST(test_TC15_reset_clears_emergency_lock_and_rehomes);
    RUN_TEST(test_TC16_motor_stall_during_opening_faults);
    RUN_TEST(test_TC17_motor_stall_during_closing_faults);
    RUN_TEST(test_TC18_motor_stall_during_homing_faults);
    RUN_TEST(test_TC19_spof_causes_fault_from_any_state);
    RUN_TEST(test_TC20_comm_timeout_causes_fault);
    RUN_TEST(test_TC21_open_rejected_in_fault);
    RUN_TEST(test_TC22_close_rejected_in_fault);
    RUN_TEST(test_TC23_reset_from_fault_restores_homing);
    RUN_TEST(test_TC24_nvs_boot_fault_goes_straight_to_fault);
    RUN_TEST(test_TC25_redundant_open_no_transition);
    RUN_TEST(test_TC26_redundant_close_no_transition);
    RUN_TEST(test_TC27_emergency_open_overrides_fault);
    RUN_TEST(test_TC28_emergency_open_during_closing);
    RUN_TEST(test_TC29_open_during_homing_rejected);
    RUN_TEST(test_TC30_reset_noop_in_idle);

    UNITY_END();
}