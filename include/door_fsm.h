/**
 * @file door_fsm.h
 * @brief Door control Finite State Machine public interface.
 *
 * The FSM runs exclusively inside the ControlTask (Core 1, Priority 4).
 * All external reads of the FSM state go through fsm_get_state() which
 * acquires the FSM mutex — direct struct reads from other tasks are forbidden
 * (NFR-6).
 */

#ifndef DOOR_FSM_H
#define DOOR_FSM_H

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "system_event.h"

/* ---------------------------------------------------------------------------
 * FSM States (description.md §4)
 * --------------------------------------------------------------------------- */
typedef enum {
    FSM_STATE_INIT      = 0, /**< Startup; physical position unknown.         */
    FSM_STATE_HOMING    = 1, /**< Performing boot homing routine (SR-4).      */
    FSM_STATE_IDLE      = 2, /**< Homing done; awaiting first command.        */
    FSM_STATE_OPENING   = 3, /**< Motor driving door toward open limit.       */
    FSM_STATE_OPEN      = 4, /**< Door at fully-open limit switch.            */
    FSM_STATE_CLOSING   = 5, /**< Motor driving door toward closed limit.     */
    FSM_STATE_CLOSED    = 6, /**< Door at fully-closed limit switch.          */
    FSM_STATE_FAULT     = 7, /**< Abnormal; no motion until RESET (SR-4..7). */
} fsm_state_t;

/* ---------------------------------------------------------------------------
 * Emergency Lock Flag
 * SR-6: Once EMERGENCY_OPEN is received, the system must hold OPEN until
 * an explicit RESET.  This flag is shared (mutex-protected).
 * --------------------------------------------------------------------------- */
typedef enum {
    EMERGENCY_LOCK_OFF = 0,
    EMERGENCY_LOCK_ON  = 1,
} emergency_lock_t;

/* ---------------------------------------------------------------------------
 * Fault Codes — stored in NVS (NFR-3)
 * --------------------------------------------------------------------------- */
typedef enum {
    FAULT_NONE              = 0x00,
    FAULT_MOTOR_STALL       = 0x01,
    FAULT_SPOF              = 0x02,
    FAULT_COMM_TIMEOUT      = 0x03,
    FAULT_OBSTRUCTION       = 0x04,  /* Stale obstruction after timeout.      */
    FAULT_NVS_BOOT          = 0x05,  /* Fault persisted from previous session.*/
} fault_code_t;

/* ---------------------------------------------------------------------------
 * Initialisation & Task Entry
 * --------------------------------------------------------------------------- */

/**
 * @brief Initialise FSM context, mutex, and motor stall timer.
 *        Must be called before fsm_control_task is created.
 * @return ESP_OK on success.
 */
esp_err_t fsm_init(void);

/**
 * @brief FreeRTOS task entry point for ControlTask (Core 1, Priority 4).
 * @param pvParameters  Pointer to QueueHandle_t (the dispatcher queue).
 */
void fsm_control_task(void *pvParameters);

/* ---------------------------------------------------------------------------
 * Thread-safe state accessor (FR-5, NFR-6)
 * --------------------------------------------------------------------------- */

/**
 * @brief Return the current FSM state under mutex protection.
 * Safe to call from any task (Dispatcher, Logger, Safety Monitor).
 */
fsm_state_t fsm_get_state(void);

/**
 * @brief Return the current emergency lock status (thread-safe).
 */
emergency_lock_t fsm_get_emergency_lock(void);

/**
 * @brief Return the last recorded fault code (thread-safe).
 */
fault_code_t fsm_get_fault_code(void);

#endif /* DOOR_FSM_H */