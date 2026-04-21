/**
 * @file hal.h
 * @brief Hardware Abstraction Layer public interface.
 *
 * ARCHITECTURAL MANDATE (from description.md §2):
 *   "The application layer must never interact directly with peripheral
 *    registers. All inputs/outputs pass through a cleanly defined HAL."
 *
 * This header defines the ONLY surface through which the application touches
 * hardware.  Swapping the mock UART implementation for real GPIO/I2C/SPI
 * sensors requires only a different .c file implementing these functions —
 * zero changes to FSM, Safety, or Dispatcher code.
 *
 * Actuator interface: The HAL exposes three named actuator calls so that the
 * FSM never writes to a GPIO register directly.
 *
 * TX interface: State feedback strings are handed to hal_tx_enqueue() which
 * puts them on the UART TX queue.  The HAL TX task drains that queue with
 * backpressure handling (NFR-5).
 */

#ifndef HAL_H
#define HAL_H

#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "system_event.h"

/* ---------------------------------------------------------------------------
 * HAL Initialisation
 * --------------------------------------------------------------------------- */

/**
 * @brief Initialise all HAL subsystems (UART, debounce state, TX queue).
 * @param dispatcher_queue  FreeRTOS queue handle; HAL pushes events here.
 * @return ESP_OK on success, ESP_FAIL on peripheral error.
 */
esp_err_t hal_init(QueueHandle_t dispatcher_queue);

/* ---------------------------------------------------------------------------
 * HAL Tasks (created by app_main, pinned to CORE_COMMS)
 * --------------------------------------------------------------------------- */

/**
 * @brief UART RX task entry point.
 * Reads raw UART frames, validates CRC-8, debounces sensors, then pushes
 * typed system_event_t structs to the dispatcher queue.
 * Must be pinned to CORE_COMMS.
 */
void hal_uart_rx_task(void *pvParameters);

/**
 * @brief UART TX task entry point.
 * Drains the TX queue and writes frames to UART.  Implements backpressure:
 * if the queue is full the oldest entry is dropped and a log warning is emitted
 * (NFR-5).
 * Must be pinned to CORE_COMMS.
 */
void hal_uart_tx_task(void *pvParameters);

/* ---------------------------------------------------------------------------
 * Actuator Commands (called by FSM — translated to GPIO/motor driver in HAL)
 * --------------------------------------------------------------------------- */

/** @brief Command motor to drive door toward the open position. */
void hal_motor_open(void);

/** @brief Command motor to drive door toward the closed position. */
void hal_motor_close(void);

/**
 * @brief Immediately cut actuator power.
 * SR-1, SR-2: Must be callable from any task; re-entrant.
 */
void hal_motor_stop(void);

/* ---------------------------------------------------------------------------
 * State Feedback TX
 * --------------------------------------------------------------------------- */

/**
 * @brief Enqueue a state string for transmission to the Supervisor.
 * @param state_str  Null-terminated ASCII frame, e.g. "$STATE,STATE=FAULT\n".
 *                   Must be ≤ UART_FRAME_MAX_LEN bytes.
 * @return pdTRUE if enqueued, pdFALSE if queue was full (logged as overflow).
 */
BaseType_t hal_tx_enqueue(const char *state_str);

/* ---------------------------------------------------------------------------
 * Acknowledgement helpers (FR-6)
 * --------------------------------------------------------------------------- */

/** @brief Transmit ACK for a received command sequence number. */
void hal_send_ack(void);

/** @brief Transmit NACK for a malformed / CRC-invalid frame. */
void hal_send_nack(void);

#endif /* HAL_H */