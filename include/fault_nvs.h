/**
 * @file fault_nvs.h
 * @brief Non-Volatile Storage interface for fault persistence (NFR-3, SR-4).
 *
 * Fatal faults are written to NVS immediately upon detection.  On every boot,
 * app_main reads NVS before creating tasks; if a persisted fault is found the
 * FSM boots directly into FAULT state (SR-4).
 */

#ifndef FAULT_NVS_H
#define FAULT_NVS_H

#include <stdint.h>
#include "esp_err.h"
#include "door_fsm.h"  /* for fault_code_t */

/**
 * @brief Open NVS handle in read-write mode.
 * Must be called AFTER nvs_flash_init() in app_main.
 */
esp_err_t fault_nvs_open(void);

/**
 * @brief Persist a fatal fault to NVS.
 * @param code  Fault code to record.
 * @return ESP_OK on success.
 */
esp_err_t fault_nvs_write(fault_code_t code);

/**
 * @brief Read back the persisted fault code.
 * @param[out] code  Set to FAULT_NONE if no fault was stored.
 * @return ESP_OK even when no fault stored (code = FAULT_NONE).
 */
esp_err_t fault_nvs_read(fault_code_t *code);

/**
 * @brief Clear any persisted fault (called after successful RESET + re-homing).
 */
esp_err_t fault_nvs_clear(void);

/**
 * @brief Close the NVS handle (call from app shutdown / test teardown).
 */
void fault_nvs_close(void);

#endif /* FAULT_NVS_H */