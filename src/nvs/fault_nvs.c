#include <string.h>

#include "nvs_flash.h"
#include "nvs.h"
#include "esp_log.h"

#include "fault_nvs.h"
#include "config.h"

static const char *TAG = "NVS";

static nvs_handle_t s_nvs_handle = 0;
static uint8_t      s_open       = 0u;

esp_err_t fault_nvs_open(void)
{
    esp_err_t ret = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &s_nvs_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "nvs_open failed: %s", esp_err_to_name(ret));
        return ret;
    }
    s_open = 1u;
    ESP_LOGI(TAG, "NVS namespace '%s' opened", NVS_NAMESPACE);
    return ESP_OK;
}

esp_err_t fault_nvs_write(fault_code_t code)
{
    if (!s_open) {
        ESP_LOGE(TAG, "NVS not open — cannot persist fault");
        return ESP_ERR_INVALID_STATE;
    }

    /* Write fault state flag (1 = fault active). */
    esp_err_t ret = nvs_set_u8(s_nvs_handle, NVS_KEY_FAULT_STATE, 1u);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "nvs_set fault_state failed: %s", esp_err_to_name(ret));
        return ret;
    }

    /* Write fault code. */
    ret = nvs_set_u8(s_nvs_handle, NVS_KEY_FAULT_CODE, (uint8_t)code);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "nvs_set fault_code failed: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = nvs_commit(s_nvs_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "nvs_commit failed: %s", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "Fault code %d persisted to NVS", (int)code);
    return ESP_OK;
}

esp_err_t fault_nvs_read(fault_code_t *code)
{
    if (!s_open || code == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t fault_active = 0u;
    esp_err_t ret = nvs_get_u8(s_nvs_handle, NVS_KEY_FAULT_STATE, &fault_active);
    if (ret == ESP_ERR_NVS_NOT_FOUND) {
        /* Key does not exist — clean boot, no fault. */
        *code = FAULT_NONE;
        return ESP_OK;
    }
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "nvs_get fault_state failed: %s", esp_err_to_name(ret));
        return ret;
    }

    if (!fault_active) {
        *code = FAULT_NONE;
        return ESP_OK;
    }

    uint8_t raw_code = 0u;
    ret = nvs_get_u8(s_nvs_handle, NVS_KEY_FAULT_CODE, &raw_code);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "nvs_get fault_code failed: %s", esp_err_to_name(ret));
        *code = FAULT_NVS_BOOT; /* Assume worst case. */
        return ESP_OK;
    }

    *code = (fault_code_t)raw_code;
    ESP_LOGW(TAG, "Persisted fault found on boot: code=%d", (int)*code);
    return ESP_OK;
}

esp_err_t fault_nvs_clear(void)
{
    if (!s_open) {
        return ESP_ERR_INVALID_STATE;
    }

    esp_err_t ret = nvs_set_u8(s_nvs_handle, NVS_KEY_FAULT_STATE, 0u);
    if (ret != ESP_OK) {
        return ret;
    }
    ret = nvs_commit(s_nvs_handle);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "NVS fault cleared");
    }
    return ret;
}

void fault_nvs_close(void)
{
    if (s_open) {
        nvs_close(s_nvs_handle);
        s_open = 0u;
    }
}