#pragma once

#include "esp_err.h"
#include "esp_spp_api.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialize Bluetooth OTA module with SPP profile
 * @param device_name Bluetooth device name for discovery
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t ota_bt_init(const char *device_name);

/**
 * @brief Deinitialize Bluetooth OTA module
 * @return ESP_OK on success
 */
esp_err_t ota_bt_deinit(void);

/**
 * @brief Finish OTA update and restart device
 * @return ESP_OK on success
 */
esp_err_t ota_bt_finish_update(void);

#ifdef __cplusplus
}
#endif
