#include <stdio.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "ota_bt_update.h"

static const char *TAG = "app_main";

void app_main(void)
{
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, " ESP32 Bluetooth OTA (SPP) - Final v2.1");
    ESP_LOGI(TAG, "========================================");

    // Inicializar NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_LOGI(TAG, "Formateando NVS...");
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // Inicializar Bluetooth OTA
    ESP_ERROR_CHECK(ota_bt_init("ESP32_OTA_SPP"));

    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "✅ Sistema listo para OTA Bluetooth");
    ESP_LOGI(TAG, "   PIN: 1234");
    ESP_LOGI(TAG, "   Nombre: ESP32_OTA_SPP");
    ESP_LOGI(TAG, "========================================");

    // Loop principal
    uint32_t loop_count = 0;
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(10000));
        loop_count++;
        ESP_LOGI(TAG, "Activo... (%" PRIu32 " min)", loop_count / 6);
    }
}
