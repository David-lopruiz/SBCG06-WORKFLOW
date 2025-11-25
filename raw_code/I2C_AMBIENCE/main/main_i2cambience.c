#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c_master.h"
#include "rom/ets_sys.h"
#include "bme68x.h"
#include "esp_log.h"

#define TAG "BME68x"

// Pines I2C
#define I2C_PORT                0
#define SDA_PIN                 21
#define SCL_PIN                 22
#define I2C_FREQ_HZ             100000
#define BME68X_I2C_ADDR         BME68X_I2C_ADDR_LOW  // 0x76 o 0x77

// Handles
static i2c_master_bus_handle_t i2c_bus;
static i2c_master_dev_handle_t bme68x_dev_handle;

/******************* Funciones de interfaz Bosch API *******************/
static int8_t i2c_read(uint8_t reg_addr, uint8_t *data, uint32_t len, void *intf_ptr)
{
    esp_err_t ret;

    // Enviar dirección de registro
    ret = i2c_master_transmit(bme68x_dev_handle, &reg_addr, 1, -1);
    if (ret != ESP_OK) return BME68X_E_COM_FAIL;

    // Leer datos
    ret = i2c_master_receive(bme68x_dev_handle, data, len, -1);
    return (ret == ESP_OK) ? BME68X_OK : BME68X_E_COM_FAIL;
}

static int8_t i2c_write(uint8_t reg_addr, const uint8_t *data, uint32_t len, void *intf_ptr)
{
    esp_err_t ret;
    uint8_t buf[len + 1];

    buf[0] = reg_addr;
    memcpy(&buf[1], data, len);

    ret = i2c_master_transmit(bme68x_dev_handle, buf, len + 1, -1);
    return (ret == ESP_OK) ? BME68X_OK : BME68X_E_COM_FAIL;
}

static void delay_us(uint32_t period, void *intf_ptr)
{
    ets_delay_us(period);
}

/******************* Inicialización del nuevo driver I2C *******************/
static void init_i2c(void)
{
    i2c_master_bus_config_t bus_cfg = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = I2C_PORT,
        .sda_io_num = SDA_PIN,
        .scl_io_num = SCL_PIN,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };

    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_cfg, &i2c_bus));

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = BME68X_I2C_ADDR,
        .scl_speed_hz = I2C_FREQ_HZ,
    };

    ESP_ERROR_CHECK(i2c_master_bus_add_device(i2c_bus, &dev_cfg, &bme68x_dev_handle));
}

/******************* Función principal *******************/
void app_main(void)
{
    ESP_LOGI(TAG, "Inicializando I2C...");
    init_i2c();

    struct bme68x_dev dev;
    uint8_t dev_addr = BME68X_I2C_ADDR;

    dev.intf = BME68X_I2C_INTF;
    dev.intf_ptr = &dev_addr;
    dev.read = i2c_read;
    dev.write = i2c_write;
    dev.delay_us = delay_us;
    dev.amb_temp = 25; // temperatura ambiente inicial

    ESP_LOGI(TAG, "Inicializando sensor BME68x...");
    int8_t rslt = bme68x_init(&dev);
    if (rslt != BME68X_OK) {
        ESP_LOGE(TAG, "Error inicializando BME68x: %d", rslt);
        return;
    }

    ESP_LOGI(TAG, "Configurando oversampling...");
    struct bme68x_conf conf_sensor = {
        .filter = BME68X_FILTER_OFF,
        .os_hum = BME68X_OS_2X,
        .os_pres = BME68X_OS_4X,
        .os_temp = BME68X_OS_8X,
        .odr = BME68X_ODR_NONE
    };
    bme68x_set_conf(&conf_sensor, &dev);

    ESP_LOGI(TAG, "Configurando calentador...");
    struct bme68x_heatr_conf heatr_conf = {
        .enable = BME68X_ENABLE,
        .heatr_temp = 300,   // °C
        .heatr_dur = 100     // ms
    };
    bme68x_set_heatr_conf(BME68X_FORCED_MODE, &heatr_conf, &dev);

    ESP_LOGI(TAG, "Iniciando lectura de datos...");

    while (1)
    {
        bme68x_set_op_mode(BME68X_FORCED_MODE, &dev);
        vTaskDelay(pdMS_TO_TICKS(200));  // espera la conversión

        struct bme68x_data data;
        uint8_t n_data = 0;

        rslt = bme68x_get_data(BME68X_FORCED_MODE, &data, &n_data, &dev);
        if (rslt == BME68X_OK && n_data > 0) {
            ESP_LOGI(TAG, "Temp: %.2f °C | Hum: %.2f %% | Pres: %.2f hPa | Gas: %.2f Ω",
                     data.temperature,
                     data.humidity,
                     data.pressure / 100.0,
                     data.gas_resistance);
        } else {
            ESP_LOGW(TAG, "No hay nuevos datos (rslt=%d)", rslt);
        }

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
