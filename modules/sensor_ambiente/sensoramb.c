#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
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
    /* Use the intf_ptr provided by the bme68x_dev structure. We store the
     * i2c device handle (i2c_master_dev_handle_t) in dev.intf_ptr when
     * initializing. Cast it back here and use it for I2C operations so the
     * driver is reentrant and usable as a library. */
    i2c_master_dev_handle_t i2c_dev = (i2c_master_dev_handle_t)intf_ptr;
    if (!i2c_dev) return BME68X_E_COM_FAIL;

    // Enviar dirección de registro
    ret = i2c_master_transmit(i2c_dev, &reg_addr, 1, -1);
    if (ret != ESP_OK) return BME68X_E_COM_FAIL;

    // Leer datos
    ret = i2c_master_receive(i2c_dev, data, len, -1);
    return (ret == ESP_OK) ? BME68X_OK : BME68X_E_COM_FAIL;
}

static int8_t i2c_write(uint8_t reg_addr, const uint8_t *data, uint32_t len, void *intf_ptr)
{
    esp_err_t ret;
    i2c_master_dev_handle_t i2c_dev = (i2c_master_dev_handle_t)intf_ptr;
    if (!i2c_dev) return BME68X_E_COM_FAIL;

    uint8_t buf[len + 1];
    buf[0] = reg_addr;
    memcpy(&buf[1], data, len);

    ret = i2c_master_transmit(i2c_dev, buf, len + 1, -1);
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

/******************* Task-based API para uso como librería *******************/

typedef struct {
    struct bme68x_dev *dev;               // sensor device (heap allocated)
    struct bme68x_data *shared_data;      // puntero proporcionado por el llamador
    SemaphoreHandle_t lock;               // mutex para proteger shared_data
    TaskHandle_t task;                    // handle de la task creada
    volatile bool stop_requested;         // señal para parar la task
} sensor_ctx_t;

/* Contexto global del módulo. Permite implementar sensoramb_read/stop. */
static sensor_ctx_t *global_ctx = NULL;

static void sensor_task(void *arg)
{
    sensor_ctx_t *ctx = (sensor_ctx_t *)arg;
    struct bme68x_data sample;
    uint8_t n_data = 0;
    int8_t rslt;

    if (!ctx || !ctx->dev || !ctx->shared_data) {
        ESP_LOGE(TAG, "sensor_task: contexto inválido");
        vTaskDelete(NULL);
        return;
    }

    while (1) {
        if (ctx->stop_requested) break;
        rslt = bme68x_set_op_mode(BME68X_FORCED_MODE, ctx->dev);
        if (rslt != BME68X_OK) {
            ESP_LOGW(TAG, "bme68x_set_op_mode error: %d", rslt);
        }

        vTaskDelay(pdMS_TO_TICKS(200));  // espera la conversión

        n_data = 0;
        rslt = bme68x_get_data(BME68X_FORCED_MODE, &sample, &n_data, ctx->dev);
        if (rslt == BME68X_OK && n_data > 0) {
            // Copiar al buffer compartido protegido por mutex
            if (xSemaphoreTake(ctx->lock, pdMS_TO_TICKS(10)) == pdTRUE) {
                *(ctx->shared_data) = sample;
                xSemaphoreGive(ctx->lock);
            }

            ESP_LOGI(TAG, "Temp: %.2f °C | Hum: %.2f %% | Pres: %.2f hPa | Gas: %.2f Ω",
                     sample.temperature,
                     sample.humidity,
                     sample.pressure / 100.0,
                     sample.gas_resistance);
        } else {
            ESP_LOGW(TAG, "No hay nuevos datos (rslt=%d)", rslt);
        }

        vTaskDelay(pdMS_TO_TICKS(1000));
    }

    ESP_LOGI(TAG, "sensor_task: finalizando");
    vTaskDelete(NULL);
}

// Inicia el sensor y lanza la tarea. El llamador debe proporcionar un puntero
// a una estructura `struct bme68x_data` donde el task volcará las lecturas.
// Devuelve 0 en éxito o <0 en error.
int sensoramb_start(struct bme68x_data *shared_data)
{
    if (!shared_data) return -1;

    ESP_LOGI(TAG, "Inicializando I2C...");
    init_i2c();

    struct bme68x_dev *dev = calloc(1, sizeof(*dev));
    if (!dev) return -2;

    // Configurar dispositivo BME68x. Pasamos el handle I2C como intf_ptr para
    // que las funciones i2c_read/i2c_write lo usen.
    dev->intf = BME68X_I2C_INTF;
    dev->intf_ptr = (void *)bme68x_dev_handle;
    dev->read = i2c_read;
    dev->write = i2c_write;
    dev->delay_us = delay_us;
    dev->amb_temp = 25;

    ESP_LOGI(TAG, "Inicializando sensor BME68x...");
    int8_t rslt = bme68x_init(dev);
    if (rslt != BME68X_OK) {
        ESP_LOGE(TAG, "Error inicializando BME68x: %d", rslt);
        free(dev);
        return -3;
    }

    struct bme68x_conf conf_sensor = {
        .filter = BME68X_FILTER_OFF,
        .os_hum = BME68X_OS_2X,
        .os_pres = BME68X_OS_4X,
        .os_temp = BME68X_OS_8X,
        .odr = BME68X_ODR_NONE
    };
    bme68x_set_conf(&conf_sensor, dev);

    struct bme68x_heatr_conf heatr_conf = {
        .enable = BME68X_ENABLE,
        .heatr_temp = 300,
        .heatr_dur = 100
    };
    bme68x_set_heatr_conf(BME68X_FORCED_MODE, &heatr_conf, dev);

    sensor_ctx_t *ctx = calloc(1, sizeof(*ctx));
    if (!ctx) {
        free(dev);
        return -4;
    }
    ctx->dev = dev;
    ctx->shared_data = shared_data;
    ctx->lock = xSemaphoreCreateMutex();
    if (!ctx->lock) {
        free(dev);
        free(ctx);
        return -5;
    }
    ctx->stop_requested = false;
    ctx->task = NULL;

    /* Guardar referencia global para sensoramb_read/stop */
    global_ctx = ctx;

    BaseType_t ok = xTaskCreate(sensor_task, "sensor_task", 4096, ctx, tskIDLE_PRIORITY + 5, &(ctx->task));
    if (ok != pdPASS) {
        ESP_LOGE(TAG, "Error creando task del sensor");
        vSemaphoreDelete(ctx->lock);
        free(dev);
        free(ctx);
        return -6;
    }

    return 0;
}

// Copia de forma segura los datos actuales a `out`. timeout_ms es el tiempo
// máximo para adquirir el mutex (en ms). Devuelve 0 en éxito, <0 en error.
int sensoramb_read(struct bme68x_data *out, TickType_t timeout_ms)
{
    if (!out) return -1;
    if (!global_ctx) return -2;

    if (xSemaphoreTake(global_ctx->lock, pdMS_TO_TICKS(timeout_ms)) != pdTRUE) return -3;
    *out = *(global_ctx->shared_data);
    xSemaphoreGive(global_ctx->lock);
    return 0;
}

// Para la tarea del sensor y libera recursos. Devuelve 0 en éxito.
int sensoramb_stop(void)
{
    if (!global_ctx) return -1;

    // Señalamos a la tarea que pare (si está corriendo) y la eliminamos si no responde
    global_ctx->stop_requested = true;
    if (global_ctx->task != NULL) {
        // Dar un pequeño margen para que la tarea termine de forma limpia
        vTaskDelay(pdMS_TO_TICKS(200));
        // Si sigue viva, forzamos la eliminación
        vTaskDelete(global_ctx->task);
    }

    if (global_ctx->lock) vSemaphoreDelete(global_ctx->lock);
    if (global_ctx->dev) free(global_ctx->dev);
    free(global_ctx);
    global_ctx = NULL;
    return 0;
}
