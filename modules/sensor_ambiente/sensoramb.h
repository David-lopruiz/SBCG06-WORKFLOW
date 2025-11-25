#ifndef SENSORAMB_H
#define SENSORAMB_H

#include "bme68x.h"
#include "freertos/FreeRTOS.h"

// Inicia el sensor y crea la tarea en background. `shared_data` debe apuntar
// a una estructura persistente donde se volcarán las lecturas.
// Retorna 0 en éxito o <0 en caso de error.
int sensoramb_start(struct bme68x_data *shared_data);

// Lee de forma segura los datos actuales. timeout_ms es el tiempo máximo en ms
// para adquirir el mutex interno.
int sensoramb_read(struct bme68x_data *out, TickType_t timeout_ms);

// Para la tarea del sensor y libera los recursos asociados.
int sensoramb_stop(void);

#endif // SENSORAMB_H

