# sensor_ambiente

Documentación del módulo `sensoramb.c` (BME68x) — interfaz task-based para ESP-IDF.

Contenido principal

- `sensoramb_start(struct bme68x_data *shared_data)` — inicializa I2C y lanza una tarea en background que vuelca lecturas periódicas en `shared_data`.
- `sensoramb_read(struct bme68x_data *out, TickType_t timeout_ms)` — copia de forma segura los datos actuales al struct `out`.
- `sensoramb_stop(void)` — para la tarea y libera los recursos asociados.

Requisitos

- ESP-IDF (FreeRTOS, drivers I2C).
- Archivos en este repo: `bme68x.c`, `bme68x.h`, `bme68x_defs.h`, `sensoramb.c`, `sensoramb.h`.

Uso básico

1. Incluye el header público y crea una estructura persistente `struct bme68x_data`:

```c
#include "sensoramb.h"
#include "bme68x.h"

void app_main(void)
{
    static struct bme68x_data shared_data; // persistente en RAM

    int r = sensoramb_start(&shared_data);
    if (r != 0) {
        // manejo de error (r < 0)
    }

    // La tarea del sensor corre en background.

    // Ejemplo de lectura segura cada 2 s:
    while (1) {
        struct bme68x_data snapshot;
        if (sensoramb_read(&snapshot, 100) == 0) {
            // usar snapshot.temperature, snapshot.humidity, etc.
        }
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}
```

API y comportamiento

- sensoramb_start(struct bme68x_data *shared_data):
  - Inicia el bus I2C y el sensor, crea un contexto interno y una tarea FreeRTOS que realiza lecturas periódicas en modo FORCED.
  - `shared_data` debe apuntar a una estructura válida y persistente en memoria.
  - Retorna 0 en éxito, <0 en error.

- sensoramb_read(struct bme68x_data *out, TickType_t timeout_ms):
  - Copia de forma segura los datos actuales al puntero `out` bajo mutex.
  - `timeout_ms` especifica cuánto tiempo (ms) se intentará tomar el mutex.
  - Retorna 0 en éxito, <0 en error (por ejemplo, timeout o módulo no inicializado).

- sensoramb_stop(void):
  - Señala a la tarea que pare y libera los recursos internos (mutex, contexto, estructuras heap).
  - Retorna 0 en éxito, <0 si no había contexto.

Concurrencia y seguridad

- La tarea del sensor escribe las lecturas periódicamente en `shared_data`. El módulo crea un mutex interno para proteger la escritura.
- Usa `sensoramb_read()` para leer de forma segura. Evita acceder directamente a `shared_data` desde fuera si no controlas la sincronización.

Parar y liberar recursos

- Llama a `sensoramb_stop()` para detener la tarea y liberar la memoria asignada por `sensoramb_start()`.
- Después de `sensoramb_stop()` necesitarás llamar de nuevo a `sensoramb_start()` para reiniciar el sensor.

Build e integración

- Añade `sensoramb.c` y los archivos BME68x a tu componente (por ejemplo `main/`) o compónlos como componente separado.
- Usa el flujo estándar de ESP-IDF (`idf.py build`, `idf.py flash`). Asegúrate de tener el includePath configurado en el editor si ves advertencias sobre headers de FreeRTOS/ESP.

Notas y siguientes mejoras posibles

- Se podrían añadir códigos de error simbólicos (enum) y más logging configurables.
- Si quieres, puedo añadir un ejemplo `app_main.c` separado que muestre `sensoramb_start`, lecturas con `sensoramb_read` y `sensoramb_stop`.

Licencia

Mantén la licencia que corresponda a los archivos BME68x y a tu proyecto. Este README no impone una licencia.

