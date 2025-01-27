#include "localBmi160.h"
#include <freertos/FreeRTOS.h>

extern "C" void app_main();

/**
 * @brief Función principal de la aplicación.
 * 
 * Flujo:
 * 1. Espera 1 segundo para estabilizar el hardware.
 * 2. Inicializa el SPI y el sensor BMI160.
 * 3. Entra en un bucle infinito.
 */
void app_main() {
    vTaskDelay(pdMS_TO_TICKS(1000)); // Espera para evitar problemas de inicialización
    bmi_initSpi(); // Configura SPI y BMI160

    for(;;) {
        vTaskDelay(pdMS_TO_TICKS(1000)); // Reduce el consumo de CPU
    }
}