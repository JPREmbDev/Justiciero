#include <localBmi160.h>
#include <driver/spi_master.h>
#include <driver/gpio.h>
#include <esp_err.h>
#include <esp_log.h>

const char TAG[] = "BMI_DRIVER"; // Etiqueta para logs del driver

/**
 * @brief Inicializa el bus SPI y configura el dispositivo BMI160.
 * 
 * Detalles:
 * - Configura los pines SPI (MISO: GPIO19, MOSI: GPIO23, SCLK: GPIO18, CS: GPIO5).
 * - Usa SPI3_HOST (bus SPI 3 del ESP32) y modo 0 (CPOL=0, CPHA=0).
 * - Realiza una transacción de prueba para verificar la comunicación.
 */
void bmi_initSpi() {
    spi_bus_config_t busSpis = {0};
    spi_device_handle_t spiHandle;
    spi_device_interface_config_t spiInter = {0};

    // Configuración del bus SPI (pines y parámetros)
    busSpis.miso_io_num = GPIO_NUM_19;    // Pin MISO (entrada de datos del sensor)
    busSpis.mosi_io_num = GPIO_NUM_23;    // Pin MOSI (salida de datos al sensor)
    busSpis.sclk_io_num = GPIO_NUM_18;    // Pin de reloj SPI
    busSpis.quadhd_io_num = -1;           // No se usa modo Quad-SPI
    busSpis.quadwp_io_num = -1;           // No se usa modo Quad-SPI
    busSpis.max_transfer_sz = 4092;       // Tamaño máximo de transferencia

    // Inicializa el bus SPI (SPI3_HOST) sin DMA
    if(spi_bus_initialize(SPI3_HOST, &busSpis, SPI_DMA_DISABLED) != ESP_OK) {
        ESP_LOGE(TAG, "Error al inicializar el bus SPI");
        return;
    }
    ESP_LOGI(TAG, "Bus SPI inicializado correctamente");

    // Configuración del dispositivo SPI (BMI160)
    spiInter.spics_io_num = GPIO_NUM_5;   // Pin CS (Chip Select)
    spiInter.clock_speed_hz = 1000000;    // Frecuencia de reloj: 1 MHz (recomendado para BMI160)
    spiInter.mode = 0;                    // Modo SPI 0 (CPOL=0, CPHA=0)
    spiInter.queue_size = 10;             // Máximo de transacciones en cola
    spiInter.address_bits = 8;            // Dirección de 8 bits (compatible con BMI160)

    // Añade el dispositivo al bus SPI
    if (spi_bus_add_device(SPI3_HOST, &spiInter, &spiHandle) != ESP_OK) {
        ESP_LOGE(TAG, "Error al añadir el dispositivo SPI");
        return;
    }
    ESP_LOGI(TAG, "Dispositivo SPI añadido correctamente");

    // Transacción 1: Lectura del registro 0x7F (dirección + bit de lectura 0x80)
    uint8_t data = 0x00;
    spi_transaction_t spiTrans = {0};
    spiTrans.addr = 0x7F | 0x80;  // Máscara 0x80 para operación de lectura (ver datasheet BMI160)
    spiTrans.length = 8;          // Longitud de la dirección (8 bits)
    spiTrans.rxlength = 8;        // Longitud de los datos recibidos (8 bits)
    spiTrans.rx_buffer = &data;   // Buffer de recepción

    // Envía la transacción
    spi_device_acquire_bus(spiHandle, portMAX_DELAY);
    if(spi_device_polling_transmit(spiHandle, &spiTrans) != ESP_OK) {
        ESP_LOGE(TAG, "Error en transacción SPI");
    } else {
        ESP_LOGI(TAG, "Dato recibido: 0x%02X", data); // Debería ser 0xD1 (ID del BMI160)
    }
    spi_device_release_bus(spiHandle);

    // Transacción 2: Lectura del registro 0x00 (prueba adicional)
    spiTrans.addr = 0x00 | 0x80;  // Otra dirección de ejemplo
    spi_device_acquire_bus(spiHandle, portMAX_DELAY);
    if(spi_device_polling_transmit(spiHandle, &spiTrans) != ESP_OK) {
        ESP_LOGE(TAG, "Error en transacción SPI");
    }
    spi_device_release_bus(spiHandle);
}

/**
 * @brief Obtiene el ID del sensor BMI160 (registro 0x00).
 * @return uint8_t - ID del sensor (0xD1 para BMI160).
 */
uint8_t bmi_getId() {
    // Implementación pendiente: usar SPI para leer el registro 0x00
    return 0;
}