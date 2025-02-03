#include "classBmi160.h"
#include <esp_log.h>

static const char* TAG = "BMI160";


/***** PRIVATE FUNCTION *****/
/**
 * @brief Función de lectura SPI para el sensor BMI160.
 * @return uint8_t - Valor de retorno de la operación.
 */
int8_t bmi_read_spi(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, uint16_t len) {

    spi_transaction_t spiTrans = {0};
    spiTrans.addr = reg_addr;  // Máscara 0x80 para operación de lectura (ver datasheet BMI160)
    spiTrans.length = 8*len;          // Longitud de la dirección (8 bits)
    spiTrans.rxlength = 8*len;        // Longitud de los datos recibidos (8 bits)
    spiTrans.rx_buffer = data;   // Buffer de recepción
    // Envía la transacción
    spi_device_acquire_bus(Bmi160::spiHandle, portMAX_DELAY);
    spi_device_polling_transmit(Bmi160::spiHandle, &spiTrans);
    spi_device_release_bus(Bmi160::spiHandle);

    return 0;
}


int8_t bmi_write_spi(uint8_t dev_addr, uint8_t reg_addr, uint8_t *read_data, uint16_t len) {

    spi_transaction_t spiTrans = {0};
    spiTrans.addr = reg_addr;  // Máscara 0x80 para operación de lectura (ver datasheet BMI160)
    spiTrans.length = 8*len;          // Longitud de la dirección (8 bits)
    spiTrans.tx_buffer = read_data;   // Buffer de recepción
    // Envía la transacción
    spi_device_acquire_bus(Bmi160::spiHandle, portMAX_DELAY);
    spi_device_polling_transmit(Bmi160::spiHandle, &spiTrans);
    spi_device_release_bus(Bmi160::spiHandle);

    return 0;
}


void bmi_delayms_spi(uint32_t period) {

    // Esto lo hacemos así porque el contador mínimo del FreeRTOS en el ESP32 es de 10 ms
    // Por lo que el periodo mínimo que podemos esperar es de 10 ms así como sus multiplos
    if(period < 10) period = 10;
    vTaskDelay(pdMS_TO_TICKS(period));
}



/***** PUBLIC FUNCTION *****/

spi_device_handle_t Bmi160::spiHandle = (nullptr);



uint8_t Bmi160::init(Bmi160SpiConfig spiConfig) {
    
    // La configuracion del spiBus nos viene por parametro 
    spiBus.miso_io_num = spiConfig.misoPin;    // Pin MISO (entrada de datos del sensor)
    spiBus.mosi_io_num = spiConfig.mosiPin;    // Pin MOSI (salida de datos al sensor)
    spiBus.sclk_io_num = spiConfig.sclkPin;    // Pin de reloj SPI
    spiBus.quadhd_io_num = -1;           // No se usa modo Quad-SPI
    spiBus.quadwp_io_num = -1;           // No se usa modo Quad-SPI
    spiBus.max_transfer_sz = 4092;       // Tamaño máximo de transferencia

    // Inicializa el bus SPI (SPI3_HOST) sin DMA
    if(spi_bus_initialize(spiConfig.spiHost, &spiBus, SPI_DMA_DISABLED) != ESP_OK) {
        ESP_LOGE(TAG, "Error!! Bus SPI cannot be initialized");
        return 1;
    }
    ESP_LOGE(TAG, "Bus SPI Initialized correctly!");

    // Configuración del dispositivo SPI (BMI160)
    spiInter.spics_io_num = spiConfig.csPin;         // Pin CS (Chip Select)
    spiInter.clock_speed_hz = spiConfig.spiSpeed;      // Frecuencia de reloj: 1 MHz (recomendado para BMI160)
    spiInter.mode = 0;                          // Modo SPI 0 (CPOL=0, CPHA=0)
    spiInter.queue_size = 10;                   // Máximo de transacciones en cola
    spiInter.address_bits = 8;                  // Dirección de 8 bits (compatible con BMI160)

    // Añade el dispositivo al bus SPI
    if (spi_bus_add_device(spiConfig.spiHost, &spiInter, &spiHandle) != ESP_OK) {
        ESP_LOGE(TAG, "Error!!! Device cannot be added");
        return 1;
    }
    ESP_LOGE(TAG, "SPI Device added correctly!");

    bmi160Dev.intf      = BMI160_SPI_INTF;          // Interfaz SPI
    bmi160Dev.read      = bmi_read_spi;             // Función de lectura SPI
    bmi160Dev.write     = bmi_write_spi;            // Función de escritura SPI
    bmi160Dev.delay_ms  = bmi_delayms_spi;          // Función delay SPI  

    return Configure();

}
uint8_t Bmi160::init(Bmi160I2cConfig i2cConfig) {

    return Configure();
}

/*
    * @brief Función de configuración del sensor BMI160.
    * @return uint8_t - Valor de retorno de la operación.

*/
uint8_t Bmi160::Configure() {

    int8_t rslt = bmi160_init(&bmi160Dev);;

    if (rslt == BMI160_OK)
    {
        ESP_LOGE(TAG, "BMI160 initialization success !\n");
        ESP_LOGE(TAG, "Chip ID 0x%X\n", bmi160Dev.chip_id);
    }
    else
    {
        ESP_LOGE(TAG, "BMI160 initialization failure !\n");
    }

    /* Select the Output data rate, range of accelerometer sensor */
    bmi160Dev.accel_cfg.odr         = BMI160_ACCEL_ODR_1600HZ;          /*! output data rate, cada cuanto el accelerometro va a darme un dato*/
    bmi160Dev.accel_cfg.range       = BMI160_ACCEL_RANGE_16G;           /*! Rango, la escala de medición del accelerometro*/
    bmi160Dev.accel_cfg.bw          = BMI160_ACCEL_BW_OSR4_AVG1;         /*! Ancho de banda, la frecuencia de corte del filtro del accelerometro*/

    /* Select the power mode of accelerometer sensor */
    bmi160Dev.accel_cfg.power       = BMI160_ACCEL_NORMAL_MODE;

    /* Select the Output data rate, range of Gyroscope sensor */
    bmi160Dev.gyro_cfg.odr          = BMI160_GYRO_ODR_3200HZ;
    bmi160Dev.gyro_cfg.range        = BMI160_GYRO_RANGE_2000_DPS;
    bmi160Dev.gyro_cfg.bw           = BMI160_GYRO_BW_NORMAL_MODE;    

    accscale = 16.0f / float(((1<<15)-1));
    gyroscale = 2000.0f / float(((1<<15)-1));

    /* Select the power mode of Gyroscope sensor */
    bmi160Dev.gyro_cfg.power        = BMI160_GYRO_NORMAL_MODE;

    /* Set the sensor configuration */
    rslt = bmi160_set_sens_conf(&bmi160Dev);

    if( rslt != BMI160_OK)
    {
        ESP_LOGE(TAG, "BMI160 configuration failure !\n");
        return rslt;
    }
    else
    {
        ESP_LOGE(TAG, "BMI160 configuration success !\n");
        return rslt;
    }

}


uint8_t Bmi160::getRawData(bmi160_sensor_data &accel, bmi160_sensor_data &gyro) {

    uint8_t ret = bmi160_get_sensor_data(BMI160_ACCEL_SEL | BMI160_GYRO_SEL | BMI160_TIME_SEL, &accel, &gyro, &bmi160Dev);
    ESP_LOGE(TAG,"Accel data      X: %6u, Y: %6u, Z: %6u, Time: %6lu\n", accel.x, accel.y, accel.z, accel.sensortime);
    return ret;
}

// Data esta dentro de la clase Bmi160, por lo que para poder usarla necesitamos específicar que Data es de la clase Bmi160
// y getData es una función de la clase Bmi160
uint8_t Bmi160::getData(Data &accel, Data &gyro) {
    
    bmi160_sensor_data accelRaw, gyroRaw;

    uint8_t ret = getRawData(accelRaw, gyroRaw);

    accel.accX = accelRaw.x * accscale;
    accel.accY = accelRaw.y * accscale;
    accel.accZ = accelRaw.z * accscale;
    accel.time = accelRaw.sensortime * sensorTimeScale;

    gyro.gyroX = gyroRaw.x * gyroscale;
    gyro.gyroY = gyroRaw.y * gyroscale;
    gyro.gyroZ = gyroRaw.z * gyroscale;
    gyro.time = gyroRaw.sensortime * sensorTimeScale;

    return ret;
}