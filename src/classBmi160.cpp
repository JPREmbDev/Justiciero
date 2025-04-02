#include "classBmi160.h"
#include <esp_log.h>

static const char* TAG = "BMI160";


/***** PRIVATE FUNCTION *****/
/**
 * @brief Función de lectura SPI para el sensor BMI160.
 * @param dev_addr Dirección del dispositivo (no usado en SPI, pero requerido por la API).
 * @param reg_addr Dirección del registro a leer.
 * @param data Buffer donde se almacenarán los datos leídos.
 * @param len Longitud de datos a leer en bytes.
 * @return int8_t 0 si la operación fue exitosa, código de error en caso contrario.
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

/**
 * @brief Función de escritura SPI para el sensor BMI160.
 * @param dev_addr Dirección del dispositivo (no usado en SPI, pero requerido por la API).
 * @param reg_addr Dirección del registro a escribir.
 * @param read_data Buffer que contiene los datos a escribir.
 * @param len Longitud de datos a escribir en bytes.
 * @return int8_t 0 si la operación fue exitosa, código de error en caso contrario.
 */
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

/**
 * @brief Función de retardo para el sensor BMI160.
 * @param period Tiempo de retardo en milisegundos.
 * @note El periodo mínimo es de 10ms debido a limitaciones del FreeRTOS en ESP32.
 */
void bmi_delayms_spi(uint32_t period) {

    // Esto lo hacemos así porque el contador mínimo del FreeRTOS en el ESP32 es de 10 ms
    // Por lo que el periodo mínimo que podemos esperar es de 10 ms así como sus multiplos
    if(period < 10) period = 10;
    vTaskDelay(pdMS_TO_TICKS(period));
}



/***** PUBLIC FUNCTION *****/

spi_device_handle_t Bmi160::spiHandle = (nullptr);

/**
 * @brief Inicializa el sensor BMI160 mediante interfaz SPI.
 * @param spiConfig Estructura con la configuración SPI.
 * @return uint8_t 0 si la inicialización fue exitosa, código de error en caso contrario.
 */
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

/**
 * @brief Inicializa el sensor BMI160 mediante interfaz I2C.
 * @param i2cConfig Estructura con la configuración I2C.
 * @return uint8_t 0 si la inicialización fue exitosa, código de error en caso contrario.
 * @note Actualmente esta función no implementa la inicialización I2C.
 */
uint8_t Bmi160::init(Bmi160I2cConfig i2cConfig) {
    // Nota: La implementación I2C no está completada
    return Configure();
}

/**
 * @brief Configura los parámetros del sensor BMI160.
 * @return uint8_t 0 si la configuración fue exitosa, código de error en caso contrario.
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
    /*
     * (1<<15) desplazamiento de 15 bits ==> 1000 0000 0000 0000
     * 1<<15 - 1 = 32767
     * 16.0f / 32767 = 0.00048828125 float
     * Si el aceleremetro mide 16384, entonces el rango es de 16g:
     *      16384 * (16.0 / 32767) ≈ 8.0g
     * Si el giroscopio mide -8192, entonces el rango es de 2000 dps:
     *      -8192 * (2000.0 / 32767) ≈ -500°/s
     * 2000.0f / 32767 = 0.06103515625 float
    */
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

/**
 * @brief Obtiene los datos crudos del sensor BMI160.
 * @param accel Estructura donde se almacenarán los datos crudos del acelerómetro.
 * @param gyro Estructura donde se almacenarán los datos crudos del giroscopio.
 * @return uint8_t 0 si la lectura fue exitosa, código de error en caso contrario.
 */
uint8_t Bmi160::getRawData(bmi160_sensor_data &accel, bmi160_sensor_data &gyro) {

    uint8_t ret = bmi160_get_sensor_data(BMI160_ACCEL_SEL | BMI160_GYRO_SEL | BMI160_TIME_SEL, &accel, &gyro, &bmi160Dev);
    ESP_LOGE(TAG,"Accel data      X: %6u, Y: %6u, Z: %6u, Time: %6lu\n", accel.x, accel.y, accel.z, accel.sensortime);
    return ret;
}

/**
 * @brief Obtiene los datos procesados del sensor BMI160 con las unidades físicas correctas.
 * @param accel Estructura donde se almacenarán los datos del acelerómetro en unidades g.
 * @param gyro Estructura donde se almacenarán los datos del giroscopio en grados por segundo.
 * @return uint8_t 0 si la lectura fue exitosa, código de error en caso contrario.
 * @note Los datos son convertidos de los valores crudos a unidades físicas usando los factores de escala.
 */
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

/**
 * @brief Calcula los ángulos de orientación utilizando datos del acelerómetro y giroscopio.
 * @param angles Estructura donde se almacenarán los ángulos calculados (roll, pitch, yaw).
 * @param dt Intervalo de tiempo entre mediciones en segundos.
 * @param alpha Factor de peso para el filtro complementario (0.0-1.0).
 * @return uint8_t 0 si el cálculo fue exitoso, código de error en caso contrario.
 * @note El filtro complementario combina datos del acelerómetro (estables a largo plazo) 
 *       con datos del giroscopio (precisos a corto plazo).
 */
uint8_t Bmi160::getAngles(AngleData &angles, float dt, float alpha) {
    // Obtenemos los datos del acelerómetro y giroscopio
    Data accel, gyro;
    uint8_t ret = getData(accel, gyro);
    
    if (ret != 0) {
        /*
         * Error al leer los datos del sensor
         * Aquí debemos de meter ESP_LOGE para que nos muestre el error
        */ 
        return ret;
    }
    
    // Cálculo de los ángulos a partir del acelerómetro
    // Convertimos de radianes a grados (multiplicando por 180/PI = 57.296)
    float accelRoll = atan2f(accel.accY, accel.accZ) * 57.296f;
    float accelPitch = atan2f(-accel.accX, sqrtf(accel.accY * accel.accY + accel.accZ * accel.accZ)) * 57.296f;
    
    // No se puede calcular el yaw directamente del acelerómetro
    
    if (angles.time == 0) {
        // Primera medición - inicializamos con los valores del acelerómetro
        angles.roll = accelRoll;
        angles.pitch = accelPitch;
        angles.yaw = 0; // El yaw inicial lo establecemos en 0
    } 
    else {
        // Aplicamos el filtro complementario
        
        // Para roll y pitch, combinamos acelerómetro con giroscopio
        angles.roll = alpha * (angles.roll + gyro.gyroX * dt) + (1 - alpha) * accelRoll;
        angles.pitch = alpha * (angles.pitch + gyro.gyroY * dt) + (1 - alpha) * accelPitch;
        
        // Para yaw, solo usamos el giroscopio (no hay referencia absoluta del acelerómetro)
        angles.yaw += gyro.gyroZ * dt;
        
        // Opcional: Normalizar yaw a rango de -180 a 180 grados
        if (angles.yaw > 180.0f) angles.yaw -= 360.0f;
        if (angles.yaw < -180.0f) angles.yaw += 360.0f;
    }
    
    // Actualizamos el tiempo de la última medición
    angles.time = accel.time;
    
    return 0;
}