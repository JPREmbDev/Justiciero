#ifndef CLASS_BMI160_H__
#define CLASS_BMI160_H__

#include <driver/gpio.h>
#include <driver/spi_master.h>
#include <driver/i2c_master.h>

#include <bmi160.h>

// Estructuras de configuracion
typedef struct {
    spi_host_device_t spiHost; // SPI host device

    gpio_num_t csPin;          // Chip select pin
    gpio_num_t sclkPin;        // Serial clock pin
    gpio_num_t misoPin;        // Master in slave out pin
    gpio_num_t mosiPin;        // Master out slave in pin

    uint32_t spiSpeed;         // SPI speed in Hz

} Bmi160SpiConfig;

typedef struct {
    gpio_num_t sdaPin;         // Serial data pin
    gpio_num_t sclPin;         // Serial clock pin
    gpio_num_t i2cAddress;     // I2C address

} Bmi160I2cConfig;

class Bmi160 {

    public:

        // Declaramos la estructura de datos sin 'typedef'
        struct Data {
            float accX;    // Accelerometer X-axis data
            float accY;    // Accelerometer Y-axis data
            float accZ;    // Accelerometer Z-axis data
            float gyroX;   // Gyroscope X-axis data
            float gyroY;   // Gyroscope Y-axis data
            float gyroZ;   // Gyroscope Z-axis data
            float time;    // Timestamp
        };   
            
        // Constructor, mejor solo inicializar variables, pero no hacer nada mas
        // Constructor con lista de inicialización
        Bmi160(): 
            bmi160Dev(),            // Se asume que bmi160_dev tiene un constructor por defecto
            spiBus(),               // Inicializa spiBus
            spiInter()              // Inicializa spiInter
            
        {
            // Inicializaciones adicionales, si son necesarias.
        }

        // Gracias a la sobrecarga de funciones, se pueden tener dos funciones con el mismo nombre
        // y solo se usa aquella a la que se le pasa la configuracion correcta
        /**
         * @brief Initialize the BMI160 sensor with SPI configuration
         * 
         * @param spiConfig Configuration structure for SPI
         * @return uint8_t Status of the initialization
         */
        uint8_t init(Bmi160SpiConfig spiConfig);

        /**
         * @brief Initialize the BMI160 sensor with I2C configuration
         * 
         * @param i2cConfig Configuration structure for I2C
         * @return uint8_t Status of the initialization
         */
        uint8_t init(Bmi160I2cConfig i2cConfig);

        // Métodos para obtener datos, marcados como const si no modifican el objeto
        // Necesitariamos leer los datos en raw, para luego convertirlos a valores reales
        /**
         * @brief Get raw data from the BMI160 sensor
         * 
         * @param accel Structure to store raw accelerometer data
         * @param gyro Structure to store raw gyroscope data
         * @return uint8_t Status of the operation
         */
        uint8_t getRawData(bmi160_sensor_data &accel, bmi160_sensor_data &gyro);

        // Funcion que parsea los datos de la estructura bmi160_sensor_data y nos lo devulve con las unidades correctas
        /**
         * @brief Get parsed data from the BMI160 sensor
         * 
         * @param accel Structure to store parsed accelerometer data
         * @param gyro Structure to store parsed gyroscope data
         * @return uint8_t Status of the operation
         */
        uint8_t getData(Data &accel, Data &gyro);

        /**
         * @brief Configure the BMI160 sensor
         * 
         * @return uint8_t Status of the configuration
         */
        uint8_t Configure();

        // Le damos la condicion de static para que sea común para todos los objetos de la clase y tengo la misma dirección de memoria
        // Así no se duplica la variable para cada objeto, sino que es compartida por todos, por lo que es una propiedad de la clase no del objeto instanciado
        // Como vamos a tener un Handle que sea para todos el mismo, una vez que se inicialice.
        // Static handle for SPI device
        static spi_device_handle_t spiHandle;

    private:

        bmi160_dev bmi160Dev;                                   // BMI160 device structure  
        static constexpr float sensorTimeScale = 0.0390625f;    // Sensor time scale constant
        float accscale;                                         // Accelerometer scale factor 
        float gyroscale;                                        // Gyroscope scale factor
        spi_bus_config_t spiBus;                                // SPI bus configuration
        spi_device_interface_config_t spiInter;                 // SPI device interface configuration
};

#endif // CLASS_BMI160_H__