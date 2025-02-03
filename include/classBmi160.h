#ifndef CLASS_BMI160_H__
#define CLASS_BMI160_H__

#include <driver/gpio.h>
#include <driver/spi_master.h>
#include <driver/i2c_master.h>

#include <bmi160.h>

// Estructuras de configuracion
typedef struct {
    
    // Necesitamos marcar en cual SPI nos conectamos:
    spi_host_device_t spiHost;

    gpio_num_t csPin;
    gpio_num_t sclkPin;
    gpio_num_t misoPin;
    gpio_num_t mosiPin;

    uint32_t spiSpeed;

} Bmi160SpiConfig;

typedef struct {
    
    gpio_num_t sdaPin;
    gpio_num_t sclPin;
    gpio_num_t i2cAddress;

} Bmi160I2cConfig;


class  Bmi160 {

    public:

        // Declaramos la estructura de datos sin 'typedef'
        struct Data {
            float accX;
            float accY;
            float accZ;
            float gyroX;
            float gyroY;
            float gyroZ;
            float time;
        };   
            
        // Constructor, mejor solo inicializar variables, pero no hacer nada mas
        // Constructor con lista de inicialización
        Bmi160()
            : bmi160Dev(),          // Se asume que bmi160_dev tiene un constructor por defecto
            spiBus(),               // Inicializa spiBus
            spiInter()              // Inicializa spiInter
            
        {
            // Inicializaciones adicionales, si son necesarias.
        }

        // Gracias a la sobrecarga de funciones, se pueden tener dos funciones con el mismo nombre
        // y solo se usa aquella a la que se le pasa la configuracion correcta
        uint8_t init(Bmi160SpiConfig spiConfig);
        uint8_t init(Bmi160I2cConfig i2cConfig);

        // Métodos para obtener datos, marcados como const si no modifican el objeto
        // Necesitariamos leer los datos en raw, para luego convertirlos a valores reales
        uint8_t getRawData(bmi160_sensor_data &accel, bmi160_sensor_data &gyro);
        // Funcion que parsea los datos de la estructura bmi160_sensor_data y nos lo devulve con las unidades correctas
        uint8_t getData(Data &accel, Data &gyro);

        uint8_t Configure();

        // Le damos la condicion de static para que sea común para todos los objetos de la clase y tengo la misma dirección de memoria
        // Así no se duplica la variable para cada objeto, sino que es compartida por todos, por lo que es una propiedad de la clase no del objeto instanciado
        // Como vamos a tener un Handle que sea para todos el mismo, una vez que se inicialice,
        static spi_device_handle_t spiHandle;

    private:

        bmi160_dev bmi160Dev;
        static constexpr float sensorTimeScale = 0.0390625f;
        float accscale;
        float gyroscale;

        // SPI
        spi_bus_config_t spiBus;
        spi_device_interface_config_t spiInter;
        
};




#endif // CLASS_BMI160_H__