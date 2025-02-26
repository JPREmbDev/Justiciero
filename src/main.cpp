#include "classBmi160.h"
#include "classServo.h"
#include "esp_log.h"

#include <freertos/FreeRTOS.h>

extern "C" void app_main();


Bmi160 imu;
Servo servo;

void app_main() {

    vTaskDelay(pdMS_TO_TICKS(1000));

    servo.initHw();
    
    // Llama a calibrate con valores de ejemplo
    // Ajusta estos valores según las especificaciones de tu servo
    servo.calibrate(500, 2500);

    uint32_t angle = 0;
    for(;;)
    {
        ESP_LOGE("MAIN", "Angle: %lu", angle);
        servo.setPos(angle);
        angle ++;
        if (angle > 180)
        {
            angle = 0;
        }
        vTaskDelay(pdMS_TO_TICKS(50)); // Más tiempo para observar cada posición
    }


    return;

    Bmi160SpiConfig config = {
        .spiHost = SPI3_HOST,
        .csPin = GPIO_NUM_21,
        .sclkPin = GPIO_NUM_18,
        .misoPin = GPIO_NUM_19,
        .mosiPin = GPIO_NUM_23,
        .spiSpeed = 4*1000*1000
    };

    imu.init(config);

    bmi160_sensor_data acc, gyr;

    for(;;)
    {
        imu.getRawData(acc, gyr);
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}