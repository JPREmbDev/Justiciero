
#include "classBmi160.h"

#include <freertos/FreeRTOS.h>

extern "C" void app_main();


Bmi160 imu;

void app_main() {

    vTaskDelay(pdMS_TO_TICKS(1000));

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