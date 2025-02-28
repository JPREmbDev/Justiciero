#include "classBmi160.h"
#include "classServo.h"
#include "motorController.h"
#include "esp_log.h"

#include <freertos/FreeRTOS.h>

extern "C" void app_main();

/* MOTOR 1*/
constexpr gpio_num_t    motor1Ain1      = GPIO_NUM_33;
constexpr gpio_num_t    motor1Ain2      = GPIO_NUM_25;
constexpr gpio_num_t    motor1Pwm       = GPIO_NUM_32;
constexpr ledc_timer_t  rightTimer       = LEDC_TIMER_2;

/* MOTOR 2*/
constexpr gpio_num_t    motor2Bin1      = GPIO_NUM_26;
constexpr gpio_num_t    motor2Bin2      = GPIO_NUM_27;
constexpr gpio_num_t    motor2Pwm       = GPIO_NUM_14;
constexpr ledc_timer_t  leftTimer       = LEDC_TIMER_1;


Bmi160 imu;
Servo servo;
motorController motor;

void app_main() {
    
    ESP_LOGI("MAIN", "Iniciando programa principal");
    vTaskDelay(pdMS_TO_TICKS(1000));

    servo.initHw();
    motor.gpioPinDefinition(motor1Ain1, motor1Ain2, motor1Pwm, motor2Bin1, motor2Bin2, motor2Pwm);
    motor.initHw(leftTimer, rightTimer);
    
    for(;;)
    {
        ESP_LOGI("MAIN", "Setting direction: left=true, right=false");
        motor.setDirection(true, false);
        motor.setSpeed(512, 512); // Ajusta la velocidad de los motores
        vTaskDelay(pdMS_TO_TICKS(2000));

        ESP_LOGI("MAIN", "Setting direction: left=false, right=true");
        motor.setDirection(false, true);
        motor.setSpeed(512, 512); // Ajusta la velocidad de los motores
        vTaskDelay(pdMS_TO_TICKS(2000));
    }

    return;
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