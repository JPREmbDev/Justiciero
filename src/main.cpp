#include "classBmi160.h"
#include "classServo.h"
#include "motorController.h"
#include "pid.h"
#include "esp_log.h"
#include "esp_sleep.h" 

#include <freertos/FreeRTOS.h>

extern "C" void app_main();

// --- Constantes para Reintentos ---
#define MAX_INIT_RETRIES 3
#define INIT_RETRY_DELAY_MS 500
// ---------------------------------

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
PID pid(1.0, 0.0, 0.0);

// --- Función Auxiliar para Deep Sleep ---
void enter_deep_sleep(const char* reason) {
    ESP_LOGE("MAIN", "Error CRÍTICO: %s. Entrando en Deep Sleep.", reason);
    vTaskDelay(pdMS_TO_TICKS(1000)); // Pausa para log
    esp_deep_sleep_start();
}
// --------------------------------------


void app_main() {
    // --- Inicialización de Logs ---
    ESP_LOGI("MAIN", "Iniciando programa principal");
    vTaskDelay(pdMS_TO_TICKS(1000));

    // --- Inicialización IMU con Reintentos ---
    ESP_LOGI("MAIN", "Iniciando IMU...");
    // Iniciamos perifericos
    Bmi160SpiConfig config = {
        .spiHost = SPI3_HOST,
        .csPin = GPIO_NUM_22,
        .sclkPin = GPIO_NUM_18,
        .misoPin = GPIO_NUM_19,
        .mosiPin = GPIO_NUM_23,
        .spiSpeed = 4*1000*1000
    }; // 4 MHz, recomendado para BMI160
    bool imu_ok = false;
    for (int attempt = 1; attempt <= MAX_INIT_RETRIES; ++attempt) {
        if (imu.init(config) == 0) {
            imu_ok = true;
            ESP_LOGI("MAIN", "IMU inicializado correctamente (intento %d)", attempt);
            break;
        }
        ESP_LOGW("MAIN", "Fallo al inicializar IMU (intento %d/%d). Reintentando...", attempt, MAX_INIT_RETRIES);
        if (attempt < MAX_INIT_RETRIES) {
            vTaskDelay(pdMS_TO_TICKS(INIT_RETRY_DELAY_MS));
        }
    }
    if (!imu_ok) {
        enter_deep_sleep("Fallo al inicializar IMU después de varios intentos");
    }
    // -----------------------------------------

    // --- Inicialización Pines Motor con Reintentos ---
    ESP_LOGI("MAIN", "Configurando pines de motores...");
    bool pins_ok = false;
    for (int attempt = 1; attempt <= MAX_INIT_RETRIES; ++attempt) {
        // Asumiendo que gpioPinDefinition devuelve true (1) en éxito, false (0) en fallo
        // NOTA: Tu implementación actual siempre devuelve true. Debería modificarse
        // en motorController.cpp para devolver false si gpio_config falla.
        if (motor.gpioPinDefinition(motor1Ain1, motor1Ain2, motor1Pwm, motor2Bin1, motor2Bin2, motor2Pwm)) {
            pins_ok = true;
            ESP_LOGI("MAIN", "Pines de motor configurados correctamente (intento %d)", attempt);
            break;
        }
        ESP_LOGW("MAIN", "Fallo al configurar pines de motor (intento %d/%d). Reintentando...", attempt, MAX_INIT_RETRIES);
        if (attempt < MAX_INIT_RETRIES) {
            vTaskDelay(pdMS_TO_TICKS(INIT_RETRY_DELAY_MS));
        }
    }
    if (!pins_ok) {
        enter_deep_sleep("Fallo al configurar pines de motor después de varios intentos");
    }
    // ---------------------------------------------

    // --- Inicialización Hardware Motor (PWM) con Reintentos ---
    ESP_LOGI("MAIN", "Inicializando hardware de motores (PWM)...");
    bool hw_ok = false;
    for (int attempt = 1; attempt <= MAX_INIT_RETRIES; ++attempt) {
        // Asumiendo que initHw devuelve true (1) en éxito, false (0) en fallo
        if (motor.initHw(leftTimer, rightTimer)) {
            hw_ok = true;
            ESP_LOGI("MAIN", "Hardware de motor inicializado correctamente (intento %d)", attempt);
            break;
        }
        ESP_LOGW("MAIN", "Fallo al inicializar hardware de motor (intento %d/%d). Reintentando...", attempt, MAX_INIT_RETRIES);
        if (attempt < MAX_INIT_RETRIES) {
            vTaskDelay(pdMS_TO_TICKS(INIT_RETRY_DELAY_MS));
        }
    }
    if (!hw_ok) {
        enter_deep_sleep("Fallo al inicializar hardware de motor después de varios intentos");
    }
    // ------------------------------------------------------

    ESP_LOGI("MAIN", "Todas las inicializaciones completadas con éxito.");
    vTaskDelay(pdMS_TO_TICKS(1000));

    float alpha = 0.0f; // Inicializa alpha a 0.0 en grados
    float factor = 0.995f; // Inicializa factor a 0.0
    Bmi160::Data acc, gyro;
    float lastime = 0.0f;
    float dt = 0.0f;

    int32_t motorL, motorR; // Variables para almacenar la velocidad de los motores
     
    for(;;)
    {
        ESP_LOGI("MAIN", "Leyendo datos del IMU...");
        // Paso 1: Calculamos alpha, angulo desde la vertical
        imu.getData(acc, gyro);
        /*
            gyro.x --> lo que estamos girando en º/s

        */
        ESP_LOGI("MAIN", "--------------------------------");
        ESP_LOGI("MAIN", "Giroscopio: x=%8.2f, y=%8.2f, z=%8.2f", gyro.gyroX, gyro.gyroY, gyro.gyroZ);
        ESP_LOGI("MAIN", "--------------------------------");
        ESP_LOGI("MAIN", "Acelerómetro: x=%8.2f, y=%8.2f, z=%8.2f", acc.accX, acc.accY, acc.accZ);  
        ESP_LOGI("MAIN", "--------------------------------");
        ESP_LOGI("MAIN", "Acelerómetro: x=%8.2f",acc.accX);
        ESP_LOGE("MAIN", "--------------------------------");
        ESP_LOGI("MAIN", "Giroscopio: y=%8.2f",gyro.gyroY);
        /*
            Según tenemos colocado el BMI160, ACC.X es el que nos interesa del acelerómetro
            En el caso del giroscopio es el gyro.Y el que nos interesa
        */
        dt = gyro.time - lastime; // Tiempo entre lecturas

        //       factor al estar cerca de 1 hace que el giroscopio tenga más peso
        //       acc.z --> el valor del acelerómetro en z, que es el que nos da la inclinación
        //             --> Nos vamos a ir calibrando en el tiempo con lo que nos dice el acelerómetro
        alpha = ((alpha + gyro.gyroY*dt/1000.0f) * factor) + acc.accX*9.8f* (1 - factor); // Filtro complementario
        ESP_LOGE("MAIN", "--------------------------------");
        ESP_LOGI("MAIN", "Alpha: %8.2f", alpha); // Imprime el valor de alpha

        // Paso 2: Calculamos el movimiento de los motores según alpha
        motorL = pid.update(alpha, dt); // Actualiza el PID con el error y el tiempo
        motorR = pid.update(alpha, dt); // Actualiza el PID con el error y el tiempo
        ESP_LOGE("MAIN", "--------------------------------");
        ESP_LOGI("MAIN", "MotorL: %ld, MotorR: %ld", motorL, motorR);
        motor.setSpeed(motorL, motorR); // Establece la velocidad de los motores
        
        lastime = gyro.time; // Actualiza el tiempo de la última lectura
        vTaskDelay(2);
    }

    
}