#include "motorController.h"
#include "esp_log.h"

static const char* TAG = "MotorController";

motorController::motorController(){}

uint8_t motorController::gpioPinDefinition(gpio_num_t leftA, gpio_num_t leftB, gpio_num_t leftPwm,
    gpio_num_t rightA, gpio_num_t rightB, gpio_num_t rightPwm)
{
    ESP_LOGI(TAG, "Configurando pines GPIO para el motor...");

    if (leftA < 0 || leftB < 0 || leftPwm < 0 || rightA < 0 || rightB < 0 || rightPwm < 0) {
        ESP_LOGE(TAG, "Error: uno o más pines GPIO son inválidos.");
        return false;
    }

    leftPinA = leftA;
    leftPinB = leftB;
    leftPinPwm = leftPwm;
    rightPinA = rightA;
    rightPinB = rightB;
    rightPinPwm = rightPwm;
    leftPwmChannel = LEDC_CHANNEL_1;
    rightPwmChannel = LEDC_CHANNEL_2;

    ESP_LOGI(TAG, "Configuración de pines GPIO completada:");
    ESP_LOGI(TAG, "  leftPinA: %d", leftPinA);
    ESP_LOGI(TAG, "  leftPinB: %d", leftPinB);
    ESP_LOGI(TAG, "  leftPinPwm: %d", leftPinPwm);
    ESP_LOGI(TAG, "  rightPinA: %d", rightPinA);
    ESP_LOGI(TAG, "  rightPinB: %d", rightPinB);
    ESP_LOGI(TAG, "  rightPinPwm: %d", rightPinPwm);
    ESP_LOGI(TAG, "  leftPwmChannel: %d", leftPwmChannel);
    ESP_LOGI(TAG, "  rightPwmChannel: %d", rightPwmChannel);

    return true;
}

uint8_t motorController::initHw(ledc_timer_t leftTimer, ledc_timer_t rightTimer)
{
    ESP_LOGI(TAG, "Inicializando hardware del motor...");

    ledc_timer_config_t ledc_timer_left = {
        .speed_mode       = LEDC_LOW_SPEED_MODE,
        .duty_resolution  = LEDC_TIMER_10_BIT,
        .timer_num        = leftTimer,
        .freq_hz          = 500,
        .clk_cfg          = LEDC_AUTO_CLK
    };
    esp_err_t err = ledc_timer_config(&ledc_timer_left);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error al configurar el timer LEDC izquierdo: %s", esp_err_to_name(err));
        return false;
    }
    ESP_LOGI(TAG, "Timer LEDC izquierdo configurado correctamente.");

    ledc_timer_config_t ledc_timer_right = {
        .speed_mode       = LEDC_LOW_SPEED_MODE,
        .duty_resolution  = LEDC_TIMER_10_BIT,
        .timer_num        = rightTimer,
        .freq_hz          = 500,
        .clk_cfg          = LEDC_AUTO_CLK
    };
    err = ledc_timer_config(&ledc_timer_right);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error al configurar el timer LEDC derecho: %s", esp_err_to_name(err));
        return false;
    }
    ESP_LOGI(TAG, "Timer LEDC derecho configurado correctamente.");

    ledc_channel_config_t ledcChannelLeft = {
        .gpio_num               = leftPinPwm,
        .speed_mode             = LEDC_LOW_SPEED_MODE,
        .channel                = leftPwmChannel,
        .intr_type              = LEDC_INTR_DISABLE,
        .timer_sel              = leftTimer,
        .duty                   = 0,
        .hpoint                 = 0,
        .flags = {
            .output_invert      = 0
        }
    };
    err = ledc_channel_config(&ledcChannelLeft);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error al configurar el canal LEDC izquierdo: %s", esp_err_to_name(err));
        return false;
    }
    ESP_LOGI(TAG, "Canal LEDC izquierdo configurado correctamente.");

    ledc_channel_config_t ledcChannelRight = {
        .gpio_num               = rightPinPwm,
        .speed_mode             = LEDC_LOW_SPEED_MODE,
        .channel                = rightPwmChannel,
        .intr_type              = LEDC_INTR_DISABLE,
        .timer_sel              = rightTimer,
        .duty                   = 0,
        .hpoint                 = 0,
        .flags = {
            .output_invert      = 0
        }
    };
    err = ledc_channel_config(&ledcChannelRight);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error al configurar el canal LEDC derecho: %s", esp_err_to_name(err));
        return false;
    }
    ESP_LOGI(TAG, "Canal LEDC derecho configurado correctamente.");

    gpio_config_t gpioConfig = {
        .pin_bit_mask   = ((uint64_t)1 << leftPinA) | ((uint64_t)1 << leftPinB) | ((uint64_t)1 << rightPinA) | ((uint64_t)1 << rightPinB),
        .mode           = GPIO_MODE_OUTPUT,
        .pull_up_en     = GPIO_PULLUP_DISABLE,
        .pull_down_en   = GPIO_PULLDOWN_DISABLE,
        .intr_type      = GPIO_INTR_DISABLE
    };
    err = gpio_config(&gpioConfig);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error al configurar los pines GPIO: %s", esp_err_to_name(err));
        return false;
    }
    ESP_LOGI(TAG, "Pines GPIO configurados correctamente.");

    gpio_set_level(leftPinA, 0);
    gpio_set_level(leftPinB, 1);
    gpio_set_level(rightPinA, 0);
    gpio_set_level(rightPinB, 1);

    ESP_LOGI(TAG, "Hardware del motor inicializado correctamente.");
    return true;
}

uint8_t motorController::setSpeed(int32_t leftSpeed, int32_t rightSpeed)
{
    ESP_LOGI(TAG, "Estableciendo velocidad del motor: izquierda=%ld, derecha=%ld", leftSpeed, rightSpeed);

    // Asegurarse de que la velocidad esté en el rango permitido
    if (leftSpeed < 0) {
        leftSpeed = -leftSpeed;
    }

    if (rightSpeed < 0) {
        rightSpeed = -rightSpeed;
    }

    // Configuración de la velocidad del motor izquierdo
    esp_err_t err = ledc_set_duty(LEDC_LOW_SPEED_MODE, leftPwmChannel, leftSpeed);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error al establecer el duty cycle del motor izquierdo: %s", esp_err_to_name(err));
        return false;
    }
    err = ledc_update_duty(LEDC_LOW_SPEED_MODE, leftPwmChannel);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error al actualizar el duty cycle del motor izquierdo: %s", esp_err_to_name(err));
        return false;
    }
    ESP_LOGI(TAG, "Velocidad del motor izquierdo establecida correctamente.");

    // Configuración de la velocidad del motor derecho
    err = ledc_set_duty(LEDC_LOW_SPEED_MODE, rightPwmChannel, rightSpeed);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error al establecer el duty cycle del motor derecho: %s", esp_err_to_name(err));
        return false;
    }
    err = ledc_update_duty(LEDC_LOW_SPEED_MODE, rightPwmChannel);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error al actualizar el duty cycle del motor derecho: %s", esp_err_to_name(err));
        return false;
    }
    ESP_LOGI(TAG, "Velocidad del motor derecho establecida correctamente.");

    return true;
}

uint8_t motorController::setDirection(bool left, bool right)
{
    ESP_LOGI(TAG, "Setting motor direction: left=%d, right=%d", left, right);

    if (left) {
        gpio_set_level(leftPinA, 1);
        gpio_set_level(leftPinB, 0);
    } else {
        gpio_set_level(leftPinA, 0);
        gpio_set_level(leftPinB, 1);
    }

    if (right) {
        gpio_set_level(rightPinA, 0);
        gpio_set_level(rightPinB, 1);
    } else {
        gpio_set_level(rightPinA, 1);
        gpio_set_level(rightPinB, 0);
    }

    return true;
}