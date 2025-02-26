#include "classServo.h"
#include <driver/gpio.h>
#include "driver/ledc.h"
#include "esp_err.h"




Servo::Servo() {
    min_pulse_width = 500;  // Valor predeterminado para 0 grados (500µs)
    max_pulse_width = 2500; // Valor predeterminado para 180 grados (2500µs)
}

void Servo::initHw() {

    // Prepare and then apply the LEDC PWM timer configuration
    ledc_timer_config_t ledc_timer = {
        .speed_mode       = LEDC_HIGH_SPEED_MODE,
        .duty_resolution  = LEDC_TIMER_10_BIT,
        .timer_num        = LEDC_TIMER_0,
        .freq_hz          = 50,  // 20 ms --> 50 Hz
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    // Prepare and then apply the LEDC PWM channel configuration
    ledc_channel_config_t ledc_channel = {
        .gpio_num       = GPIO_NUM_15,
        .speed_mode     = LEDC_HIGH_SPEED_MODE,
        .channel        = LEDC_CHANNEL_0,
        .intr_type      = LEDC_INTR_DISABLE,
        .timer_sel      = LEDC_TIMER_0,
        .duty           = 0, // Set duty to 0%
        .hpoint         = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0, 75));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0));
}

void Servo::calibrate(uint32_t min, uint32_t max) {
    min_pulse_width = min;
    max_pulse_width = max;
}

void Servo::setPos(uint32_t pos) {
    // Asegúrate de que el mapeo sea correcto para tu servo
    // uint32_t pulse_width = min_pulse_width + (pos * (max_pulse_width - min_pulse_width) / 180);
    // ESP_ERROR_CHECK(ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0, pulse_width));
    // ESP_ERROR_CHECK(ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0));
    // Limitar el valor de pos entre 0 y 180
    if (pos > 180) pos = 180;
    
    // Calcular el ancho de pulso en microsegundos basado en la posición
    uint32_t pulse_width_us = min_pulse_width + (pos * (max_pulse_width - min_pulse_width) / 180);
    
    // Convertir microsegundos a valor de ciclo de trabajo para LEDC
    // El período completo es 20000 µs (20 ms), y la resolución es 10 bits (1024 valores)
    uint32_t duty = (pulse_width_us * 1024) / 20000;
    
    // Aplicar el ciclo de trabajo
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0, duty));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0));
}