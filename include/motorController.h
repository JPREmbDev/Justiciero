#ifndef _MOTORCONTROLLER_H__
#define _MOTORCONTROLLER_H__

#include <driver/gpio.h>
#include "driver/ledc.h"


class motorController {

    public:
        /**
         * @brief Construct a new motor Controller object
         */
        motorController();

        /**
         * @brief Define the GPIO pins for the motor controller
         * 
         * @param leftA GPIO pin for left motor A
         * @param leftB GPIO pin for left motor B
         * @param leftPwm GPIO pin for left motor PWM
         * @param rightA GPIO pin for right motor A
         * @param rightB GPIO pin for right motor B
         * @param rightPwm GPIO pin for right motor PWM
         * @return uint8_t Status of the operation
         */
        uint8_t gpioPinDefinition(gpio_num_t leftA, gpio_num_t leftB, gpio_num_t leftPwm,
                                     gpio_num_t rightA, gpio_num_t rightB, gpio_num_t rightPwm);
        
        /**
         * @brief Initialize PWM and GPIOs, call it only once.
         * 
         * @param leftTimer Timer to use for left motor PWM
         * @param rightTimer Timer to use for right motor PWM
         * @return uint8_t Status of the initialization
         */
        uint8_t initHw(ledc_timer_t leftTimer, ledc_timer_t rightTimer);

        /**
         * @brief Set each motor speed, the value can be negative
         * 
         * @param leftSpeed Speed for the left motor
         * @param rightSpeed Speed for the right motor
         * @return uint8_t Status of the operation
         */
        uint8_t setSpeed(int32_t leftSpeed, int32_t rightSpeed);

        /**
         * @brief Set the direction of the motors
         * 
         * @param left Direction for the left motor (true for forward, false for backward)
         * @param right Direction for the right motor (true for forward, false for backward)
         * @return uint8_t Status of the operation
         */
        uint8_t setDirection(bool left, bool right);
        


    private:
        
        // GPIO pin for left motor A
        gpio_num_t leftPinA;
        // GPIO pin for left motor B
        gpio_num_t leftPinB;
        // GPIO pin for left motor PWM
        gpio_num_t leftPinPwm;
        // GPIO pin for right motor A
        gpio_num_t rightPinA;
        // GPIO pin for right motor B
        gpio_num_t rightPinB;
        // GPIO pin for right motor PWM
        gpio_num_t rightPinPwm;
        // PWM channel for left motor
        ledc_channel_t leftPwmChannel;
        // PWM channel for right motor
        ledc_channel_t rightPwmChannel;

};


#endif // _MOTORCONTROLLER_H__