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

        uint8_t gpioPinDefinition(gpio_num_t leftA, gpio_num_t leftB, gpio_num_t leftPwm,
                                     gpio_num_t rightA, gpio_num_t rightB, gpio_num_t rightPwm);
        
        /**
         * @brief initialize PWM and GPIOs, call it only once.
         * 
         * @param timer Timer to use for PWM
         */
        uint8_t initHw(ledc_timer_t leftTimer, ledc_timer_t rightTimer);
        /**
         * @brief Set each motor speed, the value can be negative
         */
        uint8_t setSpeed (int32_t leftSpeed, int32_t rightSpeed);
        /**
         * @brief 
         */
        uint8_t setDirection(bool left, bool right);
        


    private:
        gpio_num_t leftPinA;
        gpio_num_t leftPinB;
        gpio_num_t leftPinPwm;
        gpio_num_t rightPinA;
        gpio_num_t rightPinB;
        gpio_num_t rightPinPwm;
        ledc_channel_t leftPwmChannel;
        ledc_channel_t rightPwmChannel;

};


#endif // _MOTORCONTROLLER_H__