#ifndef SERVO_H__
#define SERVO_H__

#include <stdint.h>

class Servo {

    public:
        /**
         * @brief Construct a new Servo object
         */
        Servo();

        /**
         * @brief Initialize LEDC, call it only once.
         */
        void initHw();
        
        /**
         * @brief Set the min and max pulse width in microseconds for the servo control
         * 
         * @param min Minimum pulse width in microseconds for the servo
         * @param max Maximum pulse width in microseconds for the servo
         */
        void calibrate(uint32_t min, uint32_t max);

        /**
         * @brief Set the position of the servo in percentage
         * 
         * @param pos Position in percentage (100% = max, 0% = min)
         */
        void setPos(uint32_t pos);

    private:
        // Minimum pulse width in microseconds for the servo
        uint32_t min_pulse_width;
        // Maximum pulse width in microseconds for the servo
        uint32_t max_pulse_width;
};

#endif // SERVO_H__