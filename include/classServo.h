#ifndef SERVO_H__
#define SERVO_H__

#include <stdint.h>


class Servo {

    public:

        Servo();

        /** @brief initialize LEDC, call it only once. */
        void initHw();
        
        /** @brief Set the min and max 'us' for the servo control 
         *  @param min [in] Minimum us for the servo
         *  @param max [in] Maximum us for the servo
        */
        void calibrate(uint32_t min, uint32_t max);

        /** @brief Position in percentage                   
         *  @param pos [in] Position in percentage 100% = max, 0% = min 
         */
        void setPos(uint32_t pos);

    private:
        uint32_t min_pulse_width;
        uint32_t max_pulse_width;
};


#endif // SERVO_H__ 