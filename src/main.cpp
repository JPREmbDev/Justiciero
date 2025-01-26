
#include "localBmi160.h"
#include <freertos/FreeRTOS.h>


extern "C" void app_main();

void app_main() {

    vTaskDelay(pdMS_TO_TICKS(1000));
    bmi_initSpi();
    
    for(;;)
    {
        /* code */
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
    

}