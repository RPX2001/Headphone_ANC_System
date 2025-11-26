#include <Arduino.h>
#include "anc_core.h"

ANCCore anc;

void setup() {
    // Initialize ANC system
    anc.begin();
    
    // Longer delay for hardware stabilization
    delay(500);
    
    // Start ANC processing
    anc.start();
}

void loop() {
    // All processing handled by hardware timer + FreeRTOS tasks
    // Primary output: I2S_NUM_0 (GPIO 25)
    // Secondary output: I2S_NUM_1 (GPIO 32)
    
    vTaskDelay(portMAX_DELAY); 
}