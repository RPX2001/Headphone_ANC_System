#ifndef ANC_CORE_H
#define ANC_CORE_H

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <driver/i2s.h>
#include <driver/adc.h>
#include "anc_config.h"
#include "lms_filter.h"

// Sample data structure
struct Sample {
    float primary;
    float error;
    float secondary;
};

class ANCCore {
public:
    ANCCore();
    
    void begin();
    void start();
    void stop();

private:
    // Hardware timer
    static hw_timer_t* timer;
    
    // LMS filter
    static LMSFilter lms;
    
    // Pre-computed primary signal
    static float primaryTable[SAMPLES_PER_CYCLE];
    static volatile int primaryIdx;
    
    // Current signal values
    static volatile float currentPrimary;
    static volatile float currentError;
    static volatile float currentSecondary;
    
    // I2S handles
    static i2s_port_t primaryPort;
    static i2s_port_t secondaryPort;
    
    // FreeRTOS
    static QueueHandle_t sampleQueue;
    static TaskHandle_t dspTaskHandle;
    static TaskHandle_t i2sTaskHandle;
    
    // State
    static volatile bool running;
    
    // Instance pointer
    static ANCCore* instance;
    
    // Functions
    static void IRAM_ATTR timerISR();
    static void dspTask(void* param);
    static void i2sTask(void* param);
    
    void initPrimarySignal();
    void setupI2S();
};

#endif // ANC_CORE_H
