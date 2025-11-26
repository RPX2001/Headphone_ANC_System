#include "anc_core.h"
#include <math.h>

// Static member initialization
ANCCore* ANCCore::instance = nullptr;
hw_timer_t* ANCCore::timer = nullptr;
LMSFilter ANCCore::lms;
float ANCCore::primaryTable[SAMPLES_PER_CYCLE];
volatile int ANCCore::primaryIdx = 0;
volatile float ANCCore::currentPrimary = 0.0f;
volatile float ANCCore::currentError = 0.0f;
volatile float ANCCore::currentSecondary = 0.0f;
i2s_port_t ANCCore::primaryPort = I2S_NUM_0;
i2s_port_t ANCCore::secondaryPort = I2S_NUM_1;
QueueHandle_t ANCCore::sampleQueue = nullptr;
TaskHandle_t ANCCore::dspTaskHandle = nullptr;
TaskHandle_t ANCCore::i2sTaskHandle = nullptr;
volatile bool ANCCore::running = false;

ANCCore::ANCCore() {
    instance = this;
}

void ANCCore::begin() {
    // Initialize ADC
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(ADC1_CHANNEL_6, ADC_ATTEN_DB_11);  // 0-3.3V range
    
    // Initialize LMS filter
    lms.begin();
    
    // Generate primary signal table
    initPrimarySignal();
    
    // Setup I2S
    setupI2S();
    
    // Create FreeRTOS queue
    sampleQueue = xQueueCreate(QUEUE_SIZE, sizeof(Sample));
}

void ANCCore::initPrimarySignal() {
    for (int i = 0; i < SAMPLES_PER_CYCLE; i++) {
        float phase = (2.0f * PI * i) / SAMPLES_PER_CYCLE;
        primaryTable[i] = sinf(phase);
    }
    primaryIdx = 0;
}

void ANCCore::setupI2S() {
    // I2S configuration - SINGLE PORT ONLY (avoiding dual-port conflicts)
    i2s_config_t i2s_config = {
        .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX),
        .sample_rate = SAMPLE_RATE,
        .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
        .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,  // Stereo: left=primary, right=secondary
        .communication_format = (i2s_comm_format_t)(I2S_COMM_FORMAT_STAND_I2S),
        .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
        .dma_buf_count = 4,
        .dma_buf_len = 64,
        .use_apll = false,
        .tx_desc_auto_clear = true,
        .fixed_mclk = 0
    };
    
    // Single I2S port - use stereo channels for primary/secondary
    i2s_pin_config_t pin_config = {
        .bck_io_num = PRIMARY_BCK,
        .ws_io_num = PRIMARY_WS,
        .data_out_num = PRIMARY_DATA,
        .data_in_num = I2S_PIN_NO_CHANGE
    };
    
    i2s_driver_install(primaryPort, &i2s_config, 0, NULL);
    i2s_set_pin(primaryPort, &pin_config);
    i2s_zero_dma_buffer(primaryPort);
}

void ANCCore::start() {
    running = true;
    
    // Wait to ensure I2S and queues are ready
    vTaskDelay(pdMS_TO_TICKS(100));
    
    // Create DSP task on Core 0 (high priority)
    xTaskCreatePinnedToCore(
        dspTask,
        "DSP",
        4096,
        nullptr,
        PRIORITY_DSP,
        &dspTaskHandle,
        CORE_DSP
    );
    
    // Wait before creating Core 1 task
    vTaskDelay(pdMS_TO_TICKS(100));
    
    // Create I2S task on Core 1 (medium priority)
    xTaskCreatePinnedToCore(
        i2sTask,
        "I2S",
        8192,  // Increased stack size
        nullptr,
        PRIORITY_I2S,
        &i2sTaskHandle,
        CORE_I2S
    );
    
    // Wait for tasks to initialize
    vTaskDelay(pdMS_TO_TICKS(200));
    
    // Start hardware timer (1MHz tick rate)
    timer = timerBegin(0, 80, true);
    timerAttachInterrupt(timer, &timerISR, true);
    timerAlarmWrite(timer, TIMER_INTERVAL_US, true);
    timerAlarmEnable(timer);
}

void ANCCore::stop() {
    running = false;
    
    if (timer) {
        timerAlarmDisable(timer);
        timerDetachInterrupt(timer);
        timerEnd(timer);
    }
    
    if (dspTaskHandle) vTaskDelete(dspTaskHandle);
    if (i2sTaskHandle) vTaskDelete(i2sTaskHandle);
}

// ====================================================
// TIMER ISR - Sampling at exactly 500Hz
// ====================================================
void IRAM_ATTR ANCCore::timerISR() {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    
    // 1. Read microphone (error signal) - FAST
    int rawMic = adc1_get_raw(ADC1_CHANNEL_6);
    currentError = ((float)rawMic - 2048.0f) / 2048.0f;
    
    // 2. Get primary signal from lookup
    currentPrimary = primaryTable[primaryIdx];
    primaryIdx = (primaryIdx + 1) % SAMPLES_PER_CYCLE;
    
    // 3. Send to DSP task (non-blocking)
    Sample sample;
    sample.primary = currentPrimary;
    sample.error = currentError;
    sample.secondary = currentSecondary;
    
    xQueueSendFromISR(sampleQueue, &sample, &xHigherPriorityTaskWoken);
    
    if (xHigherPriorityTaskWoken) {
        portYIELD_FROM_ISR();
    }
}

// ====================================================
// DSP TASK - LMS Processing on Core 0
// ====================================================
void ANCCore::dspTask(void* param) {
    Sample sample;
    
    // Small delay to ensure everything is initialized
    vTaskDelay(pdMS_TO_TICKS(50));
    
    while (running) {
        if (xQueueReceive(sampleQueue, &sample, portMAX_DELAY) == pdTRUE) {
            // Run LMS algorithm - optimized
            float antiNoise = lms.process(sample.primary, sample.error);
            
            // Update global variable for I2S task
            currentSecondary = antiNoise;
        }
    }
    
    vTaskDelete(nullptr);
}

// ====================================================
// I2S TASK - Audio Output on Core 1
// ====================================================
void ANCCore::i2sTask(void* param) {
    int16_t stereoBuf[2];  // [0]=left=primary, [1]=right=secondary
    size_t bytes_written;
    
    while (running) {
        // Fill stereo buffer: left channel = primary, right channel = secondary
        stereoBuf[0] = (int16_t)(currentPrimary * 10000.0f);    // Left = primary
        stereoBuf[1] = (int16_t)(currentSecondary * 10000.0f);  // Right = secondary
        
        // Single I2S write (stereo)
        i2s_write(primaryPort, stereoBuf, 4, &bytes_written, 0);
        
        // Small delay (2ms = 500Hz rate)
        vTaskDelay(pdMS_TO_TICKS(2));
    }
    
    vTaskDelete(nullptr);
}
