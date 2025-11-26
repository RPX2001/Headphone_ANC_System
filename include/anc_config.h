#ifndef ANC_CONFIG_H
#define ANC_CONFIG_H

// ====================================================
// SIGNAL PROCESSING PARAMETERS
// ====================================================
#define SAMPLE_RATE         500      // 500 Hz sampling rate
#define PRIMARY_FREQ        100      // 100 Hz primary noise frequency
#define FILTER_TAPS         32       // 32-tap FIR filter
#define LMS_MU              0.01f    // LMS step size (learning rate)

// Derived constants
#define SAMPLES_PER_CYCLE   (SAMPLE_RATE / PRIMARY_FREQ)  // 5 samples per cycle
#define TIMER_INTERVAL_US   (1000000 / SAMPLE_RATE)       // 2000 us per sample

// ====================================================
// PIN DEFINITIONS
// ====================================================
// Microphone (ADC)
#define MIC_PIN             34       // GPIO34 = ADC1_CH6

// Primary Speaker (I2S_NUM_0)
#define PRIMARY_BCK         26
#define PRIMARY_WS          27
#define PRIMARY_DATA        25

// Secondary Speaker (I2S_NUM_1)
#define SECONDARY_BCK       33
#define SECONDARY_WS        14
#define SECONDARY_DATA      32

// ====================================================
// TASK CONFIGURATION
// ====================================================
#define CORE_DSP            0        // Core 0: DSP processing
#define CORE_I2S            1        // Core 1: I2S output

#define PRIORITY_ISR        25       // Highest priority
#define PRIORITY_DSP        20       // High priority
#define PRIORITY_I2S        15       // Medium priority

// ====================================================
// BUFFER CONFIGURATION
// ====================================================
#define I2S_BUFFER_SIZE     8        // Small buffer for low latency
#define QUEUE_SIZE          4        // Minimal queue depth

#endif // ANC_CONFIG_H
