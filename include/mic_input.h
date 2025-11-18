#ifndef MIC_INPUT_H
#define MIC_INPUT_H

#include <Arduino.h>

class MicInput {
public:
    // Initialize ADC for microphone input
    void begin();
    
    // Calibrate DC offset (call during quiet period)
    void calibrate();
    
    // Read normalized microphone signal (-1.0 to +1.0)
    float read();
    
    // Read raw ADC value (0-4095)
    int readRaw();
    
    // Get DC offset value
    float getDCOffset() const { return dc_offset; }
    
    // Check if calibrated
    bool isCalibrated() const { return calibrated; }

private:
    static const int MIC_PIN = 39;        // GPIO39 = ADC1_CH3
    static const int ADC_RESOLUTION = 12; // 12-bit ADC (0-4095)
    static const int CAL_SAMPLES = 1000;  // Calibration samples
    
    float dc_offset = 2048.0f;  // Default center value
    bool calibrated = false;
};

#endif // MIC_INPUT_H
