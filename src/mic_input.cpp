#include "mic_input.h"

void MicInput::begin() {
    // Configure ADC resolution
    analogReadResolution(ADC_RESOLUTION);
    
    // Optional: Set attenuation for better dynamic range
    // ADC_11db gives full scale voltage of ~3.3V
    // Uncomment if needed:
    analogSetPinAttenuation(MIC_PIN, ADC_11db);
    
    Serial.println("Microphone ADC initialized on GPIO39");
}

void MicInput::calibrate() {
    Serial.println("Calibrating microphone DC offset...");
    Serial.println("Keep microphone in quiet environment.");
    
    delay(500);  // Wait for environment to settle
    
    int avg = 0;
    for (int j=0; j<3;j++){
        int sum = 0;
        for (int i = 0; i < CAL_SAMPLES; i++) {
            int raw = analogRead(MIC_PIN);
            sum += raw;
            Serial.print(raw);
            Serial.print(" ");
            delayMicroseconds(100);  // Small delay between samples
        }
        
        dc_offset = sum / CAL_SAMPLES;
        Serial.println(dc_offset);
        avg += dc_offset;
        Serial.println(avg);
    }
    dc_offset = avg/3;
    calibrated = true;
    
    Serial.print("DC offset calibrated: ");
    Serial.println(dc_offset);
}

float MicInput::read() {
    int raw = analogRead(MIC_PIN);
    
    // Remove DC offset and normalize to ±1.0
    // Apply software gain to expand the signal
    float normalized = ((float)raw - dc_offset) * gain / 2048.0f;
    
    // Clamp to ±1.0 range
    if (normalized > 1.0f) normalized = 1.0f;
    if (normalized < -1.0f) normalized = -1.0f;
    
    return normalized;
}

int MicInput::readRaw() {
    return analogRead(MIC_PIN);
}
