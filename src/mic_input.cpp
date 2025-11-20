#include "mic_input.h"

void MicInput::begin() {
    // Configure ADC resolution
    analogReadResolution(ADC_RESOLUTION);
    
    // Optional: Set attenuation for better dynamic range
    // ADC_11db gives full scale voltage of ~3.3V
    // Uncomment if needed:
    analogSetPinAttenuation(MIC_PIN, ADC_11db);
    
    #ifdef DEBUG_MIC
    Serial.println("Microphone ADC initialized on GPIO39");
    #endif
}

void MicInput::calibrate() {
    #ifdef DEBUG_MIC
    Serial.println("Calibrating microphone DC offset...");
    Serial.println("Keep microphone in quiet environment.");
    #endif
    
    delay(500);  // Wait for environment to settle
    
    int avg = 0;
    for (int j=0; j<3;j++){
        int sum = 0;
        for (int i = 0; i < CAL_SAMPLES; i++) {
            int raw = analogRead(MIC_PIN);
            sum += raw;
            #ifdef DEBUG_MIC
            Serial.print(raw);
            Serial.print(" ");
            #endif
            delayMicroseconds(100);  // Small delay between samples
        }
        
        dc_offset = sum / CAL_SAMPLES;
        #ifdef DEBUG_MIC
        Serial.println(dc_offset);
        #endif
        avg += dc_offset;
        #ifdef DEBUG_MIC
        Serial.println(avg);
        #endif
    }
    dc_offset = avg/3;
    calibrated = true;
    
    #ifdef DEBUG_MIC
    Serial.print("DC offset calibrated: ");
    Serial.println(dc_offset);
    #endif
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
