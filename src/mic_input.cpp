#include "mic_input.h"

void MicInput::begin() {
    // Configure ADC resolution
    analogReadResolution(ADC_RESOLUTION);
    
    // Optional: Set attenuation for better dynamic range
    // ADC_11db gives full scale voltage of ~3.3V
    // Uncomment if needed:
    // analogSetPinAttenuation(MIC_PIN, ADC_11db);
    
    Serial.println("Microphone ADC initialized on GPIO39");
}

void MicInput::calibrate() {
    Serial.println("Calibrating microphone DC offset...");
    Serial.println("Keep microphone in quiet environment.");
    
    delay(500);  // Wait for environment to settle
    
    float sum = 0.0f;
    for (int i = 0; i < CAL_SAMPLES; i++) {
        int raw = analogRead(MIC_PIN);
        sum += (float)raw;
        delayMicroseconds(100);  // Small delay between samples
    }
    
    dc_offset = sum / (float)CAL_SAMPLES;
    calibrated = true;
    
    Serial.print("DC offset calibrated: ");
    Serial.println(dc_offset, 2);
}

float MicInput::read() {
    int raw = analogRead(MIC_PIN);
    
    // Remove DC offset and normalize to ±1.0
    // ADC range is 0-4095 for 12-bit
    // Center should be at dc_offset (typically ~2048)
    float normalized = ((float)raw - dc_offset) / 2048.0f;
    
    // Clamp to ±1.0 range
    if (normalized > 1.0f) normalized = 1.0f;
    if (normalized < -1.0f) normalized = -1.0f;
    
    return normalized;
}

int MicInput::readRaw() {
    return analogRead(MIC_PIN);
}
