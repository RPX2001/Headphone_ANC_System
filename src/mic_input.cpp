#include "mic_input.h"

void MicInput::begin() {
    // Configure ADC resolution
    analogReadResolution(ADC_RESOLUTION);
    
    // MAX4466 outputs 0-3.3V with DC bias at VCC/2 (~1.65V)
    // Use 11dB attenuation to measure full 0-3.3V range properly
    analogSetPinAttenuation(MIC_PIN, ADC_11db);
    
    Serial.println("Microphone ADC initialized on GPIO34");
    Serial.println("ADC range: 0-3.3V (11dB attenuation)");
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
