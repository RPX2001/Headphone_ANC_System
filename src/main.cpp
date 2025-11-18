#include <Arduino.h>
#include "driver/i2s.h"
#include "mic_input.h"

MicInput mic;

void setup() {
    Serial.begin(115200);
    delay(200);
    Serial.println("=== ESP32 Microphone Test ===");
    
    // Initialize microphone
    mic.begin();
    
    // Calibrate DC offset
    mic.calibrate();
    
    Serial.println("Starting microphone reading...");
    Serial.println("Format: Signal (normalized) | Raw ADC");
}

void loop() {
    // Read normalized signal (-1.0 to +1.0)
    float signal = mic.read();
    
    // Read raw ADC value (0-4095)
    int raw = mic.readRaw();
    
    // Print values
    Serial.print("Signal: ");
    Serial.print(signal, 4);
    Serial.print(" | Raw: ");
    Serial.println(raw);
    
    delay(10);  // Read at ~100Hz
}

