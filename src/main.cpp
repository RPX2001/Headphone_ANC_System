#include <Arduino.h>
#include "mic_input.h"
#include "speaker.h"

// Create objects
MicInput mic;
Speaker speaker1(I2S_NUM_0);  // Primary speaker
Speaker speaker2(I2S_NUM_1);  // Secondary speaker

// Pin definitions for I2S
#define SPEAKER1_BCK  26
#define SPEAKER1_WS   27
#define SPEAKER1_DATA 25

#define SPEAKER2_BCK  33
#define SPEAKER2_WS   14
#define SPEAKER2_DATA 32

void setup() {
    Serial.begin(115200);
    delay(500);
    Serial.println("=== ESP32 ANC System - Basic Test ===\n");
    
    // Initialize microphone
    Serial.println("1. Initializing Microphone...");
    mic.begin();
    mic.calibrate();
    
    // Initialize speakers
    // Serial.println("\n2. Initializing Speakers...");
    // speaker1.begin(SPEAKER1_BCK, SPEAKER1_WS, SPEAKER1_DATA, 16000);
    // speaker2.begin(SPEAKER2_BCK, SPEAKER2_WS, SPEAKER2_DATA, 16000);
    
    Serial.println("\n=== Setup Complete ===\n");
}

void loop() {
    // Example 1: Read raw microphone value
    // int raw = mic.readRaw();
    
    // Example 2: Read normalized microphone value (-1.0 to +1.0)
    Serial.println("Mic Normalized: ");
    for (int i=0; i<20; i++){
        float normalized = mic.read();
        Serial.print(normalized, 3);
        Serial.print(" ");
    }
    

    
    // // Print every 100ms
    // static unsigned long lastPrint = 0;
    // if (millis() - lastPrint > 100) {
    //     Serial.print("Mic Raw: ");
    //     Serial.print(raw);
    //     Serial.print(" | Normalized: ");
    //     Serial.println(normalized, 3);
    //     lastPrint = millis();
    // }
    
    // // Example 3: Generate and play a simple tone on speaker1
    // // (Uncomment to test speaker output)
    // /*
    // static int16_t sineBuffer[32];
    // static int phase = 0;
    // for (int i = 0; i < 32; i++) {
    //     float angle = 2.0 * PI * (phase + i) / 80.0;  // 200Hz at 16kHz
    //     sineBuffer[i] = (int16_t)(5000.0 * sin(angle));
    // }
    //  speaker1.writeBuffer(sineBuffer, 32);
    // phase = (phase + 32) % 80;
    // */
    
    delay(10);
}