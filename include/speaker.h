#ifndef SPEAKER_H
#define SPEAKER_H

#include <Arduino.h>
#include <driver/i2s.h>

class Speaker {
public:
    // Constructor - specify which I2S port to use
    Speaker(i2s_port_t port = I2S_NUM_0);
    
    // Initialize I2S interface with pin configuration
    void begin(int bck_pin, int ws_pin, int data_pin, uint32_t sample_rate = 16000);
    
    // Play a single sample (blocking)
    void writeSample(int16_t sample);
    
    // Play a buffer of samples (blocking)
    void writeBuffer(int16_t* buffer, size_t length);
    
    // Play a buffer of samples (non-blocking, returns bytes written)
    size_t writeBufferNonBlocking(int16_t* buffer, size_t length);
    
    // Stop I2S output
    void stop();
    
    // Get sample rate
    uint32_t getSampleRate() const { return sample_rate; }
    
    // Set sample rate (must call begin() again after this)
    void setSampleRate(uint32_t rate) { sample_rate = rate; }

private:
    i2s_port_t i2s_port;
    uint32_t sample_rate;
    bool initialized;
};

#endif // SPEAKER_H