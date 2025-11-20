#include "speaker.h"

Speaker::Speaker(i2s_port_t port) {
    i2s_port = port;
    sample_rate = 16000;  // Default sample rate
    initialized = false;
}

void Speaker::begin(int bck_pin, int ws_pin, int data_pin, uint32_t rate) {
    sample_rate = rate;
    
    // I2S configuration
    i2s_config_t i2s_config = {
        .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX),
        .sample_rate = sample_rate,
        .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
        .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,  // Mono output
        .communication_format = (i2s_comm_format_t)(I2S_COMM_FORMAT_STAND_I2S),
        .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
        .dma_buf_count = 4,
        .dma_buf_len = 64,
        .use_apll = false,
        .tx_desc_auto_clear = true,
        .fixed_mclk = 0
    };

    // Pin configuration
    i2s_pin_config_t pin_config = {
        .bck_io_num = bck_pin,
        .ws_io_num = ws_pin,
        .data_out_num = data_pin,
        .data_in_num = I2S_PIN_NO_CHANGE
    };

    // Install and configure I2S driver
    esp_err_t result = i2s_driver_install(i2s_port, &i2s_config, 0, NULL);
    if (result == ESP_OK) {
        i2s_set_pin(i2s_port, &pin_config);
        initialized = true;
        #ifdef DEBUG_SPEAKER
        Serial.print("Speaker I2S initialized on port ");
        Serial.print(i2s_port);
        Serial.print(" at ");
        Serial.print(sample_rate);
        Serial.println(" Hz");
        #endif
    } else {
        initialized = false;
        #ifdef DEBUG_SPEAKER
        Serial.print("Failed to initialize I2S port ");
        Serial.println(i2s_port);
        #endif
    }
}

void Speaker::writeSample(int16_t sample) {
    if (!initialized) return;
    
    size_t bytes_written;
    // I2S expects stereo data, so we send the same sample twice (L+R)
    int16_t stereo_sample[2] = {sample, sample};
    i2s_write(i2s_port, stereo_sample, sizeof(stereo_sample), &bytes_written, portMAX_DELAY);
}

void Speaker::writeBuffer(int16_t* buffer, size_t length) {
    if (!initialized || buffer == nullptr || length == 0) return;
    
    size_t bytes_written;
    size_t bytes_to_write = length * sizeof(int16_t);
    i2s_write(i2s_port, buffer, bytes_to_write, &bytes_written, portMAX_DELAY);
}

size_t Speaker::writeBufferNonBlocking(int16_t* buffer, size_t length) {
    if (!initialized || buffer == nullptr || length == 0) return 0;
    
    size_t bytes_written;
    size_t bytes_to_write = length * sizeof(int16_t);
    i2s_write(i2s_port, buffer, bytes_to_write, &bytes_written, 0);  // No blocking
    return bytes_written / sizeof(int16_t);  // Return number of samples written
}

void Speaker::stop() {
    if (initialized) {
        i2s_zero_dma_buffer(i2s_port);
        i2s_driver_uninstall(i2s_port);
        initialized = false;
        #ifdef DEBUG_SPEAKER
        Serial.print("Speaker I2S port ");
        Serial.print(i2s_port);
        Serial.println(" stopped");
        #endif
    }
}