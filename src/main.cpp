#include <Arduino.h>
#include <driver/i2s.h>
#include <driver/adc.h>
#include <math.h>

// ============= Configuration Parameters =============
#define SAMPLE_RATE 44100          // Sample rate in Hz
#define BUFFER_SIZE 256            // Buffer size for processing
#define LMS_FILTER_LENGTH 128      // Length of adaptive filter
#define LMS_MU 0.001               // LMS step size (learning rate)
#define PRIMARY_FREQ 1000.0        // Primary noise frequency (1 kHz)
#define TWO_PI 6.283185307179586   

// ============= I2S Configuration =============
// Primary Source (I2S_NUM_0) - Plays the known noise signal
#define I2S_PRIMARY I2S_NUM_0
#define I2S_PRIMARY_BCK_PIN 26     // Bit clock pin for primary
#define I2S_PRIMARY_WS_PIN 27      // Word select pin for primary
#define I2S_PRIMARY_DATA_PIN 25    // Data out pin for primary speaker

// Secondary Source (I2S_NUM_1) - Plays the anti-noise signal
#define I2S_SECONDARY I2S_NUM_1
#define I2S_SECONDARY_BCK_PIN 33   // Bit clock pin for secondary
#define I2S_SECONDARY_WS_PIN 14    // Word select pin for secondary
#define I2S_SECONDARY_DATA_PIN 32  // Data out pin for secondary speaker

// ============= ADC Configuration =============
#define ERROR_MIC_PIN ADC1_CHANNEL_0  // GPIO36 for error microphone
#define ADC_SAMPLES 128            // Number of ADC samples per batch

// ============= Global Variables =============
float lms_weights[LMS_FILTER_LENGTH];     // LMS filter coefficients
float reference_buffer[LMS_FILTER_LENGTH]; // Reference signal buffer
float error_signal = 0.0;                  // Current error signal
float secondary_output = 0.0;              // Anti-noise signal
uint32_t sample_count = 0;                 // Sample counter for sine generation

// ============= Function Prototypes =============
void setup_i2s_primary();
void setup_i2s_secondary();
void setup_adc();
float generate_primary_signal();
float read_error_microphone();
float lms_filter(float reference);
void update_lms_weights(float error, float reference);

// ============= Setup I2S for Primary Source =============
void setup_i2s_primary() {
  i2s_config_t i2s_config = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX),
    .sample_rate = SAMPLE_RATE,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
    .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,  // Mono output
    .communication_format = I2S_COMM_FORMAT_STAND_I2S,
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
    .dma_buf_count = 8,
    .dma_buf_len = BUFFER_SIZE,
    .use_apll = false,
    .tx_desc_auto_clear = true,
    .fixed_mclk = 0
  };

  i2s_pin_config_t pin_config = {
    .bck_io_num = I2S_PRIMARY_BCK_PIN,
    .ws_io_num = I2S_PRIMARY_WS_PIN,
    .data_out_num = I2S_PRIMARY_DATA_PIN,
    .data_in_num = I2S_PIN_NO_CHANGE
  };

  // Install and start I2S driver for primary
  i2s_driver_install(I2S_PRIMARY, &i2s_config, 0, NULL);
  i2s_set_pin(I2S_PRIMARY, &pin_config);
  i2s_zero_dma_buffer(I2S_PRIMARY);
}

// ============= Setup I2S for Secondary Source =============
void setup_i2s_secondary() {
  i2s_config_t i2s_config = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX),
    .sample_rate = SAMPLE_RATE,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
    .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,  // Mono output
    .communication_format = I2S_COMM_FORMAT_STAND_I2S,
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
    .dma_buf_count = 8,
    .dma_buf_len = BUFFER_SIZE,
    .use_apll = false,
    .tx_desc_auto_clear = true,
    .fixed_mclk = 0
  };

  i2s_pin_config_t pin_config = {
    .bck_io_num = I2S_SECONDARY_BCK_PIN,
    .ws_io_num = I2S_SECONDARY_WS_PIN,
    .data_out_num = I2S_SECONDARY_DATA_PIN,
    .data_in_num = I2S_PIN_NO_CHANGE
  };

  // Install and start I2S driver for secondary
  i2s_driver_install(I2S_SECONDARY, &i2s_config, 0, NULL);
  i2s_set_pin(I2S_SECONDARY, &pin_config);
  i2s_zero_dma_buffer(I2S_SECONDARY);
}

// ============= Setup ADC for Microphone Input =============
void setup_adc() {
  adc1_config_width(ADC_WIDTH_BIT_12);
  adc1_config_channel_atten(ERROR_MIC_PIN, ADC_ATTEN_DB_11);
}

// ============= Generate Primary Noise Signal =============
// Generates sin(2*pi*1000*t)
float generate_primary_signal() {
  float t = (float)sample_count / (float)SAMPLE_RATE;
  float signal = sin(TWO_PI * PRIMARY_FREQ * t);
  sample_count++;
  return signal;
}

// ============= Read Error Microphone Signal =============
float read_error_microphone() {
  // Read ADC value (0-4095 for 12-bit)
  int adc_value = adc1_get_raw(ERROR_MIC_PIN);
  
  // Convert to normalized float (-1.0 to 1.0)
  float normalized = ((float)adc_value - 2048.0) / 2048.0;
  
  return normalized;
}

// ============= LMS Adaptive Filter =============
// Computes the filter output y(n) = sum(w[i] * x[n-i])
float lms_filter(float reference) {
  // Shift reference buffer (FIFO)
  for (int i = LMS_FILTER_LENGTH - 1; i > 0; i--) {
    reference_buffer[i] = reference_buffer[i - 1];
  }
  reference_buffer[0] = reference;
  
  // Compute filter output
  float output = 0.0;
  for (int i = 0; i < LMS_FILTER_LENGTH; i++) {
    output += lms_weights[i] * reference_buffer[i];
  }
  
  return output;
}

// ============= Update LMS Filter Weights =============
// Updates weights using: w(n+1) = w(n) + mu * e(n) * x(n)
void update_lms_weights(float error, float reference) {
  for (int i = 0; i < LMS_FILTER_LENGTH; i++) {
    lms_weights[i] += LMS_MU * error * reference_buffer[i];
    
    // Optional: Add weight constraints to prevent divergence
    if (lms_weights[i] > 10.0) lms_weights[i] = 10.0;
    if (lms_weights[i] < -10.0) lms_weights[i] = -10.0;
  }
}

// ============= Setup Function =============
void setup() {
  Serial.begin(115200);
  Serial.println("=== ESP32 ANC System Initializing ===");
  
  // Initialize I2S for primary source (noise generator)
  setup_i2s_primary();
  Serial.println("I2S Primary initialized");
  
  // Initialize I2S for secondary source (anti-noise)
  setup_i2s_secondary();
  Serial.println("I2S Secondary initialized");
  
  // Initialize ADC for microphone input
  setup_adc();
  Serial.println("ADC initialized");
  
  // Initialize LMS filter weights to zero
  for (int i = 0; i < LMS_FILTER_LENGTH; i++) {
    lms_weights[i] = 0.0;
    reference_buffer[i] = 0.0;
  }
  Serial.println("LMS filter initialized");
  
  Serial.println("=== ANC System Ready ===");
  delay(1000);
}

// ============= Main Loop =============
void loop() {
  int16_t primary_buffer[BUFFER_SIZE * 2];    // Buffer for primary source (mono in stereo format)
  int16_t secondary_buffer[BUFFER_SIZE * 2];  // Buffer for secondary source (mono in stereo format)
  size_t bytes_written;
  
  // Process audio in batches
  for (int i = 0; i < BUFFER_SIZE; i++) {
    // Step 1: Generate primary noise signal (reference)
    float primary_signal = generate_primary_signal();
    
    // Step 2: Apply LMS filter to generate anti-noise signal
    secondary_output = lms_filter(primary_signal);
    
    // Step 3: Read error microphone
    // The error microphone captures: primary_signal + secondary_output (acoustic superposition)
    error_signal = read_error_microphone();
    
    // Step 4: Update LMS weights based on error
    update_lms_weights(error_signal, primary_signal);
    
    // Step 5: Prepare I2S outputs
    int16_t primary_output_int = (int16_t)(primary_signal * 16000.0);
    int16_t secondary_output_int = (int16_t)(secondary_output * 16000.0);
    
    // Fill buffers (mono signal on both L+R channels for compatibility)
    primary_buffer[i * 2] = primary_output_int;
    primary_buffer[i * 2 + 1] = primary_output_int;
    
    secondary_buffer[i * 2] = secondary_output_int;
    secondary_buffer[i * 2 + 1] = secondary_output_int;
  }
  
  // Write to both I2S interfaces
  i2s_write(I2S_PRIMARY, primary_buffer, BUFFER_SIZE * 4, &bytes_written, portMAX_DELAY);
  i2s_write(I2S_SECONDARY, secondary_buffer, BUFFER_SIZE * 4, &bytes_written, portMAX_DELAY);
  
  // Debug output every 1000 samples (~22ms at 44.1kHz)
  static int debug_counter = 0;
  debug_counter++;
  if (debug_counter >= 1000) {
    Serial.print("Error: ");
    Serial.print(error_signal, 4);
    Serial.print(" | Secondary: ");
    Serial.print(secondary_output, 4);
    Serial.print(" | Weight[0]: ");
    Serial.println(lms_weights[0], 4);
    debug_counter = 0;
  }
}
