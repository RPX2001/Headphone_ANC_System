#include <driver/i2s.h>
#include <driver/adc.h>
#include <Arduino.h>

// ====================================================
// PIN CONFIGURATION (I2S)
// ====================================================
// Speaker 1 (Primary Noise Source) - I2S_NUM_0
#define I2S0_BCK_PIN  26
#define I2S0_WS_PIN   27
#define I2S0_DO_PIN   25  // Connected to DIN of Amp 1

// Speaker 2 (Anti-Noise Source) - I2S_NUM_1
#define I2S1_BCK_PIN  33
#define I2S1_WS_PIN   14
#define I2S1_DO_PIN   32  // Connected to DIN of Amp 2

#define MIC_PIN       34  // ADC1_CH6

// ====================================================
// PHYSICS & CONSTANTS
// ====================================================
const int FREQUENCY = 200;        
const int SAMPLE_RATE = 16000;    // Standard I2S rate (Higher quality than 10k)
const int SAMPLES_PER_CYCLE = SAMPLE_RATE / FREQUENCY; // 80 samples at 16k

// ====================================================
// GLOBALS
// ====================================================
// Audio Buffers
int16_t sineTable[SAMPLES_PER_CYCLE];

// Control Variables (Modified by Main Loop, Read by Audio Task)
volatile int phaseOffset = 0;       // Shifts the wave in time
volatile float amplitudeGain = 0.0; // Volume of anti-noise (0.0 to 1.0)

// Optimization Logic
int phaseDir = 1;
float gainDir = 0.05;
int stepCounter = 0;

// Forward declaration
void optimizeANC(float currentVol);

// ====================================================
// I2S SETUP FUNCTIONS
// ====================================================
void setupI2S() {
  // Common Configuration for both ports
  i2s_config_t i2s_config = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX),
    .sample_rate = SAMPLE_RATE,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
    .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT, // Mono
    .communication_format = (i2s_comm_format_t)(I2S_COMM_FORMAT_STAND_I2S), // Standard I2S
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
    .dma_buf_count = 4,       // Low buffer count for lower latency
    .dma_buf_len = 64,        // Small buffer size for fast updates
    .use_apll = false,
    .tx_desc_auto_clear = true
  };

  // --- Setup I2S Port 0 (Primary Noise) ---
  i2s_pin_config_t pin_config0 = {
    .bck_io_num = I2S0_BCK_PIN,
    .ws_io_num = I2S0_WS_PIN,
    .data_out_num = I2S0_DO_PIN,
    .data_in_num = I2S_PIN_NO_CHANGE
  };
  i2s_driver_install(I2S_NUM_0, &i2s_config, 0, NULL);
  i2s_set_pin(I2S_NUM_0, &pin_config0);

  // --- Setup I2S Port 1 (Anti-Noise) ---
  i2s_pin_config_t pin_config1 = {
    .bck_io_num = I2S1_BCK_PIN,
    .ws_io_num = I2S1_WS_PIN,
    .data_out_num = I2S1_DO_PIN,
    .data_in_num = I2S_PIN_NO_CHANGE
  };
  i2s_driver_install(I2S_NUM_1, &i2s_config, 0, NULL);
  i2s_set_pin(I2S_NUM_1, &pin_config1);
}

// ====================================================
// AUDIO GENERATION TASK (Runs on Core 0)
// ====================================================
// This task keeps the I2S buffers full. 
// It replaces the hardware timer interrupt.
void i2sAudioTask(void *parameter) {
  size_t bytes_written;
  int waveIndex = 0;
  
  // Buffer to hold a small chunk of audio
  // Writing in chunks is more efficient than 1-by-1
  const int CHUNK_SIZE = 32; 
  int16_t primaryBuf[CHUNK_SIZE];
  int16_t secondaryBuf[CHUNK_SIZE];

  while (true) {
    // Fill the chunks
    for(int i=0; i<CHUNK_SIZE; i++) {
      // 1. Primary Noise (Fixed)
      primaryBuf[i] = sineTable[waveIndex];

      // 2. Anti-Noise (Variable Phase & Gain)
      // We calculate the shifted index dynamically
      int shiftedIndex = (waveIndex + phaseOffset) % SAMPLES_PER_CYCLE;
      // Ensure positive modulo
      if(shiftedIndex < 0) shiftedIndex += SAMPLES_PER_CYCLE; 
      
      secondaryBuf[i] = (int16_t)(sineTable[shiftedIndex] * amplitudeGain);

      // Advance index
      waveIndex++;
      if (waveIndex >= SAMPLES_PER_CYCLE) waveIndex = 0;
    }

    // Write to Hardware
    // These functions block if the buffer is full, keeping timing perfect
    i2s_write(I2S_NUM_0, primaryBuf, sizeof(primaryBuf), &bytes_written, portMAX_DELAY);
    i2s_write(I2S_NUM_1, secondaryBuf, sizeof(secondaryBuf), &bytes_written, portMAX_DELAY);
  }
}

// ====================================================
// SETUP
// ====================================================
void setup() {
  Serial.begin(115200);
  Serial.println("Initializing I2S ANC System...");

  // 1. Initialize ADC for Mic
  adc1_config_width(ADC_WIDTH_BIT_12);
  adc1_config_channel_atten(ADC1_CHANNEL_6, ADC_ATTEN_DB_12);

  // 2. Generate High-Res Sine Table
  // Amplitude +/- 10000 (Max is 32767)
  for (int i = 0; i < SAMPLES_PER_CYCLE; i++) {
    float angle = (2.0 * PI * i) / SAMPLES_PER_CYCLE;
    sineTable[i] = (int16_t)(10000.0 * sin(angle)); 
  }

  // 3. Start I2S Hardware
  setupI2S();

  // 4. Launch Audio Task on Core 0 (Main loop runs on Core 1)
  // This ensures audio generation never stops while we calculate logic
  xTaskCreatePinnedToCore(
    i2sAudioTask,   // Function
    "AudioGen",     // Name
    4096,           // Stack size
    NULL,           // Parameters
    1,              // Priority
    NULL,           // Handle
    0               // Pin to Core 0
  );

  Serial.println("--- ANC RUNNING ---");
}

// ====================================================
// MAIN LOOP (Optimization Logic)
// ====================================================
void loop() {
  // 1. Measure Error (RMS Volume)
  long sum = 0;
  int readings = 200;
  
  for(int i=0; i<readings; i++) {
    int val = adc1_get_raw(ADC1_CHANNEL_6);
    int centered = val - 2048; // Calibrate this value (Silence value)
    sum += abs(centered);
    // No delay needed here, I2S runs in background
  }
  float avgVolume = (float)sum / readings;

  // Debug Output (Plotter friendly)
  // Print: Error, Phase, Gain
  Serial.print(avgVolume);
  Serial.print(",");
  Serial.print(phaseOffset);
  Serial.print(",");
  Serial.println(amplitudeGain * 1000); // Scale up for visibility

  // 2. Run Gradient Descent
  optimizeANC(avgVolume);
  
  // Small delay to stabilize the feedback loop
  delay(20); 
}

void optimizeANC(float currentVol) {
  static float prevVol = 99999;
  
  // Control Logic
  // Every 4 steps, try Phase. Else, try Gain.
  if (stepCounter % 4 == 0) {
    if (currentVol < prevVol) {
      // Improvement: Keep going
      phaseOffset = (phaseOffset + phaseDir) % SAMPLES_PER_CYCLE;
    } else {
      // Worsened: Reverse
      phaseDir = -phaseDir;
      phaseOffset = (phaseOffset + phaseDir) % SAMPLES_PER_CYCLE;
    }
  } else {
     if (currentVol < prevVol) {
      amplitudeGain += gainDir;
    } else {
      gainDir = -gainDir;
      amplitudeGain += gainDir;
    }
    // Safety Limits
    if (amplitudeGain > 1.5) amplitudeGain = 1.5; // Max volume boost
    if (amplitudeGain < 0.0) amplitudeGain = 0.0;
  }

  // Handle Phase Wrap-around (Negative modulo fix)
  if (phaseOffset < 0) phaseOffset += SAMPLES_PER_CYCLE;

  prevVol = currentVol;
  stepCounter++;
}