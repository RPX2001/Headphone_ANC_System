#include <Arduino.h>
#include "driver/i2s.h"
#include <Preferences.h>

// ----------- tunable ANC params (SIMPLIFIED LMS) -----------
static const uint32_t FS_HZ        = 44100;
static const uint16_t FRAME        = 64;       // block size
static const float    F0_HZ        = 1000.0f;  // primary tone (known signal to cancel)
static const int      N_TAPS       = 32;       // LMS FIR length (reduced for single tone)
static const float    MU           = 0.005f;  // LMS step size (tune for convergence)
static const float    LEAK         = 0.0f;     // leaky LMS (0..1); 0 = off
static const float    OUT_CLIP     = 0.95f;    // clip before I2S (±1.0 full-scale)
static const int      HPF_DC_N     = 1024;     // simple DC remover length for mic

// ----------- Pins ---------------
static const i2s_port_t I2S_PRIMARY   = I2S_NUM_0;   // primary (noise) speaker
static const i2s_port_t I2S_SECONDARY = I2S_NUM_1;   // secondary (anti-noise) speaker
// MAX98357 #1 (primary)
static const int BCK1 = 26;
static const int WS1  = 27;
static const int DIN1 = 25;
// MAX98357 #2 (secondary)
static const int BCK2 = 33;
static const int WS2  = 14;
static const int DIN2 = 32;

// ----------- I2S helpers -----------
static inline int16_t f32_to_i16(float x) {
  // hard clip to [-1, 1], then scale
  if (x >  1.0f) x =  1.0f;
  if (x < -1.0f) x = -1.0f;
  return (int16_t)(x * 32767.0f);
}

// Simple 1st-order DC remover (moving average) for mic
struct DCRemover {
  float acc = 0.0f;
  int   n   = 0;
  void   reset(){ acc=0.0f; n=0; }
  float  step(float x){
    // incremental mean
    n = min(n+1, HPF_DC_N);
    acc += (x - acc) / (float)n;
    return x - acc;
  }
} dc;

// ----------- ANC state  -----------
static float w[N_TAPS];            // LMS filter coefficients
static float xBuf[N_TAPS + 256];   // ring buffer for reference signal
static int   idx = 0;              // ring index

// phase accumulator for 1 kHz
static double phase = 0.0;
static const double TWOPI = 6.283185307179586;
static double dphi;

// ----------- Non-Volatile Storage -----------
Preferences preferences;
static const char* NVS_NAMESPACE = "anc_data";
static const char* NVS_WEIGHTS_KEY = "weights";
static const char* NVS_PHASE_KEY = "phase";
static const char* NVS_VALID_KEY = "valid";
static unsigned long last_save_time = 0;
static const unsigned long SAVE_INTERVAL_MS = 10000; // Save every 10 seconds

// Save weights and signal parameters to NVS
void save_to_nvs() {
  preferences.begin(NVS_NAMESPACE, false); // Read/Write mode
  
  // Save weights array
  preferences.putBytes(NVS_WEIGHTS_KEY, w, sizeof(w));
  
  // Save phase accumulator (for signal continuity)
  preferences.putDouble(NVS_PHASE_KEY, phase);
  
  // Mark as valid
  preferences.putBool(NVS_VALID_KEY, true);
  
  preferences.end();
  Serial.println("ANC state saved to NVS");
}

// Load weights and signal parameters from NVS
bool load_from_nvs() {
  preferences.begin(NVS_NAMESPACE, true); 
  
  // Check for valid data exists
  bool valid = preferences.getBool(NVS_VALID_KEY, false);
  
  if (valid) {
    // Load weights
    size_t len = preferences.getBytes(NVS_WEIGHTS_KEY, w, sizeof(w));
    
    // Load phase
    phase = preferences.getDouble(NVS_PHASE_KEY, 0.0);
    
    preferences.end();
    
    if (len == sizeof(w)) {
      Serial.println("ANC state loaded from NVS");
      Serial.print("Restored phase: ");
      Serial.println(phase, 6);
      Serial.print("Restored W[0]: ");
      Serial.println(w[0], 6);
      return true;
    }
  }
  
  preferences.end();
  Serial.println("No valid ANC state found in NVS - starting fresh");
  return false;
}

// Clear NVS data
void clear_nvs() {
  preferences.begin(NVS_NAMESPACE, false);
  preferences.clear();
  preferences.end();
  Serial.println("NVS cleared");
}

// ----------- I2S setup -----------
void setup_i2s_tx(i2s_port_t port, int bck, int ws, int din) {
  i2s_config_t cfg = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX),
    .sample_rate = (int)FS_HZ,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
    .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT, 
    .communication_format = I2S_COMM_FORMAT_STAND_I2S,
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
    .dma_buf_count = 8,
    .dma_buf_len = FRAME,
    .use_apll = false,
    .tx_desc_auto_clear = true,
    .fixed_mclk = 0
  };
  i2s_driver_install(port, &cfg, 0, nullptr);

  i2s_pin_config_t pins = {
    .bck_io_num = bck,
    .ws_io_num = ws,
    .data_out_num = din,
    .data_in_num = I2S_PIN_NO_CHANGE
  };
  i2s_set_pin(port, &pins);
  i2s_set_clk(port, FS_HZ, I2S_BITS_PER_SAMPLE_16BIT, I2S_CHANNEL_STEREO);
}

void write_i2s_mono_stereo(i2s_port_t port, const int16_t* mono, size_t n) {
  // duplicate mono to L and R
  static int16_t interleaved[FRAME * 2];
  for (size_t i = 0; i < n; ++i) {
    interleaved[2*i + 0] = mono[i];
    interleaved[2*i + 1] = mono[i];
  }
  size_t bytesWritten = 0;
  i2s_write(port, interleaved, n * 2 * sizeof(int16_t), &bytesWritten, portMAX_DELAY);
}

// ----------- ADC mic signal input -----------
static const int MIC_PIN = 39; // ADC1_CH3

inline float read_mic_norm() {
  // 12-bit ADC: center around 0 and normalize to ±1
  int raw = analogRead(MIC_PIN);
  float x = (float)raw;
  // Map 0..4095 -> -1..+1 
  return ( (x - 2048.0f) / 2048.0f );
}

// -----LMS----------
void anc_process_block() {
  int16_t bufPrimary[FRAME]  = {0}; // to primary I2S 
  int16_t bufSecondary[FRAME]= {0}; // to secondary I2S (anti-noise)
  
  static int print_counter = 0;      // for periodic serial printing
  static float error_avg = 0.0f;     // average error 
  static float output_avg = 0.0f;    // average output

  for (int n = 0; n < FRAME; ++n) {
    // Generate primary reference x[n] = sin(2π f0 t)
    float x = (float)sin(phase);
    phase += dphi;
    if (phase >= TWOPI) phase -= TWOPI;

    // Push reference into ring buffer
    xBuf[idx] = x;

    // Generate control output y[n] = sum_k w[k] * x[n-k]
    float y = 0.0f;
    int j = idx;
    for (int k = 0; k < N_TAPS; ++k) {
      // ring index (wrap)
      int jj = j - k;
      if (jj < 0) jj += (int)(sizeof(xBuf)/sizeof(xBuf[0]));
      y += w[k] * xBuf[jj];
    }

    //Play signals: primary and secondary (y)
    float primary_out = 0.50f * x;   // make primary smaller to avoid clipping
    float secondary_out = y;

    // Clip before convert
    if (primary_out  >  OUT_CLIP) primary_out  =  OUT_CLIP;
    if (primary_out  < -OUT_CLIP) primary_out  = -OUT_CLIP;
    if (secondary_out>  OUT_CLIP) secondary_out=  OUT_CLIP;
    if (secondary_out< -OUT_CLIP) secondary_out= -OUT_CLIP;

    bufPrimary[n]   = f32_to_i16(primary_out);
    bufSecondary[n] = f32_to_i16(secondary_out);

    //Measure error mic (residual at ear)
    float e = read_mic_norm();
    e = dc.step(e);     // remove DC drift

    // Accumulate for averaging
    error_avg += fabs(e);
    output_avg += fabs(secondary_out);

    //SIMPLIFIED LMS weight update: w[k] += mu * e[n] * x[n-k] - leak*w[k]
    int jx = idx;
    for (int k = 0; k < N_TAPS; ++k) {
      int jj = jx - k;
      if (jj < 0) jj += (int)(sizeof(xBuf)/sizeof(xBuf[0]));
      float xk = xBuf[jj];
      w[k] = (1.0f - LEAK) * w[k] + MU * e * xk;
    }

    // 7) advance ring index
    idx++;
    if (idx >= (int)(sizeof(xBuf)/sizeof(xBuf[0]))) idx = 0;
  }

  // 8) push out blocks to I2S
  write_i2s_mono_stereo(I2S_PRIMARY,   bufPrimary,   FRAME);
  write_i2s_mono_stereo(I2S_SECONDARY, bufSecondary, FRAME);

  // 9) Print to serial every ~100ms (at 44.1kHz with FRAME=64, ~689 blocks/sec)
  print_counter++;
  if (print_counter >= 70) {  // ~70 blocks ≈ 100ms
    error_avg /= (float)(FRAME * 70);
    output_avg /= (float)(FRAME * 70);
    
    Serial.print("Error(avg): ");
    Serial.print(error_avg, 6);
    Serial.print(" | Output(avg): ");
    Serial.print(output_avg, 6);
    Serial.print(" | W[0]: ");
    Serial.println(w[0], 6);
    
    print_counter = 0;
    error_avg = 0.0f;
    output_avg = 0.0f;
  }
  
  // 10) Periodically save to NVS (every 10 seconds)
  unsigned long current_time = millis();
  if (current_time - last_save_time >= SAVE_INTERVAL_MS) {
    save_to_nvs();
    last_save_time = current_time;
  }
}

// ----------- Setup & loop -----------
void setup() {
  // Serial just for debug
  Serial.begin(115200);
  delay(200);
  Serial.println("=== ESP32 ANC System Starting ===");

  // ADC
  analogReadResolution(12);
  
  // I2S
  setup_i2s_tx(I2S_PRIMARY,   BCK1, WS1, DIN1);
  setup_i2s_tx(I2S_SECONDARY, BCK2, WS2, DIN2);

  // load previous state from NVS
  bool loaded = load_from_nvs();
  
  if (!loaded) {
    // Init ANC state from scratch
    memset(w, 0, sizeof(w));
    phase = 0.0;
    Serial.println("Initialized with zero weights");
  }
  
  // Always clear buffers and reset index
  memset(xBuf, 0, sizeof(xBuf));
  idx = 0;
  dc.reset();

  // Phase increment for 1 kHz
  dphi = TWOPI * (double)F0_HZ / (double)FS_HZ;
  
  // Initialize save timer
  last_save_time = millis();

  Serial.println("ANC (Simplified LMS) ready for 1kHz cancellation…");
  Serial.println("Weights will auto-save every 10 seconds");
}

void loop() {
  anc_process_block();   // real-time block processing
  // (runs at I2S DMA pace.)
}
