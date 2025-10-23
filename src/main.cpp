#include <Arduino.h>
#include "driver/i2s.h"

// ----------- User-tunable ANC params -----------
static const uint32_t FS_HZ        = 44100;
static const uint16_t FRAME        = 64;       // block size
static const float    F0_HZ        = 1000.0f;  // primary tone
static const int      N_TAPS       = 64;       // ANC FIR length (control filter)
static const int      S_DELAY      = 12;       // samples of secondary-path delay (start ~ 10–20)
static const float    MU           = 2.5e-7f;  // LMS step size (start tiny; increase cautiously)
static const float    LEAK         = 0.0f;     // optional leaky FXLMS (0..1); 0 = off
static const float    OUT_CLIP     = 0.95f;    // clip before I2S (±1.0 full-scale)
static const int      HPF_DC_N     = 1024;     // simple DC remover length for mic

// ----------- Pins (your mapping) ---------------
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

// ----------- Fixed-point/I2S helpers -----------
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
    // incremental mean (Welford simplified for mean only)
    n = min(n+1, HPF_DC_N);
    acc += (x - acc) / (float)n;
    return x - acc;
  }
} dc;

// ----------- ANC state -----------
static float w[N_TAPS];            // control filter coefficients
static float xBuf[N_TAPS + 256];   // ring buffer for reference (pad for simplicity)
static float xfBuf[N_TAPS + 256];  // filtered-x (here delay-only)
static int   idx = 0;              // ring index

// phase accumulator for 1 kHz
static double phase = 0.0;
static const double TWOPI = 6.283185307179586;
static double dphi;

// ----------- I2S setup -----------
void setup_i2s_tx(i2s_port_t port, int bck, int ws, int din) {
  i2s_config_t cfg = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX),
    .sample_rate = (int)FS_HZ,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
    .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT, // MAX98357 wants stereo LR; we'll duplicate mono
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

// ----------- ADC (mic on GPIO39) -----------
static const int MIC_PIN = 39; // ADC1_CH3

inline float read_mic_norm() {
  // 12-bit ADC: 0..4095; center around 0 and normalize to ±1
  int raw = analogRead(MIC_PIN);
  float x = (float)raw;
  // Map 0..4095 -> -1..+1 (center ~ 2048), also light scaling to avoid clipping
  return ( (x - 2048.0f) / 2048.0f );
}

// ----------- ANC core (block processing) -----------
void anc_process_block() {
  int16_t bufPrimary[FRAME]  = {0}; // to primary I2S (the "noise")
  int16_t bufSecondary[FRAME]= {0}; // to secondary I2S (the anti-noise)

  for (int n = 0; n < FRAME; ++n) {
    // 1) Generate primary reference x[n] = sin(2π f0 t)
    float x = (float)sin(phase);
    phase += dphi;
    if (phase >= TWOPI) phase -= TWOPI;

    // 2) Push reference into ring buffer
    xBuf[idx] = x;

    // 3) Generate control output y[n] = sum_k w[k] * x[n-k]
    float y = 0.0f;
    int j = idx;
    for (int k = 0; k < N_TAPS; ++k) {
      // ring index (wrap)
      int jj = j - k;
      if (jj < 0) jj += (int)(sizeof(xBuf)/sizeof(xBuf[0]));
      y += w[k] * xBuf[jj];
    }

    // 4) Play signals: primary (x scaled) and secondary (y)
    float primary_out = 0.50f * x;   // make primary smaller to avoid clipping
    float secondary_out = y;

    // Clip before convert
    if (primary_out  >  OUT_CLIP) primary_out  =  OUT_CLIP;
    if (primary_out  < -OUT_CLIP) primary_out  = -OUT_CLIP;
    if (secondary_out>  OUT_CLIP) secondary_out=  OUT_CLIP;
    if (secondary_out< -OUT_CLIP) secondary_out= -OUT_CLIP;

    bufPrimary[n]   = f32_to_i16(primary_out);
    bufSecondary[n] = f32_to_i16(secondary_out);

    // 5) Measure error mic (residual at ear)
    float e = read_mic_norm();
    e = dc.step(e);     // remove DC drift

    // 6) Filtered-X ref: delay-only model (x_f[n] = x[n - S_DELAY])
    int jdel = idx - S_DELAY;
    if (jdel < 0) jdel += (int)(sizeof(xBuf)/sizeof(xBuf[0]));
    float xf = xBuf[jdel];

    // 7) LMS weight update: w[k] += mu * e[n] * x_f[n-k] - leak*w[k]
    int jf = jdel;
    for (int k = 0; k < N_TAPS; ++k) {
      int jjf = jf - k;
      if (jjf < 0) jjf += (int)(sizeof(xBuf)/sizeof(xBuf[0]));
      float xfk = xBuf[jjf];                 // because filtered-x == delayed x
      w[k] = (1.0f - LEAK) * w[k] + MU * e * xfk;
    }

    // 8) advance ring index
    idx++;
    if (idx >= (int)(sizeof(xBuf)/sizeof(xBuf[0]))) idx = 0;
  }

  // 9) push out blocks to I2S
  write_i2s_mono_stereo(I2S_PRIMARY,   bufPrimary,   FRAME);
  write_i2s_mono_stereo(I2S_SECONDARY, bufSecondary, FRAME);
}

// ----------- Setup & loop -----------
void setup() {
  // Serial just for debug
  Serial.begin(115200);
  delay(200);

  // ADC
  analogReadResolution(12);
  // (Optionally) analogSetPinAttenuation(MIC_PIN, ADC_11db);

  // I2S
  setup_i2s_tx(I2S_PRIMARY,   BCK1, WS1, DIN1);
  setup_i2s_tx(I2S_SECONDARY, BCK2, WS2, DIN2);

  // Init ANC state
  memset(w, 0, sizeof(w));
  memset(xBuf, 0, sizeof(xBuf));
  idx = 0;
  dc.reset();

  // Phase increment for 1 kHz
  dphi = TWOPI * (double)F0_HZ / (double)FS_HZ;

  Serial.println("ANC (FXLMS, delay-only S^) starting…");
}

void loop() {
  anc_process_block();   // real-time block processing
  // (No delay; runs at I2S DMA pace.)
}
