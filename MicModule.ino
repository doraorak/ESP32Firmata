// Mic module (id 0x05): sound level off a microphone — RMS windows -> decibels.
// Mirrors ESP32FirmataSwift's MicModuleHandler byte-for-byte.

/* ==== Mic module ==========================================================
   Burst-samples on-device each window, computes the DC-removed RMS, converts
   to decibels, and drops both into scheduler registers — so tasks branch on
   loudness and the host just reads registers. dB is relative full-scale
   (0 dB ~ full-scale sine); a stored offset shifts it to ~SPL after a
   one-point calibration.
   Two mic kinds share this module — an ANALOG mic on an ADC pin (op 0x00) and
   a DIGITAL I2S MEMS mic like the INMP441 (op 0x03). Both reduce to the same
   RMS->dB path; only the sampling source and the full-scale reference differ.
   Ops:
     0x00 <pin> <dbFReg> <rmsReg> <winLo> <winHi>            analog configure + windows (ms, >=50)
     0x01                                                    read now -> reply [0x01 ok db:5 rms:5]
     0x02 <offBits:5>                                        set calibration offset (dB, IEEE-754)
     0x03 <bclk> <ws> <sd> <dbFReg> <rmsReg> <winLo> <winHi> [<rate:3>]  I2S configure (INMP441);
                                                             optional 3x7-bit sample rate (default 16 kHz)
     0x04                                                    I2S diagnostic -> reply [0x04 peak:5]
     0x05 <hzFReg | 0x7F>                                    I2S dominant-frequency (FFT) -> F[hzFReg]; 0x7F = off
   A window's burst blocks ~16 ms — same class as a sonar ping, fine at tick cadence. */

#include "driver/i2s_std.h"    // I2S RX for digital MEMS mics (INMP441 etc.)

// ---- I2S RX helpers for a digital MEMS mic — mirror firmata_shim.cpp -------
static i2s_chan_handle_t micI2sRx = NULL;

static bool micI2sBegin(int bclk, int ws, int sd, int sampleRate) {
  if (micI2sRx) { i2s_channel_disable(micI2sRx); i2s_del_channel(micI2sRx); micI2sRx = NULL; }
  i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_AUTO, I2S_ROLE_MASTER);
  if (i2s_new_channel(&chan_cfg, NULL, &micI2sRx) != ESP_OK) return false;
  // STEREO on purpose: the INMP441's data lands on whichever slot its L/R pin selects,
  // and that mapping is inconsistent on the ESP32. RMS over both slots sidesteps the
  // guessing — the silent slot just contributes zeros.
  i2s_std_config_t std_cfg = {
    .clk_cfg  = I2S_STD_CLK_DEFAULT_CONFIG((uint32_t)sampleRate),
    .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_32BIT, I2S_SLOT_MODE_STEREO),
    .gpio_cfg = {
      .mclk = I2S_GPIO_UNUSED,
      .bclk = (gpio_num_t)bclk,
      .ws   = (gpio_num_t)ws,
      .dout = I2S_GPIO_UNUSED,
      .din  = (gpio_num_t)sd,
      .invert_flags = { .mclk_inv = false, .bclk_inv = false, .ws_inv = false },
    },
  };
  if (i2s_channel_init_std_mode(micI2sRx, &std_cfg) != ESP_OK) {
    i2s_del_channel(micI2sRx); micI2sRx = NULL; return false;
  }
  if (i2s_channel_enable(micI2sRx) != ESP_OK) {
    i2s_del_channel(micI2sRx); micI2sRx = NULL; return false;
  }
  return true;
}

static float micI2sRms() {
  if (!micI2sRx) return -1;
  static int32_t buf[256];
  size_t bytesRead = 0;
  if (i2s_channel_read(micI2sRx, buf, sizeof(buf), &bytesRead, 100) != ESP_OK) return -1;
  int n = (int)(bytesRead / sizeof(int32_t));
  if (n < 8) return -1;
  double sum = 0, sumsq = 0;                     // DC-removed RMS over the block
  for (int i = 0; i < n; i++) { double s = (double)(buf[i] >> 8); sum += s; sumsq += s * s; }
  double mean = sum / n;
  double var = sumsq / n - mean * mean;
  if (var < 0) var = 0;
  return (float)sqrt(var);
}

// Diagnostic: largest-magnitude RAW 32-bit sample in a block (pre-shift). Zero means SD
// delivered nothing (dead/unpowered mic or wiring fault); a large value with no RMS
// reaction points at the bit extraction instead.
static int32_t micI2sPeakRaw() {
  if (!micI2sRx) return -1;
  static int32_t buf[256];
  size_t bytesRead = 0;
  if (i2s_channel_read(micI2sRx, buf, sizeof(buf), &bytesRead, 100) != ESP_OK) return -1;
  int n = (int)(bytesRead / sizeof(int32_t));
  int32_t peak = 0;
  for (int i = 0; i < n; i++) { int32_t a = buf[i] < 0 ? -buf[i] : buf[i]; if (a > peak) peak = a; }
  return peak;
}

// Dominant frequency (Hz) of the I2S mic's left channel over a 512-sample window: Hann-window +
// radix-2 FFT, biggest magnitude bin, parabolically interpolated. 0 = no bin dominates (peak not
// >= 6x the mean), -1 = read error. Mirrors firmata_shim.cpp fm_i2s_mic_dominant_hz.
static const int MIC_FFT_N = 512;
static float micI2sDominantHz(int sampleRate) {
  if (!micI2sRx) return -1;
  static float re[MIC_FFT_N], im[MIC_FFT_N];
  static int32_t rd[256];
  const float kTwoPi = 6.283185307179586f;
  int got = 0, attempts = 0;
  while (got < MIC_FFT_N && attempts < 8) {                 // one DMA read is < 512 frames; collect
    size_t br = 0;
    if (i2s_channel_read(micI2sRx, rd, sizeof(rd), &br, 100) != ESP_OK) break;
    int frames = (int)(br / sizeof(int32_t)) / 2;           // STEREO: 2 int32 per frame
    for (int i = 0; i < frames && got < MIC_FFT_N; i++) { re[got] = (float)(rd[2 * i] >> 8); im[got] = 0.0f; got++; }
    attempts++;
  }
  if (got < MIC_FFT_N) return -1;
  double mean = 0; for (int i = 0; i < MIC_FFT_N; i++) mean += re[i]; mean /= MIC_FFT_N;
  for (int i = 0; i < MIC_FFT_N; i++) {                     // DC-remove + Hann window
    float w = 0.5f * (1.0f - cosf(kTwoPi * i / (MIC_FFT_N - 1)));
    re[i] = (re[i] - (float)mean) * w;
  }
  for (int i = 1, j = 0; i < MIC_FFT_N; i++) {              // bit-reversal permutation
    int bit = MIC_FFT_N >> 1;
    for (; j & bit; bit >>= 1) j ^= bit;
    j ^= bit;
    if (i < j) { float t; t = re[i]; re[i] = re[j]; re[j] = t; t = im[i]; im[i] = im[j]; im[j] = t; }
  }
  for (int len = 2; len <= MIC_FFT_N; len <<= 1) {          // Cooley-Tukey butterflies
    float ang = -kTwoPi / len, wlR = cosf(ang), wlI = sinf(ang);
    for (int i = 0; i < MIC_FFT_N; i += len) {
      float wR = 1, wI = 0;
      for (int k = 0; k < len / 2; k++) {
        int a = i + k, b = a + len / 2;
        float vR = re[b] * wR - im[b] * wI, vI = re[b] * wI + im[b] * wR;
        re[b] = re[a] - vR; im[b] = im[a] - vI;
        re[a] += vR;        im[a] += vI;
        float nwR = wR * wlR - wI * wlI; wI = wR * wlI + wI * wlR; wR = nwR;
      }
    }
  }
  int minBin = (int)(50.0f * MIC_FFT_N / sampleRate); if (minBin < 1) minBin = 1;   // ignore < 50 Hz
  int maxBin = MIC_FFT_N / 2, peakBin = minBin;
  float peakMag = -1; double magSum = 0;
  for (int b = minBin; b < maxBin; b++) {
    float m = re[b] * re[b] + im[b] * im[b];
    magSum += m;
    if (m > peakMag) { peakMag = m; peakBin = b; }
  }
  double meanMag = magSum / (maxBin - minBin);
  if (peakMag < 6.0 * meanMag) return 0;                   // no bin dominates -> no tone
  float delta = 0;                                         // parabolic peak interpolation
  if (peakBin > minBin && peakBin < maxBin - 1) {
    float a = re[peakBin-1]*re[peakBin-1] + im[peakBin-1]*im[peakBin-1];
    float b = peakMag;
    float c = re[peakBin+1]*re[peakBin+1] + im[peakBin+1]*im[peakBin+1];
    float d = a - 2*b + c;
    if (d != 0) delta = 0.5f * (a - c) / d;
  }
  return (peakBin + delta) * (float)sampleRate / MIC_FFT_N;
}

struct MicModuleHandler : ModuleHandler {
  uint8_t id()    const override { return 0x05; }
  uint8_t major() const override { return 1; }
  uint8_t minor() const override { return 2; }   // 1.2: I2S sample rate + op 0x05 dominant frequency
  const char *name() const override { return "mic"; }

  int      pin = -1;                 // -1 = unconfigured (analog)
  bool     i2sMode = false;          // true = digital I2S mic (INMP441), pin ignored
  int      sampleRate = 16000;       // I2S audio rate (Hz); host-settable, drives Hz mapping
  int      hzFReg = -1;              // -1 = dominant-frequency detection off (I2S only)
  int      dbFReg = 0;
  int      rmsReg = 0;
  uint32_t windowMs = 250;
  uint32_t nextWindowMs = 0;
  float    offsetDb = 0;             // calibration: survives reset() on purpose

  bool configured() { return i2sMode || pin >= 0; }

  // One block of samples -> DC-removed RMS. I2S reads a DMA block (24-bit MEMS data);
  // analog burst-samples the ADC for ~16 ms. RMS is around the measured mean, so the
  // mic's DC bias cancels out.
  float measureRms() {
    if (i2sMode) return micI2sRms();
    if (pin < 0) return -1;
    int64_t sum = 0, sumSq = 0, n = 0;
    uint32_t start = millis();
    while ((uint32_t)(millis() - start) < 16) {
      int64_t v = analogRead((uint8_t)pin);
      sum += v; sumSq += v * v; n++;
    }
    if (n < 8) return -1;
    double mean = (double)sum / (double)n;
    double variance = (double)sumSq / (double)n - mean * mean;
    if (variance < 0) variance = 0;
    return (float)sqrt(variance);
  }

  // rms -> dB relative to full scale, floored so silence gives a real number not -inf.
  // Reference differs by source: 12-bit ADC full-scale sine ~ 2048/sqrt(2); I2S 24-bit ~ 2^23.
  float decibels(float rms) {
    float r = rms < 0.5f ? 0.5f : rms;
    float ref = i2sMode ? 8388608.0f : 1448.0f;
    return 20.0f * log10f(r / ref) + offsetDb;
  }

  void handle(const uint8_t *payload, int length) override {
    if (length < 1) return;
    switch (payload[0]) {
      case 0x00:                    // analog configure: pin, F[db], R[rms], window ms
        if (length >= 6) {
          i2sMode = false;
          pin     = payload[1] & 0x7F;
          dbFReg  = payload[2] & 0x0F;
          rmsReg  = payload[3] & 0x1F;
          windowMs = (uint32_t)(payload[4] & 0x7F) | ((uint32_t)(payload[5] & 0x7F) << 7);
          if (windowMs < 50) windowMs = 50;
          analogReadResolution(12);
          nextWindowMs = millis();
        }
        break;
      case 0x03:                    // I2S configure: bclk, ws, sd, F[db], R[rms], winLo, winHi, [rate:3]
        if (length >= 8) {
          dbFReg  = payload[4] & 0x0F;
          rmsReg  = payload[5] & 0x1F;
          windowMs = (uint32_t)(payload[6] & 0x7F) | ((uint32_t)(payload[7] & 0x7F) << 7);
          if (windowMs < 50) windowMs = 50;
          int rate = 16000;                          // default; optional 3x7-bit rate follows
          if (length >= 11)
            rate = (int)((uint32_t)(payload[8] & 0x7F) | ((uint32_t)(payload[9] & 0x7F) << 7)
                         | ((uint32_t)(payload[10] & 0x7F) << 14));
          if (rate < 8000) rate = 8000;
          if (rate > 48000) rate = 48000;
          if (micI2sBegin(payload[1] & 0x7F, payload[2] & 0x7F, payload[3] & 0x7F, rate)) {
            i2sMode = true; pin = -1;
            sampleRate = rate;
            nextWindowMs = millis();
          }
        }
        break;
      case 0x01: {                  // one-shot: measure now, reply db+rms to the host
        float rms = measureRms();
        uint8_t ok = rms < 0 ? 0 : 1;
        float db = rms < 0 ? 0.0f : decibels(rms);
        uint8_t out[16];
        int n = 0;
        out[n++] = START_SYSEX; out[n++] = MODULE_DATA; out[n++] = id(); out[n++] = 0x01; out[n++] = ok;
        uint32_t db_;
        memcpy(&db_, &db, 4);                       // dB as 5x7-bit limbs of the IEEE-754 bits
        for (int i = 0; i < 5; i++) { out[n++] = db_ & 0x7F; db_ >>= 7; }
        uint32_t r = rms < 0 ? 0 : (uint32_t)rms;   // raw RMS counts as 5x7-bit limbs
        for (int i = 0; i < 5; i++) { out[n++] = r & 0x7F; r >>= 7; }
        out[n++] = END_SYSEX;
        sendFrame(out, n);
        break;
      }
      case 0x02:                    // calibration offset (dB), IEEE-754 bits in 5 limbs
        if (length >= 6) {
          uint32_t bits = 0;
          for (int i = 5; i >= 1; i--) bits = (bits << 7) | (uint32_t)(payload[i] & 0x7F);
          memcpy(&offsetDb, &bits, 4);
        }
        break;
      case 0x04: {                  // I2S diagnostic: reply the raw peak sample [0x04 peak:5]
        int32_t peak = i2sMode ? micI2sPeakRaw() : -1;
        uint8_t out[12];
        int n = 0;
        out[n++] = START_SYSEX; out[n++] = MODULE_DATA; out[n++] = id(); out[n++] = 0x04;
        uint32_t p = (uint32_t)peak;
        for (int i = 0; i < 5; i++) { out[n++] = p & 0x7F; p >>= 7; }
        out[n++] = END_SYSEX;
        sendFrame(out, n);
        break;
      }
      case 0x05:                    // enable/disable dominant-frequency (I2S) -> F[hzFReg]; 0x7F = off
        if (length >= 2) hzFReg = (payload[1] == 0x7F) ? -1 : (payload[1] & 0x0F);
        break;
      default: break;
    }
  }

  void tick() override {
    if (!configured()) return;
    uint32_t now = millis();
    if ((int32_t)(now - nextWindowMs) < 0) return;
    float rms = measureRms();
    if (rms >= 0) {
      scheduler.fregs[dbFReg] = decibels(rms);
      scheduler.regs[rmsReg] = (int32_t)rms;
    }
    if (i2sMode && hzFReg >= 0) {
      float hz = micI2sDominantHz(sampleRate);
      if (hz >= 0) scheduler.fregs[hzFReg] = hz;   // 0 = no dominant tone; <0 = read error (skip)
    }
    nextWindowMs = now + windowMs;
  }

  void reset() override {
    pin = -1; i2sMode = false; hzFReg = -1;   // offsetDb kept: calibration outlives sessions
  }
};
static MicModuleHandler micModule;
