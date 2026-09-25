// =============================================================================
//  SIH 2026 — Edge voice activator (wake word: "hey comet")
//  ESP32-S3 firmware — Wokwi simulation build
// =============================================================================
//
//  Pipeline demonstrated:
//
//    AudioSource (SIMULATED in Wokwi)  ->  DMA-style block buffering
//      -> Keyword spotting (MOCK_KWS in Wokwi / Edge Impulse on real build)
//      -> keyword detected -> MQTT event -> short capture -> MQTT audio packet
//      -> cloud server (MOCK_ASR / real ASR) -> MQTT command -> act on it
//      -> back to listening
//
//  HONESTY NOTES (read before demoing):
//    * Wokwi has no INMP441 / I2S microphone model. The audio in this build is
//      a SOFTWARE-GENERATED test signal (silence, noise, tone patterns). It is
//      NOT human speech and NOT real microphone audio.
//    * MOCK_KWS is a deterministic tone-sequence detector (Goertzel filters).
//      It only exercises the pipeline. It is NOT a speech model.
//    * Wi-Fi and MQTT are REAL: the simulated chip really connects to
//      Wokwi-GUEST and a real MQTT broker on the internet.
//
//  The two replacement points are marked with:
//    ===== REPLACEMENT POINT A =====   simulated microphone -> INMP441 over I2S
//    ===== REPLACEMENT POINT B =====   MOCK_KWS -> Edge Impulse model
// =============================================================================

#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <math.h>
#include <string.h>

// Optional local overrides (credentials, broker, device id...).
// Copy secrets.example.h -> secrets.h for LOCAL builds only.
// Never put real secrets in a public/shared Wokwi project.
#if __has_include("secrets.h")
#include "secrets.h"
#endif

// -----------------------------------------------------------------------------
// 1. BUILD SWITCHES
// -----------------------------------------------------------------------------

// 0 = SimulatedAudioSource (Wokwi). 1 = INMP441AudioSource (real hardware).
#ifndef USE_INMP441
#define USE_INMP441 0
#endif

// 1 = always use MOCK_KWS even if an Edge Impulse library is installed.
#ifndef FORCE_MOCK_KWS
#define FORCE_MOCK_KWS 0
#endif

// ===== REPLACEMENT POINT B (part 1 of 2) =====================================
// Edge Impulse exports an Arduino library whose main header is named
// <your_project_name>_inferencing.h. If your Edge Impulse project is not named
// "hey_comet", change the header name on BOTH lines below.
// If the header is not found, the build falls back to MOCK_KWS automatically.
#if !FORCE_MOCK_KWS && __has_include(<hey_comet_inferencing.h>)
#include <hey_comet_inferencing.h>
#define KWS_USE_EDGE_IMPULSE 1
#else
#define KWS_USE_EDGE_IMPULSE 0
#endif
// =============================================================================

// -----------------------------------------------------------------------------
// 2. NETWORK CONFIGURATION (overridable from secrets.h / build flags)
// -----------------------------------------------------------------------------

#ifndef WIFI_SSID
#define WIFI_SSID "Wokwi-GUEST"      // Wokwi's simulated open access point
#endif
#ifndef WIFI_PASSWORD
#define WIFI_PASSWORD ""             // Wokwi-GUEST has no password
#endif
#ifndef WIFI_CHANNEL
#define WIFI_CHANNEL 6               // Wokwi-GUEST is on channel 6 (skips scan).
#endif                               // Use 0 (= any channel) on real hardware.

// Default broker is a PUBLIC test broker: no credentials, no privacy.
// Anyone can read these topics. Fine for a tone-pattern demo; do NOT send
// real user audio to it. Use your own broker for real audio.
#ifndef MQTT_BROKER
#define MQTT_BROKER "test.mosquitto.org"
#endif
#ifndef MQTT_PORT
#define MQTT_PORT 1883
#endif
#ifndef MQTT_USERNAME
#define MQTT_USERNAME ""             // empty = connect without auth
#endif
#ifndef MQTT_PASSWORD
#define MQTT_PASSWORD ""
#endif
#ifndef MQTT_CLIENT_ID
#define MQTT_CLIENT_ID ""            // empty = auto "sih-device01-<chip id>"
#endif

#ifndef DEVICE_ID
#define DEVICE_ID "device01"
#endif

#define TOPIC_STATUS  "sih/" DEVICE_ID "/status"   // retained ONLINE / OFFLINE (LWT)
#define TOPIC_EVENT   "sih/" DEVICE_ID "/event"    // "KEYWORD_DETECTED"
#define TOPIC_AUDIO   "sih/" DEVICE_ID "/audio"    // audio packet (JSON summary here)
#define TOPIC_COMMAND "sih/" DEVICE_ID "/command"  // cloud -> device JSON command

// -----------------------------------------------------------------------------
// 3. PINS
// -----------------------------------------------------------------------------

static const int PIN_LED_WIFI   = 4;   // green : Wi-Fi status
static const int PIN_LED_KWS    = 5;   // yellow: keyword detected / capture
static const int PIN_LED_MQTT   = 6;   // blue  : MQTT link / cloud traffic
static const int PIN_LED_LIGHT  = 7;   // white : the "light" controlled by commands
static const int PIN_BTN_INJECT = 14;  // push button: inject keyword test pattern

// Future INMP441 wiring (NOT connected in Wokwi — no such part there):
//   INMP441 SCK -> GPIO 12   (I2S bit clock, BCLK)
//   INMP441 WS  -> GPIO 11   (I2S word select, LRCLK)
//   INMP441 SD  -> GPIO 10   (I2S data in)
//   INMP441 L/R -> GND       (select LEFT channel)
//   INMP441 VDD -> 3V3, GND -> GND
static const int PIN_I2S_SCK = 12;
static const int PIN_I2S_WS  = 11;
static const int PIN_I2S_SD  = 10;

// -----------------------------------------------------------------------------
// 4. AUDIO / KWS / TIMING PARAMETERS
// -----------------------------------------------------------------------------

static const uint32_t SAMPLE_RATE_HZ           = 16000;  // 16 kHz mono
static const size_t   AUDIO_BLOCK_SAMPLES      = 512;    // 32 ms per block (one "DMA buffer")
static const size_t   AUDIO_MAX_BACKLOG_BLOCKS = 4;      // like I2S dma_buf_count

static const float    KWS_THRESHOLD            = 0.80f;
static const char*    KWS_KEYWORD_LABEL        = "hey_comet";  // must match your EI label

static const uint32_t CAPTURE_DURATION_MS      = 1000;   // post-keyword command audio
static const uint32_t ASR_TIMEOUT_MS           = 10000;
static const uint32_t COMMAND_DISPLAY_MS       = 1200;   // stay in COMMAND_RECEIVED briefly

static const uint32_t WIFI_CONNECT_TIMEOUT_MS  = 15000;
static const uint32_t WIFI_RETRY_BACKOFF_MS    = 3000;
static const uint32_t MQTT_RETRY_BACKOFF_MS    = 3000;
static const uint16_t MQTT_SOCKET_TIMEOUT_S    = 5;
static const uint16_t MQTT_KEEPALIVE_S         = 30;
static const uint16_t MQTT_BUFFER_BYTES        = 1024;

static const float kTwoPi = 6.28318530718f;

// -----------------------------------------------------------------------------
// 5. NON-BLOCKING STATUS LED
// -----------------------------------------------------------------------------

class StatusLed {
 public:
  explicit StatusLed(int pin) : pin_(pin) {}
  void begin() { pinMode(pin_, OUTPUT); off(); }
  void off() { mode_ = MODE_OFF; write(false); }
  void on()  { mode_ = MODE_ON;  write(true); }
  void blink(uint16_t periodMs) {
    if (mode_ == MODE_BLINK && period_ == periodMs) return;  // keep phase
    mode_ = MODE_BLINK; period_ = periodMs; last_ = millis(); write(true);
  }
  // Flash `count` times, then settle ON (endOn=true) or OFF.
  void pulse(uint8_t count, uint16_t periodMs, bool endOn) {
    mode_ = MODE_PULSE; period_ = periodMs; endOn_ = endOn;
    toggles_ = (uint8_t)(count * 2 - 1); last_ = millis(); write(true);
  }
  void update(uint32_t now) {
    if (mode_ != MODE_BLINK && mode_ != MODE_PULSE) return;
    if (now - last_ < (uint32_t)(period_ / 2)) return;
    last_ = now;
    if (mode_ == MODE_PULSE) {
      if (toggles_ == 0) { mode_ = endOn_ ? MODE_ON : MODE_OFF; write(endOn_); return; }
      toggles_--;
    }
    write(!level_);
  }
 private:
  enum Mode : uint8_t { MODE_OFF, MODE_ON, MODE_BLINK, MODE_PULSE };
  void write(bool level) { level_ = level; digitalWrite(pin_, level ? HIGH : LOW); }
  int pin_;
  Mode mode_ = MODE_OFF;
  bool level_ = false, endOn_ = false;
  uint16_t period_ = 500;
  uint8_t toggles_ = 0;
  uint32_t last_ = 0;
};

static StatusLed ledWifi(PIN_LED_WIFI);
static StatusLed ledKws(PIN_LED_KWS);
static StatusLed ledMqtt(PIN_LED_MQTT);
static StatusLed ledLight(PIN_LED_LIGHT);

// -----------------------------------------------------------------------------
// 6. AUDIO SOURCE ABSTRACTION
//
//    AudioSource
//     ├── SimulatedAudioSource   (Wokwi, deterministic test signal)
//     └── INMP441AudioSource     (real hardware, I2S — placeholder)
//
//  Contract: read() is NON-BLOCKING and returns 16-bit signed mono PCM at
//  SAMPLE_RATE_HZ. It returns how many samples were copied (0 = none ready).
//  The rest of the firmware only sees this interface.
// -----------------------------------------------------------------------------

class AudioSource {
 public:
  virtual ~AudioSource() {}
  virtual bool begin(uint32_t sampleRateHz) = 0;
  virtual size_t read(int16_t* dst, size_t maxSamples) = 0;
  virtual const char* name() const = 0;
  virtual bool isSimulated() const = 0;
  virtual uint32_t overruns() const { return 0; }
};

// ---- 6a. SIMULATED SOURCE ---------------------------------------------------
//
// Produces samples at exactly 16 kHz of *simulated* time (paced by micros()),
// the same way an I2S DMA ring fills up in the background. If the firmware
// does not read for a while, at most AUDIO_MAX_BACKLOG_BLOCKS are kept and
// older samples are dropped (counted as an overrun) — like a real DMA ring.
//
// Every sample is a pure function of its absolute sample index, so the signal
// is bit-exact reproducible across runs, even if samples are dropped.
//
// 10-second repeating timeline (starts when LISTENING is first entered):
//   0.0 – 1.0 s  near-silence (±4 LSB dither)
//   1.0 – 10 s   background noise (uniform ±1000) + 50 Hz mains hum
//   3.0 – 3.2 s  DISTRACTOR: 1000 Hz tone only (partial pattern, must be rejected)
//   6.0 – 6.6 s  KEYWORD TEST PATTERN: 1000 Hz -> 1500 Hz -> 2000 Hz, 200 ms each
// The push button injects the keyword test pattern immediately.
//
// The keyword test pattern is a synthetic tone sequence. It is NOT speech.

class SimulatedAudioSource : public AudioSource {
 public:
  static const uint32_t CYCLE_MS           = 10000;
  static const uint32_t SILENCE_END_MS     = 1000;
  static const uint32_t DISTRACTOR_START_MS = 3000;
  static const uint32_t KEYWORD_START_MS   = 6000;
  static const uint32_t TONE_SEGMENT_MS    = 200;
  static const int32_t  NOISE_AMP          = 1000;
  static constexpr float HUM_AMP           = 300.0f;
  static constexpr float TONE_AMP          = 8000.0f;
  static const uint32_t TONE_HZ[3];

  bool begin(uint32_t sampleRateHz) override {
    rate_ = sampleRateHz;
    lastUs_ = micros();
    rateAccum_ = 0; backlog_ = 0; index_ = 0; overruns_ = 0;
    injectRequested_ = false; hasInject_ = false; injectStart_ = 0;
    lastSeg_ = SEG_UNSET;
    Serial.printf("[MIC-SIM] SimulatedAudioSource started: %lu Hz, 16-bit mono, %u-sample blocks\n",
                  (unsigned long)rate_, (unsigned)AUDIO_BLOCK_SAMPLES);
    Serial.println("[MIC-SIM] NOTE: synthetic test signal, not a real microphone, not speech");
    return true;
  }

  size_t read(int16_t* dst, size_t maxSamples) override {
    // Advance the simulated "DMA" by the elapsed time.
    const uint32_t now = micros();
    const uint32_t elapsedUs = now - lastUs_;  // wrap-safe
    lastUs_ = now;
    rateAccum_ += (uint64_t)elapsedUs * rate_;
    backlog_ += rateAccum_ / 1000000ULL;
    rateAccum_ %= 1000000ULL;

    const uint64_t maxBacklog = (uint64_t)AUDIO_BLOCK_SAMPLES * AUDIO_MAX_BACKLOG_BLOCKS;
    if (backlog_ > maxBacklog) {           // reader was too slow: drop oldest audio
      index_ += backlog_ - maxBacklog;     // timeline keeps moving, like a real mic
      backlog_ = maxBacklog;
      overruns_++;
    }

    if (injectRequested_) {                // button: start pattern at the next sample
      injectRequested_ = false;
      hasInject_ = true;
      injectStart_ = index_;
    }

    size_t n = (backlog_ < (uint64_t)maxSamples) ? (size_t)backlog_ : maxSamples;
    for (size_t i = 0; i < n; ++i) dst[i] = sampleAt(index_ + i);
    index_ += n;
    backlog_ -= n;
    return n;
  }

  void injectKeyword() { injectRequested_ = true; }
  const char* name() const override { return "SimulatedAudioSource"; }
  bool isSimulated() const override { return true; }
  uint32_t overruns() const override { return overruns_; }

 private:
  enum Segment : uint8_t { SEG_UNSET, SEG_SILENCE, SEG_NOISE, SEG_DISTRACTOR, SEG_KEYWORD, SEG_INJECTED };

  uint32_t ms2samples(uint32_t ms) const { return (uint32_t)((uint64_t)ms * rate_ / 1000); }

  static uint32_t hash32(uint32_t x) {     // stateless deterministic "noise"
    x *= 0x9E3779B1u; x ^= x >> 16; x *= 0x85EBCA6Bu; x ^= x >> 13;
    x *= 0xC2B2AE35u; x ^= x >> 16;
    return x;
  }

  float sinAt(uint32_t hz, uint64_t idx) const {  // exact phase for integer Hz
    const uint32_t phase = (uint32_t)(((uint64_t)hz * (idx % rate_)) % rate_);
    return sinf(kTwoPi * (float)phase / (float)rate_);
  }

  int16_t sampleAt(uint64_t idx) {
    const uint32_t cyc    = (uint32_t)(idx % ms2samples(CYCLE_MS));
    const uint32_t segLen = ms2samples(TONE_SEGMENT_MS);
    Segment seg;
    uint32_t toneHz = 0;
    float s;

    if (cyc < ms2samples(SILENCE_END_MS)) {
      seg = SEG_SILENCE;
      s = (float)((int32_t)(hash32((uint32_t)idx) % 9u) - 4);
    } else {
      seg = SEG_NOISE;
      s = (float)((int32_t)(hash32((uint32_t)idx) % (uint32_t)(2 * NOISE_AMP + 1)) - NOISE_AMP)
          + HUM_AMP * sinAt(50, idx);
      const uint32_t d0 = ms2samples(DISTRACTOR_START_MS);
      const uint32_t k0 = ms2samples(KEYWORD_START_MS);
      if (cyc >= d0 && cyc < d0 + segLen) {
        seg = SEG_DISTRACTOR; toneHz = TONE_HZ[0];
      } else if (cyc >= k0 && cyc < k0 + 3 * segLen) {
        seg = SEG_KEYWORD; toneHz = TONE_HZ[(cyc - k0) / segLen];
      }
    }
    if (hasInject_ && idx >= injectStart_ && idx < injectStart_ + 3ULL * segLen) {
      seg = SEG_INJECTED; toneHz = TONE_HZ[(uint32_t)(idx - injectStart_) / segLen];
    }
    if (toneHz) s += TONE_AMP * sinAt(toneHz, idx);

    if (seg != lastSeg_) { logSegment(seg, idx); lastSeg_ = seg; }

    if (s > 32767.0f) s = 32767.0f;
    if (s < -32768.0f) s = -32768.0f;
    return (int16_t)s;
  }

  void logSegment(Segment seg, uint64_t idx) const {
    static const char* names[] = {"?", "SILENCE", "BACKGROUND_NOISE",
                                  "DISTRACTOR (1000 Hz only)",
                                  "KEYWORD_TEST_PATTERN (1000>1500>2000 Hz)",
                                  "KEYWORD_TEST_PATTERN (button-injected)"};
    Serial.printf("[MIC-SIM] t=%.3fs segment -> %s\n", (double)idx / rate_, names[seg]);
  }

  uint32_t rate_ = SAMPLE_RATE_HZ;
  uint32_t lastUs_ = 0;
  uint64_t rateAccum_ = 0, backlog_ = 0, index_ = 0, injectStart_ = 0;
  uint32_t overruns_ = 0;
  bool injectRequested_ = false, hasInject_ = false;
  Segment lastSeg_ = SEG_UNSET;
};
const uint32_t SimulatedAudioSource::TONE_HZ[3] = {1000, 1500, 2000};

// ---- 6b. INMP441 SOURCE (real hardware) — PLACEHOLDER ------------------------
//
// ===== REPLACEMENT POINT A ===================================================
// Implement this class on real hardware and build with -DUSE_INMP441=1
// (or change the #define at the top). Nothing else in the firmware changes.
//
// INMP441 facts that matter:
//   * I2S standard (Philips) format, 24-bit data left-justified in a 32-bit slot
//   * L/R pin to GND -> data on the LEFT slot
//   * read 32-bit words, convert to int16: (int16_t)(raw >> 16) is full-scale;
//     a smaller shift (e.g. >> 14) adds ~12 dB gain — clamp to int16 range
//
// Reference outline for arduino-esp32 core 3.x (ESP_I2S.h). Verify the exact
// signatures against the core version you install; this is NOT compiled here:
//
//   #include <ESP_I2S.h>
//   I2SClass i2s;
//   // begin():
//   i2s.setPins(PIN_I2S_SCK, PIN_I2S_WS, -1 /*dout*/, PIN_I2S_SD /*din*/);
//   i2s.begin(I2S_MODE_STD, sampleRateHz, I2S_DATA_BIT_WIDTH_32BIT,
//             I2S_SLOT_MODE_MONO, I2S_STD_SLOT_LEFT);
//   // read(): read up to maxSamples 32-bit words, convert, return count.
//   //   Keep it non-blocking (check available bytes first) so the state
//   //   machine and MQTT keep running.
// =============================================================================
class INMP441AudioSource : public AudioSource {
 public:
  bool begin(uint32_t sampleRateHz) override {
    (void)sampleRateHz;
    Serial.printf("[MIC] INMP441AudioSource is a PLACEHOLDER (SCK=%d WS=%d SD=%d). "
                  "Implement I2S in begin()/read().\n", PIN_I2S_SCK, PIN_I2S_WS, PIN_I2S_SD);
    return false;
  }
  size_t read(int16_t* dst, size_t maxSamples) override {
    (void)dst; (void)maxSamples;
    return 0;  // INSERT I2S READ + 32->16 bit conversion here
  }
  const char* name() const override { return "INMP441AudioSource"; }
  bool isSimulated() const override { return false; }
};

#if USE_INMP441
static INMP441AudioSource micSource;
#else
static SimulatedAudioSource micSource;
#endif
static AudioSource* audioSource = &micSource;
static bool audioReady = false;
static int16_t frameBuf[AUDIO_BLOCK_SAMPLES];

// -----------------------------------------------------------------------------
// 7. KEYWORD SPOTTING
//
//   audio samples (16 kHz mono int16)
//     -> features + model   (EI: MFE/MFCC + NN   | MOCK: Goertzel tone features)
//     -> classification     (label probabilities | tone-sequence score)
//     -> keyword probability
//     -> threshold (KWS_THRESHOLD)
//     -> keyword detected
// -----------------------------------------------------------------------------

struct KwsResult {
  bool evaluated;     // a decision was made on this block
  float probability;  // probability/score of the keyword label (0..1)
};

// ---- 7a. MOCK_KWS -----------------------------------------------------------
// Looks ONLY at the samples it is given (no peeking at the simulator state).
// Per 32 ms frame, Goertzel filters measure what fraction of the frame's
// energy sits at 1000 / 1500 / 2000 Hz. The keyword score is how completely
// the ordered sequence A -> B -> C was observed (>= 5 frames of each = 1.0).
// The 1000 Hz-only distractor scores ~0.33 and is rejected.
class MockKeywordSpotter {
 public:
  static const uint8_t EXPECTED_FRAMES = 5;    // ~160 ms per tone
  static const uint8_t MAX_GAP_FRAMES  = 2;
  static constexpr float TONE_RATIO    = 0.5f;  // fraction of frame energy in bin
  static constexpr float MIN_MEAN_SQ   = 1.0e4f; // ignore near-silent frames

  void begin(uint32_t sampleRateHz, size_t frameLen) {
    for (int k = 0; k < 3; ++k) {
      const float bin = roundf((float)SimulatedAudioSource::TONE_HZ[k] * frameLen / sampleRateHz);
      coeff_[k] = 2.0f * cosf(kTwoPi * bin / (float)frameLen);
    }
    reset();
  }
  void reset() { active_ = false; stage_ = 0; gap_ = 0; counts_[0] = counts_[1] = counts_[2] = 0; }

  KwsResult process(const int16_t* x, size_t n) {
    const int label = classifyFrame(x, n);
    if (label >= 0) {
      if (!active_) {
        if (label != 0) return {false, 0.0f};  // a sequence must start with tone A
        active_ = true; stage_ = 0;
      }
      if (label == stage_) { counts_[stage_]++; gap_ = 0; }
      else if (label == stage_ + 1) { stage_++; counts_[stage_]++; gap_ = 0; }
      else { gap_++; }                          // out-of-order tone
    } else if (active_) {
      gap_++;
    }
    if (active_ && gap_ > MAX_GAP_FRAMES) {     // candidate finished: score it
      const float expected = (float)EXPECTED_FRAMES;
      float score = 0.0f;
      for (int k = 0; k < 3; ++k) {
        const float c = (float)counts_[k];
        score += (c < expected ? c : expected) / expected;
      }
      score /= 3.0f;
      Serial.printf("[MOCK_KWS] candidate: A=%u B=%u C=%u frames -> p=%.2f (%s)\n",
                    counts_[0], counts_[1], counts_[2], score,
                    score >= KWS_THRESHOLD ? "ACCEPT" : "reject, below threshold");
      reset();
      return {true, score};
    }
    return {false, 0.0f};
  }

 private:
  int classifyFrame(const int16_t* x, size_t n) const {
    float energy = 0.0f, s1[3] = {0, 0, 0}, s2[3] = {0, 0, 0};
    for (size_t i = 0; i < n; ++i) {
      const float v = (float)x[i];
      energy += v * v;
      for (int k = 0; k < 3; ++k) {
        const float s0 = v + coeff_[k] * s1[k] - s2[k];
        s2[k] = s1[k]; s1[k] = s0;
      }
    }
    if (n == 0 || energy / n < MIN_MEAN_SQ) return -1;
    int best = -1; float bestRatio = 0.0f;
    for (int k = 0; k < 3; ++k) {
      const float power = s1[k] * s1[k] + s2[k] * s2[k] - coeff_[k] * s1[k] * s2[k];
      const float ratio = 2.0f * power / ((float)n * energy);  // 1.0 = pure tone
      if (ratio > bestRatio) { bestRatio = ratio; best = k; }
    }
    return bestRatio >= TONE_RATIO ? best : -1;
  }

  float coeff_[3] = {0, 0, 0};
  bool active_ = false;
  uint8_t stage_ = 0, gap_ = 0, counts_[3] = {0, 0, 0};
};

#if !KWS_USE_EDGE_IMPULSE
static MockKeywordSpotter mockKws;
#endif

// ---- 7b. EDGE IMPULSE --------------------------------------------------------
// ===== REPLACEMENT POINT B (part 2 of 2) =====================================
// Compiled only when the Edge Impulse header is found (see section 1).
// Uses the standard Edge Impulse C++ SDK API: signal_t, run_classifier(),
// ei_impulse_result_t, EI_CLASSIFIER_* macros.
// A 1 s sliding window is kept in a ring buffer and classified every
// EI_STRIDE_SAMPLES. For production, consider run_classifier_continuous()
// (see Edge Impulse's "continuous audio" Arduino example).
#if KWS_USE_EDGE_IMPULSE
static_assert(EI_CLASSIFIER_FREQUENCY == 16000, "Edge Impulse model must be trained at 16 kHz");
static const size_t EI_STRIDE_SAMPLES = 4096;   // ~256 ms between inferences
static int16_t eiWindow[EI_CLASSIFIER_RAW_SAMPLE_COUNT];
static size_t eiWrite = 0, eiFilled = 0, eiSinceLast = 0;

static int eiGetData(size_t offset, size_t length, float* out) {
  // Oldest sample is at eiWrite (ring buffer).
  for (size_t i = 0; i < length; ++i)
    out[i] = (float)eiWindow[(eiWrite + offset + i) % EI_CLASSIFIER_RAW_SAMPLE_COUNT];
  return 0;
}

static KwsResult runKwsEdgeImpulse(const int16_t* x, size_t n) {
  for (size_t i = 0; i < n; ++i) {
    eiWindow[eiWrite] = x[i];
    eiWrite = (eiWrite + 1) % EI_CLASSIFIER_RAW_SAMPLE_COUNT;
  }
  eiFilled = (eiFilled + n > EI_CLASSIFIER_RAW_SAMPLE_COUNT) ? EI_CLASSIFIER_RAW_SAMPLE_COUNT : eiFilled + n;
  eiSinceLast += n;
  if (eiFilled < EI_CLASSIFIER_RAW_SAMPLE_COUNT || eiSinceLast < EI_STRIDE_SAMPLES) return {false, 0.0f};
  eiSinceLast = 0;

  signal_t signal;
  signal.total_length = EI_CLASSIFIER_RAW_SAMPLE_COUNT;
  signal.get_data = &eiGetData;
  ei_impulse_result_t result = {0};
  EI_IMPULSE_ERROR err = run_classifier(&signal, &result, false);
  if (err != EI_IMPULSE_OK) {
    Serial.printf("[KWS] run_classifier failed: %d\n", (int)err);
    return {false, 0.0f};
  }
  float p = 0.0f;
  for (size_t i = 0; i < EI_CLASSIFIER_LABEL_COUNT; ++i)
    if (strcmp(result.classification[i].label, KWS_KEYWORD_LABEL) == 0) p = result.classification[i].value;
  return {true, p};
}
#endif
// =============================================================================

static const char* kwsBackendName() {
#if KWS_USE_EDGE_IMPULSE
  return "EDGE_IMPULSE";
#else
  return "MOCK_KWS";
#endif
}

static void kwsReset() {
#if KWS_USE_EDGE_IMPULSE
  eiWrite = eiFilled = eiSinceLast = 0;   // require a fresh 1 s window
#else
  mockKws.reset();
#endif
}

// -----------------------------------------------------------------------------
// 8. STATE MACHINE
// -----------------------------------------------------------------------------

enum DeviceState : uint8_t {
  BOOT, WIFI_CONNECTING, MQTT_CONNECTING, LISTENING, KEYWORD_DETECTED,
  CAPTURING, UPLOADING, WAITING_FOR_ASR, COMMAND_RECEIVED
};
static const char* STATE_NAMES[] = {
  "BOOT", "WIFI_CONNECTING", "MQTT_CONNECTING", "LISTENING", "KEYWORD_DETECTED",
  "CAPTURING", "UPLOADING", "WAITING_FOR_ASR", "COMMAND_RECEIVED"
};

struct CloudCommand {
  char command[24];
  char text[96];
  char requestId[32];
};

static WiFiClient wifiClient;
static PubSubClient mqtt(wifiClient);
static char mqttClientId[48];

static DeviceState state = BOOT;
static uint32_t stateEnteredAt = 0;
static uint32_t wifiAttemptStart = 0, wifiRetryAt = 0, mqttRetryAt = 0;
static bool wifiAttemptActive = false;

static float lastKwsProbability = 0.0f;
static uint32_t requestSeq = 0;
static char requestId[32] = "";

static uint32_t capturedSamples = 0, captureTarget = 0;
static double captureSumSq = 0.0;
static int32_t capturePeak = 0;

static volatile bool commandPending = false;
static CloudCommand pendingCommand;
static bool lightOn = false;
static uint32_t lastOverrunsReported = 0;

static void copyStr(char* dst, size_t cap, const char* src) {
  size_t i = 0;
  for (; src && src[i] && i + 1 < cap; ++i) dst[i] = src[i];
  dst[i] = '\0';
}

static void applyStateLeds(DeviceState s) {
  switch (s) {
    case BOOT:             ledWifi.off();      ledMqtt.off();      ledKws.off(); break;
    case WIFI_CONNECTING:  ledWifi.blink(500); ledMqtt.off();      ledKws.off(); break;
    case MQTT_CONNECTING:  ledWifi.on();       ledMqtt.blink(500); ledKws.off(); break;
    case LISTENING:        ledWifi.on();       ledMqtt.on();       ledKws.off(); break;
    case KEYWORD_DETECTED:
    case CAPTURING:        ledWifi.on();       ledMqtt.on();       ledKws.on();  break;
    case UPLOADING:
    case WAITING_FOR_ASR:  ledWifi.on();       ledMqtt.blink(150); ledKws.on();  break;
    case COMMAND_RECEIVED: ledWifi.on();       ledMqtt.on();       ledKws.off(); break;
  }
}

static void onEnterState(DeviceState s);

static void setState(DeviceState s) {
  Serial.printf("[STATE] %s -> %s\n", STATE_NAMES[state], STATE_NAMES[s]);
  state = s;
  stateEnteredAt = millis();
  applyStateLeds(s);
  onEnterState(s);
}

// -----------------------------------------------------------------------------
// 9. HARDWARE-FACING FUNCTIONS (the migration surface)
// -----------------------------------------------------------------------------

// Starts the audio front end. On real hardware this starts I2S DMA.
void initMicrophone() {
  audioReady = audioSource->begin(SAMPLE_RATE_HZ);
  if (!audioReady) {
    Serial.printf("[MIC] ERROR: %s failed to start — keyword spotting disabled\n", audioSource->name());
  }
#if !KWS_USE_EDGE_IMPULSE
  mockKws.begin(SAMPLE_RATE_HZ, AUDIO_BLOCK_SAMPLES);
#endif
}

// Accumulates samples into `frame`. Returns true when a full
// AUDIO_BLOCK_SAMPLES block (32 ms) is ready. Non-blocking.
bool readAudioSamples(int16_t* frame) {
  static size_t fill = 0;
  if (!audioReady) return false;
  fill += audioSource->read(frame + fill, AUDIO_BLOCK_SAMPLES - fill);
  if (fill < AUDIO_BLOCK_SAMPLES) return false;
  fill = 0;
  return true;
}

// Runs one block through the active KWS backend.
KwsResult runKWS(const int16_t* frame, size_t n) {
#if KWS_USE_EDGE_IMPULSE
  return runKwsEdgeImpulse(frame, n);
#else
  return mockKws.process(frame, n);
#endif
}

// Starts one (non-blocking) Wi-Fi connection attempt.
void connectWiFi() {
  Serial.println("[WIFI] Connecting...");
  Serial.printf("[WIFI] SSID \"%s\", timeout %lu ms\n", WIFI_SSID, (unsigned long)WIFI_CONNECT_TIMEOUT_MS);
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD, WIFI_CHANNEL);
}

static const char* mqttStateName(int rc) {
  switch (rc) {
    case -4: return "CONNECTION_TIMEOUT"; case -3: return "CONNECTION_LOST";
    case -2: return "CONNECT_FAILED";     case -1: return "DISCONNECTED";
    case 0:  return "CONNECTED";          case 1:  return "BAD_PROTOCOL";
    case 2:  return "BAD_CLIENT_ID";      case 3:  return "UNAVAILABLE";
    case 4:  return "BAD_CREDENTIALS";    case 5:  return "UNAUTHORIZED";
    default: return "UNKNOWN";
  }
}

// One MQTT connection attempt. Note: PubSubClient::connect() blocks for up
// to MQTT_SOCKET_TIMEOUT_S; audio is buffered/dropped meanwhile (not listening).
bool connectMQTT() {
  Serial.println("[MQTT] Connecting...");
  Serial.printf("[MQTT] Broker %s:%d, client id %s\n", MQTT_BROKER, MQTT_PORT, mqttClientId);
  const char* user = MQTT_USERNAME[0] ? MQTT_USERNAME : nullptr;
  const char* pass = MQTT_PASSWORD[0] ? MQTT_PASSWORD : nullptr;
  // Last Will: broker publishes retained "OFFLINE" if we vanish.
  if (!mqtt.connect(mqttClientId, user, pass, TOPIC_STATUS, 1, true, "OFFLINE")) {
    const int rc = mqtt.state();
    Serial.printf("[MQTT] Connect failed, rc=%d (%s), retry in %lu ms\n", rc, mqttStateName(rc),
                  (unsigned long)MQTT_RETRY_BACKOFF_MS);
    return false;
  }
  Serial.println("[MQTT] Connected");
  const bool sub = mqtt.subscribe(TOPIC_COMMAND, 1);
  Serial.printf("[MQTT] Subscribe %s: %s\n", TOPIC_COMMAND, sub ? "ok" : "FAILED");
  mqtt.publish(TOPIC_STATUS, "ONLINE", true);
  return true;
}

// Sends the captured audio to the cloud.
//
// WOKWI BUILD: sends ONE compact JSON summary of the simulated capture
// (no raw PCM) so the public broker is not flooded.
//
// ===== WHERE REAL PCM STREAMING GOES ========================================
// Real device: in onCaptureFrame() (below), publish each 32 ms block as it is
// captured instead of only accumulating statistics:
//   topic   : sih/device01/audio
//   payload : binary, 12-byte header + PCM
//             [u32 LE request_seq][u32 LE chunk_index][u32 LE sample_count]
//             [sample_count x int16 LE PCM @16 kHz mono]
//   end     : a final JSON message {"type":"AUDIO_END","request_id":...}
// 512 samples = 1024 B PCM + header -> raise MQTT_BUFFER_BYTES to >= 1100.
// Raw 16 kHz/16-bit is 256 kbit/s; streaming while capturing (not after)
// keeps end-to-end latency low. See docs/architecture.md.
// ============================================================================
bool publishAudio() {
  const double rms = capturedSamples ? sqrt(captureSumSq / capturedSamples) : 0.0;
  JsonDocument doc;
  doc["type"]            = audioSource->isSimulated() ? "SIMULATED_AUDIO_SUMMARY" : "AUDIO_SUMMARY";
  doc["simulated"]       = audioSource->isSimulated();
  doc["device_id"]       = DEVICE_ID;
  doc["request_id"]      = requestId;
  doc["seq"]             = requestSeq;
  doc["sample_rate"]     = SAMPLE_RATE_HZ;
  doc["channels"]        = 1;
  doc["format"]          = "pcm_s16le";
  doc["samples"]         = capturedSamples;
  doc["duration_ms"]     = (uint32_t)((uint64_t)capturedSamples * 1000 / SAMPLE_RATE_HZ);
  doc["rms"]             = (int32_t)(rms + 0.5);
  doc["peak"]            = capturePeak;
  doc["kws_backend"]     = kwsBackendName();
  doc["kws_label"]       = KWS_KEYWORD_LABEL;
  doc["kws_probability"] = roundf(lastKwsProbability * 100.0f) / 100.0f;
  doc["audio_source"]    = audioSource->name();
  doc["uptime_ms"]       = millis();

  char buf[512];
  const size_t len = serializeJson(doc, buf, sizeof(buf));
  if (len == 0 || len >= sizeof(buf)) {
    Serial.println("[MQTT] ERROR: audio packet does not fit buffer");
    return false;
  }
  const bool ok = mqtt.publish(TOPIC_AUDIO, (const uint8_t*)buf, (unsigned int)len, false);
  Serial.printf("[MQTT] %s %s (%u bytes): %s\n", ok ? "Published" : "FAILED to publish",
                TOPIC_AUDIO, (unsigned)len, buf);
  return ok;
}

// Acts on a command from the cloud. Add your real actuators here.
void handleCloudCommand(const CloudCommand& c) {
  Serial.printf("[ASR] Command received: %s\n", c.command);
  Serial.printf("[ASR] Transcript: \"%s\" (request %s)\n", c.text, c.requestId);
  if (strcmp(c.command, "TURN_ON") == 0) {
    lightOn = true;
    ledLight.pulse(2, 200, true);
    Serial.printf("[CMD] Light ON (GPIO %d)\n", PIN_LED_LIGHT);
  } else if (strcmp(c.command, "TURN_OFF") == 0) {
    lightOn = false;
    ledLight.pulse(2, 200, false);
    Serial.printf("[CMD] Light OFF (GPIO %d)\n", PIN_LED_LIGHT);
  } else if (strcmp(c.command, "STATUS") == 0) {
    ledMqtt.pulse(3, 150, true);
    Serial.printf("[CMD] STATUS: uptime %lu ms, light %s, RSSI %d dBm, KWS %s, audio %s\n",
                  (unsigned long)millis(), lightOn ? "ON" : "OFF", (int)WiFi.RSSI(),
                  kwsBackendName(), audioSource->name());
  } else {
    ledKws.pulse(3, 150, false);
    Serial.printf("[CMD] Unknown command \"%s\" — ignored\n", c.command);
  }
}

// -----------------------------------------------------------------------------
// 10. EVENT HANDLERS
// -----------------------------------------------------------------------------

static void onMqttMessage(char* topic, byte* payload, unsigned int length) {
  if (strcmp(topic, TOPIC_COMMAND) != 0) return;
  JsonDocument doc;
  const DeserializationError err = deserializeJson(doc, (const char*)payload, length);
  if (err) {
    Serial.printf("[MQTT] Invalid command JSON (%s) — ignored\n", err.c_str());
    return;
  }
  const char* rid = doc["request_id"] | "";
  if (state != WAITING_FOR_ASR) {
    Serial.printf("[MQTT] Command while not waiting (state %s) — ignored\n", STATE_NAMES[state]);
    return;
  }
  if (rid[0] && strcmp(rid, requestId) != 0) {
    Serial.printf("[MQTT] Stale command for %s (expecting %s) — ignored\n", rid, requestId);
    return;
  }
  copyStr(pendingCommand.command, sizeof(pendingCommand.command), doc["command"] | "");
  copyStr(pendingCommand.text, sizeof(pendingCommand.text), doc["text"] | "");
  copyStr(pendingCommand.requestId, sizeof(pendingCommand.requestId), rid);
  commandPending = true;  // handled by the state machine, not in this callback
}

static void onCaptureFrame(const int16_t* x, size_t n) {
  for (size_t i = 0; i < n; ++i) {
    const int32_t v = x[i];
    captureSumSq += (double)v * v;
    const int32_t a = v < 0 ? -v : v;
    if (a > capturePeak) capturePeak = a;
  }
  capturedSamples += n;
  // Real device: stream this block here (see publishAudio() comment).
}

static void onEnterState(DeviceState s) {
  const uint32_t now = millis();
  switch (s) {
    case WIFI_CONNECTING:
      wifiAttemptActive = false;
      wifiRetryAt = now;
      break;
    case MQTT_CONNECTING:
      mqttRetryAt = now;
      break;
    case LISTENING:
      if (!audioReady) initMicrophone();
      kwsReset();
      Serial.printf("[KWS] Listening... (backend %s%s, label \"%s\", threshold %.2f)\n",
                    kwsBackendName(), KWS_USE_EDGE_IMPULSE ? "" : " — MOCK, not a speech model",
                    KWS_KEYWORD_LABEL, KWS_THRESHOLD);
      break;
    case KEYWORD_DETECTED:
      Serial.printf("[KWS] Keyword detected: \"%s\" p=%.2f >= %.2f [%s]\n",
                    KWS_KEYWORD_LABEL, lastKwsProbability, KWS_THRESHOLD, kwsBackendName());
      break;
    case CAPTURING:
      capturedSamples = 0; captureSumSq = 0.0; capturePeak = 0;
      captureTarget = (uint32_t)((uint64_t)SAMPLE_RATE_HZ * CAPTURE_DURATION_MS / 1000);
      Serial.printf("[CAPTURE] Capturing %lu ms of post-keyword audio (%s)\n",
                    (unsigned long)CAPTURE_DURATION_MS,
                    audioSource->isSimulated() ? "SIMULATED signal" : "microphone");
      break;
    case WAITING_FOR_ASR:
      commandPending = false;
      Serial.printf("[ASR] Waiting for response (request %s, timeout %lu ms)\n",
                    requestId, (unsigned long)ASR_TIMEOUT_MS);
      break;
    case COMMAND_RECEIVED:
      handleCloudCommand(pendingCommand);
      break;
    default:
      break;
  }
}

// Pull every ready audio block and route it according to the current state.
// The "DMA" keeps running in every state; blocks outside LISTENING/CAPTURING
// are discarded, exactly as a real always-on microphone would behave.
static void pumpAudio() {
  if (!audioReady) return;
  for (size_t guard = 0; guard <= AUDIO_MAX_BACKLOG_BLOCKS; ++guard) {
    if (!readAudioSamples(frameBuf)) break;
    if (state == LISTENING) {
      const KwsResult r = runKWS(frameBuf, AUDIO_BLOCK_SAMPLES);
      if (r.evaluated && r.probability >= KWS_THRESHOLD) {
        lastKwsProbability = r.probability;
        setState(KEYWORD_DETECTED);
        break;
      }
    } else if (state == CAPTURING) {
      onCaptureFrame(frameBuf, AUDIO_BLOCK_SAMPLES);
    }
  }
  const uint32_t ov = audioSource->overruns();
  if (ov != lastOverrunsReported) {
    if (state == LISTENING || state == CAPTURING)
      Serial.printf("[AUDIO] Buffer overrun (total %lu): audio dropped while not being read (reconnect or slow loop)\n", (unsigned long)ov);
    lastOverrunsReported = ov;
  }
}

// Returns false (and changes state) if Wi-Fi or MQTT dropped.
static bool linksOk() {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("[WIFI] Connection lost — reconnecting");
    mqtt.disconnect();
    WiFi.disconnect();
    setState(WIFI_CONNECTING);
    return false;
  }
  if (!mqtt.connected()) {
    Serial.printf("[MQTT] Connection lost (rc=%d %s) — reconnecting\n", mqtt.state(), mqttStateName(mqtt.state()));
    setState(MQTT_CONNECTING);
    return false;
  }
  return true;
}

static void serviceButton(uint32_t now) {
  static bool lastRaw = HIGH, stable = HIGH;
  static uint32_t changedAt = 0;
  const bool raw = digitalRead(PIN_BTN_INJECT);
  if (raw != lastRaw) { lastRaw = raw; changedAt = now; }
  if (now - changedAt < 30 || raw == stable) return;
  stable = raw;
  if (stable != LOW) return;  // act on press only
#if USE_INMP441
  Serial.println("[BTN] Inject button has no effect with a real microphone — speak instead");
#else
  if (!audioReady) { Serial.println("[BTN] Not listening yet (waiting for Wi-Fi/MQTT)"); return; }
  Serial.println("[BTN] Injecting keyword test pattern into the simulated audio stream");
  micSource.injectKeyword();
#endif
}

// -----------------------------------------------------------------------------
// 11. SETUP / LOOP
// -----------------------------------------------------------------------------

void setup() {
  Serial.begin(115200);
  ledWifi.begin(); ledKws.begin(); ledMqtt.begin(); ledLight.begin();
  pinMode(PIN_BTN_INJECT, INPUT_PULLUP);

  Serial.println();
  Serial.println("[BOOT]");
  Serial.println("[BOOT] SIH 2026 edge voice activator — ESP32-S3");
  Serial.printf("[BOOT] Audio source : %s%s\n", audioSource->name(),
                audioSource->isSimulated() ? " (SIMULATED — synthetic test signal, not a real mic)" : "");
  Serial.printf("[BOOT] KWS backend  : %s%s\n", kwsBackendName(),
                KWS_USE_EDGE_IMPULSE ? "" : " (MOCK — tone-pattern detector, not a speech model)");
  Serial.printf("[BOOT] Audio format : %lu Hz, mono, int16, %u-sample blocks\n",
                (unsigned long)SAMPLE_RATE_HZ, (unsigned)AUDIO_BLOCK_SAMPLES);
  Serial.printf("[BOOT] Threshold    : %.2f\n", KWS_THRESHOLD);
  Serial.printf("[BOOT] MQTT broker  : %s:%d\n", MQTT_BROKER, MQTT_PORT);
  Serial.println("[BOOT] Topics       : sih/" DEVICE_ID "/{status,event,audio,command}");

  if (MQTT_CLIENT_ID[0]) {
    copyStr(mqttClientId, sizeof(mqttClientId), MQTT_CLIENT_ID);
  } else {
    snprintf(mqttClientId, sizeof(mqttClientId), "sih-%s-%08lX", DEVICE_ID,
             (unsigned long)(ESP.getEfuseMac() & 0xFFFFFFFFULL));
  }
  mqtt.setServer(MQTT_BROKER, MQTT_PORT);
  mqtt.setCallback(onMqttMessage);
  mqtt.setBufferSize(MQTT_BUFFER_BYTES);
  mqtt.setSocketTimeout(MQTT_SOCKET_TIMEOUT_S);
  mqtt.setKeepAlive(MQTT_KEEPALIVE_S);

  setState(WIFI_CONNECTING);
}

void loop() {
  const uint32_t now = millis();
  ledWifi.update(now); ledKws.update(now); ledMqtt.update(now); ledLight.update(now);
  serviceButton(now);

  if (state >= LISTENING) {
    if (!linksOk()) return;
    mqtt.loop();            // keepalive + incoming commands
  }
  pumpAudio();

  switch (state) {
    case BOOT:
      break;

    case WIFI_CONNECTING:
      if (WiFi.status() == WL_CONNECTED) {
        Serial.printf("[WIFI] Connected (IP %s, RSSI %d dBm)\n",
                      WiFi.localIP().toString().c_str(), (int)WiFi.RSSI());
        setState(MQTT_CONNECTING);
      } else if (!wifiAttemptActive) {
        if ((int32_t)(now - wifiRetryAt) >= 0) {
          connectWiFi();
          wifiAttemptActive = true;
          wifiAttemptStart = now;
        }
      } else if (now - wifiAttemptStart >= WIFI_CONNECT_TIMEOUT_MS) {
        Serial.printf("[WIFI] Timeout, retrying in %lu ms\n", (unsigned long)WIFI_RETRY_BACKOFF_MS);
        WiFi.disconnect();
        wifiAttemptActive = false;
        wifiRetryAt = now + WIFI_RETRY_BACKOFF_MS;
      }
      break;

    case MQTT_CONNECTING:
      if (WiFi.status() != WL_CONNECTED) {
        Serial.println("[WIFI] Connection lost — reconnecting");
        WiFi.disconnect();
        setState(WIFI_CONNECTING);
      } else if ((int32_t)(now - mqttRetryAt) >= 0) {
        if (connectMQTT()) setState(LISTENING);
        else mqttRetryAt = millis() + MQTT_RETRY_BACKOFF_MS;
      }
      break;

    case LISTENING:
      break;  // work happens in pumpAudio() -> runKWS()

    case KEYWORD_DETECTED:
      requestSeq++;
      snprintf(requestId, sizeof(requestId), "%s-%04lu", DEVICE_ID, (unsigned long)requestSeq);
      if (mqtt.publish(TOPIC_EVENT, "KEYWORD_DETECTED"))
        Serial.printf("[MQTT] Published %s: KEYWORD_DETECTED\n", TOPIC_EVENT);
      else
        Serial.printf("[MQTT] FAILED to publish %s\n", TOPIC_EVENT);
      setState(CAPTURING);
      break;

    case CAPTURING:
      if (capturedSamples >= captureTarget) {
        Serial.printf("[CAPTURE] Done: %lu samples, peak %ld\n",
                      (unsigned long)capturedSamples, (long)capturePeak);
        setState(UPLOADING);
      } else if (now - stateEnteredAt > CAPTURE_DURATION_MS * 3) {
        Serial.println("[CAPTURE] Audio stalled — aborting capture");
        setState(LISTENING);
      }
      break;

    case UPLOADING:
      Serial.println("[MQTT] Sending audio/command");
      if (publishAudio()) setState(WAITING_FOR_ASR);
      else setState(LISTENING);
      break;

    case WAITING_FOR_ASR:
      if (commandPending) {
        commandPending = false;
        setState(COMMAND_RECEIVED);
      } else if (now - stateEnteredAt >= ASR_TIMEOUT_MS) {
        Serial.printf("[ASR] Timeout after %lu ms — is cloud_server/server.py running?\n",
                      (unsigned long)ASR_TIMEOUT_MS);
        setState(LISTENING);
      }
      break;

    case COMMAND_RECEIVED:
      if (now - stateEnteredAt >= COMMAND_DISPLAY_MS) setState(LISTENING);
      break;
  }
}
