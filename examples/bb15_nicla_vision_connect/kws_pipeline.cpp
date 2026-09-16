#include "kws_pipeline.h"

#include <PDM.h>
#include <math.h>
#include <string.h>

#include "akida/program_info.h"
#include "mfcc.h"

namespace kws {
namespace {

// The DFSDM half-transfer hands the PDM library 256 samples at a time, and
// py_audio_init() raises anything smaller to this, so it is both the minimum
// and the required multiple of 512 bytes.
constexpr size_t kPdmBufferBytes = 1024u;
constexpr size_t kPdmBufferSamples = kPdmBufferBytes / sizeof(int16_t);
// The library turns this into a right shift of the DFSDM output, so only every
// third step changes anything. 12 puts speech at arm's length at about the
// level the model's training corpus holds.
constexpr int kPdmGain = 12;

constexpr uint32_t kSampleRateHz = 16000u;
constexpr uint16_t kMfccHopSamples = 320u;
constexpr uint16_t kBlockSamples = 960u;
constexpr uint8_t kMfccFramesPerBlock = kBlockSamples / kMfccHopSamples;
constexpr uint16_t kSpectrogramFrames = 49u;
constexpr uint8_t kSpectrogramCoefficients = 10u;
constexpr uint8_t kInferencePeriodBlocks = 3u;

// A cleared spectrogram holds the byte that float zero quantizes to, so the
// model is fed what a silent ring would produce.
constexpr uint8_t kClearedFeature = 128u;

// First-order DC blocking high-pass coefficient.
constexpr int32_t kDcBlockAlphaQ15 = 32700;

// One min/max pair per window, so the block reduces to kWaveformValues values.
constexpr uint16_t kWaveformPairs = kWaveformValues / 2u;
constexpr uint16_t kWaveformWindowSamples = kBlockSamples / kWaveformPairs;

constexpr const char* kKeywordLabels[] = {"down", "go",  "left",    "no",
                                          "off",  "on",  "right",   "stop",
                                          "up",   "yes", "silence", "unknown"};
constexpr uint8_t kKeywordLabelCount =
    sizeof(kKeywordLabels) / sizeof(kKeywordLabels[0]);

/** @brief Voice activity state, driving whether a block reaches the model. */
enum class SpeechState : uint8_t {
  Idle,
  Active,
};

/** @brief Detector state carried across blocks. */
struct Classifier {
  float smoothed[kMaxClasses] = {};
  uint8_t chiming[kMaxClasses] = {};
  uint32_t lastTriggerMs = 0u;
  uint8_t blocksSinceInference = 0u;
};

/** @brief Running state of the DC blocking high-pass. */
struct DcBlockState {
  int32_t previousInput = 0;
  int32_t previousOutput = 0;
};

Config g_config;
ModelParameters g_model;
BB15Runner* g_runner = nullptr;
const int32_t* g_output_shifts = nullptr;
const float* g_output_scales = nullptr;

int16_t g_pdm_samples[kPdmBufferSamples];
volatile bool g_pdm_ready = false;
// [0, hop) is the previous block's last hop, [hop, hop + block) is this one.
int16_t g_mfcc_input[kMfccHopSamples + kBlockSamples];
int16_t g_waveform[kWaveformValues];
uint8_t g_spectrogram[kSpectrogramFrames * kSpectrogramCoefficients];
uint16_t g_spectrogram_index = 0u;
uint8_t g_model_input[kSpectrogramFrames * kSpectrogramCoefficients];

SpeechState g_speech_state = SpeechState::Idle;
uint32_t g_speech_started_ms = 0u;
Classifier g_classifier;
DcBlockState g_dc_block;
size_t g_block_fill = 0u;
int64_t g_block_energy = 0;
bool g_inference_running = false;
bool g_waveform_streaming = false;
WaveformHandler g_on_waveform = nullptr;
DetectionHandler g_on_detection = nullptr;

/** @brief Note that the PDM library has filled a buffer, from interrupt. */
void on_pdm_data() { g_pdm_ready = true; }

/**
 * @brief Apply the DC blocking high-pass in place.
 *
 * The filter state carries between calls, so a buffer boundary does not reset
 * it.
 *
 * @param samples  Samples to filter, overwritten with the filtered signal.
 * @param count    Number of samples to filter.
 */
void remove_dc_offset(int16_t* samples, size_t count) {
  for (size_t index = 0u; index < count; ++index) {
    int32_t output = samples[index] - g_dc_block.previousInput +
                     ((kDcBlockAlphaQ15 * g_dc_block.previousOutput) >> 15);
    g_dc_block.previousInput = samples[index];
    g_dc_block.previousOutput = output;
    if (output > 32767) {
      output = 32767;
    }
    if (output < -32768) {
      output = -32768;
    }
    samples[index] = static_cast<int16_t>(output);
  }
}

/** @brief Reduce the completed block to the min/max envelope the phone draws.
 */
void compute_waveform_envelope() {
  const int16_t* block = &g_mfcc_input[kMfccHopSamples];
  for (uint16_t pair = 0u; pair < kWaveformPairs; ++pair) {
    const int16_t* window = &block[pair * kWaveformWindowSamples];
    int16_t lowest = INT16_MAX;
    int16_t highest = INT16_MIN;
    for (uint16_t index = 0u; index < kWaveformWindowSamples; ++index) {
      if (window[index] < lowest) {
        lowest = window[index];
      }
      if (window[index] > highest) {
        highest = window[index];
      }
    }
    g_waveform[2u * pair] = lowest;
    g_waveform[2u * pair + 1u] = highest;
  }
}

/**
 * @brief Quantize one MFCC coefficient to the model's uint8 input scale.
 *
 * @param coefficient  Coefficient as the front end produced it.
 * @return The byte the model expects.
 */
uint8_t quantize_feature(float coefficient) {
  float scaled = ((coefficient / g_model.mfccFullScale) + 1.0f) * 128.0f;
  if (scaled < 0.0f) {
    scaled = 0.0f;
  } else if (scaled > 255.0f) {
    scaled = 255.0f;
  }
  return static_cast<uint8_t>(scaled);
}

/** @brief Throw away the part-built model input. */
void clear_spectrogram() {
  memset(g_spectrogram, kClearedFeature, sizeof(g_spectrogram));
  g_spectrogram_index = 0u;
}

/** @brief Turn the filled block into MFCC frames and push them to the ring. */
void extract_features() {
  float coefficients[kMfccFeatures];
  for (uint8_t frame = 0u; frame < kMfccFramesPerBlock; ++frame) {
    mfcc_compute(&g_mfcc_input[frame * kMfccHopSamples], coefficients);
    uint8_t* pushed =
        &g_spectrogram[g_spectrogram_index * kSpectrogramCoefficients];
    for (uint8_t index = 0u; index < kSpectrogramCoefficients; ++index) {
      pushed[index] = quantize_feature(coefficients[index]);
    }
    if (++g_spectrogram_index >= kSpectrogramFrames) {
      g_spectrogram_index = 0u;
    }
  }
}

/**
 * @brief Dequantize the raw Akida potentials, as the engine does.
 *
 * @param potentials  One raw potential per class.
 * @param out         Receives the dequantized values.
 */
void dequantize_potentials(const int32_t* potentials, float* out) {
  for (uint8_t index = 0u; index < g_model.classCount; ++index) {
    out[index] =
        static_cast<float>(potentials[index] - g_output_shifts[index]) /
        g_output_scales[index];
  }
}

/**
 * @brief Numerically stable softmax in place.
 *
 * @param values  One value per class, replaced by the normalized result.
 */
void softmax_in_place(float* values) {
  float highest = values[0];
  for (uint8_t index = 1u; index < g_model.classCount; ++index) {
    if (values[index] > highest) {
      highest = values[index];
    }
  }
  float total = 0.0f;
  for (uint8_t index = 0u; index < g_model.classCount; ++index) {
    values[index] = expf(values[index] - highest);
    total += values[index];
  }
  for (uint8_t index = 0u; index < g_model.classCount; ++index) {
    values[index] /= total;
  }
}

/** @brief Say whether the post-detection cooldown has expired. */
bool debounce_complete() {
  return g_classifier.lastTriggerMs == 0u ||
         (millis() - g_classifier.lastTriggerMs) > g_config.debounceMs;
}

/** @brief Clear the scores, counters and part-built model input. */
void reset_inference_state() {
  memset(g_classifier.smoothed, 0, sizeof(g_classifier.smoothed));
  memset(g_classifier.chiming, 0, sizeof(g_classifier.chiming));
  clear_spectrogram();
}

/**
 * @brief Copy the model input out of the ring, oldest frame first.
 *
 * The write position is also the oldest frame, so the unroll starts there.
 */
void build_model_input() {
  for (uint16_t frame = 0u; frame < kSpectrogramFrames; ++frame) {
    const uint16_t source = static_cast<uint16_t>(
        (frame + g_spectrogram_index) % kSpectrogramFrames);
    memcpy(&g_model_input[frame * kSpectrogramCoefficients],
           &g_spectrogram[source * kSpectrogramCoefficients],
           kSpectrogramCoefficients);
  }
}

/**
 * @brief Fire a detection once a class has held above threshold long enough.
 *
 * Silence and unknown are scored like any other class but can never trigger.
 */
void apply_decision() {
  uint8_t triggered = kNoPrediction;
  float best = 0.0f;
  for (uint8_t index = 0u; index < g_model.classCount; ++index) {
    if (index == g_model.silenceClass || index == g_model.unknownClass) {
      continue;
    }
    if (g_classifier.smoothed[index] >= g_config.scoreThreshold) {
      ++g_classifier.chiming[index];
    } else {
      g_classifier.chiming[index] = 0u;
    }
    if (g_classifier.chiming[index] >= g_config.chimingThreshold &&
        (triggered == kNoPrediction || g_classifier.smoothed[index] > best)) {
      triggered = index;
      best = g_classifier.smoothed[index];
    }
  }
  if (triggered == kNoPrediction) {
    return;
  }

  g_classifier.lastTriggerMs = millis();
  if (g_on_detection != nullptr) {
    Detection detection;
    detection.classIndex = triggered;
    detection.confidence = best;
    g_on_detection(detection);
  }
  reset_inference_state();
}

/** @brief Run one inference on the current model input and fold in the result.
 */
void run_inference() {
  build_model_input();
  BB15Input input;
  input.data = g_model_input;
  input.type = akida::TensorType::uint8;
  input.dimensions = {1u, kSpectrogramFrames, kSpectrogramCoefficients, 1u};

  const BB15RunResult result = g_runner->infer(input);
  if (!result.ok() || result.type != akida::TensorType::int32 ||
      result.elementCount() < g_model.classCount) {
    return;
  }

  float scores[kMaxClasses];
  dequantize_potentials(result.data<int32_t>(), scores);
  softmax_in_place(scores);
  for (uint8_t index = 0u; index < g_model.classCount; ++index) {
    g_classifier.smoothed[index] =
        g_config.smoothingAlpha * scores[index] +
        (1.0f - g_config.smoothingAlpha) * g_classifier.smoothed[index];
  }
  apply_decision();
}

/**
 * @brief Decide whether this block's audio should reach the front end.
 *
 * A loud block restarts the utterance timer, and a quiet one is still
 * processed for speechTimeoutMs after the last loud block, which is what
 * captures the tail of a word.
 *
 * @param rms  This block's RMS, taken after the DC blocker.
 * @return True when the block should become features.
 */
bool gate_block(uint16_t rms) {
  if (!debounce_complete()) {
    // Idle, not merely ungated: the cooldown must not leave a stale utterance
    // timer that the tail of the same word could walk back into.
    g_speech_state = SpeechState::Idle;
    return false;
  }
  if (rms >= g_config.rmsThreshold) {
    g_speech_state = SpeechState::Active;
    g_speech_started_ms = millis();
    return true;
  }
  if (g_speech_state == SpeechState::Idle) {
    return false;
  }
  if (millis() - g_speech_started_ms > g_config.speechTimeoutMs) {
    g_speech_state = SpeechState::Idle;
    reset_inference_state();
    return false;
  }
  return true;
}

/** @brief Score the filled block, report it, and start collecting the next. */
void publish_filled_block() {
  const uint16_t rms = static_cast<uint16_t>(
      sqrtf(static_cast<float>(g_block_energy) / kBlockSamples));

  if (g_inference_running && modelReady() && gate_block(rms)) {
    extract_features();
    if (++g_classifier.blocksSinceInference >= kInferencePeriodBlocks) {
      g_classifier.blocksSinceInference = 0u;
      run_inference();
    }
  }

  // Advances whether or not the block became features, or the next window
  // would splice this block onto one already a block stale.
  memcpy(&g_mfcc_input[0], &g_mfcc_input[kBlockSamples],
         kMfccHopSamples * sizeof(int16_t));

  if (g_waveform_streaming && g_on_waveform != nullptr) {
    compute_waveform_envelope();
    g_on_waveform(g_waveform);
  }

  g_block_fill = 0u;
  g_block_energy = 0;
}

/**
 * @brief Append filtered samples to the pending block, publishing when full.
 *
 * @param samples  Filtered samples to append.
 * @param count    Number of samples available.
 */
void accumulate_samples(const int16_t* samples, size_t count) {
  size_t consumed = 0u;
  while (consumed < count) {
    const size_t room = kBlockSamples - g_block_fill;
    const size_t taking = (count - consumed) < room ? (count - consumed) : room;
    for (size_t index = 0u; index < taking; ++index) {
      const int16_t sample = samples[consumed + index];
      g_mfcc_input[kMfccHopSamples + g_block_fill + index] = sample;
      g_block_energy += static_cast<int32_t>(sample) * sample;
    }
    g_block_fill += taking;
    consumed += taking;
    if (g_block_fill == kBlockSamples) {
      publish_filled_block();
    }
  }
}

}  // namespace

bool begin() {
  clear_spectrogram();
  if (!mfcc_begin()) {
    return false;
  }
  PDM.onReceive(on_pdm_data);
  PDM.setBufferSize(kPdmBufferBytes);
  PDM.setGain(kPdmGain);
  return PDM.begin(1, kSampleRateHz) == 1;
}

bool setModel(BB15Runner* runner, const ModelParameters& parameters) {
  g_runner = runner;
  g_output_shifts = nullptr;
  g_output_scales = nullptr;
  reset();

  if (runner == nullptr) {
    g_model = ModelParameters();
    return false;
  }

  g_model = parameters;
  if (g_model.program == nullptr || g_model.programBytes < 8u ||
      g_model.classCount == 0u || g_model.classCount > kMaxClasses ||
      g_model.mfccFullScale == 0.0f) {
    g_runner = nullptr;
    return false;
  }

  const akida::ProgramInfo info(g_model.program, g_model.programBytes);
  if (!info.is_valid() || info.shifts().size < g_model.classCount ||
      info.scales().size < g_model.classCount) {
    g_runner = nullptr;
    return false;
  }
  g_output_shifts = info.shifts().data;
  g_output_scales = info.scales().data;
  return true;
}

bool modelReady() {
  return g_runner != nullptr && g_output_shifts != nullptr &&
         g_output_scales != nullptr;
}

const Config& config() { return g_config; }

void setConfig(const Config& updated) {
  g_config = updated;
  reset();
}

void resetConfig() { setConfig(Config()); }

void setInferenceRunning(bool running) {
  if (g_inference_running == running) {
    return;
  }
  g_inference_running = running;
  reset();
}

bool inferenceRunning() { return g_inference_running; }

void setWaveformStreaming(bool streaming) { g_waveform_streaming = streaming; }

void setHandlers(WaveformHandler onWaveform, DetectionHandler onDetection) {
  g_on_waveform = onWaveform;
  g_on_detection = onDetection;
}

bool poll() {
  if (!g_pdm_ready) {
    return false;
  }
  const int read_bytes = PDM.read(g_pdm_samples, sizeof(g_pdm_samples));
  g_pdm_ready = false;

  const size_t count = static_cast<size_t>(read_bytes) / sizeof(int16_t);
  remove_dc_offset(g_pdm_samples, count);
  accumulate_samples(g_pdm_samples, count);
  return true;
}

const char* classLabel(uint8_t classIndex) {
  if (classIndex < kKeywordLabelCount) {
    return kKeywordLabels[classIndex];
  }
  static char numbered[12];
  snprintf(numbered, sizeof(numbered), "class %u",
           static_cast<unsigned>(classIndex));
  return numbered;
}

uint8_t classCount() { return g_model.classCount; }

void reset() {
  g_dc_block = DcBlockState();
  g_classifier = Classifier();
  g_speech_state = SpeechState::Idle;
  g_speech_started_ms = 0u;
  g_block_fill = 0u;
  g_block_energy = 0;
  g_pdm_ready = false;
  clear_spectrogram();
}

}  // namespace kws
