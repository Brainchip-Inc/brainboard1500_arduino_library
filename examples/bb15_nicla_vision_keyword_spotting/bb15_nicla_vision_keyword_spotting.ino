#include <Arduino.h>
#include <BB15.h>
#include <PDM.h>

#include "akida/program_info.h"
#include "mfcc.h"
#include "model_metadata.h"
#include "program.h"

#ifndef ARDUINO_NICLA_VISION
#error "bb15_nicla_vision_keyword_spotting requires Arduino Nicla Vision."
#endif

namespace {

constexpr uint32_t kSerialBaud = 921600u;
constexpr uint32_t kSerialWaitMs = 3000u;
constexpr uint32_t kBootSettleMs = 250u;

// So a listener can tell an idle board from a wedged one.
constexpr uint32_t kIdleReportMs = 2000u;

// The DFSDM half-transfer hands the PDM library 256 samples at a time, and
// py_audio_init() raises anything smaller to this, so it is both the minimum
// and the required multiple of 512 bytes.
constexpr size_t kPdmBufferBytes = 1024u;
constexpr size_t kPdmBufferSamples = kPdmBufferBytes / sizeof(int16_t);
// The library turns this into a right shift of the DFSDM output,
// attenuation = 8 - gain / 3 clamped at 0, over a default attenuation of 5, so
// only every third step changes anything and 24 or above is the loudest it
// goes. 12 gives an attenuation of 4, chosen so speech at arm's length reaches
// the feature front end at about the level the model's training corpus holds.
constexpr int kPdmGain = 12;

// spark's values: rates from its source/Kconfig, the rest from
// source/core/common/kws_config.c.
constexpr uint32_t kSampleRateHz = 16000u;
constexpr uint16_t kMfccHopSamples = 320u;
constexpr uint16_t kBlockSamples = 960u;
constexpr uint8_t kMfccFramesPerBlock = kBlockSamples / kMfccHopSamples;
constexpr uint16_t kSpectrogramFrames = 49u;
constexpr uint8_t kSpectrogramCoefficients = 10u;
constexpr uint8_t kClassCount = 12u;
constexpr uint8_t kInferencePeriodBlocks = 3u;
// Reported like any other class, but neither can trigger a detection.
constexpr uint8_t kSilenceClass = 10u;
constexpr uint8_t kUnknownClass = 11u;
// Scaled with kPdmGain instead of taken from spark: the gain is a right shift
// on every sample, so speech and the room floor move together and the gate has
// to move by the same factor. The two constants only mean anything together.
constexpr uint16_t kRmsThreshold = 2200u;
constexpr uint16_t kSpeechActiveTimeMs = 1300u;
constexpr uint16_t kSmoothingAlphaQ15 = 22938u;
constexpr uint16_t kScoreThresholdQ15 = 16384u;
constexpr float kSmoothingAlpha = 0.70f;
constexpr float kScoreThreshold = 0.50f;
constexpr uint16_t kDebounceMs = 300u;
constexpr uint8_t kChimingThreshold = 3u;

// Divides an MFCC coefficient down to the model's uint8 input, as spark's
// do_inference() does. The value is the model's own, from its info.yaml, and is
// not a tuning knob.
constexpr float kMfccFullScale = 123.56967163085938f;

// spark clears a float spectrogram, and float zero quantizes to 128, so a
// cleared ring holds 128 to feed the model the bytes spark would.
constexpr uint8_t kClearedFeature = 128u;

constexpr uint32_t kAkidaSpiClockHz = 25000000u;

// spark's DC blocker coefficient, from its dc_block_process().
constexpr int32_t kDcBlockAlphaQ15 = 32700;

// One min/max pair per 10 samples of the block.
constexpr uint16_t kWaveformPoints = 96u;
constexpr uint16_t kWaveformWindowSamples = kBlockSamples / kWaveformPoints;

// USB protocol v1: the Nicla Vision human-detection demo's header and commands,
// with message types of its own for audio.
constexpr uint8_t kProtocolMagic[] = {'B', 'B', '1', '5'};
constexpr uint8_t kProtocolVersion = 1u;
enum class PacketType : uint8_t {
  StartStream = 1u,
  StopStream = 2u,
  RequestConfig = 3u,
  Error = 0x83u,
  AudioConfig = 0x84u,
  AudioResult = 0x85u,
};
constexpr size_t kPacketHeaderBytes = 10u;
constexpr size_t kAudioConfigBytes = 24u;
constexpr size_t kAudioResultMetadataBytes = 28u;

// An audio-result packet is the largest this demo sends, and it is built whole
// before being handed over: on native USB CDC every Serial.write() is its own
// blocking transfer, so writing a packet field by field costs hundreds of them.
constexpr size_t kMaxPacketBytes =
    kPacketHeaderBytes + kAudioResultMetadataBytes +
    2u * sizeof(int16_t) * kWaveformPoints +
    kMfccFramesPerBlock * kSpectrogramCoefficients +
    sizeof(int16_t) * kClassCount;

// Tells the shared desktop tool which board it is drawing.
constexpr uint8_t kBoardIdNiclaVision = 2u;

// Sent in the predicted-index field when nothing has been detected.
constexpr uint8_t kNoPrediction = 0xFFu;
constexpr uint8_t kStatusOk = 0u;

constexpr uint8_t kStatusMfccInitFailed = 0x81u;
constexpr uint8_t kStatusMicrophoneFailed = 0x82u;
constexpr uint8_t kStatusAkidaFailed = 0x83u;

// spark's kws_new_tags[], in the order the model's info.yaml gives.
constexpr const char* kClassLabels[kClassCount] = {
    "down",  "go",   "left", "no",  "off",     "on",
    "right", "stop", "up",   "yes", "silence", "unknown"};

constexpr const char* kSketchName = "bb15_nicla_vision_keyword_spotting";
constexpr const char* kLogPrefix = "[bb15_nicla_vision_keyword_spotting]";

/** @brief One completed 60 ms audio block, ready to stream to the host. */
struct AudioBlock {
  uint32_t sequence = 0u;
  uint32_t deviceMs = 0u;
  uint16_t rms = 0u;
  uint16_t peak = 0u;
  uint16_t captureMs = 0u;
  uint16_t featureMs = 0u;
  bool speechActive = false;
};

/** @brief spark's voice activity state, its SPEECH_IDLE and SPEECH_ACTIVE. */
enum class SpeechState : uint8_t {
  Idle,
  Active,
};

/** @brief What the RGB LED is saying about the demo's state. */
enum class StatusLed : uint8_t {
  Off,
  Red,
  Green,
  Blue,
};

/** @brief The Akida runtime state this demo keeps across blocks. */
struct Classifier {
  float smoothed[kClassCount] = {};
  uint8_t chiming[kClassCount] = {};
  uint32_t lastTriggerMs = 0u;
  uint8_t blocksSinceInference = 0u;
  uint8_t predicted = kNoPrediction;
  // Only ever increments, and wraps, so the desktop tool can tell a fresh
  // detection from the same keyword still on screen.
  uint8_t detections = 0u;
  uint16_t inferMs = 0u;
};

/** @brief Running state of spark's first-order DC blocking high-pass. */
struct DcBlockState {
  int32_t previousInput = 0;
  int32_t previousOutput = 0;
};

uint8_t g_packet[kMaxPacketBytes];
size_t g_packet_fill = 0u;
int16_t g_pdm_samples[kPdmBufferSamples];
// Written by the PDM interrupt, read by loop().
volatile bool g_pdm_ready = false;
volatile uint16_t g_pdm_overruns = 0u;
// spark's mfcc_process_input() layout: [0, hop) is the previous block's last
// hop, [hop, hop + block) is this one.
int16_t g_mfcc_input[kMfccHopSamples + kBlockSamples];
int16_t g_waveform[2u * kWaveformPoints];
// Held quantized, the only form the model uses. g_spectrogram_index is the
// write position, and so also the oldest frame the unroll starts from.
uint8_t g_spectrogram[kSpectrogramFrames * kSpectrogramCoefficients];
uint16_t g_spectrogram_index = 0u;
uint8_t g_new_features[kMfccFramesPerBlock * kSpectrogramCoefficients];
uint8_t g_new_frame_count = 0u;
SpeechState g_speech_state = SpeechState::Idle;
uint32_t g_speech_started_ms = 0u;
// The model input, unrolled oldest frame first the way spark builds it.
uint8_t g_model_input[kSpectrogramFrames * kSpectrogramCoefficients];
Classifier g_classifier;
BB15Pinout g_pinout = BB15Pinout::niclaVisionDefaults();
BB15Config g_config = []() {
  BB15Config config = BB15Config::defaults();
  config.spiClockHz = kAkidaSpiClockHz;
  return config;
}();
BB15* g_bb15 = nullptr;
BB15Runner* g_runner = nullptr;
BB15Model g_model(program, static_cast<size_t>(program_len));
// Read from the program itself rather than hard-coded.
const int32_t* g_output_shifts = nullptr;
const float* g_output_scales = nullptr;
DcBlockState g_dc_block;
size_t g_block_fill = 0u;
int64_t g_block_energy = 0;
uint16_t g_block_peak = 0u;
uint32_t g_block_capture_ms = 0u;
uint32_t g_sequence = 0u;
bool g_streaming = false;
// Set when setup could not finish, so host commands are answered with the
// reason.
uint8_t g_setup_failure = kStatusOk;
uint32_t g_next_idle_report_ms = 0u;

/**
 * @brief Clamp a millisecond duration into the packet's 16-bit field.
 *
 * @param value  Duration in milliseconds.
 * @return The value, saturated at 65535.
 */
uint16_t clamp_u16(uint32_t value) {
  return value > 0xFFFFu ? 0xFFFFu : static_cast<uint16_t>(value);
}

/**
 * @brief Give the host a moment to open the native USB CDC port.
 */
void wait_for_serial() {
  const uint32_t start_ms = millis();
  while (!Serial && (millis() - start_ms) < kSerialWaitMs) {
  }
}

/**
 * @brief Show the demo's state on the RGB LED, whose pins are active low.
 *
 * @param color  Colour to display, or `Off` to clear it.
 */
void set_led(StatusLed color) {
  digitalWrite(LEDR, color == StatusLed::Red ? LOW : HIGH);
  digitalWrite(LEDG, color == StatusLed::Green ? LOW : HIGH);
  digitalWrite(LEDB, color == StatusLed::Blue ? LOW : HIGH);
}

/**
 * @brief Record a setup failure and report it on the serial port.
 *
 * @param stage   Name of the setup step that failed.
 * @param status  Status the error packet should carry.
 */
void fail_setup(const char* stage, uint8_t status) {
  g_setup_failure = status;
  Serial.print(kLogPrefix);
  Serial.print(" result=FAIL stage=");
  Serial.println(stage);
}

/**
 * @brief Say what the board is doing, at most once per kIdleReportMs.
 *
 * Text rather than a packet, so a plain terminal shows it and the desktop
 * tool's framing skips it the way it skips the boot banner.
 */
void report_idle_state() {
  if (static_cast<int32_t>(millis() - g_next_idle_report_ms) < 0) {
    return;
  }
  g_next_idle_report_ms = millis() + kIdleReportMs;
  Serial.print(kLogPrefix);
  if (g_setup_failure != kStatusOk) {
    Serial.print(" idle result=FAIL status=0x");
    Serial.println(g_setup_failure, HEX);
    return;
  }
  Serial.println(" idle ready=1 waiting_for_start_stream");
}

/**
 * @brief Append one byte to the packet being built.
 *
 * @param value  Byte to append.
 */
void append_u8(uint8_t value) { g_packet[g_packet_fill++] = value; }

/**
 * @brief Append a little-endian unsigned 16-bit value to the packet.
 *
 * @param value  Value to append.
 */
void append_u16(uint16_t value) {
  append_u8(static_cast<uint8_t>(value & 0xFFu));
  append_u8(static_cast<uint8_t>((value >> 8) & 0xFFu));
}

/**
 * @brief Append a little-endian unsigned 32-bit value to the packet.
 *
 * @param value  Value to append.
 */
void append_u32(uint32_t value) {
  for (uint8_t shift = 0u; shift < 32u; shift += 8u) {
    append_u8(static_cast<uint8_t>((value >> shift) & 0xFFu));
  }
}

/**
 * @brief Append a little-endian signed 16-bit value to the packet.
 *
 * @param value  Value to append.
 */
void append_i16(int16_t value) { append_u16(static_cast<uint16_t>(value)); }

/**
 * @brief Append a run of bytes to the packet being built.
 *
 * @param values  Bytes to append.
 * @param count   Number of bytes to append.
 */
void append_bytes(const uint8_t* values, size_t count) {
  memcpy(&g_packet[g_packet_fill], values, count);
  g_packet_fill += count;
}

/**
 * @brief Start a packet with the fixed ten-byte protocol header.
 *
 * @param type           Message type that follows.
 * @param payload_bytes  Size of the payload after the header.
 */
void begin_packet(PacketType type, uint32_t payload_bytes) {
  g_packet_fill = 0u;
  append_bytes(kProtocolMagic, sizeof(kProtocolMagic));
  append_u8(kProtocolVersion);
  append_u8(static_cast<uint8_t>(type));
  append_u32(payload_bytes);
}

/**
 * @brief Hand the built packet to the host in a single write.
 */
void send_packet() { Serial.write(g_packet, g_packet_fill); }

/**
 * @brief Send the pipeline description the desktop tool needs to draw itself.
 */
void send_audio_config_packet() {
  begin_packet(PacketType::AudioConfig,
               static_cast<uint32_t>(kAudioConfigBytes));
  append_u32(kSampleRateHz);
  append_u16(kBlockSamples);
  append_u16(kWaveformPoints);
  append_u16(kSpectrogramFrames);
  append_u8(kSpectrogramCoefficients);
  append_u8(kClassCount);
  append_u16(kRmsThreshold);
  append_u16(kSpeechActiveTimeMs);
  append_u16(kSmoothingAlphaQ15);
  append_u16(kScoreThresholdQ15);
  append_u16(kDebounceMs);
  append_u8(kChimingThreshold);
  append_u8(kBoardIdNiclaVision);
  send_packet();
}

/**
 * @brief Report a device-side failure to the desktop tool.
 *
 * @param status  Status code for the failed operation.
 */
void send_error_packet(uint8_t status) {
  begin_packet(PacketType::Error, 1u);
  append_u8(status);
  send_packet();
}

/**
 * @brief Send one completed audio block and its waveform envelope.
 *
 * @param block  Statistics for the block held in `g_waveform`.
 */
void send_audio_result_packet(const AudioBlock& block) {
  const uint32_t waveform_bytes =
      static_cast<uint32_t>(sizeof(int16_t)) * 2u * kWaveformPoints;
  const uint32_t feature_bytes =
      static_cast<uint32_t>(g_new_frame_count) * kSpectrogramCoefficients;
  const uint32_t score_bytes =
      static_cast<uint32_t>(sizeof(int16_t)) * kClassCount;
  begin_packet(PacketType::AudioResult,
               static_cast<uint32_t>(kAudioResultMetadataBytes) +
                   waveform_bytes + feature_bytes + score_bytes);
  append_u32(block.sequence);
  append_u32(block.deviceMs);
  append_u16(block.rms);
  append_u16(block.peak);
  append_u16(g_pdm_overruns);
  append_u16(block.captureMs);
  append_u16(block.featureMs);
  append_u16(g_classifier.inferMs);
  append_u8(static_cast<uint8_t>(block.speechActive ? 1u : 0u));
  append_u8(kStatusOk);
  append_u8(g_classifier.predicted);
  append_u8(kClassCount);
  append_u8(static_cast<uint8_t>(kWaveformPoints));
  append_u8(g_new_frame_count);
  append_u8(g_classifier.predicted == kNoPrediction
                ? 0u
                : g_classifier.chiming[g_classifier.predicted]);
  append_u8(g_classifier.detections);
  for (uint16_t point = 0u; point < 2u * kWaveformPoints; ++point) {
    append_i16(g_waveform[point]);
  }
  append_bytes(g_new_features, static_cast<size_t>(g_new_frame_count) *
                                   kSpectrogramCoefficients);
  for (uint8_t index = 0u; index < kClassCount; ++index) {
    append_i16(static_cast<int16_t>(g_classifier.smoothed[index] * 32767.0f));
  }
  send_packet();
}

/**
 * @brief Read at most one host command from the serial input.
 *
 * Parsing byte by byte lets the boot banner pass through harmlessly when a tool
 * opens the port while it is still buffered.
 *
 * @return The command received, or type 0 when no complete command is pending.
 */
PacketType poll_host_command() {
  static uint8_t header[kPacketHeaderBytes];
  static size_t header_size = 0u;
  while (Serial.available() > 0) {
    const uint8_t value = static_cast<uint8_t>(Serial.read());
    if (header_size < sizeof(kProtocolMagic)) {
      if (value == kProtocolMagic[header_size]) {
        header[header_size++] = value;
      } else {
        header_size = value == kProtocolMagic[0] ? 1u : 0u;
        if (header_size == 1u) {
          header[0] = value;
        }
      }
      continue;
    }
    header[header_size++] = value;
    if (header_size != kPacketHeaderBytes) {
      continue;
    }
    header_size = 0u;
    if (header[4] != kProtocolVersion) {
      continue;
    }
    const uint32_t payload = static_cast<uint32_t>(header[6]) |
                             (static_cast<uint32_t>(header[7]) << 8) |
                             (static_cast<uint32_t>(header[8]) << 16) |
                             (static_cast<uint32_t>(header[9]) << 24);
    if (payload != 0u) {
      continue;
    }
    return static_cast<PacketType>(header[5]);
  }
  return static_cast<PacketType>(0u);
}

/**
 * @brief Apply spark's DC blocking high-pass to captured samples in place.
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

/**
 * @brief Reduce the completed block to a min/max envelope for the live view.
 */
void compute_waveform_envelope() {
  const int16_t* block = &g_mfcc_input[kMfccHopSamples];
  for (uint16_t point = 0u; point < kWaveformPoints; ++point) {
    const int16_t* window = &block[point * kWaveformWindowSamples];
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
    g_waveform[2u * point] = lowest;
    g_waveform[2u * point + 1u] = highest;
  }
}

/**
 * @brief Quantize one MFCC coefficient to the model's uint8 input scale.
 *
 * @param coefficient  MFCC coefficient as the front end produced it.
 * @return The byte the model expects.
 */
uint8_t quantize_feature(float coefficient) {
  float scaled = ((coefficient / kMfccFullScale) + 1.0f) * 128.0f;
  if (scaled < 0.0f) {
    scaled = 0.0f;
  } else if (scaled > 255.0f) {
    scaled = 255.0f;
  }
  return static_cast<uint8_t>(scaled);
}

/**
 * @brief Throw away the part-built model input, as spark does when it gives up
 *        on an utterance.
 */
void clear_spectrogram() {
  memset(g_spectrogram, kClearedFeature, sizeof(g_spectrogram));
  g_spectrogram_index = 0u;
}

/**
 * @brief Turn the filled block into MFCC frames and push them to the model
 *        input.
 *
 * @return Milliseconds spent on the feature computation.
 */
uint16_t extract_features() {
  const uint32_t started_ms = millis();
  float coefficients[kMfccFeatures];

  for (uint8_t frame = 0u; frame < kMfccFramesPerBlock; ++frame) {
    mfcc_compute(&g_mfcc_input[frame * kMfccHopSamples], coefficients);
    uint8_t* pushed =
        &g_spectrogram[g_spectrogram_index * kSpectrogramCoefficients];
    for (uint8_t index = 0u; index < kSpectrogramCoefficients; ++index) {
      const uint8_t feature = quantize_feature(coefficients[index]);
      pushed[index] = feature;
      g_new_features[g_new_frame_count * kSpectrogramCoefficients + index] =
          feature;
    }
    ++g_new_frame_count;
    if (++g_spectrogram_index >= kSpectrogramFrames) {
      g_spectrogram_index = 0u;
    }
  }
  return clamp_u16(millis() - started_ms);
}

/**
 * @brief Turn the raw Akida potentials into the dequantized values spark
 *        softmaxes.
 *
 * The engine's own formula, from HardwareDeviceImpl::dequantize().
 *
 * @param potentials  kClassCount raw potentials from the runner.
 * @param out         Receives the dequantized values.
 */
void dequantize_potentials(const int32_t* potentials, float* out) {
  for (uint8_t index = 0u; index < kClassCount; ++index) {
    out[index] =
        static_cast<float>(potentials[index] - g_output_shifts[index]) /
        g_output_scales[index];
  }
}

/**
 * @brief Numerically stable softmax in place, spark's softmax() from
 *        source/core/common/inference/infer_utils.c.
 *
 * @param values  kClassCount values, replaced by the normalized result.
 */
void softmax_in_place(float* values) {
  float highest = values[0];
  for (uint8_t index = 1u; index < kClassCount; ++index) {
    if (values[index] > highest) {
      highest = values[index];
    }
  }
  float total = 0.0f;
  for (uint8_t index = 0u; index < kClassCount; ++index) {
    values[index] = expf(values[index] - highest);
    total += values[index];
  }
  for (uint8_t index = 0u; index < kClassCount; ++index) {
    values[index] /= total;
  }
}

/**
 * @brief Say whether spark's post-detection cooldown has expired.
 *
 * From is_kws_debounce_complete() in spark's main.cpp. spark skips its whole
 * front end while the cooldown runs, so this gates a block before its RMS.
 *
 * @return True when audio may be processed again.
 */
bool debounce_complete() {
  return g_classifier.lastTriggerMs == 0u ||
         (millis() - g_classifier.lastTriggerMs) > kDebounceMs;
}

/**
 * @brief Clear the scores, counters and part-built model input.
 *
 * spark's reset_stale_inference_data(), called both when an utterance times
 * out and immediately after a detection.
 */
void reset_inference_state() {
  memset(g_classifier.smoothed, 0, sizeof(g_classifier.smoothed));
  memset(g_classifier.chiming, 0, sizeof(g_classifier.chiming));
  clear_spectrogram();
}

/**
 * @brief Copy the model input out of the ring, oldest frame first.
 *
 * spark unrolls its circular spectrogram as idx = (i + spectrogram_index) % 49
 * when it builds the tensor, and the write position is the oldest frame.
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
 * @brief Apply spark's decision logic to one set of smoothed scores.
 *
 * kws_post_processing() in spark's main.cpp. A class fires once it has held
 * above the score threshold for kChimingThreshold consecutive inferences.
 */
void apply_decision() {
  uint8_t triggered = kNoPrediction;
  float best = 0.0f;
  for (uint8_t index = 0u; index < kClassCount; ++index) {
    if (index == kSilenceClass || index == kUnknownClass) {
      continue;
    }
    if (g_classifier.smoothed[index] >= kScoreThreshold) {
      ++g_classifier.chiming[index];
    } else {
      g_classifier.chiming[index] = 0u;
    }
    if (g_classifier.chiming[index] >= kChimingThreshold &&
        (triggered == kNoPrediction || g_classifier.smoothed[index] > best)) {
      triggered = index;
      best = g_classifier.smoothed[index];
    }
  }
  if (triggered == kNoPrediction) {
    return;
  }
  g_classifier.predicted = triggered;
  ++g_classifier.detections;
  g_classifier.lastTriggerMs = millis();
  Serial.print(kLogPrefix);
  Serial.print(" keyword=");
  Serial.println(kClassLabels[triggered]);
  reset_inference_state();
}

/**
 * @brief Run one inference on the current model input and fold in the result.
 *
 * @return True when the runner produced usable scores.
 */
bool run_inference() {
  build_model_input();
  BB15Input input;
  input.data = g_model_input;
  input.type = akida::TensorType::uint8;
  input.dimensions = {1u, kSpectrogramFrames, kSpectrogramCoefficients, 1u};

  const uint32_t started_ms = millis();
  const BB15RunResult result = g_runner->infer(input);
  g_classifier.inferMs = clamp_u16(millis() - started_ms);
  if (!result.ok() || result.type != akida::TensorType::int32 ||
      result.elementCount() < kClassCount) {
    return false;
  }

  float scores[kClassCount];
  dequantize_potentials(result.data<int32_t>(), scores);
  softmax_in_place(scores);
  for (uint8_t index = 0u; index < kClassCount; ++index) {
    g_classifier.smoothed[index] =
        kSmoothingAlpha * scores[index] +
        (1.0f - kSmoothingAlpha) * g_classifier.smoothed[index];
  }
  apply_decision();
  return true;
}

/**
 * @brief Decide whether this block's audio should reach the MFCC front end.
 *
 * spark's rule, from its audio_process_thread(). A loud block restarts the
 * timer, and a quiet one is still processed for kSpeechActiveTimeMs after the
 * last loud block, which is what captures the tail of a word.
 *
 * @param rms  This block's RMS, taken after the DC blocker as spark does.
 * @return True when the block should be turned into features.
 */
bool gate_block(uint16_t rms) {
  if (!debounce_complete()) {
    // Idle, not merely ungated: the cooldown must not leave a stale utterance
    // timer that the tail of the same word could walk straight back into.
    g_speech_state = SpeechState::Idle;
    return false;
  }
  if (rms >= kRmsThreshold) {
    g_speech_state = SpeechState::Active;
    g_speech_started_ms = millis();
    return true;
  }
  if (g_speech_state == SpeechState::Idle) {
    return false;
  }
  if (millis() - g_speech_started_ms > kSpeechActiveTimeMs) {
    g_speech_state = SpeechState::Idle;
    reset_inference_state();
    return false;
  }
  return true;
}

/**
 * @brief Stream the filled block and start collecting the next one.
 */
void publish_filled_block() {
  AudioBlock block;
  block.sequence = g_sequence++;
  block.deviceMs = millis();
  block.rms = static_cast<uint16_t>(
      sqrtf(static_cast<float>(g_block_energy) / kBlockSamples));
  block.peak = g_block_peak;
  block.captureMs = clamp_u16(g_block_capture_ms);

  g_new_frame_count = 0u;
  if (gate_block(block.rms)) {
    block.featureMs = extract_features();
    if (++g_classifier.blocksSinceInference >= kInferencePeriodBlocks) {
      g_classifier.blocksSinceInference = 0u;
      run_inference();
    }
  }
  block.speechActive = g_speech_state == SpeechState::Active;
  // Advances whether or not the block became features, or the next window
  // would splice this block onto one already 60 ms stale.
  memcpy(&g_mfcc_input[0], &g_mfcc_input[kBlockSamples],
         kMfccHopSamples * sizeof(int16_t));

  compute_waveform_envelope();
  send_audio_result_packet(block);

  g_block_fill = 0u;
  g_block_energy = 0u;
  g_block_peak = 0u;
  g_block_capture_ms = 0u;
}

/**
 * @brief Append filtered samples to the pending block, streaming it when full.
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
      const uint16_t magnitude =
          static_cast<uint16_t>(sample < 0 ? -sample : sample);
      if (magnitude > g_block_peak) {
        g_block_peak = magnitude;
      }
    }
    g_block_fill += taking;
    consumed += taking;
    if (g_block_fill == kBlockSamples) {
      publish_filled_block();
    }
  }
}

/**
 * @brief Note that the PDM library has filled a buffer.
 *
 * Runs in interrupt context, so it only raises a flag. A buffer that is still
 * unread when the next one lands is audio this demo never saw, and is counted
 * the way the Nicla Voice demo counts a gap in the NDP120 chunk counter.
 */
void on_pdm_data() {
  if (g_pdm_ready) {
    ++g_pdm_overruns;
  }
  g_pdm_ready = true;
}

/**
 * @brief Take the next microphone buffer from the PDM library when one is
 *        ready.
 *
 * @return True when a fresh buffer was consumed.
 */
bool capture_fresh_buffer() {
  if (!g_pdm_ready) {
    return false;
  }
  const uint32_t started_ms = millis();
  const int read_bytes = PDM.read(g_pdm_samples, sizeof(g_pdm_samples));
  g_pdm_ready = false;
  g_block_capture_ms += millis() - started_ms;

  const size_t count = static_cast<size_t>(read_bytes) / sizeof(int16_t);
  remove_dc_offset(g_pdm_samples, count);
  accumulate_samples(g_pdm_samples, count);
  return true;
}

/**
 * @brief Drop any part-built block and start a stream from a clean state.
 */
void reset_capture_state() {
  g_dc_block = DcBlockState();
  g_classifier = Classifier();
  g_speech_state = SpeechState::Idle;
  g_speech_started_ms = 0u;
  g_new_frame_count = 0u;
  clear_spectrogram();
  g_block_fill = 0u;
  g_block_energy = 0u;
  g_block_peak = 0u;
  g_block_capture_ms = 0u;
  g_pdm_overruns = 0u;
  g_pdm_ready = false;
  g_sequence = 0u;
}

/**
 * @brief Bring BB15 up, load the keyword model and read its dequantization.
 *
 * @return True when the runtime is ready to infer.
 */
bool prepare_akida() {
  static BB15 bb15(g_pinout, g_config);
  static BB15Runner runner = bb15.createRunner();
  g_bb15 = &bb15;
  g_runner = &runner;

  if (bb15.begin() != BB15Status::Ok || runner.begin() != BB15Status::Ok) {
    return false;
  }
  g_model.setStorage(BB15ModelStorage::HostMemory);
  if (runner.loadModel(g_model) != BB15Status::Ok) {
    return false;
  }

  const akida::ProgramInfo info(program, static_cast<size_t>(program_len));
  if (!info.is_valid() || info.shifts().size < kClassCount ||
      info.scales().size < kClassCount) {
    return false;
  }
  g_output_shifts = info.shifts().data;
  g_output_scales = info.scales().data;
  return true;
}

/**
 * @brief Start the onboard PDM microphone.
 *
 * The buffer size is set first because begin() captures the buffer pointer,
 * and resizing afterwards would leave the library filling freed memory.
 *
 * @return True when the microphone is streaming audio buffers.
 */
bool prepare_microphone() {
  PDM.onReceive(on_pdm_data);
  PDM.setBufferSize(kPdmBufferBytes);
  PDM.setGain(kPdmGain);
  return PDM.begin(1, kSampleRateHz) == 1;
}

}  // namespace

void setup() {
  pinMode(LEDR, OUTPUT);
  pinMode(LEDG, OUTPUT);
  pinMode(LEDB, OUTPUT);
  set_led(StatusLed::Blue);
  Serial.begin(kSerialBaud);
  wait_for_serial();
  delay(kBootSettleMs);

  Serial.println();
  Serial.println(kSketchName);
  Serial.print(kLogPrefix);
  Serial.println(" board=BB15 + Nicla Vision");
  Serial.print(kLogPrefix);
  Serial.print(" audio=");
  Serial.print(kSampleRateHz);
  Serial.print("Hz mono block=");
  Serial.print(kBlockSamples);
  Serial.println(" samples");

  if (!mfcc_begin()) {
    fail_setup("mfcc", kStatusMfccInitFailed);
    return;
  }
  clear_spectrogram();
  if (!prepare_akida()) {
    fail_setup("akida", kStatusAkidaFailed);
    Serial.print(kLogPrefix);
    Serial.print(" detail=");
    g_bb15->printLastError(Serial);
    return;
  }
  Serial.print(kLogPrefix);
  Serial.print(" akida_ready ip_version=0x");
  Serial.print(g_bb15->ipVersion(), HEX);
  Serial.print(" model=");
  Serial.print(akida_model_path);
  Serial.print(" bytes=");
  Serial.println(static_cast<long>(akida_program_length_bytes));
  if (!prepare_microphone()) {
    fail_setup("microphone", kStatusMicrophoneFailed);
    return;
  }

  set_led(StatusLed::Off);
  Serial.print(kLogPrefix);
  Serial.print(" microphone_ready buffer_bytes=");
  Serial.print(static_cast<unsigned>(PDM.getBufferSize()));
  Serial.print(" gain=");
  Serial.println(kPdmGain);
  Serial.print(kLogPrefix);
  Serial.println(" usb_protocol=BB15/v1 waiting_for_start_stream");
}

void loop() {
  const PacketType command = poll_host_command();
  if (g_setup_failure != kStatusOk) {
    // Keep answering, so a tool connecting long after boot is told why there is
    // nothing to stream.
    if (command != static_cast<PacketType>(0u)) {
      send_error_packet(g_setup_failure);
    }
    report_idle_state();
    set_led(StatusLed::Red);
    delay(50);
    set_led(StatusLed::Off);
    delay(450);
    return;
  }
  if (command == PacketType::StartStream) {
    reset_capture_state();
    g_streaming = true;
    set_led(StatusLed::Green);
    send_audio_config_packet();
    return;
  }
  if (command == PacketType::StopStream) {
    g_streaming = false;
    set_led(StatusLed::Off);
    return;
  }
  if (command == PacketType::RequestConfig) {
    send_audio_config_packet();
    return;
  }
  if (!g_streaming) {
    report_idle_state();
    delay(1);
    return;
  }

  if (!capture_fresh_buffer()) {
    delay(1);
  }
}
