#include <Arduino.h>
#include <BB15.h>
#include <NDP.h>

#include "akida/program_info.h"
#include "mfcc.h"
#include "model_metadata.h"
#include "program.h"

// nicla_voice and nicla_sense share the Arduino NICLA variant, so this guard
// can only reject other families. Building this for Nicla Sense ME compiles but
// fails at NDP.begin(), because that board has no NDP120.
#ifndef ARDUINO_NICLA
#error "bb15_nicla_voice_keyword_spotting requires Arduino Nicla Voice."
#endif

namespace {

// This example owns microphone capture, block framing and USB framing. The
// Nicla Voice microphone reaches the NDP120 only, never the nRF52832, so the
// bundled NDP library is the single route to audio (Nicla Voice datasheet
// section 4.7). BB15 inference arrives in a later stage of this demo.
constexpr uint32_t kSerialBaud = 921600u;
constexpr uint32_t kBootSettleMs = 250u;

// A board that is waiting for a stream used to say nothing at all, which makes
// it indistinguishable from a dead one to anything that is only listening. It
// now reports itself on this interval instead, so a plain terminal is enough to
// tell a healthy idle board from a wedged one, and so a setup failure is
// visible without having to ask for it at the right moment.
constexpr uint32_t kIdleReportMs = 2000u;

// The NDP120 firmware packages already stored in the board's on-board QSPI
// flash. All three are required: with only the MCU and DSP packages loaded the
// audio holding tank never advances, so extractData keeps returning the same
// stale chunk. The neural-network package carries the DSP audio flow
// configuration that starts the tank; this demo uses none of its classes.
constexpr const char* kNdpMcuFirmware = "mcu_fw_120_v90.synpkg";
constexpr const char* kNdpDspFirmware = "dsp_firmware_v90.synpkg";
constexpr const char* kNdpAudioFlowPackage =
    "alexa_334_NDP120_B0_v11_v90.synpkg";

// The NDP120 publishes one 24 ms chunk of 16 kHz mono PCM at a time and holds
// it until the next one replaces it, so polling must be paced: too slow loses
// audio, too fast burns SPI time re-reading a chunk already taken. Each chunk
// carries a four-byte annotation after the samples whose third byte is a
// monotonic counter, which is what makes a fresh chunk recognizable.
constexpr size_t kChunkBufferBytes = 1024u;
constexpr size_t kAnnotationBytes = 4u;
constexpr size_t kAnnotationCounterOffset = 2u;
constexpr uint32_t kChunkPeriodMs = 24u;
constexpr uint32_t kChunkPollLeadMs = 3u;
constexpr uint32_t kChunkRetryMs = 2u;
// A single failed read is unremarkable. A run of them is a real transport fault
// worth telling the desktop tool about, rather than leaving it to guess from a
// stalled view.
constexpr uint8_t kChunkFailureLimit = 25u;

// Front-end values taken from spark and hard-coded here on purpose: this demo
// has no parameter surface. Sample rate, hop and block size come from
// spark/source/Kconfig (SAMPLING_RATE, MFCC_SAMPLE_COUNT, AUDIO_BLOCK_MS); the
// gating and decision values come from spark/source/core/common/kws_config.c.
constexpr uint32_t kSampleRateHz = 16000u;
constexpr uint16_t kMfccHopSamples = 320u;
constexpr uint16_t kBlockSamples = 960u;
constexpr uint8_t kMfccFramesPerBlock = kBlockSamples / kMfccHopSamples;
constexpr uint16_t kSpectrogramFrames = 49u;
constexpr uint8_t kSpectrogramCoefficients = 10u;
constexpr uint8_t kClassCount = 12u;
// Inference runs every third block, about every 180 ms, from spark's
// g_inference_period in source/core/common/audio/audio_processor.c where
// ap_counter counts blocks. It is deliberately not once per MFCC frame.
constexpr uint8_t kInferencePeriodBlocks = 3u;
// spark's info.yaml for this model: the classifier reports silence and unknown
// like any other class, and neither can trigger a detection.
constexpr uint8_t kSilenceClass = 10u;
constexpr uint8_t kUnknownClass = 11u;
constexpr uint16_t kRmsThreshold = 550u;
constexpr uint16_t kSpeechActiveTimeMs = 1300u;
constexpr uint16_t kSmoothingAlphaQ15 = 22938u;
constexpr uint16_t kScoreThresholdQ15 = 16384u;
constexpr float kSmoothingAlpha = 0.70f;
constexpr float kScoreThreshold = 0.50f;
constexpr uint16_t kDebounceMs = 300u;
constexpr uint8_t kChimingThreshold = 3u;

// Quantization of an MFCC coefficient to the model's uint8 input, from
// do_inference() in spark/source/apps/demo_apps/main.cpp: clamp to 0..255 then
// truncate. The full-scale divisor is the model's own, from
// spark/source/external/model_files/kws/kws/info.yaml, and is not a tuning
// knob. It matches this front end: a silent frame gives coefficient 0 of
// -123.569649, so silence lands one step below zero on the model's scale.
constexpr float kMfccFullScale = 123.56967163085938f;

// spark clears its spectrogram to float zero, which quantizes to 128 rather
// than to 0, so a cleared ring here has to hold 128 to feed the model the same
// bytes spark would.
constexpr uint8_t kClearedFeature = 128u;

// BB15 transport. The header lines run through the Nicla Voice's TXB0108
// translators, which are weak drivers, so this demo clocks the Akida SPI more
// slowly than the Nicla Vision demo does.
constexpr uint32_t kAkidaSpiClockHz = 8000000u;

// spark's DC blocking high-pass, from dc_block_process() in
// spark/source/core/interface/audio/pdm_mic.c. Its output is what spark takes
// the RMS of and what it feeds the MFCC, so the RMS threshold above only means
// the same thing here if the same filter runs first.
constexpr int32_t kDcBlockAlphaQ15 = 32700;

// One min/max pair per 10 samples. spark sends the same shape of envelope to
// its phone app, at 32 pairs per block; the extra resolution here is affordable
// because this link is a 921600 baud UART rather than BLE.
constexpr uint16_t kWaveformPoints = 96u;
constexpr uint16_t kWaveformWindowSamples = kBlockSamples / kWaveformPoints;

// USB protocol v1. The header is the one the Nicla Vision human-detection demo
// defines: "BB15", version, type, payload size (uint32 little-endian). The
// three host commands are identical; the two device message types are new so a
// mismatched desktop tool fails to recognize them instead of misreading a
// camera payload as audio.
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

// Sent in the predicted-index field when nothing has been detected.
constexpr uint8_t kNoPrediction = 0xFFu;
constexpr uint8_t kStatusOk = 0u;

// Status values for the error packet. A chunk-capture failure reports the
// Syntiant interface library's own code, which is a small positive, so setup
// failures take a range of their own.
constexpr uint8_t kStatusMfccInitFailed = 0x81u;
constexpr uint8_t kStatusMicrophoneFailed = 0x82u;
constexpr uint8_t kStatusAkidaFailed = 0x83u;
constexpr uint8_t kReservedByte = 0u;

// From spark's kws_new_tags[] in
// source/apps/demo_apps/sample_input/kws/kws_inputs.cpp, which agrees with the
// silence and unknown class indices in the model's info.yaml.
constexpr const char* kClassLabels[kClassCount] = {
    "down",  "go",   "left", "no",  "off",     "on",
    "right", "stop", "up",   "yes", "silence", "unknown"};

constexpr const char* kSketchName = "bb15_nicla_voice_keyword_spotting";
constexpr const char* kLogPrefix = "[bb15_nicla_voice_keyword_spotting]";

/** @brief One completed 60 ms audio block, ready to stream to the desktop tool.
 */
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

/** @brief The Akida runtime state this demo keeps across blocks. */
struct Classifier {
  float smoothed[kClassCount] = {};
  uint8_t chiming[kClassCount] = {};
  uint32_t lastTriggerMs = 0u;
  uint8_t blocksSinceInference = 0u;
  uint8_t predicted = kNoPrediction;
  // Wraps, and only ever increments, so the desktop tool can tell a fresh
  // detection from the same keyword still being displayed.
  uint8_t detections = 0u;
  uint16_t inferMs = 0u;
};

/**
 * @brief How far chunk synchronization has got since the last stream start.
 *
 * The NDP120's holding tank keeps running while no stream is being served, so
 * the first counter gaps a fresh stream sees say nothing about continuity.
 */
enum class ChunkSync : uint8_t {
  NoCounter,
  Seeded,
  Running,
};

/** @brief Running state of spark's first-order DC blocking high-pass. */
struct DcBlockState {
  int32_t previousInput = 0;
  int32_t previousOutput = 0;
};

alignas(4) uint8_t g_chunk[kChunkBufferBytes];
// spark's mfcc_process_input() layout: [0, hop) keeps the last hop of the
// previous block and [hop, hop + block) holds this one. Each MFCC window is two
// hops wide and steps by one hop, so a 60 ms block yields exactly three frames
// and the final window ends at the buffer end.
int16_t g_mfcc_input[kMfccHopSamples + kBlockSamples];
int16_t g_waveform[2u * kWaveformPoints];
// The model's input, held quantized because that is the only form it is used
// in. g_spectrogram_index is the write position, which is also the oldest
// frame, so the model input unrolls from it.
uint8_t g_spectrogram[kSpectrogramFrames * kSpectrogramCoefficients];
uint16_t g_spectrogram_index = 0u;
uint8_t g_new_features[kMfccFramesPerBlock * kSpectrogramCoefficients];
uint8_t g_new_frame_count = 0u;
SpeechState g_speech_state = SpeechState::Idle;
uint32_t g_speech_started_ms = 0u;
// The model input, unrolled oldest frame first the way spark builds it.
uint8_t g_model_input[kSpectrogramFrames * kSpectrogramCoefficients];
Classifier g_classifier;
BB15Pinout g_pinout = BB15Pinout::niclaVoiceDefaults();
BB15Config g_config = []() {
  BB15Config config = BB15Config::defaults();
  config.spiClockHz = kAkidaSpiClockHz;
  return config;
}();
BB15* g_bb15 = nullptr;
BB15Runner* g_runner = nullptr;
BB15Model g_model(program, static_cast<size_t>(program_len));
// Per-neuron dequantization, read out of the program itself rather than
// hard-coded, so a re-exported model cannot leave stale constants behind.
const int32_t* g_output_shifts = nullptr;
const float* g_output_scales = nullptr;
DcBlockState g_dc_block;
size_t g_block_fill = 0u;
int64_t g_block_energy = 0;
uint16_t g_block_peak = 0u;
uint32_t g_block_capture_ms = 0u;
uint8_t g_last_chunk_counter = 0u;
uint8_t g_chunk_failures = 0u;
ChunkSync g_chunk_sync = ChunkSync::NoCounter;
uint32_t g_next_chunk_poll_ms = 0u;
uint16_t g_dropped_chunks = 0u;
uint32_t g_sequence = 0u;
bool g_streaming = false;
// Set when setup could not finish. The board then answers host commands with
// the reason instead of going quiet, because a setup message printed once at
// boot is gone by the time a desktop tool opens the port.
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
 * @brief Show the demo's state on the RGB LED.
 *
 * @param color  Colour to display, or `off` to clear it.
 */
void set_led(RGBColors color) { nicla::leds.setColor(color); }

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
 * @brief Write a little-endian unsigned 16-bit value to the host.
 *
 * @param value  Value to write.
 */
void write_u16(uint16_t value) {
  Serial.write(static_cast<uint8_t>(value & 0xFFu));
  Serial.write(static_cast<uint8_t>((value >> 8) & 0xFFu));
}

/**
 * @brief Write a little-endian unsigned 32-bit value to the host.
 *
 * @param value  Value to write.
 */
void write_u32(uint32_t value) {
  for (uint8_t shift = 0u; shift < 32u; shift += 8u) {
    Serial.write(static_cast<uint8_t>((value >> shift) & 0xFFu));
  }
}

/**
 * @brief Write a little-endian signed 16-bit value to the host.
 *
 * @param value  Value to write.
 */
void write_i16(int16_t value) { write_u16(static_cast<uint16_t>(value)); }

/**
 * @brief Write the fixed ten-byte protocol header.
 *
 * @param type           Message type that follows.
 * @param payload_bytes  Size of the payload after the header.
 */
void write_packet_header(PacketType type, uint32_t payload_bytes) {
  Serial.write(kProtocolMagic, sizeof(kProtocolMagic));
  Serial.write(kProtocolVersion);
  Serial.write(static_cast<uint8_t>(type));
  write_u32(payload_bytes);
}

/**
 * @brief Send the pipeline description the desktop tool needs to draw itself.
 */
void send_audio_config_packet() {
  write_packet_header(PacketType::AudioConfig,
                      static_cast<uint32_t>(kAudioConfigBytes));
  write_u32(kSampleRateHz);
  write_u16(kBlockSamples);
  write_u16(kWaveformPoints);
  write_u16(kSpectrogramFrames);
  Serial.write(kSpectrogramCoefficients);
  Serial.write(kClassCount);
  write_u16(kRmsThreshold);
  write_u16(kSpeechActiveTimeMs);
  write_u16(kSmoothingAlphaQ15);
  write_u16(kScoreThresholdQ15);
  write_u16(kDebounceMs);
  Serial.write(kChimingThreshold);
  Serial.write(kReservedByte);
  Serial.flush();
}

/**
 * @brief Report a device-side failure to the desktop tool.
 *
 * @param status  Syntiant interface library status for the failed operation.
 */
void send_error_packet(uint8_t status) {
  write_packet_header(PacketType::Error, 1u);
  Serial.write(status);
  Serial.flush();
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
  write_packet_header(PacketType::AudioResult,
                      static_cast<uint32_t>(kAudioResultMetadataBytes) +
                          waveform_bytes + feature_bytes + score_bytes);
  write_u32(block.sequence);
  write_u32(block.deviceMs);
  write_u16(block.rms);
  write_u16(block.peak);
  write_u16(g_dropped_chunks);
  write_u16(block.captureMs);
  write_u16(block.featureMs);
  write_u16(g_classifier.inferMs);
  Serial.write(static_cast<uint8_t>(block.speechActive ? 1u : 0u));
  Serial.write(kStatusOk);
  Serial.write(g_classifier.predicted);
  Serial.write(kClassCount);
  Serial.write(static_cast<uint8_t>(kWaveformPoints));
  Serial.write(g_new_frame_count);
  Serial.write(g_classifier.predicted == kNoPrediction
                   ? 0u
                   : g_classifier.chiming[g_classifier.predicted]);
  Serial.write(g_classifier.detections);
  for (uint16_t point = 0u; point < 2u * kWaveformPoints; ++point) {
    write_i16(g_waveform[point]);
  }
  Serial.write(g_new_features, static_cast<size_t>(g_new_frame_count) *
                                   kSpectrogramCoefficients);
  for (uint8_t index = 0u; index < kClassCount; ++index) {
    write_i16(static_cast<int16_t>(g_classifier.smoothed[index] * 32767.0f));
  }
  Serial.flush();
}

/**
 * @brief Read at most one host command from the serial input.
 *
 * Commands are zero-payload packets. Parsing byte by byte lets the boot banner
 * and the NDP library's own progress output pass through harmlessly when they
 * are still buffered as the desktop tool opens the port.
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
 * The filter is a first-order IIR carrying state between calls, so filtering
 * the stream in 24 ms chunks gives the same result as spark filtering it in
 * 60 ms blocks.
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
 * The engine's own formula, from HardwareDeviceImpl::dequantize(): each
 * neuron's potential has its shift subtracted and is divided by its scale.
 * spark reaches the same numbers through akida_predict(), which dequantizes
 * before returning; this library hands back the potentials instead.
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
 * From is_kws_debounce_complete() in spark's main.cpp. While it has not, spark
 * skips the whole front end, which is why this gates the block before the RMS
 * is even looked at.
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
 * kws_post_processing() in spark's main.cpp: a class at or above the score
 * threshold advances its chiming counter and anything below resets it, silence
 * and unknown can never trigger, and the highest scoring class to reach the
 * chiming threshold fires. A detection starts the debounce cooldown and throws
 * the accumulated state away.
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
 * spark's rule, from audio_process_thread() in
 * spark/source/core/common/audio/audio_processor.c. A block at or above the RMS
 * threshold is speech and restarts the timer. A quieter block is still
 * processed while the utterance is within kSpeechActiveTimeMs of its last loud
 * block, which is what captures the tail of a word; the first quiet block past
 * that returns to idle and throws the part-built model input away.
 *
 * @param rms  This block's RMS, taken after the DC blocker as spark does.
 * @return True when the block should be turned into features.
 */
bool gate_block(uint16_t rms) {
  if (!debounce_complete()) {
    // spark skips the whole front end during the cooldown, so a detection is
    // not immediately re-triggered by the tail of the same word.
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
  // The overlap advances whether or not the block became features, or the next
  // window would splice this block onto one that is already 60 ms stale.
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
 * @brief Take the next microphone chunk from the NDP120 when one is due.
 *
 * Polls just before the next chunk is expected and retries quickly until the
 * annotation counter changes, so each chunk is taken exactly once. A gap in
 * that counter is counted as dropped audio, which is what lets the desktop tool
 * show whether the stream stayed continuous.
 *
 * @return True when a fresh chunk was consumed.
 */
bool capture_fresh_chunk() {
  if (static_cast<int32_t>(millis() - g_next_chunk_poll_ms) < 0) {
    return false;
  }

  unsigned int length = 0u;
  const uint32_t started_ms = millis();
  const int status = NDP.extractData(g_chunk, &length);
  if (status != 0 || length == 0u ||
      length + kAnnotationBytes > sizeof(g_chunk)) {
    g_next_chunk_poll_ms = millis() + kChunkRetryMs;
    if (++g_chunk_failures >= kChunkFailureLimit) {
      g_chunk_failures = 0u;
      send_error_packet(static_cast<uint8_t>(status));
    }
    return false;
  }
  g_chunk_failures = 0u;

  const uint8_t counter = g_chunk[length + kAnnotationCounterOffset];
  if (g_chunk_sync != ChunkSync::NoCounter && counter == g_last_chunk_counter) {
    g_next_chunk_poll_ms = millis() + kChunkRetryMs;
    return false;
  }
  const uint8_t advance = static_cast<uint8_t>(counter - g_last_chunk_counter);
  g_last_chunk_counter = counter;
  g_next_chunk_poll_ms = millis() + kChunkPeriodMs - kChunkPollLeadMs;

  // Take audio only once two consecutive chunks confirm the stream is being
  // kept up with, so a stream start costs at most two discarded chunks instead
  // of reporting the tank's backlog as dropped audio.
  if (g_chunk_sync != ChunkSync::Running) {
    g_chunk_sync = g_chunk_sync == ChunkSync::Seeded && advance == 1u
                       ? ChunkSync::Running
                       : ChunkSync::Seeded;
    return false;
  }
  g_dropped_chunks = static_cast<uint16_t>(g_dropped_chunks + (advance - 1u));
  g_block_capture_ms += millis() - started_ms;

  int16_t* samples = reinterpret_cast<int16_t*>(g_chunk);
  const size_t count = length / sizeof(int16_t);
  remove_dc_offset(samples, count);
  accumulate_samples(samples, count);
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
  g_chunk_sync = ChunkSync::NoCounter;
  g_chunk_failures = 0u;
  g_dropped_chunks = 0u;
  g_sequence = 0u;
  g_next_chunk_poll_ms = millis();
}

/**
 * @brief Bring BB15 up, load the keyword model and read its dequantization.
 *
 * The model is kept in host memory rather than BB15 external flash: it is
 * 22 KB against 400 KB of free program space, so it costs flash this sketch
 * has and saves a flashing step, a companion sketch and a failure mode. The
 * Nicla Vision demo makes the opposite choice because its model is 184 KB.
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
 * @brief Load the NDP120 firmware and start the microphone.
 *
 * @return True when the microphone is streaming audio chunks.
 */
bool prepare_microphone() {
  if (NDP.begin(kNdpMcuFirmware) != 1 || NDP.load(kNdpDspFirmware) != 1 ||
      NDP.load(kNdpAudioFlowPackage) != 1) {
    return false;
  }
  if (NDP.turnOnMicrophone() != 0) {
    return false;
  }
  return NDP.getAudioChunkSize() > 0;
}

}  // namespace

void setup() {
  Serial.begin(kSerialBaud);
  // nicla::disableLDO() must not be called here, unlike in the NDP library's
  // own audio examples. That LDO is the PMIC load switch feeding VDDIO_EXT,
  // which supplies the TXB0108 level translators on every header pin (Nicla
  // Voice datasheet section 4.9), so disabling it cuts off an attached BB15.
  nicla::begin();
  nicla::leds.begin();
  set_led(blue);
  // Serial here is a UART bridged to USB by the on-board SAMD11, not a native
  // USB device, so it is ready immediately and only the host needs a moment.
  delay(kBootSettleMs);

  Serial.println();
  Serial.println(kSketchName);
  Serial.print(kLogPrefix);
  Serial.println(" board=BB15 + Nicla Voice");
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

  set_led(off);
  Serial.print(kLogPrefix);
  Serial.print(" microphone_ready chunk_bytes=");
  Serial.println(NDP.getAudioChunkSize());
  Serial.print(kLogPrefix);
  Serial.println(" usb_protocol=BB15/v1 waiting_for_start_stream");
}

void loop() {
  const PacketType command = poll_host_command();
  if (g_setup_failure != kStatusOk) {
    // Keep answering, so a tool that connects long after boot is told why the
    // board has nothing to stream rather than being left to guess.
    if (command != static_cast<PacketType>(0u)) {
      send_error_packet(g_setup_failure);
    }
    report_idle_state();
    set_led(red);
    delay(50);
    set_led(off);
    delay(450);
    return;
  }
  if (command == PacketType::StartStream) {
    reset_capture_state();
    g_streaming = true;
    set_led(green);
    send_audio_config_packet();
    return;
  }
  if (command == PacketType::StopStream) {
    g_streaming = false;
    set_led(off);
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

  if (!capture_fresh_chunk()) {
    delay(1);
  }
}
