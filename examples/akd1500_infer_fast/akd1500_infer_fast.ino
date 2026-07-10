#include <Arduino.h>

#if defined(ARDUINO_NICLA_VISION)

#include <AKD1500.h>
#include <PDM.h>

#include "akd1500_fast_flash.h"
#include "bb15_demo_board.h"
#include "camera.h"
#include "gc2145.h"
#include "model_metadata.h"
#include "program.h"

#define BB15_FLASH_PROFILE_AUTO 0
#define BB15_FLASH_PROFILE_RENESAS 1

#ifndef BB15_FLASH_PROFILE_KNOWN
#define BB15_FLASH_PROFILE_KNOWN BB15_FLASH_PROFILE_AUTO
#endif

#ifndef CLAP_INFER_START_FREE_RUNNING
#define CLAP_INFER_START_FREE_RUNNING 0
#endif

namespace {

// Serial and application timing for a readable terminal demo.
constexpr uint32_t kSerialBaud = 115200u;
// Keep USB-CDC bring-up short so standalone power-up reaches inference fast.
constexpr uint32_t kSerialWaitMs = 250u;
constexpr uint32_t kBootSettleMs = 25u;
// Keep this minimal so the demo runs quickly without starving USB serial on
// the Nicla Vision mbed core.
constexpr uint32_t kLoopDelayMs = 1u;
constexpr uint8_t kRequestFrame = 1u;
constexpr uint8_t kRequestConfig = 2u;
constexpr uint8_t kRequestStreamStart = 3u;
constexpr uint8_t kRequestStreamStop = 4u;
constexpr uint8_t kResolution = CAMERA_R160x120;
constexpr uint8_t kCameraImageMode = CAMERA_RGB565;
constexpr uint8_t kPreviewImageMode = CAMERA_GRAYSCALE;
constexpr uint16_t kCameraWidth = 160u;
constexpr uint16_t kCameraHeight = 120u;
constexpr uint8_t kPreviewResolution = kResolution;
constexpr uint16_t kPreviewWidth = kCameraWidth;
constexpr uint16_t kPreviewHeight = kCameraHeight;
constexpr size_t kPreviewFrameBytes =
    static_cast<size_t>(kPreviewWidth) * kPreviewHeight;
constexpr size_t kCaptureFrameBytes =
    static_cast<size_t>(kCameraWidth) * kCameraHeight * 2u;
constexpr int32_t kFrameRate = 30;
constexpr uint16_t kModelWidth = 96u;
constexpr uint16_t kModelHeight = 96u;
constexpr uint16_t kModelChannels = 1u;
constexpr size_t kModelInputBytes =
    static_cast<size_t>(kModelWidth) * kModelHeight * kModelChannels;
constexpr uint32_t kFlashModelOffset = 0u;
// Expose the Akida and bridge-flash SPI clocks as sketch-level knobs so the
// demo can be tuned directly from the Arduino IDE without editing the library.
constexpr uint32_t kAkidaSpiClockHz = 25000000u;
constexpr uint32_t kFlashSpiClockHz = 2000000u;
constexpr uint8_t kInferenceResultSequence[4] = {0xAC, 0x1D, 0x1A, 0xDA};
constexpr uint8_t kStartSequence[4] = {0xFA, 0xCE, 0xFE, 0xED};
constexpr uint8_t kStopSequence[4] = {0xDA, 0xBB, 0xAD, 0x00};
constexpr uint32_t kPreviewSessionHoldMs = 2000u;
constexpr uint32_t kAkidaColdResetHoldMs = 250u;
constexpr uint32_t kAkidaReloadSettleMs = 10u;
constexpr uint8_t kAkidaWakeRetryLimit = 5u;
constexpr uint32_t kBurstInferencesPerClap = 5u;
constexpr int32_t kMicSampleRateHz = 16000;
constexpr uint8_t kMicChannels = 1u;
constexpr uint8_t kMicGain = 8u;
constexpr size_t kMicBufferSamples = 512u;
constexpr uint16_t kClapPeakThreshold = 12000u;
constexpr uint16_t kClapMeanAbsThreshold = 1500u;
constexpr uint32_t kClapCooldownMs = 1500u;
constexpr const char* kSketchName = "akd1500_infer_fast";
constexpr const char* kLogPrefix = "[akd1500_infer_fast]";
constexpr const char* kBundledModelName = "presence_regular_96_gray";

// Application state shared across setup and the main inference loop.
bool g_camera_ok = false;
bool g_model_ok = false;
uint8_t g_model_input[kModelInputBytes];
uint8_t g_preview_frame[kPreviewFrameBytes];
uint16_t g_crop_x[kModelWidth];
uint16_t g_crop_y[kModelHeight];
bool g_crop_maps_ready = false;
const char* g_last_failure_stage = nullptr;
const char* g_last_failure_detail = nullptr;
uint32_t g_last_frame_signature = 0u;
uint32_t g_last_input_signature = 0u;
uint32_t g_last_output_signature = 0u;
uint32_t g_same_frame_count = 0u;
uint32_t g_same_input_count = 0u;
uint32_t g_same_output_count = 0u;
bool g_reported_stale_camera = false;
bool g_reported_stale_output = false;
uint32_t g_last_preview_request_ms = 0u;
bool g_preview_session_active = false;
bool g_preview_streaming_active = false;
bool g_akida_awake = false;
bool g_free_running_enabled = CLAP_INFER_START_FREE_RUNNING != 0;
uint32_t g_burst_inferences_remaining = 0u;
uint32_t g_clap_trigger_count = 0u;
uint32_t g_last_clap_ms = 0u;
int16_t g_mic_capture_samples[kMicBufferSamples];
int16_t g_mic_processing_samples[kMicBufferSamples];
volatile uint16_t g_mic_samples_read = 0u;

struct InferenceObservation {
  bool ok = false;
  AKD1500Status status = AKD1500Status::NotInitialized;
  size_t predicted_index = 0u;
  int32_t score0 = 0;
  int32_t score1 = 0;
  uint16_t grab_ms = 0u;
  uint16_t prep_ms = 0u;
  uint16_t infer_ms = 0u;
};

struct AudioObservation {
  bool valid = false;
  uint16_t peak = 0u;
  uint16_t mean_abs = 0u;
  uint16_t sample_count = 0u;
};

bool validate_loaded_model_geometry();
void prepare_crop_maps();
void fill_preview_and_model_input_from_rgb565(const uint8_t* frame_rgb565);
size_t capture_frame_bytes();
akd1500_fast_flash::Config make_fast_flash_config();
void on_pdm_data();
bool begin_microphone();
bool consume_audio_observation(AudioObservation* observation);
bool clap_detected(AudioObservation* observation);
bool hold_akida_off(const char* reason);
void clear_failure();
void halt_bb15_failed(const char* stage, const char* detail = nullptr);
bool power_cycle_akida_off(const char* reason, uint32_t hold_ms);
void print_runtime_controls();
bool set_free_running_mode(bool enabled, const char* source);
bool wake_akida_for_inference(const char* reason);
bool prepare_model();

// Lazily construct board-facing objects after Arduino startup to avoid
// static-initialization ordering issues across library translation units.
bb15_demo::Bb15NiclaVisionBoard& board() {
  static bb15_demo::Bb15NiclaVisionBoard instance;
  return instance;
}

GC2145& sensor() {
  static GC2145 instance;
  return instance;
}

Camera& camera() {
  static Camera instance(sensor());
  return instance;
}

FrameBuffer& framebuffer() {
  static FrameBuffer instance;
  return instance;
}

akd1500_fast_flash::FastFlashModelRunner& flashModelRunner() {
  static akd1500_fast_flash::FastFlashModelRunner instance(
      make_fast_flash_config());
  return instance;
}

// Give the USB CDC serial port a short window to enumerate on host machines.
void wait_for_serial() {
  const uint32_t start_ms = millis();
  while (!Serial && (millis() - start_ms) < kSerialWaitMs) {
  }
}

// Nicla LEDs are active-low, so invert the logical application state here.
void set_led(bool active) {
  digitalWrite(LED_BUILTIN, active ? LOW : HIGH);
}

void set_red_led(bool active) { digitalWrite(LEDR, active ? LOW : HIGH); }

void clear_failure() {
  g_last_failure_stage = nullptr;
  g_last_failure_detail = nullptr;
}

// Halt in a visible way after a fatal setup failure so bench debugging only
// needs a terminal and the status LED.
void blink_forever() {
  for (;;) {
    set_led(true);
    delay(50);
    set_led(false);
    delay(950);
    if (g_last_failure_stage != nullptr) {
      Serial.print(kLogPrefix);
      Serial.print(" halted stage=");
      Serial.print(g_last_failure_stage);
      if (g_last_failure_detail != nullptr) {
        Serial.print(" detail=");
        Serial.print(g_last_failure_detail);
      }
      if (flashModelRunner().lastError().status != AKD1500Status::Ok) {
        Serial.print(" akida=");
        flashModelRunner().printLastError(Serial);
      } else {
        Serial.println();
      }
    }
  }
}

void halt_bb15_failed(const char* stage, const char* detail) {
  g_model_ok = false;
  g_akida_awake = false;
  g_burst_inferences_remaining = 0u;
  board().holdAkidaInReset();
  set_led(false);
  set_red_led(true);
  g_last_failure_stage = stage;
  g_last_failure_detail = detail;

  Serial.print(kLogPrefix);
  Serial.print(" bb15 FAILED stage=");
  Serial.print(stage == nullptr ? "unknown" : stage);
  if (detail != nullptr) {
    Serial.print(" detail=");
    Serial.print(detail);
  }
  if (flashModelRunner().lastError().status != AKD1500Status::Ok) {
    Serial.print(" akida=");
    flashModelRunner().printLastError(Serial);
  } else {
    Serial.println();
  }

  for (;;) {
    delay(1000);
  }
}

// Print and remember the first fatal step that failed during setup or runtime.
void print_failure(const char* stage, const char* detail = nullptr) {
  g_last_failure_stage = stage;
  g_last_failure_detail = detail;
  Serial.print(kLogPrefix);
  Serial.print(" result=FAIL stage=");
  Serial.print(stage);
  if (detail != nullptr) {
    Serial.print(" detail=");
    Serial.print(detail);
  }
  if (flashModelRunner().lastError().status != AKD1500Status::Ok) {
    Serial.print(" akida=");
    flashModelRunner().printLastError(Serial);
    return;
  }
  Serial.println();
}

// Pretty-printer for Akida tensor shapes in the serial banner.
void print_shape(const akida::Shape& shape) {
  Serial.print("[");
  for (size_t i = 0; i < shape.size(); ++i) {
    if (i != 0u) {
      Serial.print(", ");
    }
    Serial.print(shape[i]);
  }
  Serial.println("]");
}

void print_spi_configuration(const AKD1500Options& options) {
  Serial.print(kLogPrefix);
  Serial.print(" spi akida_hz=");
  Serial.print(static_cast<unsigned long>(options.spiClockHz));
  Serial.print(" flash_hz=");
  Serial.println(static_cast<unsigned long>(options.flashSpiClockHz));
}

void send_chunk(const uint8_t* data, size_t size) { Serial.write(data, size); }

void send_camera_config() {
  Serial.write(kPreviewImageMode);
  Serial.write(kPreviewResolution);
}

void print_runtime_controls() {
  Serial.print(kLogPrefix);
  Serial.println(" controls=f free_run_on c clap_mode t toggle_free_run ? help");
}

size_t capture_frame_bytes() {
  const int frame_size = camera().frameSize();
  if (frame_size <= 0) {
    return 0u;
  }
  return static_cast<size_t>(frame_size);
}

void write_u16_le(uint16_t value) {
  Serial.write(static_cast<uint8_t>(value & 0xFFu));
  Serial.write(static_cast<uint8_t>((value >> 8) & 0xFFu));
}

void write_i32_le(int32_t value) {
  const uint32_t raw = static_cast<uint32_t>(value);
  Serial.write(static_cast<uint8_t>(raw & 0xFFu));
  Serial.write(static_cast<uint8_t>((raw >> 8) & 0xFFu));
  Serial.write(static_cast<uint8_t>((raw >> 16) & 0xFFu));
  Serial.write(static_cast<uint8_t>((raw >> 24) & 0xFFu));
}

void send_inference_packet(const InferenceObservation& observation) {
  send_chunk(kInferenceResultSequence, sizeof(kInferenceResultSequence));
  Serial.write(observation.ok ? 1u : 0u);
  Serial.write(static_cast<uint8_t>(observation.predicted_index & 0xFFu));
  Serial.write(static_cast<uint8_t>(observation.status));
  Serial.write(2u);
  write_i32_le(observation.score0);
  write_i32_le(observation.score1);
  write_u16_le(observation.prep_ms);
  write_u16_le(observation.infer_ms);
  write_u16_le(observation.grab_ms);
}

void print_shape3(const uint32_t* shape) {
  Serial.print("[");
  Serial.print(static_cast<unsigned long>(shape[0]));
  Serial.print(", ");
  Serial.print(static_cast<unsigned long>(shape[1]));
  Serial.print(", ");
  Serial.print(static_cast<unsigned long>(shape[2]));
  Serial.println("]");
}

bool shape_matches_metadata(const akida::Shape& runtime_shape,
                            const uint32_t* metadata_shape) {
  if (runtime_shape.size() == 3u) {
    return runtime_shape[0] == metadata_shape[0] &&
           runtime_shape[1] == metadata_shape[1] &&
           runtime_shape[2] == metadata_shape[2];
  }

  if (runtime_shape.size() == 4u && runtime_shape[0] == 1u) {
    return runtime_shape[1] == metadata_shape[0] &&
           runtime_shape[2] == metadata_shape[1] &&
           runtime_shape[3] == metadata_shape[2];
  }

  return false;
}

bool validate_loaded_model_geometry() {
  const AKD1500ModelInfo model_info = flashModelRunner().modelInfo();
  if (!shape_matches_metadata(model_info.inputDimensions, akida_input_shape)) {
    print_failure("model_input_shape_mismatch");
    Serial.print(kLogPrefix);
    Serial.print(" expected input_shape=");
    print_shape3(akida_input_shape);
    Serial.print(kLogPrefix);
    Serial.print(" actual input_shape=");
    print_shape(model_info.inputDimensions);
    return false;
  }

  if (!shape_matches_metadata(model_info.outputDimensions, akida_output_shape)) {
    print_failure("model_output_shape_mismatch");
    Serial.print(kLogPrefix);
    Serial.print(" expected output_shape=");
    print_shape3(akida_output_shape);
    Serial.print(kLogPrefix);
    Serial.print(" actual output_shape=");
    print_shape(model_info.outputDimensions);
    return false;
  }

  return true;
}

// Sample a buffer at a fixed stride so we can cheaply detect "same input over
// and over" without adding a full CRC pass to every frame.
uint32_t sampled_signature(const uint8_t* data, size_t size) {
  if (data == nullptr || size == 0u) {
    return 0u;
  }

  uint32_t hash = 2166136261u;
  constexpr size_t kMaxSamples = 128u;
  const size_t stride = (size / kMaxSamples) + 1u;
  for (size_t i = 0; i < size; i += stride) {
    hash ^= static_cast<uint32_t>(data[i]);
    hash *= 16777619u;
  }
  hash ^= static_cast<uint32_t>(size);
  hash *= 16777619u;
  return hash;
}

uint32_t output_signature(const AKD1500ClassificationResult& result) {
  uint32_t hash = 2166136261u;
  hash ^= static_cast<uint32_t>(result.status);
  hash *= 16777619u;
  hash ^= static_cast<uint32_t>(result.predictedIndex);
  hash *= 16777619u;
  if (result.scores.type == akida::TensorType::int32 &&
      result.scores.elementCount() >= 2u) {
    const int32_t* values = result.data<int32_t>();
    hash ^= static_cast<uint32_t>(values[0]);
    hash *= 16777619u;
    hash ^= static_cast<uint32_t>(values[1]);
    hash *= 16777619u;
  }
  return hash;
}

void track_repeated_signatures(uint32_t frame_signature,
                               uint32_t input_signature,
                               const AKD1500ClassificationResult& result) {
  const uint32_t out_signature = output_signature(result);

  g_same_frame_count =
      (frame_signature == g_last_frame_signature) ? (g_same_frame_count + 1u)
                                                  : 0u;
  g_same_input_count =
      (input_signature == g_last_input_signature) ? (g_same_input_count + 1u)
                                                  : 0u;
  g_same_output_count =
      (out_signature == g_last_output_signature) ? (g_same_output_count + 1u)
                                                 : 0u;
  g_last_frame_signature = frame_signature;
  g_last_input_signature = input_signature;
  g_last_output_signature = out_signature;

  if (g_same_output_count >= 8u && g_same_frame_count >= 8u &&
      !g_reported_stale_camera) {
    Serial.print(kLogPrefix);
    Serial.print(" stale_probe suspect=camera_or_framebuffer");
    Serial.print(" same_output=");
    Serial.print(static_cast<unsigned long>(g_same_output_count + 1u));
    Serial.print(" same_frame=");
    Serial.print(static_cast<unsigned long>(g_same_frame_count + 1u));
    Serial.print(" same_input=");
    Serial.println(static_cast<unsigned long>(g_same_input_count + 1u));
    g_reported_stale_camera = true;
  }

  if (g_same_output_count >= 8u && g_same_frame_count == 0u &&
      !g_reported_stale_output) {
    Serial.print(kLogPrefix);
    Serial.print(" stale_probe suspect=akida_output_reuse");
    Serial.print(" same_output=");
    Serial.print(static_cast<unsigned long>(g_same_output_count + 1u));
    Serial.print(" same_frame=");
    Serial.print(static_cast<unsigned long>(g_same_frame_count + 1u));
    Serial.print(" same_input=");
    Serial.println(static_cast<unsigned long>(g_same_input_count + 1u));
    g_reported_stale_output = true;
  }

  if (g_same_output_count == 0u) {
    g_reported_stale_output = false;
  }
  if (g_same_frame_count == 0u) {
    g_reported_stale_camera = false;
  }
}

bool capture_and_classify(InferenceObservation* observation) {
  if (observation == nullptr) {
    return false;
  }

  if (!board().setAkidaSleepGate(true)) {
    print_failure("akida_sleep_gate_assert", board().lastError());
    return false;
  }

  // Capture a fresh frame from the GC2145 sensor.
  const uint32_t grab_start_ms = millis();
  if (camera().grabFrame(framebuffer(), 3000) != 0) {
    board().setAkidaSleepGate(false);
    print_failure("grab_frame");
    return false;
  }
  const uint32_t grab_ms = millis() - grab_start_ms;
  if (!board().setAkidaSleepGate(false)) {
    print_failure("akida_sleep_gate_release", board().lastError());
    return false;
  }
  const size_t frame_bytes = capture_frame_bytes();
  if (frame_bytes != kCaptureFrameBytes) {
    Serial.print(kLogPrefix);
    Serial.print(" warn capture_frame_bytes expected=");
    Serial.print(static_cast<unsigned long>(kCaptureFrameBytes));
    Serial.print(" actual=");
    Serial.println(static_cast<unsigned long>(frame_bytes));
  }
  const uint32_t frame_signature =
      sampled_signature(framebuffer().getBuffer(), frame_bytes);

  // The sensor captures RGB565 at 160x120. Convert once to grayscale preview,
  // then center-crop the 120x120 square and resize it to 96x96.
  const uint32_t prep_start_ms = millis();
  fill_preview_and_model_input_from_rgb565(framebuffer().getBuffer());
  const uint32_t prep_ms = millis() - prep_start_ms;
  const uint32_t input_signature =
      sampled_signature(g_model_input, kModelInputBytes);

  // Run inference on the AKD1500 and measure the end-to-end runtime.
  const uint32_t infer_start_ms = millis();
  const AKD1500ClassificationResult result =
      flashModelRunner().classifyUint8(g_model_input);
  const uint32_t infer_ms = millis() - infer_start_ms;
  track_repeated_signatures(frame_signature, input_signature, result);

  observation->ok = result.ok();
  observation->status = result.status;
  observation->predicted_index = result.predictedIndex;
  observation->grab_ms = grab_ms > 0xFFFFu ? 0xFFFFu
                                           : static_cast<uint16_t>(grab_ms);
  observation->prep_ms = prep_ms > 0xFFFFu ? 0xFFFFu
                                           : static_cast<uint16_t>(prep_ms);
  observation->infer_ms = infer_ms > 0xFFFFu ? 0xFFFFu
                                             : static_cast<uint16_t>(infer_ms);

  if (result.scores.type == akida::TensorType::int32 &&
      result.scores.elementCount() >= 2u) {
    const int32_t* values = result.data<int32_t>();
    observation->score0 = values[0];
    observation->score1 = values[1];
  } else {
    observation->score0 = 0;
    observation->score1 = 0;
  }

  if (!result.ok()) {
    flashModelRunner().printLastError(Serial);
  }
  return true;
}

void send_frame_with_inference() {
  InferenceObservation observation;
  if (!capture_and_classify(&observation)) {
    return;
  }

  send_chunk(kStartSequence, sizeof(kStartSequence));
  send_chunk(g_preview_frame, kPreviewFrameBytes);
  send_chunk(kStopSequence, sizeof(kStopSequence));
  send_inference_packet(observation);
}

// Precompute source pixel coordinates once so each frame only does lookup and
// conversion instead of repeating the crop math.
void prepare_crop_maps() {
  if (g_crop_maps_ready) {
    return;
  }

  // The sensor runs at 160x120, so center-crop the shorter square region
  // before scaling it down to the 96x96 model input.
  constexpr uint16_t kCropSize =
      (kCameraWidth < kCameraHeight) ? kCameraWidth : kCameraHeight;
  constexpr uint16_t kCropOriginX = (kCameraWidth - kCropSize) / 2u;
  constexpr uint16_t kCropOriginY = (kCameraHeight - kCropSize) / 2u;

  for (uint16_t x = 0; x < kModelWidth; ++x) {
    g_crop_x[x] = kCropOriginX + static_cast<uint16_t>(
                                     (static_cast<uint32_t>(x) * kCropSize) /
                                     kModelWidth);
  }
  for (uint16_t y = 0; y < kModelHeight; ++y) {
    g_crop_y[y] = kCropOriginY + static_cast<uint16_t>(
                                     (static_cast<uint32_t>(y) * kCropSize) /
                                     kModelHeight);
  }

  g_crop_maps_ready = true;
}

// Convert the camera's 160x120 RGB565 frame into grayscale preview pixels,
// then center-crop and resize them into the dense 96x96x1 tensor.
void fill_preview_and_model_input_from_rgb565(const uint8_t* frame_rgb565) {
  prepare_crop_maps();
  for (size_t src_index = 0u, dst_index = 0u; dst_index < kPreviewFrameBytes;
       ++dst_index, src_index += 2u) {
    const uint16_t pixel = (static_cast<uint16_t>(frame_rgb565[src_index]) << 8) |
                           frame_rgb565[src_index + 1u];
    const uint8_t r5 = static_cast<uint8_t>((pixel >> 11) & 0x1Fu);
    const uint8_t g6 = static_cast<uint8_t>((pixel >> 5) & 0x3Fu);
    const uint8_t b5 = static_cast<uint8_t>(pixel & 0x1Fu);
    const uint8_t r8 = static_cast<uint8_t>((r5 * 255u) / 31u);
    const uint8_t g8 = static_cast<uint8_t>((g6 * 255u) / 63u);
    const uint8_t b8 = static_cast<uint8_t>((b5 * 255u) / 31u);
    g_preview_frame[dst_index] = static_cast<uint8_t>(
        (77u * r8 + 150u * g8 + 29u * b8) >> 8);
  }

  uint8_t* dst = g_model_input;
  for (uint16_t y = 0; y < kModelHeight; ++y) {
    const uint16_t src_y = g_crop_y[y];
    for (uint16_t x = 0; x < kModelWidth; ++x) {
      const uint16_t src_x = g_crop_x[x];
      const size_t src_index =
          static_cast<size_t>(src_y) * kCameraWidth + src_x;
      *dst++ = g_preview_frame[src_index];
    }
  }
}

// Describe the model exactly as the public AkidaNicla facade expects it.
AKD1500Model make_model() {
  AKD1500Model model;
  model.serializedProgram = program;
  model.size = static_cast<size_t>(program_len);
  model.storage = AKD1500ModelStorage::ExternalFlash;
  model.externalLocation =
      AkidaNicla::externalModelAddressFromOffset(kFlashModelOffset);
  return model;
}

akd1500_fast_flash::Config make_fast_flash_config() {
  akd1500_fast_flash::Config config;
  config.akidaSpiClockHz = kAkidaSpiClockHz;
  config.flashSpiClockHz = kFlashSpiClockHz;
  config.externalModelOffset = kFlashModelOffset;
  config.expectedIpVersion = 0xBCA10309u;
  #if BB15_FLASH_PROFILE_KNOWN == BB15_FLASH_PROFILE_RENESAS
  config.flashProfilePolicy = akd1500_fast_flash::FlashProfilePolicy::Renesas6B;
  #else
  config.flashProfilePolicy = akd1500_fast_flash::FlashProfilePolicy::Auto;
  #endif
  // Keep the default path fast. The extra flash probe duplicates the runtime
  // profile handshake and adds about one second on this board stack.
  config.verboseRuntimeDiagnostics = false;
  return config;
}

// Bring the camera online with the fixed demo capture settings.
bool begin_camera() {
  if (camera().begin(kResolution, kCameraImageMode, kFrameRate)) {
    g_camera_ok = true;
    return true;
  }
  return false;
}

void on_pdm_data() {
  const int bytes_available = PDM.available();
  if (bytes_available <= 0) {
    return;
  }

  size_t bytes_to_read = static_cast<size_t>(bytes_available);
  if (bytes_to_read > sizeof(g_mic_capture_samples)) {
    bytes_to_read = sizeof(g_mic_capture_samples);
  }

  const int bytes_read = PDM.read(g_mic_capture_samples, bytes_to_read);
  if (bytes_read > 0) {
    g_mic_samples_read = static_cast<uint16_t>(bytes_read / sizeof(int16_t));
  }
}

bool begin_microphone() {
  PDM.onReceive(on_pdm_data);
  PDM.setBufferSize(sizeof(g_mic_capture_samples));
  PDM.setGain(kMicGain);
  return PDM.begin(kMicChannels, kMicSampleRateHz) == 1;
}

bool consume_audio_observation(AudioObservation* observation) {
  if (observation == nullptr) {
    return false;
  }

  uint16_t sample_count = 0u;
  noInterrupts();
  sample_count = g_mic_samples_read;
  if (sample_count > kMicBufferSamples) {
    sample_count = static_cast<uint16_t>(kMicBufferSamples);
  }
  if (sample_count > 0u) {
    memcpy(g_mic_processing_samples, g_mic_capture_samples,
           static_cast<size_t>(sample_count) * sizeof(int16_t));
    g_mic_samples_read = 0u;
  }
  interrupts();

  if (sample_count == 0u) {
    return false;
  }

  uint32_t sum_abs = 0u;
  uint16_t peak = 0u;
  for (uint16_t i = 0u; i < sample_count; ++i) {
    const int32_t sample = static_cast<int32_t>(g_mic_processing_samples[i]);
    const uint32_t magnitude =
        static_cast<uint32_t>(sample < 0 ? -sample : sample);
    if (magnitude > peak) {
      peak = static_cast<uint16_t>(magnitude);
    }
    sum_abs += magnitude;
  }

  observation->valid = true;
  observation->peak = peak;
  observation->sample_count = sample_count;
  observation->mean_abs = sample_count == 0u
                              ? 0u
                              : static_cast<uint16_t>(sum_abs / sample_count);
  return true;
}

bool clap_detected(AudioObservation* observation) {
  AudioObservation local;
  if (!consume_audio_observation(&local)) {
    return false;
  }

  if (observation != nullptr) {
    *observation = local;
  }

  const uint32_t now_ms = millis();
  if ((now_ms - g_last_clap_ms) < kClapCooldownMs) {
    return false;
  }

  if (local.peak < kClapPeakThreshold ||
      local.mean_abs < kClapMeanAbsThreshold) {
    return false;
  }

  g_last_clap_ms = now_ms;
  ++g_clap_trigger_count;
  Serial.print(kLogPrefix);
  Serial.print(" clap_detect result=PASS trigger_index=");
  Serial.print(static_cast<unsigned long>(g_clap_trigger_count));
  Serial.print(" peak=");
  Serial.print(static_cast<unsigned long>(local.peak));
  Serial.print(" mean_abs=");
  Serial.print(static_cast<unsigned long>(local.mean_abs));
  Serial.print(" samples=");
  Serial.println(static_cast<unsigned long>(local.sample_count));
  return true;
}

// Assume the model is already staged in external flash, then link the Akida
// runtime and attempt to load it for repeated inference.
bool prepare_model() {
  const uint32_t flash_address =
      AkidaNicla::externalModelAddressFromOffset(kFlashModelOffset);

  if (static_cast<int64_t>(program_len) != akida_program_length_bytes) {
    print_failure("program_length_mismatch");
    return false;
  }

  Serial.print(kLogPrefix);
  Serial.print(" flash_slot address=0x");
  Serial.println(flash_address, HEX);
  Serial.print(kLogPrefix);
  Serial.println(" flash_verify result=SKIP source=unchecked");

  if (flashModelRunner().load(make_model()) != AKD1500Status::Ok) {
    print_failure("load_external_model");
    return false;
  }

  flashModelRunner().printSessionSummary(Serial, kLogPrefix);
  Serial.print(kLogPrefix);
  Serial.print(" akida ip_version=0x");
  Serial.println(flashModelRunner().ipVersion(), HEX);

  if (!validate_loaded_model_geometry()) {
    return false;
  }

  g_model_ok = true;
  g_akida_awake = true;
  clear_failure();
  Serial.print(kLogPrefix);
  Serial.print(" model_info ");
  flashModelRunner().printModelInfo(Serial);
  return true;
}

// One full demo iteration: capture, preprocess, classify, and print.
bool run_one_inference() {
  if (!g_camera_ok || !g_model_ok) {
    return false;
  }
  InferenceObservation observation;
  if (!capture_and_classify(&observation)) {
    delay(kLoopDelayMs);
    return false;
  }

  // Emit a compact terminal-friendly result line for screenshots and docs.
  Serial.print(kLogPrefix);
  Serial.print(" inference status=");
  Serial.print(AkidaNicla::statusName(observation.status));
  Serial.print(" predicted_index=");
  Serial.print(static_cast<unsigned long>(observation.predicted_index));
  Serial.print(" label=");
  if (observation.predicted_index == 0u) {
    Serial.print("no_presence");
  } else if (observation.predicted_index == 1u) {
    Serial.print("presence");
  } else {
    Serial.print("class_");
    Serial.print(static_cast<unsigned long>(observation.predicted_index));
  }
  Serial.print(" scores=[");
  Serial.print(observation.score0);
  Serial.print(", ");
  Serial.print(observation.score1);
  Serial.print("]");
  Serial.print(" grab_ms=");
  Serial.print(static_cast<unsigned long>(observation.grab_ms));
  Serial.print(" prep_ms=");
  Serial.print(static_cast<unsigned long>(observation.prep_ms));
  Serial.print(" infer_ms=");
  Serial.println(static_cast<unsigned long>(observation.infer_ms));
  return true;
}

bool hold_akida_off(const char* reason) {
  g_model_ok = false;
  g_akida_awake = false;
  g_burst_inferences_remaining = 0u;
  if (!board().holdAkidaInReset()) {
    print_failure("hold_akida_in_reset", board().lastError());
    return false;
  }
  Serial.print(kLogPrefix);
  Serial.print(" akida_state=reset_asserted reason=");
  Serial.println(reason == nullptr ? "unspecified" : reason);
  return true;
}

bool power_cycle_akida_off(const char* reason, uint32_t hold_ms) {
  if (!hold_akida_off(reason)) {
    return false;
  }
  delay(hold_ms);
  return true;
}

bool set_free_running_mode(bool enabled, const char* source) {
  if (g_free_running_enabled == enabled) {
    Serial.print(kLogPrefix);
    Serial.print(" run_mode unchanged=");
    Serial.print(enabled ? "free_running" : "clap_triggered");
    Serial.print(" source=");
    Serial.println(source == nullptr ? "unspecified" : source);
    return true;
  }

  g_free_running_enabled = enabled;
  g_burst_inferences_remaining = 0u;

  Serial.print(kLogPrefix);
  Serial.print(" run_mode=");
  Serial.print(enabled ? "free_running" : "clap_triggered");
  Serial.print(" source=");
  Serial.println(source == nullptr ? "unspecified" : source);

  if (enabled) {
    if (!wake_akida_for_inference("free_run_enable")) {
      return false;
    }
  } else if (!g_preview_session_active) {
    if (!hold_akida_off("free_run_disabled")) {
      return false;
    }
  }

  return true;
}

bool wake_akida_for_inference(const char* reason) {
  if (g_model_ok && g_akida_awake) {
    return true;
  }

  for (uint8_t attempt = 1u; attempt <= kAkidaWakeRetryLimit; ++attempt) {
    Serial.print(kLogPrefix);
    Serial.print(" akida_wake reason=");
    Serial.print(reason == nullptr ? "unspecified" : reason);
    Serial.print(" attempt=");
    Serial.print(static_cast<unsigned long>(attempt));
    Serial.print("/");
    Serial.print(static_cast<unsigned long>(kAkidaWakeRetryLimit));
    Serial.print(" hold_reset_ms=");
    Serial.println(static_cast<unsigned long>(kAkidaColdResetHoldMs));

    if (!board().coldBootAkidaIntoExternalFlashMode(kAkidaColdResetHoldMs)) {
      print_failure("board_cold_boot", board().lastError());
    } else {
      delay(kAkidaReloadSettleMs);
      const uint32_t reload_start_ms = millis();
      if (prepare_model()) {
        Serial.print(kLogPrefix);
        Serial.print(" akida_wake result=PASS load_ms=");
        Serial.print(static_cast<unsigned long>(millis() - reload_start_ms));
        Serial.print(" attempt=");
        Serial.println(static_cast<unsigned long>(attempt));
        return true;
      }
    }

    Serial.print(kLogPrefix);
    Serial.print(" akida_wake result=RETRY attempt=");
    Serial.print(static_cast<unsigned long>(attempt));
    Serial.print(" stage=");
    Serial.print(g_last_failure_stage == nullptr ? "unknown"
                                                 : g_last_failure_stage);
    if (g_last_failure_detail != nullptr) {
      Serial.print(" detail=");
      Serial.print(g_last_failure_detail);
    }
    if (flashModelRunner().lastError().status != AKD1500Status::Ok) {
      Serial.print(" akida=");
      flashModelRunner().printLastError(Serial);
    } else {
      Serial.println();
    }

    if (attempt < kAkidaWakeRetryLimit &&
        !power_cycle_akida_off("wake_retry_power_cycle",
                               kAkidaColdResetHoldMs)) {
      return false;
    }
  }

  power_cycle_akida_off("wake_retry_exhausted", kAkidaColdResetHoldMs);
  return false;
}

}  // namespace

void setup() {
  const uint32_t setup_start_ms = millis();
  // Put the status LEDs in a known state before touching external hardware.
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(LEDR, OUTPUT);
  set_led(false);
  digitalWrite(LEDR, HIGH);

  // Bring up USB serial first so boot progress is visible in the terminal.
  Serial.begin(kSerialBaud);
  const uint32_t serial_wait_start_ms = millis();
  wait_for_serial();
  const uint32_t serial_wait_ms = millis() - serial_wait_start_ms;
  const uint32_t boot_settle_start_ms = millis();
  delay(kBootSettleMs);
  const uint32_t boot_settle_ms = millis() - boot_settle_start_ms;

  // Print a short boot banner that explains the demo and the attached board.
  Serial.println();
  Serial.println(kSketchName);
  Serial.print(kLogPrefix);
  Serial.println(" sensor=GC2145");
  Serial.print(kLogPrefix);
  Serial.println(" mode=clap-triggered burst inference");
  Serial.print(kLogPrefix);
  Serial.print(" model=");
  Serial.println(kBundledModelName);
  board().printSummary(Serial);
  print_spi_configuration(flashModelRunner().options());
  Serial.print(kLogPrefix);
  Serial.print(" export program_len=");
  Serial.println(static_cast<long>(akida_program_length_bytes));
  Serial.print(kLogPrefix);
  Serial.print(" export input_shape=");
  print_shape3(akida_input_shape);
  Serial.print(kLogPrefix);
  Serial.print(" export output_shape=");
  print_shape3(akida_output_shape);

  // Configure the BB15 expander and reset sequence needed for AKD1500 access.
  const uint32_t board_start_ms = millis();
  if (!board().begin()) {
    print_failure("board_begin", board().lastError());
    blink_forever();
  }
  const uint32_t board_ms = millis() - board_start_ms;
  Serial.print(kLogPrefix);
  Serial.println(" board_setup result=PASS");

  // Start the camera after the board-side wiring and reset state are settled.
  const uint32_t camera_start_ms = millis();
  if (!begin_camera()) {
    print_failure("camera_begin");
    blink_forever();
  }
  const uint32_t camera_ms = millis() - camera_start_ms;
  Serial.print(kLogPrefix);
  Serial.println(" camera result=PASS");
  Serial.print(kLogPrefix);
  Serial.print(" camera_geometry width=");
  Serial.print(static_cast<unsigned long>(camera().getResolutionWidth()));
  Serial.print(" height=");
  Serial.print(static_cast<unsigned long>(camera().getResolutionHeight()));
  Serial.print(" capture_bytes=");
  Serial.print(static_cast<unsigned long>(capture_frame_bytes()));
  Serial.print(" preview_bytes=");
  Serial.println(static_cast<unsigned long>(kPreviewFrameBytes));

  const uint32_t microphone_start_ms = millis();
  if (!begin_microphone()) {
    print_failure("microphone_begin");
    blink_forever();
  }
  const uint32_t microphone_ms = millis() - microphone_start_ms;
  Serial.print(kLogPrefix);
  Serial.println(" microphone result=PASS");

  const uint32_t akida_sleep_start_ms = millis();
  if (!hold_akida_off("startup_idle")) {
    blink_forever();
  }
  const uint32_t akida_sleep_ms = millis() - akida_sleep_start_ms;
  Serial.print(kLogPrefix);
  Serial.println(" akida result=SLEEPING");

  Serial.print(kLogPrefix);
  Serial.println(
      " preprocess=rgb565_to_gray160x120 preview_native center_crop_120 resize_nn_96");
  Serial.print(kLogPrefix);
  Serial.println(
      " loop=listen -> clap -> wake -> 5x infer -> sleep or free_run or preview_stream");
  Serial.print(kLogPrefix);
  Serial.print(" startup_run_mode=");
  Serial.println(g_free_running_enabled ? "free_running" : "clap_triggered");
  Serial.print(kLogPrefix);
  Serial.print(" clap_policy peak_threshold=");
  Serial.print(static_cast<unsigned long>(kClapPeakThreshold));
  Serial.print(" mean_abs_threshold=");
  Serial.print(static_cast<unsigned long>(kClapMeanAbsThreshold));
  Serial.print(" cooldown_ms=");
  Serial.println(static_cast<unsigned long>(kClapCooldownMs));
  Serial.print(kLogPrefix);
  Serial.print(" burst_policy inferences_per_clap=");
  Serial.print(static_cast<unsigned long>(kBurstInferencesPerClap));
  Serial.print(" cold_reset_hold_ms=");
  Serial.println(static_cast<unsigned long>(kAkidaColdResetHoldMs));
  Serial.print(kLogPrefix);
  Serial.print(" microphone sample_rate_hz=");
  Serial.print(static_cast<unsigned long>(kMicSampleRateHz));
  Serial.print(" gain=");
  Serial.print(static_cast<unsigned long>(kMicGain));
  Serial.print(" samples_per_chunk=");
  Serial.println(static_cast<unsigned long>(kMicBufferSamples));
  Serial.print(kLogPrefix);
  Serial.print(" startup_timing serial_wait_ms=");
  Serial.print(static_cast<unsigned long>(serial_wait_ms));
  Serial.print(" boot_settle_ms=");
  Serial.print(static_cast<unsigned long>(boot_settle_ms));
  Serial.print(" board_ms=");
  Serial.print(static_cast<unsigned long>(board_ms));
  Serial.print(" camera_ms=");
  Serial.print(static_cast<unsigned long>(camera_ms));
  Serial.print(" microphone_ms=");
  Serial.print(static_cast<unsigned long>(microphone_ms));
  Serial.print(" akida_sleep_ms=");
  Serial.print(static_cast<unsigned long>(akida_sleep_ms));
  Serial.print(" setup_total_ms=");
  Serial.println(static_cast<unsigned long>(millis() - setup_start_ms));
  print_runtime_controls();

  if (g_free_running_enabled) {
    if (!wake_akida_for_inference("startup_free_run")) {
      halt_bb15_failed("akida_wake_retry_exhausted", g_last_failure_stage);
    }
  }
}

void loop() {
  // Re-arm the USB CDC port if a host connects after setup completed, but do
  // not block the application on the core's notion of "serial connected".
  if (!Serial) {
    Serial.begin(kSerialBaud);
  }

  if (Serial.available() > 0) {
    const int request = Serial.read();
    if (request == '\r' || request == '\n') {
      return;
    }
    if (request == 'f' || request == 'F') {
      if (!set_free_running_mode(true, "serial")) {
        halt_bb15_failed("run_mode_switch_failed", g_last_failure_stage);
      }
      return;
    }
    if (request == 'c' || request == 'C') {
      if (!set_free_running_mode(false, "serial")) {
        halt_bb15_failed("run_mode_switch_failed", g_last_failure_stage);
      }
      return;
    }
    if (request == 't' || request == 'T') {
      if (!set_free_running_mode(!g_free_running_enabled, "serial_toggle")) {
        halt_bb15_failed("run_mode_switch_failed", g_last_failure_stage);
      }
      return;
    }
    if (request == '?' || request == 'h' || request == 'H') {
      print_runtime_controls();
      return;
    }

    if (request == kRequestStreamStart) {
      g_preview_session_active = true;
      g_preview_streaming_active = true;
      g_last_preview_request_ms = millis();
      return;
    }
    if (request == kRequestStreamStop) {
      g_preview_streaming_active = false;
      g_preview_session_active = false;
      if (!g_free_running_enabled && g_burst_inferences_remaining == 0u &&
          !hold_akida_off("preview_stream_stop")) {
        blink_forever();
      }
      return;
    }

    if (request != kRequestFrame && request != kRequestConfig) {
      Serial.print(kLogPrefix);
      Serial.print(" control_ignored byte=0x");
      if (request < 0x10) {
        Serial.print('0');
      }
      Serial.println(static_cast<unsigned long>(
                         static_cast<uint8_t>(request & 0xFF)),
                     HEX);
      return;
    }

    g_preview_session_active = true;
    g_last_preview_request_ms = millis();
    set_led(true);
    if (request == kRequestFrame) {
      if (!wake_akida_for_inference("serial_preview")) {
        halt_bb15_failed("akida_wake_retry_exhausted", g_last_failure_stage);
      }
      send_frame_with_inference();
    } else if (request == kRequestConfig) {
      send_camera_config();
    }
    set_led(false);
    return;
  }

  if (g_preview_streaming_active) {
    if (!Serial) {
      g_preview_streaming_active = false;
      g_preview_session_active = false;
      if (!g_free_running_enabled && g_burst_inferences_remaining == 0u &&
          !hold_akida_off("preview_stream_serial_lost")) {
        blink_forever();
      }
      delay(kLoopDelayMs);
      return;
    }
    if (!wake_akida_for_inference("serial_preview_stream")) {
      halt_bb15_failed("akida_wake_retry_exhausted", g_last_failure_stage);
    }
    send_frame_with_inference();
    return;
  }

  if (g_preview_session_active) {
    if ((millis() - g_last_preview_request_ms) < kPreviewSessionHoldMs) {
      delay(1);
      return;
    }
    g_preview_session_active = false;
    if (!g_free_running_enabled && g_burst_inferences_remaining == 0u &&
        !hold_akida_off("preview_idle")) {
      blink_forever();
    }
  }

  if (g_free_running_enabled) {
    if (!g_akida_awake || !g_model_ok) {
      if (!wake_akida_for_inference("free_run_loop")) {
        halt_bb15_failed("akida_wake_retry_exhausted", g_last_failure_stage);
      }
    }
    set_led(true);
    const bool inference_ok = run_one_inference();
    set_led(false);
    if (!inference_ok) {
      blink_forever();
    }
    delay(kLoopDelayMs);
    return;
  }

  if (g_burst_inferences_remaining == 0u) {
    AudioObservation audio;
    if (clap_detected(&audio)) {
      if (!wake_akida_for_inference("clap_trigger")) {
        halt_bb15_failed("akida_wake_retry_exhausted", g_last_failure_stage);
      }
      g_burst_inferences_remaining = kBurstInferencesPerClap;
      Serial.print(kLogPrefix);
      Serial.print(" burst_start trigger_index=");
      Serial.print(static_cast<unsigned long>(g_clap_trigger_count));
      Serial.print(" remaining=");
      Serial.println(static_cast<unsigned long>(g_burst_inferences_remaining));
    }
    delay(kLoopDelayMs);
    return;
  }

  set_led(true);
  const bool inference_ok = run_one_inference();
  set_led(false);
  if (!inference_ok) {
    blink_forever();
  }

  if (g_burst_inferences_remaining > 0u) {
    --g_burst_inferences_remaining;
  }
  if (g_burst_inferences_remaining == 0u) {
    Serial.print(kLogPrefix);
    Serial.println(" burst_complete result=PASS");
    if (!hold_akida_off("burst_complete")) {
      blink_forever();
    }
  }

  delay(kLoopDelayMs);
}

#else

namespace {

constexpr uint32_t kSerialBaud = 115200u;
constexpr uint32_t kSerialWaitMs = 1500u;

void wait_for_serial() {
  const uint32_t start_ms = millis();
  while (!Serial && (millis() - start_ms) < kSerialWaitMs) {
  }
}

}  // namespace

void setup() {
  Serial.begin(kSerialBaud);
  wait_for_serial();
  Serial.println(
      "akd1500_infer_fast: runtime support requires Nicla Vision + BB15");
}

void loop() {}

#endif
