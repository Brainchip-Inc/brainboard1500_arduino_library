#include <Arduino.h>

#include <AKD1500.h>

#include "bb15_demo_board.h"
#include "camera.h"
#include "gc2145.h"
#include "program.h"

#ifndef ARDUINO_NICLA_VISION
#error "This example is intended for Nicla Vision."
#endif

namespace {

// Serial and application timing for a readable terminal demo.
constexpr uint32_t kSerialBaud = 115200u;
constexpr uint32_t kSerialWaitMs = 3000u;
constexpr uint32_t kBootSettleMs = 250u;
// Keep this minimal so the demo runs quickly without starving USB serial on
// the Nicla Vision mbed core.
constexpr uint32_t kLoopDelayMs = 1u;
constexpr uint16_t kChunkSize = 512u;
constexpr uint8_t kRequestFrame = 1u;
constexpr uint8_t kRequestConfig = 2u;
constexpr uint8_t kResolution = CAMERA_R320x240;
constexpr uint8_t kImageMode = CAMERA_RGB565;
constexpr uint16_t kCameraWidth = 320u;
constexpr uint16_t kCameraHeight = 240u;
constexpr size_t kCameraFrameBytes =
    static_cast<size_t>(kCameraWidth) * kCameraHeight * 2u;
constexpr int32_t kFrameRate = 30;
constexpr uint16_t kModelWidth = 224u;
constexpr uint16_t kModelHeight = 224u;
constexpr uint16_t kModelChannels = 3u;
constexpr size_t kModelInputBytes =
    static_cast<size_t>(kModelWidth) * kModelHeight * kModelChannels;
constexpr uint32_t kFlashModelOffset = 0u;
// Expose the Akida and bridge-flash SPI clocks as sketch-level knobs so the
// demo can be tuned directly from the Arduino IDE without editing the library.
constexpr uint32_t kAkidaSpiClockHz = 10000000u;
constexpr uint32_t kFlashSpiClockHz = 2000000u;
constexpr uint8_t kInferenceResultSequence[4] = {0xAC, 0x1D, 0x1A, 0xDA};
constexpr uint8_t kStartSequence[4] = {0xFA, 0xCE, 0xFE, 0xED};
constexpr uint8_t kStopSequence[4] = {0xDA, 0xBB, 0xAD, 0x00};
constexpr uint32_t kPreviewSessionHoldMs = 2000u;
constexpr const char* kSketchName = "NiclaVisionCameraFlashClassify";

// Application state shared across setup and the main inference loop.
bool g_camera_ok = false;
bool g_model_ok = false;
uint8_t g_model_input[kModelInputBytes];
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

void prepare_crop_maps();
void fill_model_input_from_rgb565_center_crop(const uint8_t* frame_rgb565);

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

AkidaNicla& niclaAkida() {
  static AkidaNicla instance;
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

// Halt in a visible way after a fatal setup failure so bench debugging only
// needs a terminal and the status LED.
void blink_forever() {
  for (;;) {
    set_led(true);
    delay(50);
    set_led(false);
    delay(950);
    if (g_last_failure_stage != nullptr) {
      Serial.print("[camera_flash_demo] halted stage=");
      Serial.print(g_last_failure_stage);
      if (g_last_failure_detail != nullptr) {
        Serial.print(" detail=");
        Serial.print(g_last_failure_detail);
      }
      if (niclaAkida().lastError().status != AKD1500Status::Ok) {
        Serial.print(" akida=");
        niclaAkida().printLastError(Serial);
      } else {
        Serial.println();
      }
    }
  }
}

// Print and remember the first fatal step that failed during setup or runtime.
void print_failure(const char* stage, const char* detail = nullptr) {
  g_last_failure_stage = stage;
  g_last_failure_detail = detail;
  Serial.print("[camera_flash_demo] result=FAIL stage=");
  Serial.print(stage);
  if (detail != nullptr) {
    Serial.print(" detail=");
    Serial.print(detail);
  }
  if (niclaAkida().lastError().status != AKD1500Status::Ok) {
    Serial.print(" akida=");
    niclaAkida().printLastError(Serial);
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
  Serial.print("[camera_flash_demo] spi akida_hz=");
  Serial.print(static_cast<unsigned long>(options.spiClockHz));
  Serial.print(" flash_hz=");
  Serial.println(static_cast<unsigned long>(options.flashSpiClockHz));
}

// This demo model produces two int32 class scores, so show them when present.
void print_scores(const AKD1500ClassificationResult& result) {
  if (result.scores.type == akida::TensorType::int32 &&
      result.scores.elementCount() >= 2u) {
    const int32_t* values = result.data<int32_t>();
    Serial.print(" scores=[");
    Serial.print(values[0]);
    Serial.print(", ");
    Serial.print(values[1]);
    Serial.print("]");
  }
}

void send_chunk(const uint8_t* data, size_t size) { Serial.write(data, size); }

void send_camera_config() {
  Serial.write(kImageMode);
  Serial.write(kResolution);
  Serial.flush();
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
  Serial.flush();
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
    Serial.print("[camera_flash_demo] stale_probe suspect=camera_or_framebuffer");
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
    Serial.print("[camera_flash_demo] stale_probe suspect=akida_output_reuse");
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

  // Capture a fresh frame from the GC2145 sensor.
  const uint32_t grab_start_ms = millis();
  if (camera().grabFrame(framebuffer(), 3000) != 0) {
    print_failure("grab_frame");
    return false;
  }
  const uint32_t grab_ms = millis() - grab_start_ms;
  const uint32_t frame_signature =
      sampled_signature(framebuffer().getBuffer(), kCameraFrameBytes);

  // Convert the frame into the exact byte layout the model expects.
  const uint32_t prep_start_ms = millis();
  fill_model_input_from_rgb565_center_crop(framebuffer().getBuffer());
  const uint32_t prep_ms = millis() - prep_start_ms;
  const uint32_t input_signature =
      sampled_signature(g_model_input, kModelInputBytes);

  // Run inference on the AKD1500 and measure the end-to-end runtime.
  const uint32_t infer_start_ms = millis();
  const AKD1500ClassificationResult result =
      niclaAkida().classifyUint8(g_model_input);
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
    niclaAkida().printLastError(Serial);
  }
  return true;
}

void send_frame_with_inference() {
  InferenceObservation observation;
  if (!capture_and_classify(&observation)) {
    return;
  }

  const uint8_t* buffer = framebuffer().getBuffer();
  const size_t buffer_size = kCameraFrameBytes;
  send_chunk(kStartSequence, sizeof(kStartSequence));
  for (size_t offset = 0; offset < buffer_size; offset += kChunkSize) {
    const size_t chunk_size =
        min(static_cast<size_t>(kChunkSize), buffer_size - offset);
    send_chunk(buffer + offset, chunk_size);
  }
  send_chunk(kStopSequence, sizeof(kStopSequence));
  send_inference_packet(observation);
}

// Precompute source pixel coordinates once so each frame only does lookup and
// conversion instead of repeating the crop math.
void prepare_crop_maps() {
  if (g_crop_maps_ready) {
    return;
  }

  // The sensor runs at 320x240, so center-crop the shorter square region
  // before scaling it down to the 224x224 model input.
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

// Convert the camera's RGB565 frame into the model's RGB888 byte tensor while
// applying the precomputed center crop and resize sampling.
void fill_model_input_from_rgb565_center_crop(const uint8_t* frame_rgb565) {
  prepare_crop_maps();

  uint8_t* dst = g_model_input;
  for (uint16_t y = 0; y < kModelHeight; ++y) {
    const uint16_t src_y = g_crop_y[y];
    for (uint16_t x = 0; x < kModelWidth; ++x) {
      const uint16_t src_x = g_crop_x[x];
      const size_t src_index =
          (static_cast<size_t>(src_y) * kCameraWidth + src_x) * 2u;
      const uint16_t value =
          (static_cast<uint16_t>(frame_rgb565[src_index]) << 8) |
          frame_rgb565[src_index + 1u];
      const uint8_t r5 = static_cast<uint8_t>((value >> 11) & 0x1Fu);
      const uint8_t g6 = static_cast<uint8_t>((value >> 5) & 0x3Fu);
      const uint8_t b5 = static_cast<uint8_t>(value & 0x1Fu);
      dst[0] = static_cast<uint8_t>((r5 << 3) | (r5 >> 2));
      dst[1] = static_cast<uint8_t>((g6 << 2) | (g6 >> 4));
      dst[2] = static_cast<uint8_t>((b5 << 3) | (b5 >> 2));
      dst += 3;
    }
  }
}

// Start from the Nicla Vision defaults and pin the external model location for
// this example's flash-staged model.
AKD1500Options make_options() {
  AKD1500Options options = AKD1500Options::niclaVisionDefaults();
  options.spiClockHz = kAkidaSpiClockHz;
  options.flashSpiClockHz = kFlashSpiClockHz;
  options.externalModelAddress =
      AkidaNicla::externalModelAddressFromOffset(kFlashModelOffset);
  return options;
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

// Bring the camera online with the fixed demo capture settings.
bool begin_camera() {
  if (camera().begin(kResolution, kImageMode, kFrameRate)) {
    g_camera_ok = true;
    return true;
  }
  return false;
}

// Stage the bundled model into external flash, verify the bytes, then link the
// Akida runtime and load the model for repeated inference.
bool prepare_model() {
  const AKD1500Options options = make_options();
  const uint32_t flash_address =
      AkidaNicla::externalModelAddressFromOffset(kFlashModelOffset);

  // Step 1: copy only the executable program payload into bridge flash.
  Serial.print("[camera_flash_demo] flash_stage address=0x");
  Serial.println(flash_address, HEX);
  if (!AkidaNicla::stageModelToFlash(options, program,
                                     static_cast<size_t>(program_len),
                                     kFlashModelOffset)) {
    print_failure("stage_model_to_flash");
    return false;
  }
  Serial.println("[camera_flash_demo] flash_stage result=PASS");

  // Step 2: read the staged bytes back through the same bridge path.
  if (!AkidaNicla::verifyModelInFlash(options, program,
                                      static_cast<size_t>(program_len),
                                      kFlashModelOffset)) {
    print_failure("verify_model_in_flash");
    return false;
  }
  Serial.println("[camera_flash_demo] flash_verify result=PASS");

  // Step 3: establish the AKD1500 link using the supported public defaults.
  if (niclaAkida().begin(options) != AKD1500Status::Ok) {
    print_failure("akida_begin");
    return false;
  }

  Serial.print("[camera_flash_demo] akida ip_version=0x");
  Serial.println(niclaAkida().ipVersion(), HEX);

  // Step 4: load the previously staged external-flash model into the runtime.
  if (niclaAkida().load(make_model()) != AKD1500Status::Ok) {
    print_failure("load_external_model");
    return false;
  }

  g_model_ok = true;
  Serial.print("[camera_flash_demo] model_info ");
  niclaAkida().printModelInfo(Serial);
  return true;
}

// One full demo iteration: capture, preprocess, classify, and print.
void run_one_inference() {
  if (!g_camera_ok || !g_model_ok) {
    return;
  }
  InferenceObservation observation;
  if (!capture_and_classify(&observation)) {
    delay(kLoopDelayMs);
    return;
  }

  // Emit a compact terminal-friendly result line for screenshots and docs.
  Serial.print("[camera_flash_demo] inference status=");
  Serial.print(AkidaNicla::statusName(observation.status));
  Serial.print(" predicted_index=");
  Serial.print(static_cast<unsigned long>(observation.predicted_index));
  Serial.print(" label=");
  if (observation.predicted_index == 0u) {
    Serial.print("no_human");
  } else if (observation.predicted_index == 1u) {
    Serial.print("human");
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
}

}  // namespace

void setup() {
  // Put the status LEDs in a known state before touching external hardware.
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(LEDR, OUTPUT);
  set_led(false);
  digitalWrite(LEDR, HIGH);

  // Bring up USB serial first so boot progress is visible in the terminal.
  Serial.begin(kSerialBaud);
  wait_for_serial();
  delay(kBootSettleMs);

  // Print a short boot banner that explains the demo and the attached board.
  Serial.println();
  Serial.println(kSketchName);
  Serial.println("[camera_flash_demo] sensor=GC2145");
  Serial.println("[camera_flash_demo] mode=serial terminal classification");
  board().printSummary(Serial);
  print_spi_configuration(make_options());

  // Configure the BB15 expander and reset sequence needed for AKD1500 access.
  if (!board().begin()) {
    print_failure("board_begin", board().lastError());
    blink_forever();
  }
  Serial.println("[camera_flash_demo] board_setup result=PASS");

  // Start the camera after the board-side wiring and reset state are settled.
  if (!begin_camera()) {
    print_failure("camera_begin");
    blink_forever();
  }
  Serial.println("[camera_flash_demo] camera result=PASS");

  // Stage, verify, link, and load the external-flash model.
  if (!prepare_model()) {
    blink_forever();
  }
  Serial.println("[camera_flash_demo] akida result=PASS");

  // Show the discovered input/output geometry so users can reason about the
  // preprocessing and output labels without opening the model blob.
  Serial.print("[camera_flash_demo] model input_shape=");
  print_shape(niclaAkida().modelInfo().inputDimensions);
  Serial.print("[camera_flash_demo] model output_shape=");
  print_shape(niclaAkida().modelInfo().outputDimensions);
  Serial.println("[camera_flash_demo] loop=grab -> preprocess -> classify -> print");
}

void loop() {
  // Re-arm the USB CDC port if a host connects after setup completed, but do
  // not block the application on the core's notion of "serial connected".
  if (!Serial) {
    Serial.begin(kSerialBaud);
  }

  if (Serial.available() > 0) {
    const int request = Serial.read();
    g_preview_session_active = true;
    g_last_preview_request_ms = millis();
    set_led(true);
    if (request == kRequestFrame) {
      send_frame_with_inference();
    } else if (request == kRequestConfig) {
      send_camera_config();
    }
    set_led(false);
    return;
  }

  if (g_preview_session_active) {
    if ((millis() - g_last_preview_request_ms) < kPreviewSessionHoldMs) {
      delay(1);
      return;
    }
    g_preview_session_active = false;
  }

  // Blink while work is active, then idle for a readable terminal cadence.
  set_led(true);
  run_one_inference();
  set_led(false);
  delay(kLoopDelayMs);
}
