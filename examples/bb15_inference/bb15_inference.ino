#include <Arduino.h>

#if defined(TARGET_NICLA_VISION) || defined(TARGET_NICLA)

#include <BB15.h>
#if defined(TARGET_NICLA) && !defined(TARGET_NICLA_VISION)
#include <Nicla_System.h>
#endif
#if defined(TARGET_NICLA_VISION)
#include "camera.h"
#include "gc2145.h"
#endif

#include "model_metadata.h"
#include "program.h"

namespace {

// This example is intentionally board-generic at the API level:
// - users define the BB15 pinout once
// - the same `BB15` / `BB15Runner` flow is used afterwards
// - host-specific capabilities, such as a validated interrupt path, stay in
//   small conditional helper sections instead of taking over the main flow

constexpr uint32_t kSerialBaud = 115200u;
constexpr uint32_t kSerialWaitMs = 3000u;
constexpr uint32_t kBootSettleMs = 250u;
constexpr uint32_t kHeartbeatToggleMs = 500u;
constexpr uint32_t kFlashModelOffset = 0u;
constexpr uint32_t kAkidaSpiClockHz = 25000000u;
constexpr uint32_t kFlashSpiClockHz = 2000000u;
constexpr uint16_t kModelWidth = 96u;
constexpr uint16_t kModelHeight = 96u;
constexpr uint16_t kModelChannels = 1u;
// Validated completion route on this BB15 stack:
// AKD GPIO3 -> expander P2 -> expander INT -> host interrupt pin.
constexpr uint8_t kExpanderInterruptPin = 2u;
constexpr uint8_t kExpanderSleepPin = 1u;
constexpr uint8_t kAkidaInterruptPadIndex = 3u;
constexpr uint8_t kExpanderRegDirection = 0x03u;
constexpr uint8_t kExpanderRegInputDefaultState = 0x09u;
constexpr uint8_t kExpanderRegPullEnable = 0x0Bu;
constexpr uint8_t kExpanderRegPullSelect = 0x0Du;
constexpr uint8_t kExpanderRegInputStatus = 0x0Fu;
constexpr uint8_t kExpanderRegInterruptMask = 0x11u;
constexpr uint8_t kExpanderRegInterruptStatus = 0x13u;
constexpr uint8_t kExpanderPinP2Mask = 0x04u;
constexpr uint8_t kExpanderInterruptMaskOnlyP2 =
    static_cast<uint8_t>(~kExpanderPinP2Mask);
constexpr uint32_t kAkidaSystemConfigBase = 0xFCE00000u;
constexpr uint32_t kAkidaRegGpioOutputEnable =
    kAkidaSystemConfigBase + 0x38u;
constexpr uint32_t kAkidaRegGpioMuxEnable = kAkidaSystemConfigBase + 0x3Cu;
constexpr uint32_t kAkidaRegGpioDriveStrength =
    kAkidaSystemConfigBase + 0x40u;
constexpr uint32_t kAkidaInterruptPadMask = 1u << kAkidaInterruptPadIndex;
constexpr uint32_t kInterruptWaitTimeoutMs = 125u;
constexpr uint32_t kFetchFallbackTimeoutMs = 125u;
constexpr uint32_t kInterruptPollDelayMs = 1u;
#if defined(TARGET_NICLA_VISION)
constexpr uint8_t kCameraResolution = CAMERA_R160x120;
constexpr uint8_t kCameraImageMode = CAMERA_RGB565;
constexpr int32_t kCameraFrameRate = 30;
constexpr uint16_t kCameraWidth = 160u;
constexpr uint16_t kCameraHeight = 120u;
constexpr size_t kCameraCaptureBytes =
    static_cast<size_t>(kCameraWidth) * kCameraHeight * 2u;
constexpr size_t kCameraPreviewBytes =
    static_cast<size_t>(kCameraWidth) * kCameraHeight;
#endif
constexpr size_t kModelInputBytes =
    static_cast<size_t>(kModelWidth) * kModelHeight * kModelChannels;
constexpr const char* kSketchName = "bb15_inference";
constexpr const char* kLogPrefix = "[bb15_inference]";
constexpr const char* kBundledModelName = "presence_regular_96_gray";

bool g_ready = false;
const char* g_last_failure_stage = nullptr;
uint8_t g_input[kModelInputBytes];
volatile bool g_akida_done = false;
volatile uint32_t g_irq_count = 0u;
volatile uint32_t g_irq_last_ms = 0u;
bool g_interrupt_mode_ready = false;
#if defined(TARGET_NICLA_VISION)
bool g_camera_ok = false;
uint8_t g_preview_frame[kCameraPreviewBytes];
uint16_t g_crop_x[kModelWidth];
uint16_t g_crop_y[kModelHeight];
bool g_crop_maps_ready = false;
#endif

BB15Pinout g_pinout =
#if defined(TARGET_NICLA_VISION)
    BB15Pinout::niclaVisionDefaults();
#else
    BB15Pinout::niclaSenseMeDefaults();
#endif
;

BB15Config g_config = []() {
  BB15Config config = BB15Config::niclaVisionDefaults();
  config.spiClockHz = kAkidaSpiClockHz;
  config.flashSpiClockHz = kFlashSpiClockHz;
  config.defaultModelAddress =
      AkidaNicla::externalModelAddressFromOffset(kFlashModelOffset);
  return config;
}();

// The examples keep the public API direct, but construction happens in
// `setup()`, not at global initialization time. On Arduino targets the global
// constructor phase runs before the board runtime is fully ready, so hardware-
// touching constructor work must wait until sketch startup.
BB15* g_bb15 = nullptr;
BB15Runner* g_runner = nullptr;
BB15Model g_model = []() {
  BB15Model model(program, static_cast<size_t>(program_len));
  model.setStorage(BB15ModelStorage::ExternalFlash)
      .setExternalAddress(AkidaNicla::externalModelAddressFromOffset(
          kFlashModelOffset));
  return model;
}();

BB15& board() { return *g_bb15; }
BB15Runner& runner() { return *g_runner; }

bool uses_camera_input() {
#if defined(TARGET_NICLA_VISION)
  return true;
#else
  return false;
#endif
}

bool uses_interrupt_completion() {
  return true;
}

const char* input_source_name() {
  return uses_camera_input() ? "camera" : "synthetic";
}

const char* completion_mode_name() {
  return uses_interrupt_completion() ? "interrupt" : "blocking";
}

void wait_for_serial() {
  const uint32_t start_ms = millis();
  while (!Serial && (millis() - start_ms) < kSerialWaitMs) {
  }
}

void set_led(bool active) { digitalWrite(LED_BUILTIN, active ? LOW : HIGH); }

void print_failure(const char* stage) {
  g_last_failure_stage = stage;
  Serial.print(kLogPrefix);
  Serial.print(" result=FAIL stage=");
  Serial.print(stage);
  Serial.print(" detail=");
  board().printLastError(Serial);
}

TwoWire& expander_bus() { return *board().config().wire; }

bool write_expander_reg8(uint8_t reg, uint8_t value) {
  expander_bus().beginTransmission(board().config().expanderAddress);
  expander_bus().write(reg);
  expander_bus().write(value);
  return expander_bus().endTransmission() == 0u;
}

bool read_expander_reg8(uint8_t reg, uint8_t& value) {
  expander_bus().beginTransmission(board().config().expanderAddress);
  expander_bus().write(reg);
  if (expander_bus().endTransmission(false) != 0u) {
    return false;
  }
  if (expander_bus().requestFrom(
          static_cast<int>(board().config().expanderAddress), 1) != 1) {
    return false;
  }
  value = static_cast<uint8_t>(expander_bus().read());
  return true;
}

void on_akida_done_interrupt() {
  g_akida_done = true;
  ++g_irq_count;
  g_irq_last_ms = millis();
}

void clear_expander_interrupt_state() {
  uint8_t ignored = 0u;
  (void)read_expander_reg8(kExpanderRegInterruptStatus, ignored);
}

bool configure_expander_interrupt_input() {
  uint8_t direction = 0u;
  uint8_t input_default = 0u;
  uint8_t pull_enable = 0u;
  uint8_t pull_select = 0u;
  if (!read_expander_reg8(kExpanderRegDirection, direction) ||
      !read_expander_reg8(kExpanderRegInputDefaultState, input_default) ||
      !read_expander_reg8(kExpanderRegPullEnable, pull_enable) ||
      !read_expander_reg8(kExpanderRegPullSelect, pull_select)) {
    return false;
  }

  // The proven board behavior is: P2 idles low and rises on inference done.
  // We leave everything else alone and only repurpose the one pin that carries
  // the completion path from AKD GPIO3.
  direction &= static_cast<uint8_t>(~kExpanderPinP2Mask);
  input_default &= static_cast<uint8_t>(~kExpanderPinP2Mask);
  pull_enable |= kExpanderPinP2Mask;
  pull_select &= static_cast<uint8_t>(~kExpanderPinP2Mask);
  if (!write_expander_reg8(kExpanderRegDirection, direction) ||
      !write_expander_reg8(kExpanderRegInputDefaultState, input_default) ||
      !write_expander_reg8(kExpanderRegPullEnable, pull_enable) ||
      !write_expander_reg8(kExpanderRegPullSelect, pull_select)) {
    return false;
  }

  // Clear any stale latch before unmasking P2, otherwise the first inference
  // can look "complete" before anything was enqueued.
  clear_expander_interrupt_state();
  return write_expander_reg8(kExpanderRegInterruptMask,
                             kExpanderInterruptMaskOnlyP2);
}

bool configure_akida_interrupt_route() {
  uint32_t gpio_oe = 0u;
  uint32_t gpio_mux = 0u;
  uint32_t gpio_drv = 0u;
  if (!runner().readRegister32(kAkidaRegGpioOutputEnable, gpio_oe) ||
      !runner().readRegister32(kAkidaRegGpioMuxEnable, gpio_mux) ||
      !runner().readRegister32(kAkidaRegGpioDriveStrength, gpio_drv)) {
    return false;
  }

  // For runtime completion signaling the pad must stay in alternate-function
  // mode. Manual GPIO mode can prove the wiring, but it blocks the real
  // internal AKD completion signal from reaching the pad.
  gpio_oe |= kAkidaInterruptPadMask;
  gpio_mux &= ~kAkidaInterruptPadMask;
  gpio_drv |= kAkidaInterruptPadMask;
  if (!runner().writeRegister32(kAkidaRegGpioOutputEnable, gpio_oe) ||
      !runner().writeRegister32(kAkidaRegGpioMuxEnable, gpio_mux) ||
      !runner().writeRegister32(kAkidaRegGpioDriveStrength, gpio_drv)) {
    return false;
  }

  uint32_t gpio_oe_check = 0u;
  uint32_t gpio_mux_check = 0u;
  uint32_t gpio_drv_check = 0u;
  return runner().readRegister32(kAkidaRegGpioOutputEnable, gpio_oe_check) &&
         runner().readRegister32(kAkidaRegGpioMuxEnable, gpio_mux_check) &&
         runner().readRegister32(kAkidaRegGpioDriveStrength, gpio_drv_check) &&
         ((gpio_oe_check & kAkidaInterruptPadMask) != 0u) &&
         ((gpio_mux_check & kAkidaInterruptPadMask) == 0u) &&
         ((gpio_drv_check & kAkidaInterruptPadMask) != 0u);
}

bool service_expander_interrupt(bool* p2_high_out) {
  uint8_t interrupt_status = 0u;
  uint8_t input_status = 0u;
  if (!read_expander_reg8(kExpanderRegInterruptStatus, interrupt_status) ||
      !read_expander_reg8(kExpanderRegInputStatus, input_status)) {
    return false;
  }

  const bool p2_high = (input_status & kExpanderPinP2Mask) != 0u;
  if (p2_high_out != nullptr) {
    *p2_high_out = p2_high;
  }
  return ((interrupt_status & kExpanderPinP2Mask) != 0u) || p2_high;
}

bool begin_interrupt_mode() {
  pinMode(board().pinout().host.interrupt, INPUT);
  clear_expander_interrupt_state();
  if (!configure_expander_interrupt_input() ||
      !configure_akida_interrupt_route()) {
    return false;
  }
  attachInterrupt(digitalPinToInterrupt(board().pinout().host.interrupt),
                  on_akida_done_interrupt, FALLING);
  g_interrupt_mode_ready = true;

  Serial.print(kLogPrefix);
  Serial.print(" interrupt_mode=ready host_pin=D");
  Serial.print(static_cast<unsigned long>(board().pinout().host.interrupt));
  Serial.print(" expander_pin=P");
  Serial.print(static_cast<unsigned long>(kExpanderInterruptPin));
  Serial.print(" akida_gpio=");
  Serial.println(static_cast<unsigned long>(kAkidaInterruptPadIndex));
  return true;
}

void blink_forever() {
  for (;;) {
    set_led(true);
    delay(50);
    set_led(false);
    delay(950);
    if (g_last_failure_stage != nullptr) {
      Serial.print(kLogPrefix);
      Serial.print(" halted stage=");
      Serial.println(g_last_failure_stage);
    }
  }
}

void print_model_summary() {
  Serial.print(kLogPrefix);
  Serial.print(" model=");
  Serial.print(kBundledModelName);
  Serial.print(" bytes=");
  Serial.println(static_cast<long>(akida_program_length_bytes));
}

void print_shape(const akida::Shape& shape) {
  Serial.print("[");
  for (size_t i = 0u; i < shape.size(); ++i) {
    if (i != 0u) {
      Serial.print(", ");
    }
    Serial.print(static_cast<unsigned long>(shape[i]));
  }
  Serial.println("]");
}

bool validate_export_geometry() {
  if (akida_input_rank != 3) {
    Serial.print(kLogPrefix);
    Serial.println(" unsupported_input_rank");
    return false;
  }
  if (akida_input_shape[0] != kModelWidth ||
      akida_input_shape[1] != kModelHeight ||
      akida_input_shape[2] != kModelChannels) {
    Serial.print(kLogPrefix);
    Serial.println(" unexpected_input_shape");
    return false;
  }
  if (static_cast<int64_t>(program_len) != akida_program_length_bytes) {
    Serial.print(kLogPrefix);
    Serial.println(" program_length_mismatch");
    return false;
  }
  return true;
}

#if defined(TARGET_NICLA_VISION)
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

void prepare_crop_maps() {
  if (g_crop_maps_ready) {
    return;
  }

  constexpr uint16_t kCropSize =
      (kCameraWidth < kCameraHeight) ? kCameraWidth : kCameraHeight;
  constexpr uint16_t kCropOriginX = (kCameraWidth - kCropSize) / 2u;
  constexpr uint16_t kCropOriginY = (kCameraHeight - kCropSize) / 2u;

  for (uint16_t x = 0u; x < kModelWidth; ++x) {
    g_crop_x[x] = kCropOriginX + static_cast<uint16_t>(
                                     (static_cast<uint32_t>(x) * kCropSize) /
                                     kModelWidth);
  }
  for (uint16_t y = 0u; y < kModelHeight; ++y) {
    g_crop_y[y] = kCropOriginY + static_cast<uint16_t>(
                                     (static_cast<uint32_t>(y) * kCropSize) /
                                     kModelHeight);
  }

  g_crop_maps_ready = true;
}

void fill_model_input_from_rgb565(const uint8_t* frame_rgb565) {
  prepare_crop_maps();

  for (size_t src_index = 0u, dst_index = 0u; dst_index < kCameraPreviewBytes;
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

  uint8_t* dst = g_input;
  for (uint16_t y = 0u; y < kModelHeight; ++y) {
    const uint16_t src_y = g_crop_y[y];
    for (uint16_t x = 0u; x < kModelWidth; ++x) {
      const uint16_t src_x = g_crop_x[x];
      const size_t src_index =
          static_cast<size_t>(src_y) * kCameraWidth + src_x;
      *dst++ = g_preview_frame[src_index];
    }
  }
}

bool begin_camera_source() {
  if (!camera().begin(kCameraResolution, kCameraImageMode, kCameraFrameRate)) {
    return false;
  }
  g_camera_ok = true;
  return true;
}

bool capture_camera_input() {
  if (!g_camera_ok) {
    return false;
  }
  if (camera().grabFrame(framebuffer(), 3000) != 0) {
    return false;
  }
  fill_model_input_from_rgb565(framebuffer().getBuffer());
  return true;
}

void print_camera_summary() {
  Serial.print(kLogPrefix);
  Serial.println(" input_source=camera");
  Serial.print(kLogPrefix);
  Serial.print(" camera_ready width=");
  Serial.print(static_cast<unsigned long>(camera().getResolutionWidth()));
  Serial.print(" height=");
  Serial.print(static_cast<unsigned long>(camera().getResolutionHeight()));
  Serial.println();
}
#endif

void build_demo_input(uint32_t pass_index) {
  for (uint16_t y = 0u; y < kModelHeight; ++y) {
    for (uint16_t x = 0u; x < kModelWidth; ++x) {
      const size_t index =
          static_cast<size_t>(y) * kModelWidth + static_cast<size_t>(x);
      const uint16_t stripe = static_cast<uint16_t>((x / 12u) + (y / 12u));
      const uint8_t base =
          ((stripe + static_cast<uint16_t>(pass_index)) & 1u) != 0u ? 208u : 32u;
      const uint8_t diagonal =
          ((x + y + static_cast<uint16_t>(pass_index * 3u)) % 29u) == 0u ? 255u
                                                                           : base;
      g_input[index] = diagonal;
    }
  }
}

bool begin_input_source() {
#if defined(TARGET_NICLA_VISION)
  return begin_camera_source();
#else
  return true;
#endif
}

bool capture_input_tensor(uint32_t pass_index) {
#if defined(TARGET_NICLA_VISION)
  (void)pass_index;
  return capture_camera_input();
#else
  build_demo_input(pass_index);
  return true;
#endif
}

void print_scores(const BB15RunResult& scores) {
  const int32_t* int32_values = scores.data<int32_t>();
  const uint8_t* uint8_values = scores.data<uint8_t>();
  const size_t count = scores.elementCount();

  Serial.print(kLogPrefix);
  Serial.print(" scores=[");
  for (size_t i = 0u; i < count; ++i) {
    if (i != 0u) {
      Serial.print(", ");
    }
    if (int32_values != nullptr) {
      Serial.print(int32_values[i]);
    } else if (uint8_values != nullptr) {
      Serial.print(uint8_values[i]);
    } else {
      Serial.print("?");
    }
  }
  Serial.println("]");
}

bool prepare_board_and_runner() {
  if (board().begin() != BB15Status::Ok) {
    print_failure("board_begin");
    return false;
  }
  if (!board().detectFlash()) {
    print_failure("detect_flash");
    return false;
  }

  const BB15FlashInfo flash = board().flashInfo();
  Serial.print(kLogPrefix);
  Serial.print(" flash_detect name=");
  Serial.print(flash.name);
  Serial.print(" jedec=0x");
  Serial.print(flash.jedec, HEX);
  Serial.println(flash.supportedProfile ? " supported=yes" : " supported=no");

  if (runner().begin() != BB15Status::Ok) {
    print_failure("runner_begin");
    return false;
  }
  if (runner().loadModel(g_model) != BB15Status::Ok) {
    print_failure("load_external_model");
    return false;
  }

  Serial.print(kLogPrefix);
  Serial.print(" akida ip_version=0x");
  Serial.println(board().ipVersion(), HEX);
  Serial.print(kLogPrefix);
  Serial.print(" model_loaded ");
  runner().printModelInfo(Serial);
  const BB15ModelInfo model_info = runner().modelInfo();
  if (!model_info.valid) {
    print_failure("model_info_invalid");
    return false;
  }
  if (model_info.input.dimensions.size() != 4u) {
    print_failure("model_input_rank");
    Serial.print(kLogPrefix);
    Serial.print(" actual input_shape=");
    print_shape(model_info.input.dimensions);
    return false;
  }
  if (model_info.input.dimensions[0] != 1u ||
      model_info.input.dimensions[1] != kModelHeight ||
      model_info.input.dimensions[2] != kModelWidth ||
      model_info.input.dimensions[3] != kModelChannels) {
    print_failure("model_input_shape");
    Serial.print(kLogPrefix);
    Serial.print(" actual input_shape=");
    print_shape(model_info.input.dimensions);
    return false;
  }
  return true;
}

void run_synchronous_inference(uint32_t pass_index) {
  if (!capture_input_tensor(pass_index)) {
    print_failure("capture_input");
    return;
  }
  const akida::Shape dims = {1u, kModelHeight, kModelWidth, kModelChannels};
  const BB15ClassificationResult result = runner().classify(g_input, dims);
  if (!result.ok()) {
    print_failure("classify");
    return;
  }

  Serial.print(kLogPrefix);
  Serial.print(" infer pass=");
  Serial.print(static_cast<unsigned long>(pass_index));
  Serial.print(" source=");
  Serial.print(input_source_name());
  Serial.print(" completion=blocking");
  Serial.print(" predicted_index=");
  Serial.println(static_cast<unsigned long>(result.predictedIndex));
  print_scores(result.scores);
}

void run_interrupt_inference(uint32_t pass_index) {
  if (!g_interrupt_mode_ready) {
    print_failure("interrupt_mode_begin_required");
    return;
  }
  if (!capture_input_tensor(pass_index)) {
    print_failure("capture_input");
    return;
  }
  if (!configure_akida_interrupt_route()) {
    print_failure("interrupt_route");
    return;
  }

  BB15Input input;
  input.data = g_input;
  input.type = akida::TensorType::uint8;
  input.dimensions = {1u, kModelHeight, kModelWidth, kModelChannels};

  noInterrupts();
  g_akida_done = false;
  interrupts();
  clear_expander_interrupt_state();

  if (runner().enqueue(input) != BB15Status::Ok) {
    print_failure("enqueue");
    return;
  }

  const uint32_t start_ms = millis();
  const uint32_t irq_deadline_ms = start_ms + kInterruptWaitTimeoutMs;
  const uint32_t irq_count_before = g_irq_count;
  const char* completion_source = "timeout";
  bool completion_seen = false;
  while (static_cast<int32_t>(millis() - irq_deadline_ms) < 0) {
    bool irq_candidate = false;
    noInterrupts();
    irq_candidate = g_akida_done;
    g_akida_done = false;
    interrupts();

    bool p2_high = false;
    if (irq_candidate && service_expander_interrupt(&p2_high)) {
      completion_source = "irq";
      completion_seen = true;
      break;
    }
    if (service_expander_interrupt(&p2_high)) {
      completion_source = "poll";
      completion_seen = true;
      break;
    }
    delay(kInterruptPollDelayMs);
  }

  const uint32_t fetch_deadline_ms = millis() + kFetchFallbackTimeoutMs;
  BB15RunResult result;
  for (;;) {
    result = runner().fetch();
    if (result.ok()) {
      if (!completion_seen) {
        completion_source = "fetch";
      }
      break;
    }
    if (result.status != BB15Status::OutputNotReady ||
        static_cast<int32_t>(millis() - fetch_deadline_ms) >= 0) {
      print_failure("fetch");
      return;
    }
    delay(kInterruptPollDelayMs);
  }

  Serial.print(kLogPrefix);
  Serial.print(" infer pass=");
  Serial.print(static_cast<unsigned long>(pass_index));
  Serial.print(" source=");
  Serial.print(input_source_name());
  Serial.print(" completion=");
  Serial.print(completion_source);
  Serial.print(" irq_count_delta=");
  Serial.print(static_cast<unsigned long>(g_irq_count - irq_count_before));
  Serial.print(" wait_ms=");
  Serial.print(static_cast<unsigned long>(millis() - start_ms));
  Serial.print(" predicted_index=");
  Serial.println(static_cast<unsigned long>(result.predictedIndex));
  print_scores(result);
}

void run_demo_inference(uint32_t pass_index) {
  run_interrupt_inference(pass_index);
}

}  // namespace

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(LEDR, OUTPUT);
  set_led(false);
  digitalWrite(LEDR, HIGH);

  Serial.begin(kSerialBaud);
  wait_for_serial();
  delay(kBootSettleMs);

  Serial.println();
  Serial.println(kSketchName);
  Serial.print(kLogPrefix);
  Serial.println(" board=BB15");
  static BB15 bb15(g_pinout, g_config);
  static BB15Runner bb15_runner = bb15.createRunner();
  g_bb15 = &bb15;
  g_runner = &bb15_runner;
  Serial.print(kLogPrefix);
  Serial.println(" constructor_reset_state=released");
  Serial.print(kLogPrefix);
  Serial.println(" purpose=load flashed model and run inference");
  print_model_summary();

  if (!validate_export_geometry()) {
    print_failure("validate_export_geometry");
    blink_forever();
  }
  if (!prepare_board_and_runner()) {
    blink_forever();
  }
  if (!begin_input_source()) {
    print_failure("input_source_begin");
    blink_forever();
  }
#if defined(TARGET_NICLA_VISION)
  print_camera_summary();
#else
  Serial.print(kLogPrefix);
  Serial.print(" input_source=");
  Serial.println(input_source_name());
#endif

  if (!begin_interrupt_mode()) {
    print_failure("interrupt_mode_begin");
    blink_forever();
  }

  Serial.print(kLogPrefix);
  Serial.print(" completion_mode=");
  Serial.println(completion_mode_name());

  run_demo_inference(0u);
  g_ready = true;
}

void loop() {
  static uint32_t inference_pass = 1u;
  static uint32_t last_heartbeat_ms = 0u;
  static bool heartbeat_on = false;

  const uint32_t now = millis();
  if ((now - last_heartbeat_ms) >= kHeartbeatToggleMs) {
    last_heartbeat_ms = now;
    heartbeat_on = !heartbeat_on;
    set_led(heartbeat_on);
  }

  if (g_ready) {
    run_demo_inference(inference_pass++);
  }
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
      "bb15_inference: runtime support requires a supported Nicla board + BB15");
}

void loop() {}

#endif
