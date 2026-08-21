#include <Arduino.h>

#include <BB15.h>

#include "camera.h"
#include "gc2145.h"
#include "model_metadata.h"
#include "program.h"

#ifndef ARDUINO_NICLA_VISION
#error "bb15_nicla_vision_human_detection requires Arduino Nicla Vision."
#endif

namespace {

// This example owns camera capture, preprocessing, interrupt completion, and
// USB framing. BB15 itself remains synchronous and hardware-focused.
constexpr uint32_t kSerialBaud = 115200u;
constexpr uint32_t kSerialWaitMs = 3000u;
constexpr uint32_t kBootSettleMs = 250u;
constexpr uint32_t kFlashModelOffset = 0u;
constexpr uint32_t kAkidaSpiClockHz = 25000000u;
constexpr uint32_t kFlashSpiClockHz = 2000000u;

constexpr uint8_t kCameraResolution = CAMERA_R320x240;
constexpr uint8_t kCameraImageMode = CAMERA_RGB565;
constexpr uint16_t kCameraWidth = 320u;
constexpr uint16_t kCameraHeight = 240u;
constexpr int32_t kCameraFrameRate = 30;
constexpr size_t kCaptureBytes =
    static_cast<size_t>(kCameraWidth) * kCameraHeight * 2u;
constexpr size_t kPreviewBytes = static_cast<size_t>(kCameraWidth) * kCameraHeight;
constexpr uint16_t kModelWidth = 96u;
constexpr uint16_t kModelHeight = 96u;
constexpr uint16_t kModelChannels = 3u;
constexpr size_t kModelInputBytes =
    static_cast<size_t>(kModelWidth) * kModelHeight * kModelChannels;

// The interrupt route is intentionally sketch-local. Akida GPIO 3 reaches
// expander P2, which drives the host interrupt pin defined in the pinout.
constexpr uint8_t kAkidaInterruptPadIndex = 3u;
constexpr uint8_t kExpanderInterruptPin = 2u;
constexpr uint8_t kExpanderRegDirection = 0x03u;
constexpr uint8_t kExpanderRegInputDefaultState = 0x09u;
constexpr uint8_t kExpanderRegPullEnable = 0x0Bu;
constexpr uint8_t kExpanderRegPullSelect = 0x0Du;
constexpr uint8_t kExpanderRegInputStatus = 0x0Fu;
constexpr uint8_t kExpanderRegInterruptMask = 0x11u;
constexpr uint8_t kExpanderRegInterruptStatus = 0x13u;
constexpr uint8_t kExpanderP2Mask = 0x04u;
constexpr uint8_t kExpanderInterruptMaskOnlyP2 =
    static_cast<uint8_t>(~kExpanderP2Mask);
constexpr uint32_t kAkidaSystemConfigBase = 0xFCE00000u;
constexpr uint32_t kAkidaRegGpioOutputEnable = kAkidaSystemConfigBase + 0x38u;
constexpr uint32_t kAkidaRegGpioMuxEnable = kAkidaSystemConfigBase + 0x3Cu;
constexpr uint32_t kAkidaRegGpioDriveStrength = kAkidaSystemConfigBase + 0x40u;
constexpr uint32_t kAkidaInterruptPadMask = 1u << kAkidaInterruptPadIndex;
constexpr uint32_t kInterruptWaitTimeoutMs = 125u;
constexpr uint32_t kFetchTimeoutMs = 125u;

// USB protocol v1. Every device message starts with this fixed header:
//   "BB15", version, type, payload size (uint32 little-endian).
// The preview tool sends the same header with a zero-length control payload.
constexpr uint8_t kProtocolMagic[] = {'B', 'B', '1', '5'};
constexpr uint8_t kProtocolVersion = 1u;
enum class PacketType : uint8_t {
  StartStream = 1u,
  StopStream = 2u,
  RequestConfig = 3u,
  Config = 0x81u,
  FrameResult = 0x82u,
  Error = 0x83u,
};
constexpr uint8_t kPreviewGray8 = 1u;
constexpr size_t kPacketHeaderBytes = 10u;
constexpr size_t kFrameMetadataBytes = 26u;

constexpr const char* kSketchName = "bb15_nicla_vision_human_detection";
constexpr const char* kLogPrefix = "[bb15_nicla_vision_human_detection]";
constexpr const char* kModelName =
    "NiclaV_VWW_PersonDet_EN_USBbottom_2026-06-14";

struct Observation {
  bool ok = false;
  BB15Status status = BB15Status::NotInitialized;
  size_t predictedIndex = 0u;
  int32_t score0 = 0;
  int32_t score1 = 0;
  uint16_t grabMs = 0u;
  uint16_t prepMs = 0u;
  uint16_t inferMs = 0u;
};

BB15Pinout g_pinout = BB15Pinout::niclaVisionDefaults();
BB15Config g_config = []() {
  BB15Config config = BB15Config::defaults();
  config.spiClockHz = kAkidaSpiClockHz;
  config.flashSpiClockHz = kFlashSpiClockHz;
  config.defaultModelAddress =
      AkidaNicla::externalModelAddressFromOffset(kFlashModelOffset);
  return config;
}();
BB15* g_bb15 = nullptr;
BB15Runner* g_runner = nullptr;
BB15Model g_model = []() {
  BB15Model model(program, static_cast<size_t>(program_len));
  model.setStorage(BB15ModelStorage::ExternalFlash)
      .setExternalAddress(AkidaNicla::externalModelAddressFromOffset(
          kFlashModelOffset));
  return model;
}();

uint8_t g_preview[kPreviewBytes];
// The Akida runtime executes after preprocessing. Keep a second immutable
// snapshot for USB so the long frame transfer cannot expose any later runtime
// writes to the working preview buffer.
uint8_t g_preview_usb[kPreviewBytes];
uint8_t g_model_input[kModelInputBytes];
uint16_t g_crop_x[kModelWidth];
uint16_t g_crop_y[kModelHeight];
bool g_crop_maps_ready = false;
volatile bool g_akida_done = false;
volatile uint32_t g_irq_count = 0u;
bool g_streaming = false;
uint32_t g_sequence = 0u;

BB15& board() { return *g_bb15; }
BB15Runner& runner() { return *g_runner; }
TwoWire& expander_bus() { return *board().config().wire; }
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

uint16_t clamp_u16(uint32_t value) {
  return value > 0xFFFFu ? 0xFFFFu : static_cast<uint16_t>(value);
}

void wait_for_serial() {
  const uint32_t start_ms = millis();
  while (!Serial && (millis() - start_ms) < kSerialWaitMs) {
  }
}

void set_led(bool active) { digitalWrite(LED_BUILTIN, active ? LOW : HIGH); }

void print_failure(const char* stage) {
  Serial.print(kLogPrefix);
  Serial.print(" result=FAIL stage=");
  Serial.print(stage);
  Serial.print(" detail=");
  board().printLastError(Serial);
}

void halt_forever(const char* stage) {
  print_failure(stage);
  for (;;) {
    set_led(true);
    delay(50);
    set_led(false);
    delay(950);
  }
}

bool write_expander_reg8(uint8_t reg, uint8_t value) {
  expander_bus().beginTransmission(board().config().expanderAddress);
  expander_bus().write(reg);
  expander_bus().write(value);
  return expander_bus().endTransmission() == 0u;
}

bool read_expander_reg8(uint8_t reg, uint8_t& value) {
  expander_bus().beginTransmission(board().config().expanderAddress);
  expander_bus().write(reg);
  if (expander_bus().endTransmission(false) != 0u ||
      expander_bus().requestFrom(
          static_cast<int>(board().config().expanderAddress), 1) != 1) {
    return false;
  }
  value = static_cast<uint8_t>(expander_bus().read());
  return true;
}

void on_akida_done_interrupt() {
  g_akida_done = true;
  ++g_irq_count;
}

bool clear_expander_interrupt_state() {
  uint8_t ignored = 0u;
  return read_expander_reg8(kExpanderRegInterruptStatus, ignored);
}

bool configure_interrupt_mode() {
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
  direction &= static_cast<uint8_t>(~kExpanderP2Mask);
  input_default &= static_cast<uint8_t>(~kExpanderP2Mask);
  pull_enable |= kExpanderP2Mask;
  pull_select &= static_cast<uint8_t>(~kExpanderP2Mask);
  if (!write_expander_reg8(kExpanderRegDirection, direction) ||
      !write_expander_reg8(kExpanderRegInputDefaultState, input_default) ||
      !write_expander_reg8(kExpanderRegPullEnable, pull_enable) ||
      !write_expander_reg8(kExpanderRegPullSelect, pull_select) ||
      !clear_expander_interrupt_state() ||
      !write_expander_reg8(kExpanderRegInterruptMask,
                           kExpanderInterruptMaskOnlyP2)) {
    return false;
  }

  pinMode(board().pinout().host.interrupt, INPUT);
  attachInterrupt(digitalPinToInterrupt(board().pinout().host.interrupt),
                  on_akida_done_interrupt, FALLING);
  return true;
}

bool configure_akida_interrupt_route() {
  uint32_t output_enable = 0u;
  uint32_t mux_enable = 0u;
  uint32_t drive_strength = 0u;
  if (!runner().readRegister32(kAkidaRegGpioOutputEnable, output_enable) ||
      !runner().readRegister32(kAkidaRegGpioMuxEnable, mux_enable) ||
      !runner().readRegister32(kAkidaRegGpioDriveStrength, drive_strength)) {
    return false;
  }
  output_enable |= kAkidaInterruptPadMask;
  mux_enable &= ~kAkidaInterruptPadMask;
  drive_strength |= kAkidaInterruptPadMask;
  return runner().writeRegister32(kAkidaRegGpioOutputEnable, output_enable) &&
         runner().writeRegister32(kAkidaRegGpioMuxEnable, mux_enable) &&
         runner().writeRegister32(kAkidaRegGpioDriveStrength, drive_strength);
}

bool completion_pending() {
  uint8_t interrupt_status = 0u;
  uint8_t input_status = 0u;
  if (!read_expander_reg8(kExpanderRegInterruptStatus, interrupt_status) ||
      !read_expander_reg8(kExpanderRegInputStatus, input_status)) {
    return false;
  }
  return (interrupt_status & kExpanderP2Mask) != 0u ||
         (input_status & kExpanderP2Mask) != 0u;
}

void write_u16(uint16_t value) {
  Serial.write(static_cast<uint8_t>(value & 0xFFu));
  Serial.write(static_cast<uint8_t>((value >> 8) & 0xFFu));
}

void write_u32(uint32_t value) {
  for (uint8_t shift = 0u; shift < 32u; shift += 8u) {
    Serial.write(static_cast<uint8_t>((value >> shift) & 0xFFu));
  }
}

void write_i32(int32_t value) { write_u32(static_cast<uint32_t>(value)); }

void write_packet_header(PacketType type, uint32_t payload_bytes) {
  Serial.write(kProtocolMagic, sizeof(kProtocolMagic));
  Serial.write(kProtocolVersion);
  Serial.write(static_cast<uint8_t>(type));
  write_u32(payload_bytes);
}

void send_config_packet() {
  write_packet_header(PacketType::Config, 9u);
  write_u16(kCameraWidth);
  write_u16(kCameraHeight);
  Serial.write(kPreviewGray8);
  write_u16(kModelWidth);
  write_u16(kModelHeight);
  Serial.flush();
}

void send_error_packet(BB15Status status) {
  write_packet_header(PacketType::Error, 1u);
  Serial.write(static_cast<uint8_t>(status));
  Serial.flush();
}

void send_frame_result_packet(const Observation& observation) {
  write_packet_header(PacketType::FrameResult,
                      static_cast<uint32_t>(kFrameMetadataBytes + kPreviewBytes));
  write_u32(g_sequence++);
  write_u16(kCameraWidth);
  write_u16(kCameraHeight);
  Serial.write(kPreviewGray8);
  Serial.write(static_cast<uint8_t>(observation.predictedIndex & 0xFFu));
  Serial.write(static_cast<uint8_t>(observation.status));
  Serial.write(2u);
  write_i32(observation.score0);
  write_i32(observation.score1);
  write_u16(observation.grabMs);
  write_u16(observation.prepMs);
  write_u16(observation.inferMs);
  Serial.write(g_preview_usb, kPreviewBytes);
  Serial.flush();
}

// Commands are zero-payload packets. A small byte-wise parser tolerates the
// boot banner that may still be buffered when the desktop tool opens the port.
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

void prepare_crop_maps() {
  if (g_crop_maps_ready) {
    return;
  }
  constexpr uint16_t kCropSize = kCameraHeight;
  constexpr uint16_t kCropOriginX = (kCameraWidth - kCropSize) / 2u;
  for (uint16_t x = 0u; x < kModelWidth; ++x) {
    g_crop_x[x] = static_cast<uint16_t>(
        kCropOriginX + (static_cast<uint32_t>(x) * kCropSize) / kModelWidth);
  }
  for (uint16_t y = 0u; y < kModelHeight; ++y) {
    g_crop_y[y] = static_cast<uint16_t>(
        (static_cast<uint32_t>(y) * kCropSize) / kModelHeight);
  }
  g_crop_maps_ready = true;
}

// Generate a readable grayscale preview and the exact RGB tensor expected by
// the VWW model from one RGB565 camera capture.
void preprocess_rgb565(const uint8_t* frame) {
  prepare_crop_maps();
  for (size_t pixel = 0u; pixel < kPreviewBytes; ++pixel) {
    const uint16_t packed = (static_cast<uint16_t>(frame[pixel * 2u]) << 8) |
                            frame[pixel * 2u + 1u];
    const uint8_t r8 = static_cast<uint8_t>(((packed >> 11) & 0x1Fu) * 255u / 31u);
    const uint8_t g8 = static_cast<uint8_t>(((packed >> 5) & 0x3Fu) * 255u / 63u);
    const uint8_t b8 = static_cast<uint8_t>((packed & 0x1Fu) * 255u / 31u);
    g_preview[pixel] = static_cast<uint8_t>((77u * r8 + 150u * g8 + 29u * b8) >> 8);
  }

  for (uint16_t y = 0u; y < kModelHeight; ++y) {
    for (uint16_t x = 0u; x < kModelWidth; ++x) {
      const size_t source =
          static_cast<size_t>(g_crop_y[y]) * kCameraWidth + g_crop_x[x];
      const uint16_t packed =
          (static_cast<uint16_t>(frame[source * 2u]) << 8) | frame[source * 2u + 1u];
      const uint8_t r8 = static_cast<uint8_t>(((packed >> 11) & 0x1Fu) * 255u / 31u);
      const uint8_t g8 = static_cast<uint8_t>(((packed >> 5) & 0x3Fu) * 255u / 63u);
      const uint8_t b8 = static_cast<uint8_t>((packed & 0x1Fu) * 255u / 31u);
      // The model was trained with the camera mounted USB-side down.
      const size_t output =
          (static_cast<size_t>(kModelHeight - 1u - y) * kModelWidth +
           (kModelWidth - 1u - x)) *
          kModelChannels;
      g_model_input[output] = r8;
      g_model_input[output + 1u] = g8;
      g_model_input[output + 2u] = b8;
    }
  }
}

bool capture_and_infer(Observation* observation) {
  const uint32_t grab_start = millis();
  if (camera().grabFrame(framebuffer(), 3000) != 0) {
    observation->status = BB15Status::TransportStateError;
    return false;
  }
  const uint32_t grab_ms = millis() - grab_start;
  const uint8_t* frame = framebuffer().getBuffer();
  if (frame == nullptr) {
    observation->status = BB15Status::TransportStateError;
    return false;
  }

  const uint32_t prep_start = millis();
  preprocess_rgb565(frame);
  memcpy(g_preview_usb, g_preview, kPreviewBytes);
  const uint32_t prep_ms = millis() - prep_start;
  if (!configure_akida_interrupt_route()) {
    observation->status = BB15Status::TransportStateError;
    return false;
  }

  BB15Input input;
  input.data = g_model_input;
  input.type = akida::TensorType::uint8;
  input.dimensions = {1u, kModelHeight, kModelWidth, kModelChannels};
  noInterrupts();
  g_akida_done = false;
  interrupts();
  if (!clear_expander_interrupt_state()) {
    observation->status = BB15Status::TransportStateError;
    return false;
  }

  const uint32_t infer_start = millis();
  if (runner().enqueue(input) != BB15Status::Ok) {
    observation->status = runner().lastError().status;
    return false;
  }
  const uint32_t interrupt_deadline = millis() + kInterruptWaitTimeoutMs;
  while (static_cast<int32_t>(millis() - interrupt_deadline) < 0) {
    bool irq_seen = false;
    noInterrupts();
    irq_seen = g_akida_done;
    g_akida_done = false;
    interrupts();
    const bool expander_seen = completion_pending();
    if (irq_seen || expander_seen) {
      break;
    }
    delay(1);
  }

  const uint32_t fetch_deadline = millis() + kFetchTimeoutMs;
  BB15RunResult result;
  do {
    result = runner().fetch();
    if (result.ok()) {
      break;
    }
    if (result.status != BB15Status::OutputNotReady) {
      observation->status = result.status;
      return false;
    }
    delay(1);
  } while (static_cast<int32_t>(millis() - fetch_deadline) < 0);
  if (!result.ok()) {
    observation->status = result.status;
    return false;
  }

  observation->ok = true;
  observation->status = BB15Status::Ok;
  observation->predictedIndex = result.predictedIndex;
  observation->grabMs = clamp_u16(grab_ms);
  observation->prepMs = clamp_u16(prep_ms);
  observation->inferMs = clamp_u16(millis() - infer_start);
  if (result.type == akida::TensorType::int32 && result.elementCount() >= 2u) {
    const int32_t* scores = result.data<int32_t>();
    observation->score0 = scores[0];
    observation->score1 = scores[1];
  }
  return true;
}

bool prepare_runtime() {
  if (board().begin() != BB15Status::Ok || !board().detectFlash() ||
      runner().begin() != BB15Status::Ok ||
      runner().loadModel(g_model) != BB15Status::Ok || !configure_interrupt_mode()) {
    return false;
  }
  return camera().begin(kCameraResolution, kCameraImageMode, kCameraFrameRate);
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
  Serial.println(" board=BB15 + Nicla Vision");
  Serial.print(kLogPrefix);
  Serial.print(" model=");
  Serial.print(kModelName);
  Serial.print(" bytes=");
  Serial.println(static_cast<long>(akida_program_length_bytes));
  static BB15 bb15(g_pinout, g_config);
  static BB15Runner bb15_runner = bb15.createRunner();
  g_bb15 = &bb15;
  g_runner = &bb15_runner;
  if (!prepare_runtime()) {
    halt_forever("setup");
  }
  Serial.print(kLogPrefix);
  Serial.print(" runtime_ready ip_version=0x");
  Serial.println(board().ipVersion(), HEX);
  Serial.print(kLogPrefix);
  Serial.println(" usb_protocol=BB15/v1 waiting_for_start_stream");
}

void loop() {
  const PacketType command = poll_host_command();
  if (command == PacketType::StartStream) {
    g_streaming = true;
    send_config_packet();
    return;
  }
  if (command == PacketType::StopStream) {
    g_streaming = false;
    set_led(false);
    return;
  }
  if (command == PacketType::RequestConfig) {
    send_config_packet();
    return;
  }
  if (!g_streaming) {
    delay(1);
    return;
  }

  set_led(true);
  Observation observation;
  if (capture_and_infer(&observation)) {
    send_frame_result_packet(observation);
  } else {
    send_error_packet(observation.status);
  }
  set_led(false);
}
