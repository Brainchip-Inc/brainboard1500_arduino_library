#include <Arduino.h>

#include <AKD1500.h>
#include <Wire.h>
#include <mbed.h>

#include "camera.h"
#include "gc2145.h"
#include "presence_classifier/program.h"

#ifndef ARDUINO_NICLA_VISION
#error "This sketch is intended for Nicla Vision."
#endif

namespace {

using namespace std::chrono_literals;

constexpr uint32_t kSerialBaud = 115200u;
constexpr uint32_t kSerialWaitMs = 3000u;
constexpr uint32_t kBootSettleMs = 250u;
constexpr uint32_t kI2cClockHz = 100000u;
constexpr uint32_t kExpanderSettleMs = 10u;
constexpr uint32_t kIdleBaselineMs = 5000u;
constexpr uint32_t kSampleIntervalMs = 100u;
constexpr uint8_t kExpanderAddress = 0x43u;
constexpr uint8_t kInaAddressDefault = 0x40u;
constexpr uint8_t kInaAddressSchematic = 0x48u;
constexpr uint8_t kAkidaResetPin = 3u;
constexpr uint8_t kAkidaCsPin = 7u;
constexpr uint8_t kBridgeCsPin = 1u;
constexpr uint32_t kFlashModelOffset = 0u;
constexpr uint32_t kAkidaSpiClockHz = 10000000u;
constexpr uint32_t kFlashSpiClockHz = 2000000u;
constexpr const char* kModelName = "presence_classifier";
constexpr uint8_t kResolution = CAMERA_R320x240;
constexpr uint8_t kImageMode = CAMERA_RGB565;
constexpr uint16_t kCameraWidth = 320u;
constexpr uint16_t kCameraHeight = 240u;
constexpr int32_t kFrameRate = 30;
constexpr uint16_t kModelWidth = 224u;
constexpr uint16_t kModelHeight = 224u;
constexpr uint16_t kModelChannels = 3u;
constexpr size_t kModelInputBytes =
    static_cast<size_t>(kModelWidth) * kModelHeight * kModelChannels;

constexpr uint32_t kAkidaResetAssertMs = 5u;
constexpr uint32_t kAkidaResetReleaseSettleMs = 10u;
constexpr PinName kProceedGpioPin = PA_10;

constexpr uint8_t kExpanderRegDeviceIdControl = 0x01u;
constexpr uint8_t kExpanderRegDirection = 0x03u;
constexpr uint8_t kExpanderRegOutputState = 0x05u;
constexpr uint8_t kExpanderRegOutputHighZ = 0x07u;
constexpr uint8_t kExpanderAllOutputs = 0xFFu;
constexpr uint8_t kExpanderAllOutputsEnabled = 0x00u;
constexpr uint8_t kExpanderDefaultOutputState = 0x02u;

constexpr float kShuntOhms = 0.010f;
constexpr float kBusVoltageLsbVolts = 0.0016f;
constexpr float kShuntVoltageLsbVolts = 0.0000025f;
constexpr uint8_t kInaRegShuntVoltageCh1 = 0x00u;
constexpr uint8_t kInaRegBusVoltageCh1 = 0x01u;
constexpr uint8_t kInaRegShuntVoltageCh2 = 0x08u;
constexpr uint8_t kInaRegBusVoltageCh2 = 0x09u;
constexpr uint8_t kInaRegManufacturerId = 0x7Eu;
constexpr uint8_t kInaRegDeviceId = 0x7Fu;

enum class RunPhase : uint8_t {
  Boot = 0,
  IdleReady,
  InferLoop,
};

struct InaChannelSample {
  int16_t raw_shunt = 0;
  uint16_t raw_bus = 0u;
  float bus_v = 0.0f;
  float shunt_v = 0.0f;
  float current_a = 0.0f;
  float power_w = 0.0f;
};

struct InaSample {
  bool ok = false;
  InaChannelSample ch1 = {};
  InaChannelSample ch2 = {};
};

struct InferenceObservation {
  AKD1500Status status = AKD1500Status::NotInitialized;
  bool ok = false;
  uint32_t frame = 0u;
  uint16_t grab_ms = 0u;
  uint16_t prep_ms = 0u;
  uint16_t infer_ms = 0u;
  size_t predicted_index = 0u;
  int32_t score0 = 0;
  int32_t score1 = 0;
};

class PioExpander6408 {
 public:
  explicit PioExpander6408(TwoWire& bus, uint8_t address = kExpanderAddress)
      : bus_(bus), address_(address) {}

  bool probe() {
    bus_.beginTransmission(address_);
    return bus_.endTransmission() == 0u;
  }

  bool configureBb15DefaultOutputs() {
    return writeRegister(kExpanderRegOutputState, 0x00u) &&
           writeRegister(kExpanderRegDirection, kExpanderAllOutputs) &&
           writeRegister(kExpanderRegOutputHighZ,
                         kExpanderAllOutputsEnabled) &&
           writeRegister(kExpanderRegOutputState, kExpanderDefaultOutputState) &&
           writeRegister(0x11u, 0xFFu);
  }

  bool pinModeOutput(uint8_t pin) {
    if (pin >= 8u) {
      return false;
    }
    uint8_t direction = 0u;
    uint8_t output_high_z = 0u;
    if (!readRegister(kExpanderRegDirection, direction) ||
        !readRegister(kExpanderRegOutputHighZ, output_high_z)) {
      return false;
    }

    const uint8_t mask = static_cast<uint8_t>(1u << pin);
    direction |= mask;
    output_high_z &= static_cast<uint8_t>(~mask);
    return writeRegister(kExpanderRegDirection, direction) &&
           writeRegister(kExpanderRegOutputHighZ, output_high_z);
  }

  bool digitalWrite(uint8_t pin, bool high) {
    if (pin >= 8u) {
      return false;
    }
    uint8_t output_state = 0u;
    if (!readRegister(kExpanderRegOutputState, output_state)) {
      return false;
    }

    const uint8_t mask = static_cast<uint8_t>(1u << pin);
    if (high) {
      output_state |= mask;
    } else {
      output_state &= static_cast<uint8_t>(~mask);
    }
    return writeRegister(kExpanderRegOutputState, output_state);
  }

 private:
  bool writeRegister(uint8_t reg, uint8_t value) {
    bus_.beginTransmission(address_);
    bus_.write(reg);
    bus_.write(value);
    return bus_.endTransmission() == 0u;
  }

  bool readRegister(uint8_t reg, uint8_t& value) {
    bus_.beginTransmission(address_);
    bus_.write(reg);
    if (bus_.endTransmission(false) != 0u) {
      return false;
    }
    if (bus_.requestFrom(static_cast<int>(address_), 1) != 1) {
      return false;
    }
    value = static_cast<uint8_t>(bus_.read());
    return true;
  }

  TwoWire& bus_;
  uint8_t address_;
};

TwoWire& g_bus = Wire;
GC2145 g_sensor;
Camera g_camera(g_sensor);
FrameBuffer g_framebuffer;
AkidaNicla g_akida;
PioExpander6408 g_expander(g_bus);
mbed::Ticker g_sample_ticker;

bool g_camera_ok = false;
bool g_akida_ok = false;
bool g_model_ok = false;
uint8_t g_ina_address = 0u;
uint8_t g_model_input[kModelInputBytes];
RunPhase g_phase = RunPhase::Boot;
uint32_t g_phase_started_ms = 0u;
InferenceObservation g_last_inference = {};
volatile bool g_inference_active = false;
volatile uint32_t g_sample_ticks_due = 0u;
volatile uint32_t g_last_tick_us = 0u;
uint32_t g_sample_sequence = 0u;
const char* g_last_failure_stage = nullptr;
const char* g_last_failure_detail = nullptr;

AKD1500Options make_akida_options() {
  AKD1500Options options = AKD1500Options::niclaVisionDefaults();
  options.akidaCsPin = kAkidaCsPin;
  options.bridgeCsPin = kBridgeCsPin;
  options.spiClockHz = kAkidaSpiClockHz;
  options.flashSpiClockHz = kFlashSpiClockHz;
  options.externalModelAddress =
      AkidaNicla::externalModelAddressFromOffset(kFlashModelOffset);
  return options;
}

AKD1500Model make_flash_model() {
  AKD1500Model model;
  model.serializedProgram = program;
  model.size = static_cast<size_t>(program_len);
  model.storage = AKD1500ModelStorage::ExternalFlash;
  model.externalLocation =
      AkidaNicla::externalModelAddressFromOffset(kFlashModelOffset);
  return model;
}

void wait_for_serial() {
  const uint32_t start_ms = millis();
  while (!Serial && (millis() - start_ms) < kSerialWaitMs) {
  }
}

void set_led(bool active) {
  digitalWrite(LED_BUILTIN, active ? LOW : HIGH);
}

void print_failure(const char* stage, const char* detail = nullptr) {
  g_last_failure_stage = stage;
  g_last_failure_detail = detail;
  Serial.print("# result=FAIL stage=");
  Serial.print(stage);
  if (detail != nullptr) {
    Serial.print(" detail=");
    Serial.print(detail);
  }
  if (g_akida.lastError().status != AKD1500Status::Ok) {
    Serial.print(" akida=");
    g_akida.printLastError(Serial);
  } else {
    Serial.println();
  }
}

void blink_forever() {
  for (;;) {
    set_led(true);
    delay(50);
    set_led(false);
    delay(950);
  }
}

const char* phase_name(RunPhase phase) {
  switch (phase) {
    case RunPhase::Boot:
      return "boot";
    case RunPhase::IdleReady:
      return "idle_ready";
    case RunPhase::InferLoop:
      return "infer_loop";
  }
  return "unknown";
}

void set_phase(RunPhase phase) {
  g_phase = phase;
  g_phase_started_ms = millis();
}

void assert_proceed_high() {
  pinMode(kProceedGpioPin, OUTPUT);
  digitalWrite(kProceedGpioPin, HIGH);
  pinMode(D2, OUTPUT);
  digitalWrite(D2, HIGH);
}

void detach_external_uart_pins() {
  Serial1.end();
  Serial2.end();
  pinMode(PA_9, OUTPUT);
  digitalWrite(PA_9, HIGH);
  pinMode(D1, OUTPUT);
  digitalWrite(D1, HIGH);
  assert_proceed_high();
}

void set_akida_reset(bool asserted) {
  pinMode(kAkidaResetPin, OUTPUT);
  digitalWrite(kAkidaResetPin, asserted ? LOW : HIGH);
}

bool write_reg16(uint8_t address, uint8_t reg, uint16_t value) {
  g_bus.beginTransmission(address);
  g_bus.write(reg);
  g_bus.write(static_cast<uint8_t>((value >> 8) & 0xFFu));
  g_bus.write(static_cast<uint8_t>(value & 0xFFu));
  return g_bus.endTransmission() == 0u;
}

bool read_reg16(uint8_t address, uint8_t reg, uint16_t& value) {
  g_bus.beginTransmission(address);
  g_bus.write(reg);
  if (g_bus.endTransmission(false) != 0u) {
    return false;
  }
  if (g_bus.requestFrom(static_cast<int>(address), 2) != 2) {
    return false;
  }
  const uint8_t msb = static_cast<uint8_t>(g_bus.read());
  const uint8_t lsb = static_cast<uint8_t>(g_bus.read());
  value = static_cast<uint16_t>((static_cast<uint16_t>(msb) << 8) | lsb);
  return true;
}

uint8_t probe_address(uint8_t address) {
  g_bus.beginTransmission(address);
  return g_bus.endTransmission();
}

uint8_t detect_ina_address() {
  if (probe_address(kInaAddressDefault) == 0u) {
    return kInaAddressDefault;
  }
  if (probe_address(kInaAddressSchematic) == 0u) {
    return kInaAddressSchematic;
  }
  return 0u;
}

bool reset_akida_into_external_flash_mode() {
  set_akida_reset(true);
  delay(kAkidaResetAssertMs);
  if (!g_expander.pinModeOutput(0u) || !g_expander.digitalWrite(0u, false)) {
    return false;
  }
  set_akida_reset(false);
  delay(kAkidaResetReleaseSettleMs);
  return true;
}

bool begin_bb15_board() {
  // Hold the board's proceed rails high before reusing the shared pins.
  assert_proceed_high();

  // Release the shared UART bridge pins before the flash/Akida SPI path uses them.
  detach_external_uart_pins();
  delay(100);
  assert_proceed_high();

  // Start the external control bus and drive the BB15 expander defaults.
  g_bus.begin();
  g_bus.setClock(kI2cClockHz);
  if (!g_expander.probe()) {
    return false;
  }
  if (!g_expander.configureBb15DefaultOutputs()) {
    return false;
  }
  delay(kExpanderSettleMs);

  // Re-strap the AKD1500 into external-flash mode before the library links to it.
  return reset_akida_into_external_flash_mode();
}

bool setup_ina() {
  g_ina_address = detect_ina_address();
  if (g_ina_address == 0u) {
    return false;
  }

  uint16_t manufacturer = 0u;
  uint16_t device_id = 0u;
  if (!read_reg16(g_ina_address, kInaRegManufacturerId, manufacturer) ||
      !read_reg16(g_ina_address, kInaRegDeviceId, device_id)) {
    return false;
  }

  Serial.print("# ina_address=0x");
  Serial.println(g_ina_address, HEX);
  Serial.print("# ina_manufacturer=0x");
  Serial.println(manufacturer, HEX);
  Serial.print("# ina_device=0x");
  Serial.println(device_id, HEX);
  return true;
}

bool read_ina_channel(uint8_t shunt_reg, uint8_t bus_reg,
                      InaChannelSample& sample) {
  uint16_t raw_shunt = 0u;
  uint16_t raw_bus = 0u;
  if (!read_reg16(g_ina_address, shunt_reg, raw_shunt) ||
      !read_reg16(g_ina_address, bus_reg, raw_bus)) {
    return false;
  }

  sample.raw_shunt = static_cast<int16_t>(raw_shunt);
  sample.raw_bus = raw_bus;
  sample.shunt_v = static_cast<float>(sample.raw_shunt) * kShuntVoltageLsbVolts;
  sample.bus_v = static_cast<float>(sample.raw_bus) * kBusVoltageLsbVolts;
  sample.current_a = sample.shunt_v / kShuntOhms;
  sample.power_w = sample.bus_v * sample.current_a;
  return true;
}

InaSample read_ina_sample() {
  InaSample sample;
  sample.ok = read_ina_channel(kInaRegShuntVoltageCh1, kInaRegBusVoltageCh1,
                               sample.ch1) &&
              read_ina_channel(kInaRegShuntVoltageCh2, kInaRegBusVoltageCh2,
                               sample.ch2);
  return sample;
}

bool begin_camera() {
  if (g_camera.begin(kResolution, kImageMode, kFrameRate)) {
    g_camera_ok = true;
    return true;
  }
  return false;
}

void rgb565_to_rgb888(const uint8_t* src, uint8_t* dst_r, uint8_t* dst_g,
                      uint8_t* dst_b) {
  const uint16_t value = (static_cast<uint16_t>(src[0]) << 8) | src[1];
  const uint8_t r5 = static_cast<uint8_t>((value >> 11) & 0x1Fu);
  const uint8_t g6 = static_cast<uint8_t>((value >> 5) & 0x3Fu);
  const uint8_t b5 = static_cast<uint8_t>(value & 0x1Fu);
  *dst_r = static_cast<uint8_t>((r5 * 255u) / 31u);
  *dst_g = static_cast<uint8_t>((g6 * 255u) / 63u);
  *dst_b = static_cast<uint8_t>((b5 * 255u) / 31u);
}

void fill_model_input_from_rgb565_center_crop(const uint8_t* frame_rgb565) {
  constexpr uint16_t kCropSize =
      (kCameraWidth < kCameraHeight) ? kCameraWidth : kCameraHeight;
  constexpr uint16_t kCropOriginX = (kCameraWidth - kCropSize) / 2u;
  constexpr uint16_t kCropOriginY = (kCameraHeight - kCropSize) / 2u;

  uint8_t* dst = g_model_input;
  for (uint16_t y = 0; y < kModelHeight; ++y) {
    const uint16_t src_y = kCropOriginY + static_cast<uint16_t>(
                                              (static_cast<uint32_t>(y) * kCropSize) /
                                              kModelHeight);
    for (uint16_t x = 0; x < kModelWidth; ++x) {
      const uint16_t src_x = kCropOriginX + static_cast<uint16_t>(
                                                (static_cast<uint32_t>(x) * kCropSize) /
                                                kModelWidth);
      const size_t src_index =
          (static_cast<size_t>(src_y) * kCameraWidth + src_x) * 2u;
      rgb565_to_rgb888(frame_rgb565 + src_index, &dst[0], &dst[1], &dst[2]);
      dst += 3;
    }
  }
}

bool ensure_model_staged() {
  const AKD1500Options options = make_akida_options();
  if (AkidaNicla::verifyModelInFlash(options, program,
                                     static_cast<size_t>(program_len),
                                     kFlashModelOffset)) {
    return true;
  }

  Serial.println("# flash_verify=miss staging_model=1");
  if (!AkidaNicla::stageModelToFlash(options, program,
                                     static_cast<size_t>(program_len),
                                     kFlashModelOffset)) {
    return false;
  }
  return AkidaNicla::verifyModelInFlash(options, program,
                                        static_cast<size_t>(program_len),
                                        kFlashModelOffset);
}

bool begin_akida_and_load_model() {
  const AKD1500Options options = make_akida_options();
  if (!ensure_model_staged()) {
    return false;
  }

  g_akida_ok = (g_akida.begin(options) == AKD1500Status::Ok);
  if (!g_akida_ok) {
    return false;
  }

  g_model_ok = (g_akida.load(make_flash_model()) == AKD1500Status::Ok);
  if (g_model_ok) {
    // Mark the steady-state idle phase as "ready" rather than leaving the
    // default status at NotInitialized before the first inference runs.
    g_last_inference.status = AKD1500Status::Ok;
  }
  return g_model_ok;
}

void on_sample_tick() {
  ++g_sample_ticks_due;
  g_last_tick_us = micros();
}

void print_boot_header() {
  Serial.println("# sketch=bb15_camera_inference_ina_timer_monitor");
  Serial.println("# board=Nicla Vision + BB15");
  Serial.println("# sensor=GC2145");
  Serial.println("# mode=csv_no_preview");
  Serial.print("# timer_interval_ms=");
  Serial.println(kSampleIntervalMs);
  Serial.print("# idle_baseline_ms=");
  Serial.println(kIdleBaselineMs);
  Serial.print("# spi_akida_hz=");
  Serial.println(kAkidaSpiClockHz);
  Serial.print("# spi_flash_hz=");
  Serial.println(kFlashSpiClockHz);
  Serial.print("# flash_model_offset=0x");
  Serial.println(kFlashModelOffset, HEX);
  Serial.print("# model=");
  Serial.println(kModelName);
  Serial.println("# phases=boot->idle_ready->infer_loop");
  Serial.println("# csv=tag,ms,phase,sample_seq,ticks_due,service_lag_us,inference_active,frame,status,grab_ms,prep_ms,infer_ms,predicted_index,score0,score1,ch1_raw_shunt,ch1_raw_bus,ch1_shunt_uv,ch1_bus_v,ch1_current_a,ch1_current_ma,ch1_power_w,ch2_raw_shunt,ch2_raw_bus,ch2_shunt_uv,ch2_bus_v,ch2_current_a,ch2_current_ma,ch2_power_w");
}

void print_csv_header() {
  Serial.println("tag,ms,phase,sample_seq,ticks_due,service_lag_us,inference_active,frame,status,grab_ms,prep_ms,infer_ms,predicted_index,score0,score1,ch1_raw_shunt,ch1_raw_bus,ch1_shunt_uv,ch1_bus_v,ch1_current_a,ch1_current_ma,ch1_power_w,ch2_raw_shunt,ch2_raw_bus,ch2_shunt_uv,ch2_bus_v,ch2_current_a,ch2_current_ma,ch2_power_w");
}

void print_csv_sample(const InaSample& sample, uint32_t ticks_due_before_service,
                      uint32_t service_lag_us) {
  Serial.print("bb15_ina_csv,");
  Serial.print(millis());
  Serial.print(",");
  Serial.print(phase_name(g_phase));
  Serial.print(",");
  Serial.print(g_sample_sequence);
  Serial.print(",");
  Serial.print(ticks_due_before_service);
  Serial.print(",");
  Serial.print(service_lag_us);
  Serial.print(",");
  Serial.print(g_inference_active ? 1u : 0u);
  Serial.print(",");
  Serial.print(g_last_inference.frame);
  Serial.print(",");
  Serial.print(AkidaNicla::statusName(g_last_inference.status));
  Serial.print(",");
  Serial.print(g_last_inference.grab_ms);
  Serial.print(",");
  Serial.print(g_last_inference.prep_ms);
  Serial.print(",");
  Serial.print(g_last_inference.infer_ms);
  Serial.print(",");
  Serial.print(static_cast<unsigned long>(g_last_inference.predicted_index));
  Serial.print(",");
  Serial.print(g_last_inference.score0);
  Serial.print(",");
  Serial.print(g_last_inference.score1);
  Serial.print(",");
  Serial.print(sample.ch1.raw_shunt);
  Serial.print(",");
  Serial.print(sample.ch1.raw_bus);
  Serial.print(",");
  Serial.print(sample.ch1.shunt_v * 1000000.0f, 3);
  Serial.print(",");
  Serial.print(sample.ch1.bus_v, 6);
  Serial.print(",");
  Serial.print(sample.ch1.current_a, 6);
  Serial.print(",");
  Serial.print(sample.ch1.current_a * 1000.0f, 3);
  Serial.print(",");
  Serial.print(sample.ch1.power_w, 6);
  Serial.print(",");
  Serial.print(sample.ch2.raw_shunt);
  Serial.print(",");
  Serial.print(sample.ch2.raw_bus);
  Serial.print(",");
  Serial.print(sample.ch2.shunt_v * 1000000.0f, 3);
  Serial.print(",");
  Serial.print(sample.ch2.bus_v, 6);
  Serial.print(",");
  Serial.print(sample.ch2.current_a, 6);
  Serial.print(",");
  Serial.print(sample.ch2.current_a * 1000.0f, 3);
  Serial.print(",");
  Serial.println(sample.ch2.power_w, 6);
}

void service_one_pending_ina_sample() {
  if (g_inference_active) {
    return;
  }

  uint32_t ticks_due_before_service = 0u;
  uint32_t last_tick_us = 0u;
  noInterrupts();
  if (g_sample_ticks_due == 0u) {
    interrupts();
    return;
  }
  ticks_due_before_service = g_sample_ticks_due;
  --g_sample_ticks_due;
  last_tick_us = g_last_tick_us;
  interrupts();

  const InaSample sample = read_ina_sample();
  if (!sample.ok) {
    Serial.println("# warning=ina_read_failed");
    return;
  }

  const uint32_t now_us = micros();
  const uint32_t service_lag_us = now_us - last_tick_us;
  ++g_sample_sequence;
  print_csv_sample(sample, ticks_due_before_service, service_lag_us);
}

void run_one_inference_iteration() {
  if (!g_camera_ok || !g_model_ok) {
    return;
  }

  // Capture a fresh RGB565 frame from the camera.
  const uint32_t grab_start_ms = millis();
  if (g_camera.grabFrame(g_framebuffer, 3000) != 0) {
    g_last_inference.status = AKD1500Status::InvalidInput;
    g_last_inference.ok = false;
    g_last_inference.grab_ms = 0u;
    g_last_inference.prep_ms = 0u;
    g_last_inference.infer_ms = 0u;
    return;
  }
  g_last_inference.grab_ms =
      static_cast<uint16_t>(millis() - grab_start_ms);

  // Convert the center crop into the dense 224x224x3 uint8 model tensor.
  const uint32_t prep_start_ms = millis();
  fill_model_input_from_rgb565_center_crop(g_framebuffer.getBuffer());
  g_last_inference.prep_ms =
      static_cast<uint16_t>(millis() - prep_start_ms);

  // Use the post-preprocess gap as another safe point for one deferred INA
  // read before entering the synchronous classify call.
  service_one_pending_ina_sample();

  // Mark only the actual Akida classify call as inference-active so the
  // ticker can continue scheduling samples without inserting I2C reads into
  // the critical path.
  const uint32_t infer_start_ms = millis();
  g_inference_active = true;
  const AKD1500ClassificationResult result = g_akida.classifyUint8(g_model_input);
  g_inference_active = false;
  g_last_inference.infer_ms =
      static_cast<uint16_t>(millis() - infer_start_ms);

  g_last_inference.status = result.status;
  g_last_inference.ok = result.ok();
  ++g_last_inference.frame;

  if (result.ok() && result.scores.type == akida::TensorType::int32 &&
      result.scores.elementCount() >= 2u) {
    const int32_t* scores = result.data<int32_t>();
    g_last_inference.predicted_index = result.predictedIndex;
    g_last_inference.score0 = scores[0];
    g_last_inference.score1 = scores[1];
    return;
  }

  g_last_inference.predicted_index = 0u;
  g_last_inference.score0 = 0;
  g_last_inference.score1 = 0;
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

  print_boot_header();

  // Bring the BB15 board into the known-good external-flash execution state.
  if (!begin_bb15_board()) {
    print_failure("bb15_board");
    blink_forever();
  }
  Serial.println("# bb15_board=PASS");

  // Discover and calibrate the INA monitor before any steady-state sampling begins.
  if (!setup_ina()) {
    print_failure("ina_setup");
    blink_forever();
  }
  Serial.println("# ina_setup=PASS");

  // Start the camera with the fixed demo resolution used by the human classifier.
  if (!begin_camera()) {
    print_failure("camera_begin");
    blink_forever();
  }
  Serial.println("# camera_begin=PASS");

  // Link to the AKD1500 and load the already-flashed human classifier model.
  if (!begin_akida_and_load_model()) {
    print_failure("akida_begin_or_load");
    blink_forever();
  }
  Serial.println("# akida_begin_load=PASS");
  Serial.print("# akida_ip_version=0x");
  Serial.println(g_akida.ipVersion(), HEX);

  // Enter the idle baseline phase first so current can be captured with the
  // model resident but with no inference running yet.
  set_phase(RunPhase::IdleReady);
  Serial.println("# phase=idle_ready");
  print_csv_header();

  // Start the ticker only after all one-time bring-up work is complete.
  g_sample_ticker.attach(mbed::callback(on_sample_tick), kSampleIntervalMs * 1ms);
}

void loop() {
  // Service at most one pending INA sample per loop iteration and only from a
  // safe gap outside the synchronous Akida classify call.
  service_one_pending_ina_sample();

  // Hold a fixed idle window so analysis has a clean baseline before inference.
  if (g_phase == RunPhase::IdleReady &&
      (millis() - g_phase_started_ms) >= kIdleBaselineMs) {
    set_phase(RunPhase::InferLoop);
    Serial.println("# phase=infer_loop");
  }

  // Once the baseline completes, keep looping through capture, preprocess,
  // classify, then allow the deferred INA service work to catch up.
  if (g_phase == RunPhase::InferLoop) {
    run_one_inference_iteration();
    service_one_pending_ina_sample();
  }
}
