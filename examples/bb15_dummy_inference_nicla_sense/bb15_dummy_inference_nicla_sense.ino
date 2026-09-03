#include <Arduino.h>
#include <BB15.h>
#include <Nicla_System.h>

#include "model_metadata.h"
#include "program.h"

namespace {

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
constexpr uint8_t kExpanderInterruptPin = 2u;
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
constexpr uint32_t kAkidaRegGpioOutputEnable = kAkidaSystemConfigBase + 0x38u;
constexpr uint32_t kAkidaRegGpioMuxEnable = kAkidaSystemConfigBase + 0x3Cu;
constexpr uint32_t kAkidaRegGpioDriveStrength = kAkidaSystemConfigBase + 0x40u;
constexpr uint32_t kAkidaInterruptPadMask = 1u << kAkidaInterruptPadIndex;
constexpr uint32_t kInterruptWaitTimeoutMs = 125u;
constexpr uint32_t kFetchFallbackTimeoutMs = 125u;
constexpr uint32_t kInterruptPollDelayMs = 1u;
constexpr size_t kModelInputBytes =
    static_cast<size_t>(kModelWidth) * kModelHeight * kModelChannels;
constexpr const char* kSketchName = "bb15_dummy_inference_nicla_sense";
constexpr const char* kLogPrefix = "[bb15_dummy_inference_nicla_sense]";
constexpr const char* kBundledModelName = "presence_regular_96_gray";

bool g_ready = false;
const char* g_last_failure_stage = nullptr;
uint8_t g_input[kModelInputBytes];
volatile bool g_akida_done = false;
volatile uint32_t g_irq_count = 0u;
bool g_interrupt_mode_ready = false;

BB15Pinout g_pinout = BB15Pinout::niclaSenseMeDefaults();

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
      .setExternalAddress(
          AkidaNicla::externalModelAddressFromOffset(kFlashModelOffset));
  return model;
}();

BB15& board() { return *g_bb15; }
BB15Runner& runner() { return *g_runner; }
TwoWire& expander_bus() { return *board().config().wire; }

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

  gpio_oe |= kAkidaInterruptPadMask;
  gpio_mux &= ~kAkidaInterruptPadMask;
  gpio_drv |= kAkidaInterruptPadMask;
  return runner().writeRegister32(kAkidaRegGpioOutputEnable, gpio_oe) &&
         runner().writeRegister32(kAkidaRegGpioMuxEnable, gpio_mux) &&
         runner().writeRegister32(kAkidaRegGpioDriveStrength, gpio_drv);
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

bool validate_export_geometry() {
  return akida_input_rank == 3 && akida_input_shape[0] == kModelWidth &&
         akida_input_shape[1] == kModelHeight &&
         akida_input_shape[2] == kModelChannels &&
         static_cast<int64_t>(program_len) == akida_program_length_bytes;
}

void print_model_summary() {
  Serial.print(kLogPrefix);
  Serial.print(" model=");
  Serial.print(kBundledModelName);
  Serial.print(" bytes=");
  Serial.println(static_cast<long>(akida_program_length_bytes));
}

void build_demo_input(uint32_t pass_index) {
  for (uint16_t y = 0u; y < kModelHeight; ++y) {
    for (uint16_t x = 0u; x < kModelWidth; ++x) {
      const size_t index = static_cast<size_t>(y) * kModelWidth + x;
      const uint16_t stripe = static_cast<uint16_t>((x / 12u) + (y / 12u));
      const uint8_t base =
          ((stripe + static_cast<uint16_t>(pass_index)) & 1u) != 0u ? 208u
                                                                    : 32u;
      const uint8_t diagonal =
          ((x + y + static_cast<uint16_t>(pass_index * 3u)) % 29u) == 0u ? 255u
                                                                         : base;
      g_input[index] = diagonal;
    }
  }
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
  return runner().modelInfo().valid;
}

void run_demo_inference(uint32_t pass_index) {
  if (!g_interrupt_mode_ready) {
    print_failure("interrupt_mode_begin_required");
    return;
  }

  build_demo_input(pass_index);
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
  Serial.print(" source=synthetic completion=");
  Serial.print(completion_source);
  Serial.print(" irq_count_delta=");
  Serial.print(static_cast<unsigned long>(g_irq_count - irq_count_before));
  Serial.print(" wait_ms=");
  Serial.print(static_cast<unsigned long>(millis() - start_ms));
  Serial.print(" predicted_index=");
  Serial.println(static_cast<unsigned long>(result.predictedIndex));
  print_scores(result);
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
  Serial.println(" purpose=load flashed model and run dummy inference");
  print_model_summary();

  if (!validate_export_geometry()) {
    print_failure("validate_export_geometry");
    blink_forever();
  }
  if (!prepare_board_and_runner()) {
    print_failure("prepare_board_and_runner");
    blink_forever();
  }
  Serial.print(kLogPrefix);
  Serial.println(" input_source=synthetic");

  if (!begin_interrupt_mode()) {
    print_failure("interrupt_mode_begin");
    blink_forever();
  }

  Serial.print(kLogPrefix);
  Serial.println(" completion_mode=interrupt");

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
