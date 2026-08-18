#include <Arduino.h>

#if defined(TARGET_NICLA_VISION) || defined(TARGET_NICLA)

#include <BB15.h>
#if defined(TARGET_NICLA) && !defined(TARGET_NICLA_VISION)
#include <Nicla_System.h>
#endif

#include "model_metadata.h"
#include "program.h"

namespace {

constexpr uint32_t kSerialBaud = 115200u;
constexpr uint32_t kSerialWaitMs = 3000u;
constexpr uint32_t kBootSettleMs = 250u;
constexpr uint32_t kFlashModelOffset = 0u;
constexpr uint32_t kAkidaSpiClockHz = 25000000u;
constexpr uint32_t kFlashSpiClockHz = 2000000u;
constexpr uint32_t kSleepHoldMs = 5000u;
constexpr uint32_t kLoopDelayMs = 1000u;
constexpr uint16_t kModelWidth = 96u;
constexpr uint16_t kModelHeight = 96u;
constexpr uint16_t kModelChannels = 1u;
constexpr size_t kModelInputBytes =
    static_cast<size_t>(kModelWidth) * kModelHeight * kModelChannels;
constexpr const char* kSketchName = "bb15_sleep_wake";
constexpr const char* kLogPrefix = "[bb15_sleep_wake]";
constexpr const char* kBundledModelName = "presence_regular_96_gray";

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

BB15Model g_model = []() {
  BB15Model model(program, static_cast<size_t>(program_len));
  model.setStorage(BB15ModelStorage::ExternalFlash)
      .setExternalAddress(AkidaNicla::externalModelAddressFromOffset(
          kFlashModelOffset));
  return model;
}();

BB15* g_bb15 = nullptr;
BB15Runner* g_runner = nullptr;
uint8_t g_input[kModelInputBytes];
uint32_t g_cycle = 0u;

BB15& board() { return *g_bb15; }
BB15Runner& runner() { return *g_runner; }

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

void blink_forever() {
  for (;;) {
    set_led(true);
    delay(100);
    set_led(false);
    delay(900);
  }
}

void print_model_summary() {
  Serial.print(kLogPrefix);
  Serial.print(" model=");
  Serial.print(kBundledModelName);
  Serial.print(" bytes=");
  Serial.println(static_cast<long>(akida_program_length_bytes));
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

// The intended public lifecycle is:
// 1. construct `BB15` once `setup()` starts
// 2. call `begin()` for full board bring-up
// 3. load a flashed model through `BB15Runner`
// 4. call `sleep()` when Akida should idle
// 5. call `wake()`, then `begin()` again before using it normally
bool begin_runtime() {
  if (board().begin() != BB15Status::Ok) {
    print_failure("board_begin");
    return false;
  }
  if (!board().detectFlash()) {
    print_failure("detect_flash");
    return false;
  }
  if (runner().begin() != BB15Status::Ok) {
    print_failure("runner_begin");
    return false;
  }
  if (runner().loadModel(g_model) != BB15Status::Ok) {
    print_failure("load_model");
    return false;
  }

  Serial.print(kLogPrefix);
  Serial.print(" runtime_ready ip_version=0x");
  Serial.println(board().ipVersion(), HEX);
  return true;
}

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

bool run_inference_once(uint32_t pass_index) {
  build_demo_input(pass_index);
  const akida::Shape dims = {1u, kModelHeight, kModelWidth, kModelChannels};
  const BB15ClassificationResult result = runner().classify(g_input, dims);
  if (!result.ok()) {
    print_failure("classify");
    return false;
  }

  Serial.print(kLogPrefix);
  Serial.print(" infer pass=");
  Serial.print(static_cast<unsigned long>(pass_index));
  Serial.print(" predicted_index=");
  Serial.println(static_cast<unsigned long>(result.predictedIndex));
  print_scores(result.scores);
  return true;
}

bool sleep_and_wake_cycle() {
  Serial.print(kLogPrefix);
  Serial.print(" sleep enter hold_ms=");
  Serial.println(static_cast<unsigned long>(kSleepHoldMs));
  if (!board().sleep()) {
    print_failure("sleep");
    return false;
  }
  delay(kSleepHoldMs);

  Serial.print(kLogPrefix);
  Serial.println(" wake begin");
  if (!board().wake()) {
    print_failure("wake");
    return false;
  }
  return begin_runtime();
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

  static BB15 bb15(g_pinout, g_config);
  static BB15Runner bb15_runner = bb15.createRunner();
  g_bb15 = &bb15;
  g_runner = &bb15_runner;

  Serial.println();
  Serial.println(kSketchName);
  Serial.print(kLogPrefix);
  Serial.println(" board=BB15");
  Serial.print(kLogPrefix);
  Serial.println(" constructor_reset_state=released");
  Serial.print(kLogPrefix);
  Serial.println(" purpose=demonstrate sleep() and wake() with a flashed model");
  print_model_summary();

  if (!validate_export_geometry()) {
    blink_forever();
  }
  if (!begin_runtime()) {
    blink_forever();
  }
}

void loop() {
  if (!run_inference_once(g_cycle++)) {
    blink_forever();
  }
  if (!sleep_and_wake_cycle()) {
    blink_forever();
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
      "bb15_sleep_wake: runtime support requires a supported Nicla board + BB15");
}

void loop() {}

#endif
