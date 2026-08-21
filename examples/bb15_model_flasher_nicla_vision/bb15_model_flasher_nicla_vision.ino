#include <Arduino.h>

#define AKD1500_LIBRARY_ENABLE_LOGS 1
#include <BB15.h>

#include "model_metadata.h"
#include "program.h"

namespace {

// This example is intentionally verbose because it is the main starting point
// for users who want to place their own Akida model in BB15 external flash.
//
// The user workflow is:
// 1. export a model to `program.h` / `program.cpp`
// 2. update the pinout below to match the host + BB15 wiring
// 3. upload this sketch once
// 4. confirm that flashing and verification pass
// 5. upload an inference sketch that loads the model from flash
//
// The three user-edit areas are:
// - `g_pinout` for board wiring differences
// - `g_config` for transport and flash address policy
// - `g_model` for the actual exported model blob

constexpr uint32_t kSerialBaud = 115200u;
constexpr uint32_t kSerialWaitMs = 3000u;
constexpr uint32_t kBootSettleMs = 250u;

// This is the offset inside BB15 bridge flash where the model will be staged.
// Keeping it at zero is the simplest default while the flash stores one model.
// If the user later wants multiple assets in flash, this is one of the first
// values they would change.
constexpr uint32_t kFlashModelOffset = 0u;

// These SPI clocks are the first transport settings most users will tune.
// They are intentionally sketch-level constants so the example remains easy to
// inspect and adapt without digging through library internals.
constexpr uint32_t kAkidaSpiClockHz = 25000000u;
constexpr uint32_t kFlashSpiClockHz = 2000000u;
constexpr const char* kSketchName = "bb15_model_flasher_nicla_vision";
constexpr const char* kLogPrefix = "[bb15_model_flasher_nicla_vision]";
// This Vision flasher installs the model used by
// `bb15_nicla_vision_human_detection`. Keep its asset files in sync with that
// sketch: the inference example deliberately only loads an already-flashed
// model and never modifies external flash.
constexpr const char* kBundledModelName =
    "NiclaV_VWW_PersonDet_EN_USBbottom_2026-06-14";

bool g_flash_ok = false;
const char* g_last_failure_stage = nullptr;
BB15Pinout g_pinout = BB15Pinout::niclaVisionDefaults();

// The sketch keeps the three user-edit surfaces as direct objects:
// - `g_pinout` describes the physical wiring
// - `g_config` describes runtime transport policy
// - `g_model` describes the exported model blob
//
// That shape mirrors the intended public API exactly, so users can copy the
// same object graph into their own application. Construction itself is delayed
// until `setup()`, because the `BB15` constructor now touches hardware state
// and that is not safe during Arduino global initialization.
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

BB15& board() { return *g_bb15; }
BB15Runner& runner() { return *g_runner; }

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
  Serial.print(" ip_version=0x");
  Serial.print(board().ipVersion(), HEX);
  Serial.print(" detail=");
  board().printLastError(Serial);
}

// Fail loudly and visibly so a user at the bench can immediately see that the
// model did not finish flashing correctly.
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

// Keep the default runtime output short. The sketch source itself carries the
// detailed guidance; the serial output should focus on the steps the user needs
// to validate at the bench.
void print_model_summary() {
  Serial.print(kLogPrefix);
  Serial.print(" model=");
  Serial.print(kBundledModelName);
  Serial.print(" bytes=");
  Serial.println(static_cast<long>(akida_program_length_bytes));
}

void print_board_configuration() {
  Serial.print(kLogPrefix);
  Serial.print(" spi akida_hz=");
  Serial.print(static_cast<unsigned long>(g_config.spiClockHz));
  Serial.print(" flash_hz=");
  Serial.println(static_cast<unsigned long>(g_config.flashSpiClockHz));

  Serial.print(kLogPrefix);
  Serial.print(" pinout ");
  board().printSummary(Serial);
}

// Bring the BB15 board online and confirm that the external flash is present
// before attempting to write any model bytes.
bool prepare_board() {
  if (board().begin() != BB15Status::Ok) {
    print_failure("board_begin");
    return false;
  }
  Serial.print(kLogPrefix);
  Serial.println(" board_setup result=PASS");

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
  return true;
}

// This is the core model staging sequence:
// - build a `BB15Model` descriptor from the exported bytes
// - sanity-check the expected program size
// - write the bytes into external flash
// - verify the bytes by reading them back
// - load the flashed model once so the user gets immediate proof that the
//   runtime can see and parse it correctly
bool flash_and_verify_model() {
  if (static_cast<int64_t>(program_len) != akida_program_length_bytes) {
    print_failure("program_length_mismatch");
    return false;
  }

  Serial.print(kLogPrefix);
  Serial.print(" flash_model address=0x");
  Serial.println(g_model.externalAddress(), HEX);
  if (!board().flashModel(g_model)) {
    print_failure("flash_model");
    return false;
  }

  if (!board().verifyModel(g_model)) {
    print_failure("verify_model");
    return false;
  }
  Serial.print(kLogPrefix);
  Serial.println(" flash_verify result=PASS");

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
  return true;
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
  print_model_summary();
  print_board_configuration();

  if (!prepare_board()) {
    blink_forever();
  }
  if (!flash_and_verify_model()) {
    blink_forever();
  }

  Serial.print(kLogPrefix);
  Serial.println(" result=PASS");
  Serial.print(kLogPrefix);
  Serial.println(" next_step=upload the inference example");
  g_flash_ok = true;
}

void loop() {
  set_led(true);
  delay(50);
  set_led(false);
  delay(g_flash_ok ? 950 : 50);
}
