#include <Arduino.h>

#if defined(TARGET_NICLA_VISION) || defined(TARGET_NICLA)

#include <BB15.h>
#if defined(TARGET_NICLA) && !defined(TARGET_NICLA_VISION)
#include <Nicla_System.h>
#endif

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
// - `make_pinout()` for board wiring differences
// - `make_config()` for transport and flash address policy
// - `make_model()` for the actual exported model blob

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
constexpr const char* kSketchName = "bb15_model_flasher";
constexpr const char* kLogPrefix = "[bb15_model_flasher]";
constexpr const char* kBundledModelName = "presence_regular_96_gray";

bool g_flash_ok = false;
const char* g_last_failure_stage = nullptr;

// Define the physical BrainBoard15 wiring here.
//
// Users should edit this function first when porting the example to another
// host board or another BB15 stack variant.
//
// Use one of the built-in pinout presets as the base wiring definition, then
// edit the returned struct only if this host stack differs from the validated
// board mapping.
BB15Pinout make_pinout() {
  BB15Pinout pinout =
#if defined(TARGET_NICLA_VISION)
      BB15Pinout::niclaVisionDefaults();
#else
      BB15Pinout::niclaSenseMeDefaults();
#endif
  return pinout;
}

// Define runtime transport policy here.
//
// This is where users tune:
// - Akida SPI speed
// - flash SPI speed
// - flash storage address
// - optional forced flash profile
//
// Most users should only need to touch the clocks or model address.
BB15Config make_config() {
  BB15Config config = BB15Config::niclaVisionDefaults();
  config.spiClockHz = kAkidaSpiClockHz;
  config.flashSpiClockHz = kFlashSpiClockHz;
  config.defaultModelAddress =
      AkidaNicla::externalModelAddressFromOffset(kFlashModelOffset);
  return config;
}

BB15& bb15() {
  static BB15 instance(make_pinout(), make_config());
  return instance;
}

BB15Runner& runner() {
  static BB15Runner instance = bb15().createRunner();
  return instance;
}

// Describe the model that will be written into BB15 external flash.
//
// To adapt this example to another model, users typically replace:
// - `program.h`
// - `program.cpp`
// - `model_metadata.h`
// - `model_metadata.cpp`
//
// Then they can keep this function structure and only change the flash address
// if they do not want to write at offset zero.
BB15Model make_model() {
  BB15Model model(program, static_cast<size_t>(program_len));
  model.setStorage(BB15ModelStorage::ExternalFlash)
      .setExternalAddress(AkidaNicla::externalModelAddressFromOffset(
          kFlashModelOffset));
  return model;
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
  bb15().printLastError(Serial);
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

// Bring the BB15 board online and confirm that the external flash is present
// before attempting to write any model bytes.
bool prepare_board() {
  if (bb15().begin() != BB15Status::Ok) {
    print_failure("board_begin");
    return false;
  }
  Serial.print(kLogPrefix);
  Serial.println(" board_setup result=PASS");

  if (!bb15().detectFlash()) {
    print_failure("detect_flash");
    return false;
  }
  const BB15FlashInfo flash = bb15().flashInfo();
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
  const BB15Model model = make_model();

  if (static_cast<int64_t>(program_len) != akida_program_length_bytes) {
    print_failure("program_length_mismatch");
    return false;
  }

  Serial.print(kLogPrefix);
  Serial.print(" flash_model address=0x");
  Serial.println(model.externalAddress(), HEX);
  if (!bb15().flashModel(model)) {
    print_failure("flash_model");
    return false;
  }

  if (!bb15().verifyModel(model)) {
    print_failure("verify_model");
    return false;
  }
  Serial.print(kLogPrefix);
  Serial.println(" flash_verify result=PASS");

  if (runner().begin() != BB15Status::Ok) {
    print_failure("runner_begin");
    return false;
  }
  if (runner().loadModel(model) != BB15Status::Ok) {
    print_failure("load_external_model");
    return false;
  }
  Serial.print(kLogPrefix);
  Serial.print(" akida ip_version=0x");
  Serial.println(bb15().ipVersion(), HEX);
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
  print_model_summary();

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
      "bb15_model_flasher: runtime support requires a supported Nicla board + BB15");
}

void loop() {}

#endif
