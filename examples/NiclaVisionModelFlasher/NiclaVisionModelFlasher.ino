#include <Arduino.h>

#include <AKD1500.h>

#include "bb15_demo_board.h"
#include "program.h"

#ifndef ARDUINO_NICLA_VISION
#error "This example is intended for Nicla Vision."
#endif

namespace {

// Serial timing for a readable first-time bring-up sketch.
constexpr uint32_t kSerialBaud = 115200u;
constexpr uint32_t kSerialWaitMs = 3000u;
constexpr uint32_t kBootSettleMs = 250u;
constexpr uint32_t kHeartbeatMs = 1000u;

// Keep the flashed model at the first external-flash slot used by the camera
// demo so both examples stay aligned.
constexpr uint32_t kFlashModelOffset = 0u;

// Expose the clocks here so users can tune the flash path from the IDE.
constexpr uint32_t kAkidaSpiClockHz = 8000000u;
constexpr uint32_t kFlashSpiClockHz = 2000000u;

constexpr const char* kSketchName = "NiclaVisionModelFlasher";

bb15_demo::Bb15NiclaVisionBoard& board() {
  static bb15_demo::Bb15NiclaVisionBoard instance;
  return instance;
}

AkidaNicla& niclaAkida() {
  static AkidaNicla instance;
  return instance;
}

const char* g_last_failure_stage = nullptr;
const char* g_last_failure_detail = nullptr;
bool g_flash_ok = false;
uint32_t g_last_heartbeat_ms = 0u;

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
  Serial.print("[model_flasher] result=FAIL stage=");
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

void blink_forever() {
  for (;;) {
    set_led(true);
    delay(50);
    set_led(false);
    delay(950);
    if (g_last_failure_stage != nullptr) {
      Serial.print("[model_flasher] halted stage=");
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

AKD1500Options make_options() {
  AKD1500Options options = AKD1500Options::niclaVisionDefaults();
  options.spiClockHz = kAkidaSpiClockHz;
  options.flashSpiClockHz = kFlashSpiClockHz;
  options.externalModelAddress =
      AkidaNicla::externalModelAddressFromOffset(kFlashModelOffset);
  return options;
}

AKD1500Model make_model() {
  AKD1500Model model;
  model.serializedProgram = program;
  model.size = static_cast<size_t>(program_len);
  model.storage = AKD1500ModelStorage::ExternalFlash;
  model.externalLocation =
      AkidaNicla::externalModelAddressFromOffset(kFlashModelOffset);
  return model;
}

void print_spi_configuration(const AKD1500Options& options) {
  Serial.print("[model_flasher] spi akida_hz=");
  Serial.print(static_cast<unsigned long>(options.spiClockHz));
  Serial.print(" flash_hz=");
  Serial.println(static_cast<unsigned long>(options.flashSpiClockHz));
}

bool flash_and_verify_model() {
  const AKD1500Options options = make_options();
  const uint32_t flash_address =
      AkidaNicla::externalModelAddressFromOffset(kFlashModelOffset);

  // Stage the bundled human-classifier payload into BB15 external flash.
  Serial.print("[model_flasher] flash_stage address=0x");
  Serial.println(flash_address, HEX);
  if (!AkidaNicla::stageModelToFlash(options, program,
                                     static_cast<size_t>(program_len),
                                     kFlashModelOffset)) {
    print_failure("stage_model_to_flash");
    return false;
  }
  Serial.println("[model_flasher] flash_stage result=PASS");

  // Read the same bytes back through the supported verify helper.
  if (!AkidaNicla::verifyModelInFlash(options, program,
                                      static_cast<size_t>(program_len),
                                      kFlashModelOffset)) {
    print_failure("verify_model_in_flash");
    return false;
  }
  Serial.println("[model_flasher] flash_verify result=PASS");

  // Link to the AKD1500 and confirm the flashed model can be loaded.
  if (niclaAkida().begin(options) != AKD1500Status::Ok) {
    print_failure("akida_begin");
    return false;
  }
  Serial.print("[model_flasher] akida ip_version=0x");
  Serial.println(niclaAkida().ipVersion(), HEX);

  if (niclaAkida().load(make_model()) != AKD1500Status::Ok) {
    print_failure("load_external_model");
    return false;
  }
  Serial.print("[model_flasher] model_info ");
  niclaAkida().printModelInfo(Serial);
  return true;
}

}  // namespace

void setup() {
  // Start the onboard status LEDs before any board-level bring-up.
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(LEDR, OUTPUT);
  set_led(false);
  digitalWrite(LEDR, HIGH);

  // Bring up USB serial so the user can watch the full flash flow.
  Serial.begin(kSerialBaud);
  wait_for_serial();
  delay(kBootSettleMs);

  const AKD1500Options options = make_options();
  Serial.println();
  Serial.println(kSketchName);
  Serial.println("[model_flasher] board=Nicla Vision + BB15");
  Serial.println("[model_flasher] bus=Wire SDA=D11 SCL=D12");
  Serial.println("[model_flasher] wiring=akida_cs=D7 bridge_cs=D1 reset=D3 expander=0x43");
  Serial.println("[model_flasher] model=human classifier bundled in this example");
  print_spi_configuration(options);

  // Configure the BB15 expander and AKD reset sequence before flash access.
  if (!board().begin()) {
    print_failure("board_begin", board().lastError());
    blink_forever();
  }
  Serial.println("[model_flasher] board_setup result=PASS");

  if (!flash_and_verify_model()) {
    blink_forever();
  }

  Serial.println("[model_flasher] result=PASS");
  Serial.println("[model_flasher] next_step=open NiclaVisionCameraFlashClassify and upload it");
  g_flash_ok = true;
  g_last_heartbeat_ms = millis();
}

void loop() {
  set_led(true);
  delay(50);
  set_led(false);
  delay(50);
  if (g_flash_ok && (millis() - g_last_heartbeat_ms) >= kHeartbeatMs) {
    g_last_heartbeat_ms = millis();
    Serial.println("[model_flasher] ready flashed_model=human_classifier");
  }
  delay(900);
}
