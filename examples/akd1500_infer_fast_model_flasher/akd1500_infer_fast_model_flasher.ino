#include <Arduino.h>

#include <AKD1500.h>

#include "bb15_demo_board.h"
#include "model_metadata.h"
#include "program.h"

#ifndef ARDUINO_NICLA_VISION
#error "This example is intended for Nicla Vision."
#endif

namespace {

constexpr uint32_t kSerialBaud = 115200u;
constexpr uint32_t kSerialWaitMs = 3000u;
constexpr uint32_t kBootSettleMs = 250u;
constexpr uint32_t kHeartbeatMs = 1000u;
constexpr uint32_t kFlashModelOffset = 0u;
constexpr uint32_t kAkidaSpiClockHz = 8000000u;
constexpr uint32_t kFlashSpiClockHz = 2000000u;
constexpr const char* kSketchName = "akd1500_infer_fast_model_flasher";
constexpr const char* kLogPrefix = "[akd1500_infer_fast_model_flasher]";
constexpr const char* kBundledModelName = "presence_regular_96_gray";

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

void set_led(bool active) { digitalWrite(LED_BUILTIN, active ? LOW : HIGH); }

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
      Serial.print(kLogPrefix);
      Serial.print(" halted stage=");
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

void print_shape3(const uint32_t* shape) {
  Serial.print("[");
  Serial.print(static_cast<unsigned long>(shape[0]));
  Serial.print(", ");
  Serial.print(static_cast<unsigned long>(shape[1]));
  Serial.print(", ");
  Serial.print(static_cast<unsigned long>(shape[2]));
  Serial.println("]");
}

void print_export_metadata() {
  Serial.print(kLogPrefix);
  Serial.print(" export model_path=");
  Serial.println(akida_model_path);
  Serial.print(kLogPrefix);
  Serial.print(" export program_len=");
  Serial.println(static_cast<long>(akida_program_length_bytes));
  Serial.print(kLogPrefix);
  Serial.print(" export input_shape=");
  print_shape3(akida_input_shape);
  Serial.print(kLogPrefix);
  Serial.print(" export output_shape=");
  print_shape3(akida_output_shape);
}

void print_spi_configuration(const AKD1500Options& options) {
  Serial.print(kLogPrefix);
  Serial.print(" spi akida_hz=");
  Serial.print(static_cast<unsigned long>(options.spiClockHz));
  Serial.print(" flash_hz=");
  Serial.println(static_cast<unsigned long>(options.flashSpiClockHz));
}

bool flash_and_verify_model() {
  const AKD1500Options options = make_options();
  const uint32_t flash_address =
      AkidaNicla::externalModelAddressFromOffset(kFlashModelOffset);

  if (static_cast<int64_t>(program_len) != akida_program_length_bytes) {
    print_failure("program_length_mismatch");
    return false;
  }

  Serial.print(kLogPrefix);
  Serial.print(" flash_stage address=0x");
  Serial.println(flash_address, HEX);
  if (!AkidaNicla::stageModelToFlash(options, program,
                                     static_cast<size_t>(program_len),
                                     kFlashModelOffset)) {
    print_failure("stage_model_to_flash");
    return false;
  }
  Serial.print(kLogPrefix);
  Serial.println(" flash_stage result=PASS");

  if (!AkidaNicla::verifyModelInFlash(options, program,
                                      static_cast<size_t>(program_len),
                                      kFlashModelOffset)) {
    print_failure("verify_model_in_flash");
    return false;
  }
  Serial.print(kLogPrefix);
  Serial.println(" flash_verify result=PASS");

  if (niclaAkida().begin(options) != AKD1500Status::Ok) {
    print_failure("akida_begin");
    return false;
  }
  Serial.print(kLogPrefix);
  Serial.print(" akida ip_version=0x");
  Serial.println(niclaAkida().ipVersion(), HEX);

  if (niclaAkida().load(make_model()) != AKD1500Status::Ok) {
    print_failure("load_external_model");
    return false;
  }
  Serial.print(kLogPrefix);
  Serial.print(" model_info ");
  niclaAkida().printModelInfo(Serial);
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

  const AKD1500Options options = make_options();
  Serial.println();
  Serial.println(kSketchName);
  Serial.print(kLogPrefix);
  Serial.println(" board=Nicla Vision + BB15");
  Serial.print(kLogPrefix);
  Serial.println(" bus=Wire SDA=D11 SCL=D12");
  Serial.print(kLogPrefix);
  Serial.println(" wiring=proceed=D2/PA_10 HIGH akida_cs=D7 bridge_cs=D1 akida_reset_n=D3 expander=0x43");
  Serial.print(kLogPrefix);
  Serial.print(" model=");
  Serial.println(kBundledModelName);
  print_spi_configuration(options);
  print_export_metadata();

  if (!board().begin()) {
    print_failure("board_begin", board().lastError());
    blink_forever();
  }
  Serial.print(kLogPrefix);
  Serial.println(" board_setup result=PASS");

  if (!flash_and_verify_model()) {
    blink_forever();
  }

  Serial.print(kLogPrefix);
  Serial.println(" result=PASS");
  Serial.print(kLogPrefix);
  Serial.println(" next_step=open akd1500_infer_fast and upload it");
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
    Serial.print(kLogPrefix);
    Serial.print(" ready flashed_model=");
    Serial.println(kBundledModelName);
  }
  delay(900);
}
