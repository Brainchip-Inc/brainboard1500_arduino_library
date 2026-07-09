#include <AKD1500.h>
#include <Arduino.h>

#include "akd1500_fast_flash.h"
#include "bb15_demo_board.h"
#include "program_info_blob.h"

namespace {

constexpr uint32_t kSerialBaud = 115200u;
constexpr uint32_t kLoopDelayMs = 1500u;
constexpr uint32_t kFlashModelOffset = 0u;

constexpr uint16_t kModelWidth = 96u;
constexpr uint16_t kModelHeight = 96u;
constexpr uint16_t kModelChannels = 3u;
constexpr size_t kModelInputBytes =
    static_cast<size_t>(kModelWidth) * kModelHeight * kModelChannels;

uint8_t g_model_input[kModelInputBytes];
bool g_model_ok = false;
uint32_t g_inference_count = 0u;

bb15_demo::Bb15NiclaVisionBoard& board() {
  static bb15_demo::Bb15NiclaVisionBoard instance;
  return instance;
}

akd1500_fast_flash::Config make_mkr_config() {
  akd1500_fast_flash::Config config;
  config.akidaSpiClockHz = 12000000u;
  config.flashSpiClockHz = 4000000u;
  config.externalModelOffset = kFlashModelOffset;
  config.expectedIpVersion = 0xBCA10309u;
  config.flashProfilePolicy = akd1500_fast_flash::FlashProfilePolicy::Auto;
  config.verboseRuntimeDiagnostics = false;
  config.akidaCsPin = 20u;
  config.bridgeCsPin = 13u;
  return config;
}

akd1500_fast_flash::FastFlashModelRunner& flashModelRunner() {
  static akd1500_fast_flash::FastFlashModelRunner instance(make_mkr_config());
  return instance;
}

AKD1500Model make_preflashed_model() {
  AKD1500Model model;
  model.serializedProgram = program_info_blob;
  model.size = static_cast<size_t>(program_info_blob_len);
  model.storage = AKD1500ModelStorage::ExternalFlash;
  model.externalLocation =
      AkidaNicla::externalModelAddressFromOffset(kFlashModelOffset);
  return model;
}

void generate_predictable_noise(uint32_t iteration) {
  if ((iteration / 5u) % 2u == 0u) {
    for (size_t i = 0; i < kModelInputBytes; ++i) {
      g_model_input[i] = static_cast<uint8_t>(12u + (i % 4u));
    }
    Serial.println(F("IN:DK"));
    return;
  }

  for (size_t i = 0; i < kModelInputBytes; ++i) {
    g_model_input[i] = (i % 2u == 0u) ? 245u : 10u;
  }
  Serial.println(F("IN:ST"));
}

bool prepare_preflashed_model() {
  if (flashModelRunner().load(make_preflashed_model()) != AKD1500Status::Ok) {
    Serial.println(F("L:ERR"));
    return false;
  }
  g_model_ok = true;
  Serial.print(F("PI:"));
  Serial.println(static_cast<unsigned long>(program_info_blob_len));
  Serial.println(F("L:OK"));
  return true;
}

}  // namespace

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  Serial.begin(kSerialBaud);
  while (!Serial && millis() < 2000u) {
  }

  Serial.println(F("START"));
  Serial.println(F("MODE:PREFLASHED"));

  if (!board().begin()) {
    Serial.println(F("B:ERR"));
    while (true) {
      delay(1000);
    }
  }
  Serial.println(F("B:OK"));

  if (!board().coldBootAkidaIntoExternalFlashMode(250u)) {
    Serial.println(F("BOOT:ERR"));
    while (true) {
      delay(1000);
    }
  }

  if (!prepare_preflashed_model()) {
    while (true) {
      delay(1000);
    }
  }
}

void loop() {
  if (!g_model_ok) {
    delay(1000);
    return;
  }

  digitalWrite(LED_BUILTIN, HIGH);

  generate_predictable_noise(g_inference_count);

  const uint32_t start_ms = millis();
  const AKD1500ClassificationResult result =
      flashModelRunner().classifyUint8(g_model_input);
  const uint32_t elapsed_ms = millis() - start_ms;

  Serial.print(F("#"));
  Serial.print(g_inference_count);
  Serial.print(F(" ST:"));
  Serial.print(static_cast<int>(result.status));
  Serial.print(F(" ID:"));
  Serial.print(result.predictedIndex);
  Serial.print(F(" MS:"));
  Serial.println(elapsed_ms);

  digitalWrite(LED_BUILTIN, LOW);

  ++g_inference_count;
  delay(kLoopDelayMs);
}
