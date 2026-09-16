#include <Arduino.h>
#include <ArduinoBLE.h>
#include <BB15.h>
#include <STM32H747_System.h>

#include "ble_model_transfer.h"
#include "ble_protocol.h"
#include "device_status.h"
#include "kws_pipeline.h"

#ifndef ARDUINO_NICLA_VISION
#error "bb15_nicla_vision_connect requires Arduino Nicla Vision."
#endif

namespace {

constexpr uint32_t kSerialBaud = 115200u;
constexpr uint32_t kSerialWaitMs = 3000u;
constexpr uint32_t kBootSettleMs = 250u;
constexpr uint32_t kAkidaSpiClockHz = 25000000u;

constexpr const char* kSketchName = "bb15_nicla_vision_connect";
constexpr const char* kLogPrefix = "[bb15_nicla_vision_connect]";

BB15Pinout g_pinout = BB15Pinout::niclaVisionDefaults();
BB15Config g_config = []() {
  BB15Config config = BB15Config::defaults();
  config.spiClockHz = kAkidaSpiClockHz;
  return config;
}();

BB15* g_board = nullptr;
BB15Runner* g_runner = nullptr;
bool g_was_connected = false;

// A spare RTC backup register, which survives a reset but not a power cycle.
// It carries a cookie saying whether the previous run ended by asking to
// restart, which is the only way to tell a commanded restart from a crash: the
// bootloader clears RCC_RSR before the sketch runs, and the reset reason it is
// meant to leave behind in DR8 is never written.
constexpr RTCBackup kRunStateRegister = RTCBackup::DR9;
constexpr uint32_t kRunStateRunning = 0x42423135u;
constexpr uint32_t kRunStateCommanded = 0x42423100u;

/**
 * @brief Say how the previous run ended, then mark this one as under way.
 *
 * @return "commanded" when the last run asked to restart, "unexpected" when it
 *         stopped without asking, and "cold" when there was no previous run.
 */
const char* take_previous_run_outcome() {
  const uint32_t cookie = STM32H747::readBackupRegister(kRunStateRegister);
  STM32H747::writeBackupRegister(kRunStateRegister, kRunStateRunning);

  switch (cookie) {
    case kRunStateCommanded:
      return "commanded";
    case kRunStateRunning:
      return "unexpected";
    default:
      return "cold";
  }
}

/** @brief Give a host a moment to open the USB serial port. */
void wait_for_serial() {
  const uint32_t start_ms = millis();
  while (!Serial && (millis() - start_ms) < kSerialWaitMs) {
  }
}

/**
 * @brief Bring the BrainBoard up ready to be given a model.
 *
 * No model is loaded here: it arrives over Bluetooth, or is restored from the
 * board's own flash by the transfer module.
 *
 * @return True when the board and its runner are ready.
 */
bool prepare_board() {
  static BB15 board(g_pinout, g_config);
  static BB15Runner runner = board.createRunner();
  g_board = &board;
  g_runner = &runner;

  return board.begin() == BB15Status::Ok && runner.begin() == BB15Status::Ok;
}

/**
 * @brief Follow the model the transfer module reports.
 *
 * @param loaded  Description of the model now on the BrainBoard, or an invalid
 *                one when the board has stopped having a model to run.
 */
void on_model_loaded(const model::Loaded& loaded) {
  if (!loaded.valid) {
    kws::setModel(nullptr, kws::ModelParameters());
    Serial.print(kLogPrefix);
    Serial.println(" model cleared, the board has none to run");
    return;
  }

  kws::ModelParameters parameters;
  parameters.programInfo = loaded.programInfo;
  parameters.programInfoBytes = loaded.programInfoBytes;
  parameters.dataAddress = loaded.dataAddress;
  parameters.classCount = loaded.classCount;
  parameters.silenceClass = loaded.silenceClass;
  parameters.unknownClass = loaded.unknownClass;
  parameters.mfccFullScale = loaded.mfccFullScale;

  const bool ready = kws::setModel(g_runner, parameters);

  Serial.print(kLogPrefix);
  Serial.print(" model=");
  Serial.print(loaded.name);
  Serial.print(" classes=");
  Serial.print(loaded.classCount);
  Serial.print(" program_bytes=");
  Serial.print(static_cast<unsigned long>(loaded.programBytes));
  Serial.print(" program_info_bytes=");
  Serial.print(static_cast<unsigned long>(loaded.programInfoBytes));
  Serial.print(" ready=");
  Serial.println(ready ? 1 : 0);
}

/**
 * @brief Show a model transfer on the LED and keep audio out of its way.
 *
 * @param busy  True while a transfer is running.
 */
void on_transfer_activity(bool busy) {
  if (busy) {
    kws::setInferenceRunning(false);
    device::setLedMode(device::LedMode::ModelTransfer);
    return;
  }
  device::setLedMode(BLE.connected() ? device::LedMode::Connected
                                     : device::LedMode::Advertising);
}

/**
 * @brief Start or stop scoring audio.
 *
 * @param running  True to run the detector.
 */
void on_deploy(bool running) { kws::setInferenceRunning(running); }

/**
 * @brief Start or stop the waveform stream.
 *
 * @param streaming  True to send the microphone envelope.
 */
void on_stream(bool streaming) { kws::setWaveformStreaming(streaming); }

/** @brief Restart the board, as the phone's reset command asks. */
void on_reset() {
  STM32H747::writeBackupRegister(kRunStateRegister, kRunStateCommanded);
  NVIC_SystemReset();
}

/**
 * @brief Send one completed audio block to the phone.
 *
 * @param values  Interleaved minimum and maximum samples.
 */
void on_waveform(const int16_t* values) {
  protocol::sendWaveform(values, kws::kWaveformValues);
}

/**
 * @brief Report a keyword the model has detected.
 *
 * @param detection  Class and score the pipeline decided on.
 */
void on_detection(const kws::Detection& detection) {
  device::flashDetection();
  protocol::sendDetection(kws::classLabel(detection.classIndex),
                          detection.confidence);
  Serial.print(kLogPrefix);
  Serial.print(" keyword=");
  Serial.println(kws::classLabel(detection.classIndex));
}

/** @brief Follow the connection on the LED and stop work on a disconnect. */
void track_connection() {
  const bool connected = BLE.connected();
  if (connected == g_was_connected) {
    return;
  }
  g_was_connected = connected;

  if (connected) {
    device::setLedMode(device::LedMode::Connected);
    Serial.print(kLogPrefix);
    Serial.println(" connected");
    return;
  }

  kws::setInferenceRunning(false);
  kws::setWaveformStreaming(false);
  device::setLedMode(device::LedMode::Advertising);
  Serial.print(kLogPrefix);
  Serial.println(" disconnected");
}

}  // namespace

void setup() {
  // Serial first: the fuel gauge and the BrainBoard are both slow to answer,
  // and a board whose USB is not up yet cannot be reflashed without the button.
  Serial.begin(kSerialBaud);
  wait_for_serial();
  delay(kBootSettleMs);

  Serial.println();
  Serial.println(kSketchName);
  Serial.print(kLogPrefix);
  Serial.print(" previous_run=");
  Serial.println(take_previous_run_outcome());

  device::begin();
  const device::Power power = device::readPower();
  Serial.print(kLogPrefix);
  Serial.print(" serial=");
  Serial.print(device::serialNumber());
  Serial.print(" usb=");
  Serial.print(power.usbPowered ? 1 : 0);
  Serial.print(" battery=");
  Serial.print(power.percent);
  Serial.print("% state=");
  Serial.println(static_cast<int>(power.state));

  if (!prepare_board()) {
    device::setLedMode(device::LedMode::Failed);
    Serial.print(kLogPrefix);
    Serial.print(" result=FAIL stage=board detail=");
    g_board->printLastError(Serial);
    return;
  }
  Serial.print(kLogPrefix);
  Serial.print(" akida_ready ip_version=0x");
  Serial.println(g_board->ipVersion(), HEX);

  if (!kws::begin()) {
    device::setLedMode(device::LedMode::Failed);
    Serial.print(kLogPrefix);
    Serial.println(" result=FAIL stage=microphone");
    return;
  }
  kws::setHandlers(on_waveform, on_detection);

  if (!BLE.begin()) {
    device::setLedMode(device::LedMode::Failed);
    Serial.print(kLogPrefix);
    Serial.println(" result=FAIL stage=ble");
    return;
  }

  model::setHandlers(on_model_loaded, on_transfer_activity);
  model::begin(*g_board, *g_runner);
  protocol::setHandlers(on_deploy, on_stream, on_reset);
  if (!protocol::begin()) {
    device::setLedMode(device::LedMode::Failed);
    Serial.print(kLogPrefix);
    Serial.println(" result=FAIL stage=advertise");
    return;
  }

  if (!model::restoreFromFlash()) {
    Serial.print(kLogPrefix);
    Serial.println(" no model in flash, waiting for one over bluetooth");
  }

  device::setLedMode(device::LedMode::Advertising);
  Serial.print(kLogPrefix);
  Serial.print(" advertising as ");
  Serial.print(protocol::kDeviceName);
  Serial.print(" at ");
  Serial.println(BLE.address());
}

void loop() {
  BLE.poll();
  track_connection();
  protocol::poll();
  model::poll();
  device::updateLed();

  if (!model::transferInProgress()) {
    kws::poll();
  }
}
