#include "ble_protocol.h"

#include <ArduinoBLE.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "ble_model_transfer.h"
#include "device_status.h"
#include "kws_pipeline.h"

namespace protocol {

const char kDeviceName[] = "BrainBoard1500";

const uint8_t kManufacturerData[12] = {'4', '1', '0', '1', '0', 'A',
                                       'K', 'D', '1', '5', '0', '0'};

namespace {

// What the board says about itself in the device information burst.
constexpr const char* kVendor = "BrainChip";
constexpr const char* kBluetoothVersion = "4.1";
constexpr const char* kFirmwareVersion = "0.1.0";

// What the board says about itself in the application list. The app takes the
// first word, lowercased, as the application id, so it has to stay "Keyword".
constexpr const char* kAppName = "Keyword Spotting";
constexpr const char* kAppDescription =
    "Voice-activated wake word detection using microphone input";

// Frame types, which the phone uses to reassemble a multi-frame answer.
constexpr uint8_t kFrameSingle = 0u;
constexpr uint8_t kFrameStart = 1u;
constexpr uint8_t kFrameMiddle = 2u;
constexpr uint8_t kFrameLast = 3u;

// Opcodes, as the phone writes them and expects them back.
constexpr int kCmdBattery = 0;
constexpr int kCmdDeviceInfo = 1;
constexpr int kCmdApps = 2;
constexpr int kCmdConfig = 4;
constexpr int kCmdAppInfo = 5;
constexpr int kCmdReset = 7;
constexpr int kCmdDeployStart = 8;
constexpr int kCmdStreamStart = 9;
constexpr int kCmdDeployStop = 10;
constexpr int kCmdStreamStop = 11;

// Answered to a start or stop command once the pipeline has really changed.
constexpr const char* kAckDone = "170";

// Binary waveform frames, told apart from the text ones by the first byte.
constexpr uint8_t kWaveMagic = 0x42u;
constexpr uint8_t kWaveCommand = 0x0Cu;
constexpr size_t kWaveHeaderBytes = 6u;

constexpr size_t kMaxCommandBytes = 248u;
constexpr size_t kCommandQueueDepth = 4u;
constexpr size_t kFrameBytes = 160u;
constexpr size_t kDataPartBytes = 128u;

BLEService g_service("6e400001-b5a3-f393-e0a9-e50e24dcca9e");
BLECharacteristic g_rx("6e400002-b5a3-f393-e0a9-e50e24dcca9e",
                       BLEWrite | BLEWriteWithoutResponse, kMaxCommandBytes);
BLECharacteristic g_tx("6e400003-b5a3-f393-e0a9-e50e24dcca9e", BLENotify,
                       kMaxCommandBytes);

DeployHandler g_on_deploy = nullptr;
StreamHandler g_on_stream = nullptr;
ResetHandler g_on_reset = nullptr;

char g_queue[kCommandQueueDepth][kMaxCommandBytes];
volatile uint8_t g_queue_head = 0u;
volatile uint8_t g_queue_tail = 0u;

uint16_t g_wave_sequence = 0u;

/** @brief One tunable the phone can read and write, and how it is carried. */
struct ConfigParameter {
  uint8_t id;
  bool isFloat;
  float minimum;
  float maximum;
};

constexpr ConfigParameter kParameters[] = {
    {0u, false, 0.0f, 65535.0f}, {1u, false, 0.0f, 65535.0f},
    {2u, true, 0.0f, 1.0f},      {3u, true, 0.0f, 1.0f},
    {4u, false, 1.0f, 255.0f},   {5u, false, 0.0f, 65535.0f},
};
constexpr size_t kParameterCount = sizeof(kParameters) / sizeof(kParameters[0]);

/**
 * @brief Send one text frame to the phone.
 *
 * @param frameType  Single, or the position in a multi-frame answer.
 * @param index      Sequence number within the answer.
 * @param dataPart   The `<opcode>:<value>` body, ending in a carriage return.
 */
void sendFrame(uint8_t frameType, uint8_t index, const char* dataPart) {
  char frame[kFrameBytes];
  snprintf(frame, sizeof(frame), "%u,%u,%u,%s",
           static_cast<unsigned>(frameType), static_cast<unsigned>(index),
           static_cast<unsigned>(strlen(dataPart)), dataPart);
  g_tx.writeValue(reinterpret_cast<const uint8_t*>(frame),
                  static_cast<int>(strlen(frame)));
}

/**
 * @brief Send a whole answer as one frame.
 *
 * @param opcode  Opcode the phone routes the answer by.
 * @param value   Body after the opcode.
 */
void sendSingle(int opcode, const char* value) {
  char dataPart[kDataPartBytes];
  snprintf(dataPart, sizeof(dataPart), "%d:%s\r", opcode, value);
  sendFrame(kFrameSingle, 0u, dataPart);
}

/**
 * @brief Send an answer the phone reassembles from several frames.
 *
 * Each value ends in a comma so that the concatenation the phone builds splits
 * cleanly back into the fields.
 *
 * @param opcode  Opcode the phone routes the answer by.
 * @param values  The fields, in order.
 * @param count   Number of fields.
 */
void sendBurst(int opcode, const char* const* values, size_t count) {
  for (size_t index = 0u; index < count; ++index) {
    char dataPart[kDataPartBytes];
    snprintf(dataPart, sizeof(dataPart), "%d:%s,\r", opcode, values[index]);
    uint8_t frameType = kFrameMiddle;
    if (index == 0u) {
      frameType = kFrameStart;
    } else if (index + 1u == count) {
      frameType = kFrameLast;
    }
    sendFrame(frameType, static_cast<uint8_t>(index), dataPart);
  }
}

/** @brief Answer the battery command with the current power state. */
void sendBattery() {
  const device::Power power = device::readPower();
  char value[32];
  snprintf(value, sizeof(value), "%u,%u", static_cast<unsigned>(power.percent),
           static_cast<unsigned>(power.state));
  sendSingle(kCmdBattery, value);
}

/** @brief Answer the device information command. */
void sendDeviceInfo() {
  const char* values[] = {kVendor, kDeviceName, kBluetoothVersion,
                          kFirmwareVersion, device::serialNumber()};
  sendBurst(kCmdDeviceInfo, values, 5u);
}

/** @brief Answer the application list command. */
void sendApps() {
  const model::Loaded& loadedModel = model::loaded();
  char sizeKb[16];
  snprintf(sizeKb, sizeof(sizeKb), "%lu",
           static_cast<unsigned long>(loadedModel.programBytes / 1024u));
  const char* values[] = {kAppName, kAppDescription, sizeKb};
  sendBurst(kCmdApps, values, 3u);
}

/** @brief Answer the application detail command with the loaded model. */
void sendAppInfo() {
  const model::Loaded& loadedModel = model::loaded();

  char inputShape[32];
  snprintf(inputShape, sizeof(inputShape), "49x10x1");

  char classes[8];
  snprintf(classes, sizeof(classes), "%u",
           static_cast<unsigned>(loadedModel.classCount));

  char keywords[128] = {0};
  for (uint8_t index = 0u; index < loadedModel.classCount; ++index) {
    if (index == loadedModel.silenceClass ||
        index == loadedModel.unknownClass) {
      continue;
    }
    if (keywords[0] != '\0') {
      strncat(keywords, ";", sizeof(keywords) - strlen(keywords) - 1u);
    }
    strncat(keywords, kws::classLabel(index),
            sizeof(keywords) - strlen(keywords) - 1u);
  }

  const char* values[] = {loadedModel.name, inputShape, classes, keywords};
  sendBurst(kCmdAppInfo, values, 4u);
}

/**
 * @brief Read one tunable out of the pipeline configuration.
 *
 * @param id  Parameter identifier the phone uses.
 * @return Its current value.
 */
float readParameter(uint8_t id) {
  const kws::Config& config = kws::config();
  switch (id) {
    case 0u:
      return config.rmsThreshold;
    case 1u:
      return config.debounceMs;
    case 2u:
      return config.smoothingAlpha;
    case 3u:
      return config.scoreThreshold;
    case 4u:
      return config.chimingThreshold;
    default:
      return config.speechTimeoutMs;
  }
}

/**
 * @brief Write one tunable into the pipeline configuration.
 *
 * @param id     Parameter identifier the phone uses.
 * @param value  Value already checked against its range.
 */
void writeParameter(uint8_t id, float value) {
  kws::Config config = kws::config();
  switch (id) {
    case 0u:
      config.rmsThreshold = static_cast<uint16_t>(value);
      break;
    case 1u:
      config.debounceMs = static_cast<uint16_t>(value);
      break;
    case 2u:
      config.smoothingAlpha = value;
      break;
    case 3u:
      config.scoreThreshold = value;
      break;
    case 4u:
      config.chimingThreshold = static_cast<uint8_t>(value);
      break;
    default:
      config.speechTimeoutMs = static_cast<uint16_t>(value);
      break;
  }
  kws::setConfig(config);
}

/**
 * @brief Format one parameter the way the phone parses it.
 *
 * @param parameter  The parameter being reported.
 * @param out        Receives the formatted value.
 * @param size       Size of that buffer.
 */
void formatParameter(const ConfigParameter& parameter, char* out, size_t size) {
  if (parameter.isFloat) {
    snprintf(out, size, "%.2f",
             static_cast<double>(readParameter(parameter.id)));
  } else {
    snprintf(out, size, "%ld", static_cast<long>(readParameter(parameter.id)));
  }
}

/** @brief Send every tunable, which the phone reads as its live mirror. */
void sendConfigBurst() {
  for (size_t index = 0u; index < kParameterCount; ++index) {
    char value[16];
    formatParameter(kParameters[index], value, sizeof(value));
    char dataPart[kDataPartBytes];
    snprintf(dataPart, sizeof(dataPart), "%d:%u:%s\r", kCmdConfig,
             static_cast<unsigned>(kParameters[index].id), value);
    uint8_t frameType = kFrameMiddle;
    if (index == 0u) {
      frameType = kFrameStart;
    } else if (index + 1u == kParameterCount) {
      frameType = kFrameLast;
    }
    sendFrame(frameType, static_cast<uint8_t>(index), dataPart);
  }
}

/**
 * @brief Apply one parameter the phone has set, and say whether it took.
 *
 * @param payload  The `<id>:<value>` body of the set command.
 */
void applyConfigSet(const char* payload) {
  const char* separator = strchr(payload, ':');
  if (separator == nullptr) {
    return;
  }
  const uint8_t id = static_cast<uint8_t>(atoi(payload));
  const float value = static_cast<float>(atof(separator + 1));

  for (size_t index = 0u; index < kParameterCount; ++index) {
    if (kParameters[index].id != id) {
      continue;
    }
    char answer[32];
    if (value < kParameters[index].minimum ||
        value > kParameters[index].maximum) {
      snprintf(answer, sizeof(answer), "%u:ERR:RANGE",
               static_cast<unsigned>(id));
    } else {
      writeParameter(id, value);
      snprintf(answer, sizeof(answer), "%u:OK", static_cast<unsigned>(id));
    }
    sendSingle(kCmdConfig, answer);
    return;
  }

  char answer[32];
  snprintf(answer, sizeof(answer), "%u:ERR:ID", static_cast<unsigned>(id));
  sendSingle(kCmdConfig, answer);
}

/**
 * @brief Handle the configuration command in all three of its forms.
 *
 * @param payload  Body after the opcode: GET, RESET, or `<id>:<value>`.
 */
void handleConfig(const char* payload) {
  if (strncmp(payload, "GET", 3) == 0) {
    sendConfigBurst();
    return;
  }
  if (strncmp(payload, "RESET", 5) == 0) {
    kws::resetConfig();
    sendSingle(kCmdConfig, "RESET:OK");
    sendConfigBurst();
    return;
  }
  applyConfigSet(payload);
}

/**
 * @brief Act on one command the phone has sent.
 *
 * @param command  The frame, with its trailing carriage return removed.
 */
void handleCommand(char* command) {
  int frameType = 0;
  int index = 0;
  int length = 0;
  int opcode = 0;
  if (sscanf(command, "%d,%d,%d,%d", &frameType, &index, &length, &opcode) !=
      4) {
    return;
  }

  const char* separator = strchr(command, ':');
  const char* payload = separator != nullptr ? separator + 1 : "";

  switch (opcode) {
    case kCmdBattery:
      sendBattery();
      break;
    case kCmdDeviceInfo:
      sendDeviceInfo();
      break;
    case kCmdApps:
      if (model::loaded().valid) {
        sendApps();
      }
      break;
    case kCmdAppInfo:
      if (model::loaded().valid) {
        sendAppInfo();
      }
      break;
    case kCmdConfig:
      handleConfig(payload);
      break;
    case kCmdReset:
      if (g_on_reset != nullptr) {
        g_on_reset();
      }
      break;
    case kCmdDeployStart:
      if (g_on_deploy != nullptr) {
        g_on_deploy(true);
      }
      sendSingle(kCmdDeployStart, kAckDone);
      break;
    case kCmdDeployStop:
      if (g_on_deploy != nullptr) {
        g_on_deploy(false);
      }
      sendSingle(kCmdDeployStop, kAckDone);
      break;
    case kCmdStreamStart:
      if (g_on_stream != nullptr) {
        g_on_stream(true);
      }
      break;
    case kCmdStreamStop:
      if (g_on_stream != nullptr) {
        g_on_stream(false);
      }
      break;
    default:
      break;
  }
}

/** @brief Queue a command for poll() to answer outside this callback. */
void onRxWritten(BLEDevice, BLECharacteristic characteristic) {
  const uint8_t next = (g_queue_head + 1u) % kCommandQueueDepth;
  if (next == g_queue_tail) {
    return;
  }
  size_t length = static_cast<size_t>(characteristic.valueLength());
  if (length > kMaxCommandBytes - 1u) {
    length = kMaxCommandBytes - 1u;
  }
  memcpy(g_queue[g_queue_head], characteristic.value(), length);
  g_queue[g_queue_head][length] = '\0';
  g_queue_head = next;
}

}  // namespace

bool begin() {
  g_service.addCharacteristic(g_rx);
  g_service.addCharacteristic(g_tx);
  BLE.addService(g_service);
  g_rx.setEventHandler(BLEWritten, onRxWritten);

  BLE.setLocalName(kDeviceName);
  BLE.setDeviceName(kDeviceName);
  BLE.setManufacturerData(kManufacturerData, sizeof(kManufacturerData));
  return BLE.advertise() != 0;
}

void setHandlers(DeployHandler onDeploy, StreamHandler onStream,
                 ResetHandler onReset) {
  g_on_deploy = onDeploy;
  g_on_stream = onStream;
  g_on_reset = onReset;
}

void poll() {
  while (g_queue_tail != g_queue_head) {
    handleCommand(g_queue[g_queue_tail]);
    g_queue_tail = (g_queue_tail + 1u) % kCommandQueueDepth;
  }
}

void sendDetection(const char* label, float confidence) {
  char value[64];
  snprintf(value, sizeof(value), "%s,%.2f", label,
           static_cast<double>(confidence * 100.0f));
  sendSingle(kCmdDeployStart, value);
}

void sendWaveform(const int16_t* values, uint16_t count) {
  uint8_t frame[kWaveHeaderBytes + kws::kWaveformValues * sizeof(int16_t)];
  frame[0] = kWaveMagic;
  frame[1] = kWaveCommand;
  frame[2] = static_cast<uint8_t>(g_wave_sequence & 0xFFu);
  frame[3] = static_cast<uint8_t>((g_wave_sequence >> 8) & 0xFFu);
  frame[4] = static_cast<uint8_t>(count & 0xFFu);
  frame[5] = static_cast<uint8_t>((count >> 8) & 0xFFu);
  ++g_wave_sequence;

  for (uint16_t index = 0u; index < count; ++index) {
    const uint16_t sample = static_cast<uint16_t>(values[index]);
    frame[kWaveHeaderBytes + index * 2u] = static_cast<uint8_t>(sample & 0xFFu);
    frame[kWaveHeaderBytes + index * 2u + 1u] =
        static_cast<uint8_t>((sample >> 8) & 0xFFu);
  }
  g_tx.writeValue(frame,
                  static_cast<int>(kWaveHeaderBytes + count * sizeof(int16_t)));
}

bool connected() { return BLE.connected() && g_tx.subscribed(); }

}  // namespace protocol
