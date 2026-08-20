#pragma once

#include <stddef.h>
#include <stdint.h>

#include <memory>
#include <vector>

#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>

#include "internal/bb15_runtime.h"

#if AKD1500_PLATFORM_SUPPORTED
namespace akida_port {
class AKD1500Board;
}
#endif

namespace bb15 {
class PioExpander6408;
}

struct BB15Pins {
  uint8_t akidaCs = 7u;
  uint8_t ttModeCs = 1u;
  uint8_t interrupt = 0u;
};

enum class BB15ResetRoute {
  HostGpio = 0,
  Expander,
};

struct BB15ResetPin {
  BB15ResetRoute route = BB15ResetRoute::HostGpio;
  uint8_t pin = 3u;
};

struct BB15ExpanderPins {
  uint8_t bootMode = 0u;
  uint8_t akidaSleep = 2u;
  uint8_t akidaInterrupt = 1u;
};

struct BB15Pinout {
  BB15Pins host = {};
  BB15ResetPin akidaReset = {};
  BB15ExpanderPins expander = {};

  static BB15Pinout niclaVisionDefaults();
  static BB15Pinout niclaSenseMeDefaults();
};

struct BB15Config {
  SPIClass* spi = &SPI;
  TwoWire* wire = &Wire;

  uint8_t expanderAddress = 0x43u;
  uint32_t spiClockHz = 8000000u;
  uint32_t flashSpiClockHz = 2000000u;
  uint32_t expectedIpVersion = 0xBCA10309u;
  uint32_t defaultModelAddress = 0x80000000u;
  uint32_t fetchTimeoutMs = 20000u;
  uint32_t fetchPollDelayMs = 1u;
  uint32_t postBeginSettleMs = 50u;
  uint32_t postLinkSettleMs = 50u;
  uint32_t i2cClockHz = 100000u;
  uint32_t resetAssertMs = 5u;
  uint32_t resetReleaseSettleMs = 10u;
  uint32_t sleepEnterSettleMs = 1u;
  uint32_t sleepExitSettleMs = 10u;

  const char* forcedFlashProfile = nullptr;
  bool assumeForcedFlashProfileReady = false;

  static BB15Config defaults();
};

enum class BB15Status {
  Ok = 0,
  NotInitialized,
  InvalidConfig,
  ExpanderMissing,
  ExpanderConfigFailed,
  ResetFailed,
  LinkFailed,
  FlashUnsupported,
  FlashStageFailed,
  FlashVerifyFailed,
  ModelInvalid,
  ModelNotLoaded,
  InvalidInput,
  EnqueueFailed,
  OutputNotReady,
  FetchTimeout,
  OutputFormatMismatch,
  TransportStateError,
};

struct BB15Error {
  BB15Status status = BB15Status::Ok;
  uint32_t detail = 0u;
  const char* message = "ok";
};

enum class BB15ModelStorage {
  HostMemory = 0,
  ExternalFlash,
};

struct BB15TensorInfo {
  akida::TensorType type = akida::TensorType::uint8;
  akida::Shape dimensions = {};
  bool dense = true;
};

struct BB15ModelInfo {
  bool valid = false;
  bool canLearn = false;
  BB15TensorInfo input = {};
  BB15TensorInfo output = {};
  size_t serializedSize = 0u;
};

struct BB15FlashInfo {
  bool detected = false;
  uint32_t jedec = 0u;
  const char* name = "unknown";
  bool supportedProfile = false;
};

class BB15Model {
 public:
  BB15Model(const uint8_t* data = nullptr, size_t size = 0u);

  bool valid() const;
  size_t size() const;
  const uint8_t* data() const;

  BB15ModelInfo info() const;
  const BB15TensorInfo& input() const;
  const BB15TensorInfo& output() const;

  BB15ModelStorage storage() const;
  BB15Model& setStorage(BB15ModelStorage storage);

  uint32_t externalAddress() const;
  BB15Model& setExternalAddress(uint32_t address);

 private:
  const uint8_t* data_ = nullptr;
  size_t size_ = 0u;
  BB15ModelStorage storage_ = BB15ModelStorage::HostMemory;
  uint32_t external_address_ = 0u;
  BB15ModelInfo info_ = {};
};

struct BB15RunResult {
  BB15Status status = BB15Status::NotInitialized;
  BB15Error error = {};
  akida::TensorType type = akida::TensorType::uint8;
  akida::Shape dimensions = {};
  std::vector<uint8_t> bytes = {};
  size_t predictedIndex = 0u;

  bool ok() const { return status == BB15Status::Ok; }
  size_t byteSize() const { return bytes.size(); }
  size_t elementCount() const;

  template <typename T>
  const T* data() const {
    return bytes.empty() ? nullptr
                         : reinterpret_cast<const T*>(bytes.data());
  }
};

struct BB15ClassificationResult {
  BB15Status status = BB15Status::NotInitialized;
  BB15Error error = {};
  size_t predictedIndex = 0u;
  BB15RunResult scores = {};

  bool ok() const { return status == BB15Status::Ok && scores.ok(); }
};

struct BB15Input {
  const void* data = nullptr;
  akida::TensorType type = akida::TensorType::uint8;
  akida::Shape dimensions = {};
};

class BB15Runner;

class BB15 {
 public:
  explicit BB15(const BB15Pinout& pinout,
                const BB15Config& config = BB15Config());
  ~BB15();

  BB15Status begin();
  bool initialized() const { return initialized_; }

  const BB15Pinout& pinout() const { return pinout_; }
  const BB15Config& config() const { return config_; }
  const BB15Error& lastError() const { return last_error_; }
  uint32_t ipVersion() const { return ip_version_; }

  bool probeExpander();
  bool configureExpanderDefaults();

  bool setAkidaReset(bool asserted);
  bool holdAkidaInReset();
  bool releaseAkidaReset();
  bool powerDown();
  bool powerUp();
  bool sleep();
  bool wake();
  bool setAkidaSleep(bool enabled);

  bool enterExternalFlashBootMode();
  bool coldBootExternalFlashMode(uint32_t holdResetMs = 250u);

  bool detectFlash();
  BB15FlashInfo flashInfo() const;
  uint32_t detectedFlashJedec() const { return detected_flash_jedec_; }
  const char* detectedFlashName() const { return detected_flash_name_; }
  bool hasSupportedFlashProfile() const { return has_supported_flash_profile_; }

  bool s2mEnter();
  bool s2mExit();
  bool s2mActive() const { return s2m_active_; }

  bool programExternalData(const uint8_t* data, size_t size, uint32_t address);
  bool verifyExternalData(const uint8_t* data, size_t size, uint32_t address);
  bool readExternalData(uint32_t address, uint8_t* out, size_t size);

  bool flashModel(const BB15Model& model);
  bool verifyModel(const BB15Model& model);

  BB15Runner createRunner();

  void printLastError(Print& out) const;
  void printSummary(Print& out) const;

 private:
  friend class BB15Runner;

  BB15Status setError(BB15Status status, const char* message,
                      uint32_t detail = 0u);
  bb15::internal::RuntimeOptions toRuntimeOptions() const;
  bool ensureLowLevelBoard();
  uint32_t normalizeAddress(uint32_t address_or_offset) const;
  uint32_t logicalFlashOffset(uint32_t address_or_offset) const;
  void prepareHostSpiPinsForAkidaAccess();
  bool ensureWireStarted();
  bool ensureExpander();
  void initializeConstructorResetState();

  BB15Pinout pinout_;
  BB15Config config_;
  BB15Error last_error_ = {};
  uint32_t ip_version_ = 0u;
  uint32_t detected_flash_jedec_ = 0u;
  const char* detected_flash_name_ = "unknown";
  bool has_supported_flash_profile_ = false;
  bool initialized_ = false;
  bool wire_started_ = false;
  bool expander_configured_ = false;
  bool s2m_active_ = false;
  std::unique_ptr<bb15::PioExpander6408> expander_;
#if AKD1500_PLATFORM_SUPPORTED
  std::unique_ptr<bb15::internal::RuntimeBoard> low_level_board_;
#endif
};

class BB15Runner {
 public:
  explicit BB15Runner(BB15& board);

  BB15Status begin();
  bool ready() const { return ready_; }

  BB15Status loadModel(const BB15Model& model);
  bool modelLoaded() const { return model_loaded_; }
  BB15ModelInfo modelInfo() const { return model_info_; }

  BB15Status enqueue(const BB15Input& input);
  BB15RunResult fetch();

  BB15RunResult infer(const BB15Input& input);
  BB15RunResult infer(const uint8_t* data, const akida::Shape& dims);

  BB15ClassificationResult classify(const BB15Input& input);
  BB15ClassificationResult classify(const uint8_t* data,
                                    const akida::Shape& dims);

  bool readRegister32(uint32_t address, uint32_t& value);
  bool writeRegister32(uint32_t address, uint32_t value);

  const BB15Error& lastError() const { return last_error_; }
  void printModelInfo(Print& out) const;

 private:
  BB15Status setError(BB15Status status, const char* message,
                      uint32_t detail = 0u);

  BB15* board_ = nullptr;
  std::unique_ptr<bb15::internal::RuntimeRunner> backend_;
  BB15Error last_error_ = {};
  BB15ModelInfo model_info_ = {};
  bool ready_ = false;
  bool model_loaded_ = false;
  bool inference_in_flight_ = false;
};
