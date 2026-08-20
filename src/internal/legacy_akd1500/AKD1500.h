#pragma once

#include <stddef.h>
#include <stdint.h>

#include <memory>
#include <vector>

#include <SPI.h>

// Keep low-level AKD1500 transport logs enabled while the Nicla Vision
// bridge-flash bring-up is still being validated. This makes real hardware
// failures report the exact sub-step instead of collapsing to a generic
// `flash_stage_failed` at the BB15 layer.
#ifndef AKD1500_LIBRARY_ENABLE_LOGS
#define AKD1500_LIBRARY_ENABLE_LOGS 1
#endif

#if defined(ARDUINO_NICLA_VISION) || defined(ARDUINO_NICLA) || \
    defined(ARDUINO_ARCH_SAMD)
#define AKD1500_PLATFORM_SUPPORTED 1
#else
#define AKD1500_PLATFORM_SUPPORTED 0
#endif

#if AKD1500_PLATFORM_SUPPORTED

#include "akida/hardware_device.h"
#include "akida/program_info.h"
#include "akida/tensor.h"
#include "akida_port/nicla_voice_akd1500_board.h"

#else

namespace akida {

class HardwareDevice;
class HardwareDriver;
class ProgramInfo;

enum class TensorType {
  bit = 0,
  int2,
  int4,
  int8,
  uint2,
  uint4,
  uint8,
  int32,
  float32,
};

using Shape = std::vector<uint32_t>;

inline size_t shape_size(const Shape& shape) {
  if (shape.empty()) {
    return 0u;
  }

  size_t elements = 1u;
  for (uint32_t dimension : shape) {
    elements *= static_cast<size_t>(dimension);
  }
  return elements;
}

struct SpiFlashRuntimeConfig {
  uint8_t reserved = 0u;
};

}  // namespace akida

#endif

class Print;

enum class AKD1500Status {
  Ok = 0,
  NotInitialized,
  LinkFailed,
  ProgramInfoInvalid,
  ProgramFailed,
  ModelNotLoaded,
  InvalidInput,
  EnqueueFailed,
  FetchTimeout,
  OutputFormatMismatch,
};

struct AKD1500Error {
  AKD1500Status status = AKD1500Status::Ok;
  uint32_t detail = 0u;
  const char* message = "ok";
};

struct AKD1500Options {
  SPIClass* spiBus = &SPI;
  uint8_t akidaCsPin = 6u;
  uint8_t bridgeCsPin = 10u;
  uint32_t spiClockHz = 2000000u;
  uint32_t flashSpiClockHz = 0u;
  uint32_t expectedIpVersion = 0xBCA10309u;
  uint32_t externalModelAddress = 0x80000000u;
  uint32_t visibleMemoryBase = 0u;
  uint32_t visibleMemorySize = 0u;
  const char* forcedFlashProfile = nullptr;
  bool assumeForcedFlashProfileReady = false;
  uint32_t postBeginSettleMs = 50u;
  uint32_t postLinkSettleMs = 50u;
  uint32_t fetchPollDelayMs = 100u;
  uint32_t fetchTimeoutMs = 20000u;

  static AKD1500Options niclaVisionDefaults();
};

struct AKD1500RunResult {
  AKD1500Status status = AKD1500Status::NotInitialized;
  AKD1500Error error = {};
  akida::TensorType type = akida::TensorType::uint8;
  akida::Shape dimensions = {};
  std::vector<uint8_t> bytes;
  size_t predictedIndex = 0u;

  bool ok() const { return status == AKD1500Status::Ok; }
  size_t byteSize() const { return bytes.size(); }
  size_t elementCount() const;

  template <typename T>
  const T* data() const {
    return bytes.empty() ? nullptr
                         : reinterpret_cast<const T*>(bytes.data());
  }
};

#if AKD1500_PLATFORM_SUPPORTED

class AKD1500RunnerBase {
 public:
  const AKD1500Options& options() const { return options_; }
  const AKD1500Error& lastError() const { return last_error_; }
  uint32_t ipVersion() const { return ip_version_; }
  bool modelLoaded() const { return model_loaded_ && program_info_.is_valid(); }
  uint32_t detectedFlashJedec() const { return board_.detected_flash_jedec(); }
  const char* detectedFlashName() const { return board_.detected_flash_name(); }
  akida::SpiFlashRuntimeConfig detectedFlashRuntimeConfig() const {
    return board_.detected_flash_runtime_config();
  }
  bool hasSupportedFlashProfile() const {
    return board_.has_supported_flash_profile();
  }
  akida::HardwareDevice* hardwareDevice() const { return device_.get(); }
  akida::HardwareDriver* hardwareDriver() const {
    return device_ != nullptr
               ? &const_cast<akida_port::AKD1500Board&>(board_).hardware_driver()
               : nullptr;
  }
  const akida::ProgramInfo& programInfo() const { return program_info_; }
  akida::Shape inputDimensions() const;
  akida::Shape outputDimensions() const;
  bool inputIsDense() const;
  bool outputIsDense() const;
  bool canLearn() const;

 protected:
  explicit AKD1500RunnerBase(const AKD1500Options& options);

  AKD1500Status beginRunner();
  AKD1500RunResult runImpl(const void* input_data, akida::TensorType input_type,
                           const akida::Shape& input_dimensions);
  AKD1500RunResult runImpl(const uint8_t* input_data,
                           const akida::Shape& input_dimensions) {
    return runImpl(input_data, akida::TensorType::uint8, input_dimensions);
  }
  void dumpFetchTimeoutState(uint32_t fetch_polls, uint32_t elapsed_us);
  void clearLoadedModel();
  void setLoadedModel(const akida::ProgramInfo& program_info);
  AKD1500Status setError(AKD1500Status status, const char* message,
                         uint32_t detail = 0u);
  bool isReady() const { return device_ != nullptr; }
  akida::HardwareDevice* device() const { return device_.get(); }

  AKD1500Options options_;
  akida_port::AKD1500Board board_;
  akida::HardwareDevicePtr device_;
  akida::ProgramInfo program_info_;
  AKD1500Error last_error_;
  uint32_t ip_version_ = 0u;
  bool model_loaded_ = false;
};

#else

class AKD1500RunnerBase {
 public:
  const AKD1500Options& options() const { return options_; }
  const AKD1500Error& lastError() const { return last_error_; }
  uint32_t ipVersion() const { return ip_version_; }
  bool modelLoaded() const { return model_loaded_; }
  uint32_t detectedFlashJedec() const { return 0u; }
  const char* detectedFlashName() const { return "unsupported"; }
  akida::SpiFlashRuntimeConfig detectedFlashRuntimeConfig() const { return {}; }
  bool hasSupportedFlashProfile() const { return false; }
  akida::HardwareDevice* hardwareDevice() const { return nullptr; }
  akida::HardwareDriver* hardwareDriver() const { return nullptr; }
  const akida::ProgramInfo* programInfo() const { return nullptr; }
  akida::Shape inputDimensions() const { return model_info_.inputDimensions; }
  akida::Shape outputDimensions() const { return model_info_.outputDimensions; }
  bool inputIsDense() const { return model_info_.inputIsDense; }
  bool outputIsDense() const { return model_info_.outputIsDense; }
  bool canLearn() const { return model_info_.canLearn; }

 protected:
  explicit AKD1500RunnerBase(const AKD1500Options& options);

  AKD1500Status beginRunner();
  AKD1500RunResult runImpl(const void* input_data, akida::TensorType input_type,
                           const akida::Shape& input_dimensions);
  AKD1500RunResult runImpl(const uint8_t* input_data,
                           const akida::Shape& input_dimensions) {
    return runImpl(input_data, akida::TensorType::uint8, input_dimensions);
  }
  void dumpFetchTimeoutState(uint32_t fetch_polls, uint32_t elapsed_us);
  void clearLoadedModel();
  AKD1500Status setError(AKD1500Status status, const char* message,
                         uint32_t detail = 0u);

  AKD1500Options options_;
  AKD1500Error last_error_ = {};
  uint32_t ip_version_ = 0u;
  bool model_loaded_ = false;
  struct StubModelInfo {
    bool inputIsDense = true;
    bool outputIsDense = true;
    bool canLearn = false;
    akida::Shape inputDimensions = {};
    akida::Shape outputDimensions = {};
  } model_info_;
};

#endif

class AKD1500HostRunner final : public AKD1500RunnerBase {
 public:
  explicit AKD1500HostRunner(const AKD1500Options& options = {});

  AKD1500Status begin();
  AKD1500Status loadModel(const uint8_t* serialized_program,
                          size_t serialized_program_size);
  AKD1500RunResult run(const void* input_data, akida::TensorType input_type,
                       const akida::Shape& input_dimensions);
  AKD1500RunResult run(const uint8_t* input_data,
                       const akida::Shape& input_dimensions);
};

class AKD1500FlashRunner final : public AKD1500RunnerBase {
 public:
  explicit AKD1500FlashRunner(const AKD1500Options& options = {});

  AKD1500Status begin();
  AKD1500Status loadExternalModel(const uint8_t* serialized_program,
                                  size_t serialized_program_size);
  AKD1500Status loadModel(const uint8_t* serialized_program,
                          size_t serialized_program_size) {
    return loadExternalModel(serialized_program, serialized_program_size);
  }
  AKD1500RunResult run(const void* input_data, akida::TensorType input_type,
                       const akida::Shape& input_dimensions);
  AKD1500RunResult run(const uint8_t* input_data,
                       const akida::Shape& input_dimensions);
};

enum class AKD1500ModelStorage {
  HostMemory = 0,
  ExternalFlash,
};

struct AKD1500Model {
  const uint8_t* serializedProgram = nullptr;
  size_t size = 0u;
  AKD1500ModelStorage storage = AKD1500ModelStorage::HostMemory;
  uint32_t externalLocation = 0u;
};

struct AKD1500ModelInfo {
  bool valid = false;
  bool inputIsDense = true;
  bool outputIsDense = true;
  bool canLearn = false;
  akida::Shape inputDimensions = {};
  akida::Shape outputDimensions = {};
};

struct AKD1500Input {
  const void* data = nullptr;
  akida::TensorType type = akida::TensorType::uint8;
  akida::Shape dimensions = {};
};

struct AKD1500ClassificationResult {
  AKD1500Status status = AKD1500Status::NotInitialized;
  AKD1500Error error = {};
  size_t predictedIndex = 0u;
  AKD1500RunResult scores = {};

  bool ok() const { return status == AKD1500Status::Ok && scores.ok(); }

  template <typename T>
  const T* data() const {
    return scores.data<T>();
  }
};

class AkidaNicla final {
 public:
  explicit AkidaNicla(const AKD1500Options& options = {});

  AKD1500Status begin();
  AKD1500Status begin(const AKD1500Options& options);
  AKD1500Status load(const AKD1500Model& model);

  AKD1500RunResult infer(const AKD1500Input& input);
  AKD1500RunResult infer(const void* input_data, akida::TensorType input_type,
                         const akida::Shape& input_dimensions);
  AKD1500RunResult inferUint8(const uint8_t* input_data,
                              const akida::Shape& input_dimensions);
  AKD1500RunResult inferUint8(const uint8_t* input_data);

  AKD1500ClassificationResult classify(const AKD1500Input& input);
  AKD1500ClassificationResult classify(
      const void* input_data, akida::TensorType input_type,
      const akida::Shape& input_dimensions);
  AKD1500ClassificationResult classifyUint8(
      const uint8_t* input_data, const akida::Shape& input_dimensions);
  AKD1500ClassificationResult classifyUint8(const uint8_t* input_data);

  const AKD1500Options& options() const { return options_; }
  const AKD1500Error& lastError() const { return last_error_; }
  uint32_t ipVersion() const { return ip_version_; }
  bool modelLoaded() const { return model_info_.valid; }
  AKD1500ModelInfo modelInfo() const { return model_info_; }
  akida::HardwareDevice* hardwareDevice() const;
  akida::HardwareDriver* hardwareDriver() const;
  const akida::ProgramInfo* programInfo() const;

  void printLastError(Print& out) const;
  void printModelInfo(Print& out) const;

  static uint32_t normalizeExternalModelAddress(uint32_t address_or_offset);
  static uint32_t externalModelAddressFromOffset(uint32_t offset);
  static bool stageModelToFlash(const AKD1500Options& options,
                                const uint8_t* serialized_program,
                                size_t serialized_program_size,
                                uint32_t address_or_offset = 0u);
  static bool stageModelToFlash(const uint8_t* serialized_program,
                                size_t serialized_program_size,
                                uint32_t address_or_offset = 0u);
  static bool verifyModelInFlash(const AKD1500Options& options,
                                 const uint8_t* serialized_program,
                                 size_t serialized_program_size,
                                 uint32_t address_or_offset = 0u);
  static bool verifyModelInFlash(const uint8_t* serialized_program,
                                 size_t serialized_program_size,
                                 uint32_t address_or_offset = 0u);
  static const char* statusName(AKD1500Status status);

 private:
  enum class ActiveRunner {
    None,
    Host,
    Flash
  };

  AKD1500RunnerBase* activeRunnerBase() const;
  AKD1500Status setError(AKD1500Status status, const char* message,
                         uint32_t detail = 0u);
  AKD1500RunResult dispatchInfer(const AKD1500Input& input);
  AKD1500ClassificationResult makeClassificationResult(
      const AKD1500RunResult& run_result) const;
  bool inputMatchesModel(const akida::Shape& input_dimensions) const;

  AKD1500Options options_;
  std::unique_ptr<AKD1500HostRunner> host_runner_;
  std::unique_ptr<AKD1500FlashRunner> flash_runner_;
  ActiveRunner active_runner_ = ActiveRunner::None;
  AKD1500ModelInfo model_info_ = {};
  AKD1500Error last_error_ = {};
  uint32_t ip_version_ = 0u;
};
