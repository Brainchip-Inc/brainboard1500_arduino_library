#include "AKD1500.h"

#include <cstring>
#include <vector>

#include <Arduino.h>

#if AKD1500_PLATFORM_SUPPORTED

#include "akida/dense.h"
#include "akida/input_conversion.h"
#include "akida/shape.h"
#include "akida/sparse.h"
#include "akida/version.h"
#include "akida_engine/dma_engine_ops.h"
#include "engine/akida_program_info_generated.h"
#include "flatbuffers/base.h"

namespace {

constexpr uint32_t kExternalModelAliasBase = 0x80000000u;
constexpr uint32_t kExternalModelWindowBase = 0xFC000000u;
constexpr uint32_t kExternalModelWindowSize = 0x00800000u;
constexpr bool kPrintRunTiming = false;

akida_port::AKD1500BoardConfig make_board_config(
    const AKD1500Options& options) {
  akida_port::AKD1500BoardConfig config;
  config.spi_bus = options.spiBus;
  config.pins.akida_cs = options.akidaCsPin;
  config.pins.bridge_cs = options.bridgeCsPin;
  config.spi_clock_hz = options.spiClockHz;
  config.flash_spi_clock_hz = options.flashSpiClockHz;
  config.external_program_data_address = options.externalModelAddress;
  config.expected_ip_version = options.expectedIpVersion;
  config.visible_memory_base = options.visibleMemoryBase;
  config.visible_memory_size = options.visibleMemorySize;
  config.forced_flash_profile = options.forcedFlashProfile;
  config.assume_forced_flash_profile_ready =
      options.assumeForcedFlashProfileReady;
  return config;
}

AKD1500Error make_error(AKD1500Status status, const char* message,
                        uint32_t detail = 0u) {
  AKD1500Error error;
  error.status = status;
  error.detail = detail;
  error.message = message;
  return error;
}

size_t serialized_program_info_size(const uint8_t* serialized_program,
                                    size_t total_size) {
  if (serialized_program == nullptr ||
      total_size < sizeof(flatbuffers::uoffset_t)) {
    return 0u;
  }

  const size_t size =
      flatbuffers::ReadScalar<flatbuffers::uoffset_t>(serialized_program) +
      sizeof(flatbuffers::uoffset_t);
  return (size <= total_size) ? size : 0u;
}

bool validate_program_info_blob(const uint8_t* serialized_program,
                                size_t serialized_program_size,
                                size_t* program_info_size_out) {
  if (program_info_size_out != nullptr) {
    *program_info_size_out = 0u;
  }

  const size_t program_info_size =
      serialized_program_info_size(serialized_program, serialized_program_size);
  if (program_info_size == 0u || program_info_size > serialized_program_size) {
    return false;
  }

  flatbuffers::Verifier verifier(serialized_program, program_info_size);
  if (!akida::fb::VerifySizePrefixedProgramInfoBuffer(verifier)) {
    return false;
  }

  const auto* program_info =
      akida::fb::GetSizePrefixedProgramInfo(serialized_program);
  if (program_info == nullptr || program_info->version() == nullptr ||
      program_info->device_version() == nullptr) {
    return false;
  }

  if (std::strcmp(program_info->version()->c_str(), akida::version()) != 0) {
    return false;
  }

  if (program_info_size_out != nullptr) {
    *program_info_size_out = program_info_size;
  }
  return true;
}

std::vector<akida::TensorUniquePtr> dense_to_sparse_inputs(
    const std::vector<akida::TensorConstPtr>& dense_inputs,
    const akida::ProgramInfo& program_info) {
  std::vector<akida::TensorUniquePtr> sparse_inputs;
  sparse_inputs.reserve(dense_inputs.size());
  for (const auto& input : dense_inputs) {
    sparse_inputs.push_back(akida::conversion::to_sparse(
        *static_cast<const akida::Dense*>(input.get()), program_info));
  }
  return sparse_inputs;
}

size_t best_index_for_dense_output(const akida::Dense& dense) {
  if (dense.size() == 0u) {
    return 0u;
  }

  size_t best = 0u;
  switch (dense.type()) {
    case akida::TensorType::int32: {
      const int32_t* values = dense.data<int32_t>();
      for (size_t i = 1; i < dense.size(); ++i) {
        if (values[i] > values[best]) {
          best = i;
        }
      }
      return best;
    }
    case akida::TensorType::float32: {
      const float* values = dense.data<float>();
      for (size_t i = 1; i < dense.size(); ++i) {
        if (values[i] > values[best]) {
          best = i;
        }
      }
      return best;
    }
    case akida::TensorType::uint8:
    case akida::TensorType::uint4:
    case akida::TensorType::uint2: {
      const uint8_t* values = dense.data<uint8_t>();
      for (size_t i = 1; i < dense.size(); ++i) {
        if (values[i] > values[best]) {
          best = i;
        }
      }
      return best;
    }
    case akida::TensorType::int8:
    case akida::TensorType::int4:
    case akida::TensorType::int2:
    case akida::TensorType::bit: {
      const int8_t* values = dense.data<int8_t>();
      for (size_t i = 1; i < dense.size(); ++i) {
        if (values[i] > values[best]) {
          best = i;
        }
      }
      return best;
    }
  }

  return 0u;
}

AKD1500RunResult make_failed_result(const AKD1500Error& error) {
  AKD1500RunResult result;
  result.status = error.status;
  result.error = error;
  return result;
}

AKD1500RunResult make_run_result(const akida::Dense& dense) {
  AKD1500RunResult result;
  result.status = AKD1500Status::Ok;
  result.error = make_error(AKD1500Status::Ok, "ok");
  result.type = dense.type();
  result.dimensions = dense.dimensions();
  result.bytes.resize(dense.buffer()->size());
  if (!result.bytes.empty()) {
    std::memcpy(result.bytes.data(), dense.buffer()->data(), result.bytes.size());
  }
  result.predictedIndex = best_index_for_dense_output(dense);
  return result;
}

uint32_t normalize_external_model_address(uint32_t address_or_offset) {
  if (address_or_offset >= kExternalModelAliasBase &&
      address_or_offset < (kExternalModelAliasBase + kExternalModelWindowSize)) {
    return address_or_offset;
  }

  if (address_or_offset >= kExternalModelWindowBase &&
      address_or_offset <
          (kExternalModelWindowBase + kExternalModelWindowSize)) {
    return kExternalModelAliasBase +
           (address_or_offset - kExternalModelWindowBase);
  }

  if (address_or_offset < kExternalModelWindowSize) {
    return kExternalModelAliasBase + address_or_offset;
  }

  return address_or_offset;
}

bool shapes_equal(const akida::Shape& lhs, const akida::Shape& rhs) {
  if (lhs.size() != rhs.size()) {
    return false;
  }

  for (size_t i = 0; i < lhs.size(); ++i) {
    if (lhs[i] != rhs[i]) {
      return false;
    }
  }

  return true;
}

}  // namespace

AKD1500RunnerBase* AkidaNicla::activeRunnerBase() const {
#if AKD1500_PLATFORM_SUPPORTED
  if (active_runner_ == ActiveRunner::Flash && flash_runner_) {
    return flash_runner_.get();
  }
  if (active_runner_ == ActiveRunner::Host && host_runner_) {
    return host_runner_.get();
  }
#endif
  return nullptr;
}

akida::HardwareDevice* AkidaNicla::hardwareDevice() const {
  AKD1500RunnerBase* runner = activeRunnerBase();
  return runner != nullptr ? runner->hardwareDevice() : nullptr;
}

akida::HardwareDriver* AkidaNicla::hardwareDriver() const {
  AKD1500RunnerBase* runner = activeRunnerBase();
  return runner != nullptr ? runner->hardwareDriver() : nullptr;
}

const akida::ProgramInfo* AkidaNicla::programInfo() const {
#if AKD1500_PLATFORM_SUPPORTED
  AKD1500RunnerBase* runner = activeRunnerBase();
  return runner != nullptr ? &runner->programInfo() : nullptr;
#else
  return nullptr;
#endif
}

AKD1500Options AKD1500Options::niclaVisionDefaults() {
  AKD1500Options options;
  options.akidaCsPin = 7u;
  options.bridgeCsPin = 3u;
  // The Nicla Vision demo path is dominated by host-to-AKD input upload, and
  // 8 MHz is a validated stable setting on connected hardware.
  options.spiClockHz = 8000000u;
  options.flashSpiClockHz = 2000000u;
  options.externalModelAddress = kExternalModelAliasBase;
  options.postBeginSettleMs = 50u;
  options.postLinkSettleMs = 50u;
  options.fetchPollDelayMs = 1u;
  return options;
}

size_t AKD1500RunResult::elementCount() const {
  if (dimensions.size() == 0u) {
    return 0u;
  }
  return akida::shape_size(dimensions);
}

AKD1500RunnerBase::AKD1500RunnerBase(const AKD1500Options& options)
    : options_(options), board_(make_board_config(options)) {
  last_error_ = make_error(AKD1500Status::Ok, "ok");
}

void AKD1500RunnerBase::dumpFetchTimeoutState(uint32_t fetch_polls,
                                              uint32_t elapsed_us) {
#ifdef ARDUINO
  Serial.print("[AKD1500][fetch_timeout] polls=");
  Serial.print(static_cast<unsigned long>(fetch_polls));
  Serial.print(" elapsed_us=");
  Serial.print(static_cast<unsigned long>(elapsed_us));
  Serial.print(" input_dense=");
  Serial.print(inputIsDense() ? "yes" : "no");
  Serial.print(" output_dense=");
  Serial.print(outputIsDense() ? "yes" : "no");
  Serial.print(" can_learn=");
  Serial.print(canLearn() ? "yes" : "no");
  if (device_ != nullptr) {
    Serial.print(" clock_counter=");
    Serial.print(static_cast<unsigned long>(device_->read_clock_counter()));
    Serial.print(" config_clock_counter=");
    Serial.print(
        static_cast<unsigned long>(device_->read_config_clock_counter()));
    const auto memory = device_->memory();
    Serial.print(" memory_cur=");
    Serial.print(static_cast<unsigned long>(memory.first));
    Serial.print(" memory_peak=");
    Serial.println(static_cast<unsigned long>(memory.second));
  } else {
    Serial.println(" device_state=not_initialized");
  }
#else
  (void)fetch_polls;
  (void)elapsed_us;
#endif
  board_.dump_runtime_state("[AKD1500][fetch_timeout]");
}

akida::Shape AKD1500RunnerBase::inputDimensions() const {
  if (!program_info_.is_valid()) {
    return akida::Shape();
  }

  const uint32_t* dims = program_info_.input_dims();
  return akida::Shape{1u, dims[0], dims[1], dims[2]};
}

akida::Shape AKD1500RunnerBase::outputDimensions() const {
  if (!program_info_.is_valid()) {
    return akida::Shape();
  }

  return program_info_.output_dims();
}

bool AKD1500RunnerBase::inputIsDense() const {
  return program_info_.is_valid() ? program_info_.input_is_dense() : false;
}

bool AKD1500RunnerBase::outputIsDense() const {
  return program_info_.is_valid() ? program_info_.output_is_dense() : false;
}

bool AKD1500RunnerBase::canLearn() const {
  return program_info_.is_valid() ? program_info_.can_learn() : false;
}

AKD1500Status AKD1500RunnerBase::setError(AKD1500Status status,
                                          const char* message,
                                          uint32_t detail) {
  last_error_ = make_error(status, message, detail);
  return status;
}

void AKD1500RunnerBase::clearLoadedModel() {
  program_info_ = akida::ProgramInfo();
  model_loaded_ = false;
}

void AKD1500RunnerBase::setLoadedModel(const akida::ProgramInfo& program_info) {
  program_info_ = program_info;
  model_loaded_ = program_info_.is_valid();
  last_error_ = make_error(AKD1500Status::Ok, "ok");
}

AKD1500Status AKD1500RunnerBase::beginRunner() {
  if (device_) {
    return setError(AKD1500Status::Ok, "ok");
  }

  board_.begin();
  if (options_.postBeginSettleMs > 0u) {
    delay(options_.postBeginSettleMs);
  }

  ip_version_ = board_.read_ip_version();
  if (options_.expectedIpVersion != 0u &&
      ip_version_ != options_.expectedIpVersion) {
    device_.reset();
    clearLoadedModel();
    return setError(AKD1500Status::LinkFailed, "unexpected_ip_version",
                    ip_version_);
  }

  if (options_.postLinkSettleMs > 0u) {
    delay(options_.postLinkSettleMs);
  }

  device_ = akida::HardwareDevice::create(&board_.hardware_driver());
  if (!device_) {
    return setError(AKD1500Status::NotInitialized, "device_create_failed");
  }

  return setError(AKD1500Status::Ok, "ok");
}

AKD1500RunResult AKD1500RunnerBase::runImpl(
    const void* input_data, akida::TensorType input_type,
    const akida::Shape& input_dimensions) {
  if (!device_) {
    return make_failed_result(
        make_error(setError(AKD1500Status::NotInitialized, "begin_required"),
                   "begin_required"));
  }
  if (!model_loaded_ || !program_info_.is_valid()) {
    return make_failed_result(
        make_error(setError(AKD1500Status::ModelNotLoaded, "model_not_loaded"),
                   "model_not_loaded"));
  }
  if (input_data == nullptr || input_dimensions.size() == 0u) {
    return make_failed_result(
        make_error(setError(AKD1500Status::InvalidInput, "invalid_input"),
                   "invalid_input"));
  }

  auto input_tensor = akida::Dense::create_view(
      reinterpret_cast<const char*>(input_data), input_type, input_dimensions,
      akida::Dense::Layout::RowMajor);
  auto input_vector = akida::Dense::split(*input_tensor);
  if (input_vector.empty()) {
    return make_failed_result(
        make_error(setError(AKD1500Status::InvalidInput, "no_input"),
                   "no_input"));
  }

  auto& driver = board_.hardware_driver();
  const uint32_t set_batch_start_us = micros();
  device_->set_batch_size(1u, driver.akida_visible_memory() == 0u);
  const uint32_t set_batch_us = micros() - set_batch_start_us;

  const uint32_t conversion_start_us = micros();
  std::vector<akida::TensorUniquePtr> sparse_inputs;
  const bool needs_sparse = !program_info_.input_is_dense();
  if (needs_sparse) {
    sparse_inputs = dense_to_sparse_inputs(input_vector, program_info_);
  }
  const uint32_t conversion_us = micros() - conversion_start_us;

  const akida::Tensor& input =
      needs_sparse ? *sparse_inputs.front() : *input_vector.front();
  const uint32_t enqueue_start_us = micros();
  if (!device_->enqueue(input)) {
    return make_failed_result(
        make_error(setError(AKD1500Status::EnqueueFailed, "enqueue_failed"),
                   "enqueue_failed"));
  }
  const uint32_t enqueue_us = micros() - enqueue_start_us;

  const uint32_t fetch_start_us = micros();
  akida::TensorUniquePtr output;
  uint32_t fetch_polls = 0u;
  while (!output) {
    output = device_->fetch();
    if (output) {
      break;
    }
    ++fetch_polls;
    if ((micros() - fetch_start_us) >
        (options_.fetchTimeoutMs * 1000u)) {
      dumpFetchTimeoutState(fetch_polls, micros() - fetch_start_us);
      return make_failed_result(
          make_error(setError(AKD1500Status::FetchTimeout, "fetch_timeout"),
                     "fetch_timeout"));
    }
    delay(options_.fetchPollDelayMs);
  }
  const uint32_t fetch_us = micros() - fetch_start_us;

  if (kPrintRunTiming) {
    Serial.print("[AKD1500] run_timing set_batch_us=");
    Serial.print(static_cast<unsigned long>(set_batch_us));
    Serial.print(" conversion_us=");
    Serial.print(static_cast<unsigned long>(conversion_us));
    Serial.print(" enqueue_us=");
    Serial.print(static_cast<unsigned long>(enqueue_us));
    Serial.print(" fetch_us=");
    Serial.print(static_cast<unsigned long>(fetch_us));
    Serial.print(" fetch_polls=");
    Serial.println(static_cast<unsigned long>(fetch_polls));
  }

  const akida::Dense* dense_output = akida::conversion::as_dense(*output);
  akida::DenseUniquePtr converted_output;
  if (dense_output == nullptr) {
    const akida::Sparse* sparse_output = akida::conversion::as_sparse(*output);
    if (sparse_output != nullptr) {
      converted_output = akida::conversion::to_dense(*sparse_output);
      dense_output = converted_output.get();
    }
  }
  if (dense_output == nullptr) {
    return make_failed_result(make_error(
        setError(AKD1500Status::OutputFormatMismatch,
                 "output_format_mismatch"),
        "output_format_mismatch"));
  }

  last_error_ = make_error(AKD1500Status::Ok, "ok");
  return make_run_result(*dense_output);
}

AKD1500HostRunner::AKD1500HostRunner(const AKD1500Options& options)
    : AKD1500RunnerBase(options) {}

AKD1500Status AKD1500HostRunner::begin() { return beginRunner(); }

AKD1500Status AKD1500HostRunner::loadModel(const uint8_t* serialized_program,
                                           size_t serialized_program_size) {
  if (beginRunner() != AKD1500Status::Ok) {
    return last_error_.status;
  }

  size_t program_info_size = 0u;
  if (!validate_program_info_blob(serialized_program, serialized_program_size,
                                  &program_info_size)) {
    clearLoadedModel();
    return setError(AKD1500Status::ProgramInfoInvalid,
                    "invalid_program_info");
  }
  (void)program_info_size;

  const auto program_info =
      device()->program(serialized_program, serialized_program_size);
  if (!program_info.is_valid()) {
    clearLoadedModel();
    return setError(AKD1500Status::ProgramFailed, "program_failed");
  }

  setLoadedModel(program_info);
  return AKD1500Status::Ok;
}

AKD1500RunResult AKD1500HostRunner::run(
    const void* input_data, akida::TensorType input_type,
    const akida::Shape& input_dimensions) {
  return runImpl(input_data, input_type, input_dimensions);
}

AKD1500RunResult AKD1500HostRunner::run(
    const uint8_t* input_data, const akida::Shape& input_dimensions) {
  return runImpl(input_data, input_dimensions);
}

AKD1500FlashRunner::AKD1500FlashRunner(const AKD1500Options& options)
    : AKD1500RunnerBase(options) {}

AKD1500Status AKD1500FlashRunner::begin() {
  if (device_) {
    return setError(AKD1500Status::Ok, "ok");
  }

  board_.begin();
  if (options_.postBeginSettleMs > 0u) {
    delay(options_.postBeginSettleMs);
  }

  ip_version_ = board_.read_ip_version();
  if (options_.expectedIpVersion != 0u &&
      ip_version_ != options_.expectedIpVersion) {
    device_.reset();
    clearLoadedModel();
    return setError(AKD1500Status::LinkFailed, "unexpected_ip_version",
                    ip_version_);
  }

  if (!board_.ensure_spi_flash_runtime_profile()) {
    clearLoadedModel();
    return setError(AKD1500Status::NotInitialized, "unsupported_flash_jedec",
                    board_.detected_flash_jedec());
  }

  if (options_.postLinkSettleMs > 0u) {
    delay(options_.postLinkSettleMs);
  }

  device_ = akida::HardwareDevice::create(&board_.hardware_driver());
  if (!device_) {
    return setError(AKD1500Status::NotInitialized, "device_create_failed");
  }

  return setError(AKD1500Status::Ok, "ok");
}

AKD1500Status AKD1500FlashRunner::loadExternalModel(
    const uint8_t* serialized_program, size_t serialized_program_size) {
  if (begin() != AKD1500Status::Ok) {
    return last_error_.status;
  }

  size_t program_info_size = 0u;
  if (!validate_program_info_blob(serialized_program, serialized_program_size,
                                  &program_info_size)) {
    clearLoadedModel();
    return setError(AKD1500Status::ProgramInfoInvalid,
                    "invalid_program_info");
  }

  const auto program_info = device()->program_external_data(
      serialized_program, program_info_size, options_.externalModelAddress);
  if (!program_info.is_valid()) {
    clearLoadedModel();
    if (akida::dma::has_runtime_fault()) {
      return setError(AKD1500Status::ProgramFailed,
                      akida::dma::runtime_fault_message());
    }
    return setError(AKD1500Status::ProgramFailed, "program_external_failed");
  }

  if (!board_.reinit_spi_flash_runtime()) {
    clearLoadedModel();
    return setError(AKD1500Status::ProgramFailed,
                    "flash_runtime_reinit_after_program",
                    board_.detected_flash_jedec());
  }

  setLoadedModel(program_info);
  return AKD1500Status::Ok;
}

AKD1500RunResult AKD1500FlashRunner::run(
    const void* input_data, akida::TensorType input_type,
    const akida::Shape& input_dimensions) {
  board_.reinit_spi_flash_runtime();
  return runImpl(input_data, input_type, input_dimensions);
}

AKD1500RunResult AKD1500FlashRunner::run(
    const uint8_t* input_data, const akida::Shape& input_dimensions) {
  board_.reinit_spi_flash_runtime();
  return runImpl(input_data, input_dimensions);
}

AkidaNicla::AkidaNicla(const AKD1500Options& options) : options_(options) {
  last_error_ = make_error(AKD1500Status::Ok, "ok");
}

AKD1500Status AkidaNicla::setError(AKD1500Status status, const char* message,
                                   uint32_t detail) {
  last_error_ = make_error(status, message, detail);
  return status;
}

AKD1500Status AkidaNicla::begin() {
  host_runner_.reset(new AKD1500HostRunner(options_));
  const AKD1500Status status = host_runner_->begin();
  flash_runner_.reset();
  active_runner_ = (status == AKD1500Status::Ok) ? ActiveRunner::Host
                                                 : ActiveRunner::None;
  ip_version_ = host_runner_->ipVersion();
  last_error_ = host_runner_->lastError();
  model_info_ = {};
  return status;
}

AKD1500Status AkidaNicla::begin(const AKD1500Options& options) {
  options_ = options;
  host_runner_.reset();
  flash_runner_.reset();
  active_runner_ = ActiveRunner::None;
  ip_version_ = 0u;
  model_info_ = {};
  return begin();
}

AKD1500Status AkidaNicla::load(const AKD1500Model& model) {
  if (model.serializedProgram == nullptr || model.size == 0u) {
    model_info_ = {};
    active_runner_ = ActiveRunner::None;
    return setError(AKD1500Status::InvalidInput, "invalid_model");
  }

  model_info_ = {};

  if (model.storage == AKD1500ModelStorage::ExternalFlash) {
    host_runner_.reset();
    active_runner_ = ActiveRunner::None;

    AKD1500Options flash_options = options_;
    flash_options.externalModelAddress =
        normalize_external_model_address(model.externalLocation);
    flash_runner_.reset(new AKD1500FlashRunner(flash_options));
    const AKD1500Status begin_status = flash_runner_->begin();
    ip_version_ = flash_runner_->ipVersion();
    last_error_ = flash_runner_->lastError();
    if (begin_status != AKD1500Status::Ok) {
      active_runner_ = ActiveRunner::None;
      return begin_status;
    }

    const AKD1500Status load_status =
        flash_runner_->loadExternalModel(model.serializedProgram, model.size);
    active_runner_ = ActiveRunner::Flash;
    ip_version_ = flash_runner_->ipVersion();
    last_error_ = flash_runner_->lastError();
    if (load_status != AKD1500Status::Ok) {
      model_info_ = {};
      return load_status;
    }

    model_info_.valid = flash_runner_->modelLoaded();
    model_info_.inputIsDense = flash_runner_->inputIsDense();
    model_info_.outputIsDense = flash_runner_->outputIsDense();
    model_info_.canLearn = flash_runner_->canLearn();
    model_info_.inputDimensions = flash_runner_->inputDimensions();
    model_info_.outputDimensions = flash_runner_->outputDimensions();
    return setError(AKD1500Status::Ok, "ok");
  }

  flash_runner_.reset();
  active_runner_ = ActiveRunner::None;

  host_runner_.reset(new AKD1500HostRunner(options_));
  const AKD1500Status begin_status = host_runner_->begin();
  ip_version_ = host_runner_->ipVersion();
  last_error_ = host_runner_->lastError();
  if (begin_status != AKD1500Status::Ok) {
    active_runner_ = ActiveRunner::None;
    return begin_status;
  }

  const AKD1500Status load_status =
      host_runner_->loadModel(model.serializedProgram, model.size);
  active_runner_ = ActiveRunner::Host;
  ip_version_ = host_runner_->ipVersion();
  last_error_ = host_runner_->lastError();
  if (load_status != AKD1500Status::Ok) {
    model_info_ = {};
    return load_status;
  }

  model_info_.valid = host_runner_->modelLoaded();
  model_info_.inputIsDense = host_runner_->inputIsDense();
  model_info_.outputIsDense = host_runner_->outputIsDense();
  model_info_.canLearn = host_runner_->canLearn();
  model_info_.inputDimensions = host_runner_->inputDimensions();
  model_info_.outputDimensions = host_runner_->outputDimensions();
  return setError(AKD1500Status::Ok, "ok");
}

bool AkidaNicla::inputMatchesModel(const akida::Shape& input_dimensions) const {
  if (!model_info_.valid || model_info_.inputDimensions.size() == 0u) {
    return true;
  }

  if (shapes_equal(input_dimensions, model_info_.inputDimensions)) {
    return true;
  }

  if (model_info_.inputDimensions.size() == 4u && input_dimensions.size() == 3u &&
      model_info_.inputDimensions[0] == 1u) {
    return input_dimensions[0] == model_info_.inputDimensions[1] &&
           input_dimensions[1] == model_info_.inputDimensions[2] &&
           input_dimensions[2] == model_info_.inputDimensions[3];
  }

  return false;
}

AKD1500RunResult AkidaNicla::dispatchInfer(const AKD1500Input& input) {
  if (!model_info_.valid || active_runner_ == ActiveRunner::None) {
    return make_failed_result(
        make_error(setError(AKD1500Status::ModelNotLoaded, "model_not_loaded"),
                   "model_not_loaded"));
  }

  if (input.data == nullptr || input.dimensions.size() == 0u) {
    return make_failed_result(
        make_error(setError(AKD1500Status::InvalidInput, "invalid_input"),
                   "invalid_input"));
  }

  if (!inputMatchesModel(input.dimensions)) {
    return make_failed_result(
        make_error(setError(AKD1500Status::InvalidInput,
                            "input_shape_mismatch"),
                   "input_shape_mismatch"));
  }

  AKD1500RunResult result;
  if (active_runner_ == ActiveRunner::Flash && flash_runner_) {
    result = flash_runner_->run(input.data, input.type, input.dimensions);
    ip_version_ = flash_runner_->ipVersion();
    last_error_ = flash_runner_->lastError();
  } else if (active_runner_ == ActiveRunner::Host && host_runner_) {
    result = host_runner_->run(input.data, input.type, input.dimensions);
    ip_version_ = host_runner_->ipVersion();
    last_error_ = host_runner_->lastError();
  } else {
    return make_failed_result(
        make_error(setError(AKD1500Status::NotInitialized, "runner_missing"),
                   "runner_missing"));
  }

  if (result.ok()) {
    last_error_ = make_error(AKD1500Status::Ok, "ok");
  } else {
    last_error_ = result.error;
  }
  return result;
}

AKD1500RunResult AkidaNicla::infer(const AKD1500Input& input) {
  return dispatchInfer(input);
}

AKD1500RunResult AkidaNicla::infer(
    const void* input_data, akida::TensorType input_type,
    const akida::Shape& input_dimensions) {
  AKD1500Input input;
  input.data = input_data;
  input.type = input_type;
  input.dimensions = input_dimensions;
  return dispatchInfer(input);
}

AKD1500RunResult AkidaNicla::inferUint8(
    const uint8_t* input_data, const akida::Shape& input_dimensions) {
  return infer(input_data, akida::TensorType::uint8, input_dimensions);
}

AKD1500RunResult AkidaNicla::inferUint8(const uint8_t* input_data) {
  if (!model_info_.valid || model_info_.inputDimensions.size() == 0u) {
    return make_failed_result(
        make_error(setError(AKD1500Status::ModelNotLoaded, "model_not_loaded"),
                   "model_not_loaded"));
  }
  return inferUint8(input_data, model_info_.inputDimensions);
}

AKD1500ClassificationResult AkidaNicla::makeClassificationResult(
    const AKD1500RunResult& run_result) const {
  AKD1500ClassificationResult result;
  result.status = run_result.status;
  result.error = run_result.error;
  result.predictedIndex = run_result.predictedIndex;
  result.scores = run_result;
  return result;
}

AKD1500ClassificationResult AkidaNicla::classify(const AKD1500Input& input) {
  return makeClassificationResult(dispatchInfer(input));
}

AKD1500ClassificationResult AkidaNicla::classify(
    const void* input_data, akida::TensorType input_type,
    const akida::Shape& input_dimensions) {
  return makeClassificationResult(
      infer(input_data, input_type, input_dimensions));
}

AKD1500ClassificationResult AkidaNicla::classifyUint8(
    const uint8_t* input_data, const akida::Shape& input_dimensions) {
  return makeClassificationResult(inferUint8(input_data, input_dimensions));
}

AKD1500ClassificationResult AkidaNicla::classifyUint8(
    const uint8_t* input_data) {
  return makeClassificationResult(inferUint8(input_data));
}

void AkidaNicla::printLastError(Print& out) const {
  out.print("status=");
  out.print(statusName(last_error_.status));
  out.print(" (");
  out.print(static_cast<int>(last_error_.status));
  out.print(")");
  out.print(" detail=");
  out.print(last_error_.detail);
  out.print(" reason=");
  out.println(last_error_.message);
}

void AkidaNicla::printModelInfo(Print& out) const {
  out.print("valid=");
  out.print(model_info_.valid ? "yes" : "no");
  out.print(" input_dense=");
  out.print(model_info_.inputIsDense ? "yes" : "no");
  out.print(" output_dense=");
  out.print(model_info_.outputIsDense ? "yes" : "no");
  out.print(" input_shape=[");
  for (size_t i = 0; i < model_info_.inputDimensions.size(); ++i) {
    if (i != 0u) {
      out.print(", ");
    }
    out.print(model_info_.inputDimensions[i]);
  }
  out.print("] output_shape=[");
  for (size_t i = 0; i < model_info_.outputDimensions.size(); ++i) {
    if (i != 0u) {
      out.print(", ");
    }
    out.print(model_info_.outputDimensions[i]);
  }
  out.println("]");
}

uint32_t AkidaNicla::normalizeExternalModelAddress(uint32_t address_or_offset) {
  return normalize_external_model_address(address_or_offset);
}

uint32_t AkidaNicla::externalModelAddressFromOffset(uint32_t offset) {
  return normalize_external_model_address(offset);
}

bool AkidaNicla::stageModelToFlash(const uint8_t* serialized_program,
                                   size_t serialized_program_size,
                                   uint32_t address_or_offset) {
  return stageModelToFlash(AKD1500Options::niclaVisionDefaults(),
                           serialized_program, serialized_program_size,
                           address_or_offset);
}

bool AkidaNicla::stageModelToFlash(const AKD1500Options& options,
                                   const uint8_t* serialized_program,
                                   size_t serialized_program_size,
                                   uint32_t address_or_offset) {
  akida_port::AKD1500BoardConfig config = make_board_config(options);
  config.external_program_data_address =
      normalize_external_model_address(address_or_offset);
  if (Serial) {
    Serial.print("[AKD1500][flash_wrap] stage begin size=");
    Serial.print(static_cast<unsigned long>(serialized_program_size));
    Serial.print(" addr=0x");
    Serial.println(
        static_cast<unsigned long>(config.external_program_data_address), HEX);
  }
  const bool ok = akida_port::stage_program_data_to_bridge_flash(
      config, serialized_program, serialized_program_size,
      config.external_program_data_address);
  if (Serial) {
    Serial.print("[AKD1500][flash_wrap] stage result=");
    Serial.println(ok ? "PASS" : "FAIL");
  }
  return ok;
}

bool AkidaNicla::verifyModelInFlash(const uint8_t* serialized_program,
                                    size_t serialized_program_size,
                                    uint32_t address_or_offset) {
  return verifyModelInFlash(AKD1500Options::niclaVisionDefaults(),
                            serialized_program, serialized_program_size,
                            address_or_offset);
}

bool AkidaNicla::verifyModelInFlash(const AKD1500Options& options,
                                    const uint8_t* serialized_program,
                                    size_t serialized_program_size,
                                    uint32_t address_or_offset) {
  akida_port::AKD1500BoardConfig config = make_board_config(options);
  config.external_program_data_address =
      normalize_external_model_address(address_or_offset);
  if (Serial) {
    Serial.print("[AKD1500][flash_wrap] verify begin size=");
    Serial.print(static_cast<unsigned long>(serialized_program_size));
    Serial.print(" addr=0x");
    Serial.println(
        static_cast<unsigned long>(config.external_program_data_address), HEX);
  }
  const bool ok = akida_port::verify_program_data_from_bridge_flash(
      config, serialized_program, serialized_program_size,
      config.external_program_data_address);
  if (Serial) {
    Serial.print("[AKD1500][flash_wrap] verify result=");
    Serial.println(ok ? "PASS" : "FAIL");
  }
  return ok;
}

const char* AkidaNicla::statusName(AKD1500Status status) {
  switch (status) {
    case AKD1500Status::Ok:
      return "Ok";
    case AKD1500Status::NotInitialized:
      return "NotInitialized";
    case AKD1500Status::LinkFailed:
      return "LinkFailed";
    case AKD1500Status::ProgramInfoInvalid:
      return "ProgramInfoInvalid";
    case AKD1500Status::ProgramFailed:
      return "ProgramFailed";
    case AKD1500Status::ModelNotLoaded:
      return "ModelNotLoaded";
    case AKD1500Status::InvalidInput:
      return "InvalidInput";
    case AKD1500Status::EnqueueFailed:
      return "EnqueueFailed";
    case AKD1500Status::FetchTimeout:
      return "FetchTimeout";
    case AKD1500Status::OutputFormatMismatch:
      return "OutputFormatMismatch";
  }
  return "Unknown";
}

#else

namespace {

constexpr uint32_t kExternalModelAliasBase = 0x80000000u;
constexpr uint32_t kExternalModelWindowBase = 0xFC000000u;
constexpr uint32_t kExternalModelWindowSize = 0x00800000u;
constexpr const char* kUnsupportedPlatformMessage =
    "unsupported_platform_requires_nicla_vision";

AKD1500Error make_error(AKD1500Status status, const char* message,
                        uint32_t detail = 0u) {
  AKD1500Error error;
  error.status = status;
  error.detail = detail;
  error.message = message;
  return error;
}

AKD1500RunResult make_failed_result(const AKD1500Error& error) {
  AKD1500RunResult result;
  result.status = error.status;
  result.error = error;
  return result;
}

uint32_t normalize_external_model_address(uint32_t address_or_offset) {
  if (address_or_offset >= kExternalModelAliasBase &&
      address_or_offset < (kExternalModelAliasBase + kExternalModelWindowSize)) {
    return address_or_offset;
  }

  if (address_or_offset >= kExternalModelWindowBase &&
      address_or_offset <
          (kExternalModelWindowBase + kExternalModelWindowSize)) {
    return kExternalModelAliasBase +
           (address_or_offset - kExternalModelWindowBase);
  }

  if (address_or_offset < kExternalModelWindowSize) {
    return kExternalModelAliasBase + address_or_offset;
  }

  return address_or_offset;
}

}  // namespace

AKD1500Options AKD1500Options::niclaVisionDefaults() {
  AKD1500Options options;
  options.akidaCsPin = 7u;
  options.bridgeCsPin = 3u;
  options.spiClockHz = 8000000u;
  options.flashSpiClockHz = 2000000u;
  options.externalModelAddress = kExternalModelAliasBase;
  options.postBeginSettleMs = 50u;
  options.postLinkSettleMs = 50u;
  options.fetchPollDelayMs = 1u;
  return options;
}

size_t AKD1500RunResult::elementCount() const {
  return akida::shape_size(dimensions);
}

AKD1500RunnerBase::AKD1500RunnerBase(const AKD1500Options& options)
    : options_(options) {
  last_error_ = make_error(AKD1500Status::Ok, "ok");
}

void AKD1500RunnerBase::dumpFetchTimeoutState(uint32_t, uint32_t) {}

void AKD1500RunnerBase::clearLoadedModel() {
  model_loaded_ = false;
  model_info_ = {};
}

AKD1500Status AKD1500RunnerBase::setError(AKD1500Status status,
                                          const char* message,
                                          uint32_t detail) {
  last_error_ = make_error(status, message, detail);
  return status;
}

AKD1500Status AKD1500RunnerBase::beginRunner() {
  clearLoadedModel();
  ip_version_ = 0u;
  return setError(AKD1500Status::LinkFailed, kUnsupportedPlatformMessage);
}

AKD1500RunResult AKD1500RunnerBase::runImpl(const void*, akida::TensorType,
                                            const akida::Shape&) {
  return make_failed_result(
      make_error(AKD1500Status::LinkFailed, kUnsupportedPlatformMessage));
}

AKD1500HostRunner::AKD1500HostRunner(const AKD1500Options& options)
    : AKD1500RunnerBase(options) {}

AKD1500Status AKD1500HostRunner::begin() { return beginRunner(); }

AKD1500Status AKD1500HostRunner::loadModel(const uint8_t*, size_t) {
  clearLoadedModel();
  return setError(AKD1500Status::LinkFailed, kUnsupportedPlatformMessage);
}

AKD1500RunResult AKD1500HostRunner::run(const void* input_data,
                                        akida::TensorType input_type,
                                        const akida::Shape& input_dimensions) {
  return runImpl(input_data, input_type, input_dimensions);
}

AKD1500RunResult AKD1500HostRunner::run(
    const uint8_t* input_data, const akida::Shape& input_dimensions) {
  return runImpl(input_data, input_dimensions);
}

AKD1500FlashRunner::AKD1500FlashRunner(const AKD1500Options& options)
    : AKD1500RunnerBase(options) {}

AKD1500Status AKD1500FlashRunner::begin() { return beginRunner(); }

AKD1500Status AKD1500FlashRunner::loadExternalModel(const uint8_t*, size_t) {
  clearLoadedModel();
  return setError(AKD1500Status::LinkFailed, kUnsupportedPlatformMessage);
}

AKD1500RunResult AKD1500FlashRunner::run(const void* input_data,
                                         akida::TensorType input_type,
                                         const akida::Shape& input_dimensions) {
  return runImpl(input_data, input_type, input_dimensions);
}

AKD1500RunResult AKD1500FlashRunner::run(
    const uint8_t* input_data, const akida::Shape& input_dimensions) {
  return runImpl(input_data, input_dimensions);
}

AkidaNicla::AkidaNicla(const AKD1500Options& options) : options_(options) {
  last_error_ = make_error(AKD1500Status::Ok, "ok");
}

AKD1500Status AkidaNicla::setError(AKD1500Status status, const char* message,
                                   uint32_t detail) {
  last_error_ = make_error(status, message, detail);
  return status;
}

AKD1500Status AkidaNicla::begin() { return begin(options_); }

AKD1500Status AkidaNicla::begin(const AKD1500Options& options) {
  options_ = options;
  host_runner_.reset();
  flash_runner_.reset();
  active_runner_ = ActiveRunner::None;
  model_info_ = {};
  ip_version_ = 0u;
  return setError(AKD1500Status::LinkFailed, kUnsupportedPlatformMessage);
}

AKD1500Status AkidaNicla::load(const AKD1500Model&) {
  model_info_ = {};
  active_runner_ = ActiveRunner::None;
  return setError(AKD1500Status::LinkFailed, kUnsupportedPlatformMessage);
}

AKD1500RunResult AkidaNicla::dispatchInfer(const AKD1500Input&) {
  return make_failed_result(
      make_error(AKD1500Status::LinkFailed, kUnsupportedPlatformMessage));
}

AKD1500RunResult AkidaNicla::infer(const AKD1500Input& input) {
  return dispatchInfer(input);
}

AKD1500RunResult AkidaNicla::infer(
    const void* input_data, akida::TensorType input_type,
    const akida::Shape& input_dimensions) {
  AKD1500Input input;
  input.data = input_data;
  input.type = input_type;
  input.dimensions = input_dimensions;
  return dispatchInfer(input);
}

AKD1500RunResult AkidaNicla::inferUint8(
    const uint8_t* input_data, const akida::Shape& input_dimensions) {
  return infer(input_data, akida::TensorType::uint8, input_dimensions);
}

AKD1500RunResult AkidaNicla::inferUint8(const uint8_t* input_data) {
  return infer(input_data, akida::TensorType::uint8, model_info_.inputDimensions);
}

AKD1500ClassificationResult AkidaNicla::makeClassificationResult(
    const AKD1500RunResult& run_result) const {
  AKD1500ClassificationResult result;
  result.status = run_result.status;
  result.error = run_result.error;
  result.predictedIndex = run_result.predictedIndex;
  result.scores = run_result;
  return result;
}

AKD1500ClassificationResult AkidaNicla::classify(const AKD1500Input& input) {
  return makeClassificationResult(infer(input));
}

AKD1500ClassificationResult AkidaNicla::classify(
    const void* input_data, akida::TensorType input_type,
    const akida::Shape& input_dimensions) {
  return makeClassificationResult(
      infer(input_data, input_type, input_dimensions));
}

AKD1500ClassificationResult AkidaNicla::classifyUint8(
    const uint8_t* input_data, const akida::Shape& input_dimensions) {
  return makeClassificationResult(inferUint8(input_data, input_dimensions));
}

AKD1500ClassificationResult AkidaNicla::classifyUint8(
    const uint8_t* input_data) {
  return makeClassificationResult(inferUint8(input_data));
}

void AkidaNicla::printLastError(Print& out) const {
  out.print("status=");
  out.print(statusName(last_error_.status));
  out.print(" (");
  out.print(static_cast<int>(last_error_.status));
  out.print(")");
  out.print(" detail=");
  out.print(last_error_.detail);
  out.print(" reason=");
  out.println(last_error_.message);
}

void AkidaNicla::printModelInfo(Print& out) const {
  out.print("valid=");
  out.print(model_info_.valid ? "yes" : "no");
  out.print(" input_dense=");
  out.print(model_info_.inputIsDense ? "yes" : "no");
  out.print(" output_dense=");
  out.print(model_info_.outputIsDense ? "yes" : "no");
  out.print(" input_shape=[");
  for (size_t i = 0; i < model_info_.inputDimensions.size(); ++i) {
    if (i != 0u) {
      out.print(", ");
    }
    out.print(model_info_.inputDimensions[i]);
  }
  out.print("] output_shape=[");
  for (size_t i = 0; i < model_info_.outputDimensions.size(); ++i) {
    if (i != 0u) {
      out.print(", ");
    }
    out.print(model_info_.outputDimensions[i]);
  }
  out.println("]");
}

uint32_t AkidaNicla::normalizeExternalModelAddress(uint32_t address_or_offset) {
  return normalize_external_model_address(address_or_offset);
}

uint32_t AkidaNicla::externalModelAddressFromOffset(uint32_t offset) {
  return normalize_external_model_address(offset);
}

bool AkidaNicla::stageModelToFlash(const uint8_t*, size_t, uint32_t) {
  return false;
}

bool AkidaNicla::stageModelToFlash(const AKD1500Options&, const uint8_t*, size_t,
                                   uint32_t) {
  return false;
}

bool AkidaNicla::verifyModelInFlash(const uint8_t*, size_t, uint32_t) {
  return false;
}

bool AkidaNicla::verifyModelInFlash(const AKD1500Options&, const uint8_t*, size_t,
                                    uint32_t) {
  return false;
}

bool AkidaNicla::inputMatchesModel(const akida::Shape& input_dimensions) const {
  return model_info_.valid && input_dimensions == model_info_.inputDimensions;
}

const char* AkidaNicla::statusName(AKD1500Status status) {
  switch (status) {
    case AKD1500Status::Ok:
      return "Ok";
    case AKD1500Status::NotInitialized:
      return "NotInitialized";
    case AKD1500Status::LinkFailed:
      return "LinkFailed";
    case AKD1500Status::ProgramInfoInvalid:
      return "ProgramInfoInvalid";
    case AKD1500Status::ProgramFailed:
      return "ProgramFailed";
    case AKD1500Status::ModelNotLoaded:
      return "ModelNotLoaded";
    case AKD1500Status::InvalidInput:
      return "InvalidInput";
    case AKD1500Status::EnqueueFailed:
      return "EnqueueFailed";
    case AKD1500Status::FetchTimeout:
      return "FetchTimeout";
    case AKD1500Status::OutputFormatMismatch:
      return "OutputFormatMismatch";
  }
  return "Unknown";
}

#endif
