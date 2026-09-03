#include "BB15.h"

#include <cstring>
#include <utility>

#if AKD1500_PLATFORM_SUPPORTED
#include "akida/dense.h"
#include "akida/input_conversion.h"
#include "akida/sparse.h"
#include "akida_port/nicla_voice_akd1500_board.h"
#endif
#include "bb15/pio_expander_6408.h"

namespace {

constexpr uint32_t kExternalModelAliasBase = 0x80000000u;
constexpr uint32_t kExternalModelWindowBase = 0xFC000000u;
constexpr uint32_t kExternalModelWindowSize = 0x00800000u;

BB15Error make_error(BB15Status status, const char* message,
                     uint32_t detail = 0u) {
  BB15Error error;
  error.status = status;
  error.detail = detail;
  error.message = message;
  return error;
}

bool shapes_equal(const akida::Shape& lhs, const akida::Shape& rhs) {
  if (lhs.size() != rhs.size()) {
    return false;
  }
  for (size_t i = 0u; i < lhs.size(); ++i) {
    if (lhs[i] != rhs[i]) {
      return false;
    }
  }
  return true;
}

#if AKD1500_PLATFORM_SUPPORTED
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
      for (size_t i = 1u; i < dense.size(); ++i) {
        if (values[i] > values[best]) {
          best = i;
        }
      }
      return best;
    }
    case akida::TensorType::float32: {
      const float* values = dense.data<float>();
      for (size_t i = 1u; i < dense.size(); ++i) {
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
      for (size_t i = 1u; i < dense.size(); ++i) {
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
      for (size_t i = 1u; i < dense.size(); ++i) {
        if (values[i] > values[best]) {
          best = i;
        }
      }
      return best;
    }
  }

  return 0u;
}

BB15RunResult make_run_result(const akida::Dense& dense) {
  BB15RunResult result;
  result.status = BB15Status::Ok;
  result.error = make_error(BB15Status::Ok, "ok");
  result.type = dense.type();
  result.dimensions = dense.dimensions();
  result.bytes.resize(dense.buffer()->size());
  if (!result.bytes.empty()) {
    std::memcpy(result.bytes.data(), dense.buffer()->data(),
                result.bytes.size());
  }
  result.predictedIndex = best_index_for_dense_output(dense);
  return result;
}
#endif

BB15Status map_status(bb15::internal::RuntimeStatus status) {
  switch (status) {
    case bb15::internal::RuntimeStatus::Ok:
      return BB15Status::Ok;
    case bb15::internal::RuntimeStatus::NotInitialized:
      return BB15Status::NotInitialized;
    case bb15::internal::RuntimeStatus::LinkFailed:
      return BB15Status::LinkFailed;
    case bb15::internal::RuntimeStatus::ProgramInfoInvalid:
      return BB15Status::ModelInvalid;
    case bb15::internal::RuntimeStatus::ProgramFailed:
      return BB15Status::ModelInvalid;
    case bb15::internal::RuntimeStatus::ModelNotLoaded:
      return BB15Status::ModelNotLoaded;
    case bb15::internal::RuntimeStatus::InvalidInput:
      return BB15Status::InvalidInput;
    case bb15::internal::RuntimeStatus::EnqueueFailed:
      return BB15Status::EnqueueFailed;
    case bb15::internal::RuntimeStatus::FetchTimeout:
      return BB15Status::FetchTimeout;
    case bb15::internal::RuntimeStatus::OutputFormatMismatch:
      return BB15Status::OutputFormatMismatch;
  }
  return BB15Status::TransportStateError;
}

BB15Error map_error(const bb15::internal::RuntimeError& error) {
  return make_error(map_status(error.status), error.message, error.detail);
}

bb15::internal::RuntimeModelStorage map_storage(BB15ModelStorage storage) {
  switch (storage) {
    case BB15ModelStorage::HostMemory:
      return bb15::internal::RuntimeModelStorage::HostMemory;
    case BB15ModelStorage::ExternalFlash:
      return bb15::internal::RuntimeModelStorage::ExternalFlash;
  }
  return bb15::internal::RuntimeModelStorage::HostMemory;
}

bb15::internal::RuntimeInput map_input(const BB15Input& input) {
  bb15::internal::RuntimeInput mapped;
  mapped.data = input.data;
  mapped.type = input.type;
  mapped.dimensions = input.dimensions;
  return mapped;
}

BB15ModelInfo map_model_info(const bb15::internal::RuntimeModelInfo& info,
                             size_t serialized_size) {
  BB15ModelInfo mapped;
  mapped.valid = info.valid;
  mapped.canLearn = info.canLearn;
  mapped.input.dense = info.inputIsDense;
  mapped.input.dimensions = info.inputDimensions;
  mapped.output.dense = info.outputIsDense;
  mapped.output.dimensions = info.outputDimensions;
  mapped.serializedSize = serialized_size;
  return mapped;
}

BB15RunResult map_run_result(const bb15::internal::RuntimeRunResult& result) {
  BB15RunResult mapped;
  mapped.status = map_status(result.status);
  mapped.error = map_error(result.error);
  mapped.type = result.type;
  mapped.dimensions = result.dimensions;
  mapped.bytes = result.bytes;
  mapped.predictedIndex = result.predictedIndex;
  return mapped;
}

BB15ClassificationResult map_classification_result(
    const bb15::internal::RuntimeClassificationResult& result) {
  BB15ClassificationResult mapped;
  mapped.status = map_status(result.status);
  mapped.error = map_error(result.error);
  mapped.predictedIndex = result.predictedIndex;
  mapped.scores = map_run_result(result.scores);
  return mapped;
}

}  // namespace

BB15Config BB15Config::defaults() {
  BB15Config config;
  config.spiClockHz = 25000000u;
  config.flashSpiClockHz = 2000000u;
  return config;
}

BB15Pinout BB15Pinout::niclaVisionDefaults() {
  BB15Pinout pinout;
  pinout.host.boardReset = 16u;
  pinout.host.akidaCs = 7u;
  pinout.host.ttModeCs = 3u;
  pinout.host.interrupt = 0u;
  pinout.akidaReset.route = BB15ResetRoute::Expander;
  pinout.akidaReset.pin = 4u;
  pinout.expander.bootMode = 0u;
  pinout.expander.akidaSleep = 3u;
  pinout.expander.akidaInterrupt = 2u;
  return pinout;
}

BB15Pinout BB15Pinout::niclaSenseMeDefaults() {
  BB15Pinout pinout;
  pinout.host.boardReset = 10u;
  pinout.host.akidaCs = 6u;
  pinout.host.ttModeCs = 0u;
  pinout.host.interrupt = 5u;
  pinout.akidaReset.route = BB15ResetRoute::Expander;
  pinout.akidaReset.pin = 4u;
  pinout.expander.bootMode = 0u;
  pinout.expander.akidaSleep = 3u;
  pinout.expander.akidaInterrupt = 2u;
  return pinout;
}

// Derived from the Nicla Voice pinout and the BB15 v2 edge connector sheet.
//
// The interrupt and TT_MODE lines are shared nets, also reaching NDP120 GPIO15
// and GPIO6 through the header translators, so they are safe only while the
// NDP120 firmware leaves those GPIOs alone.
BB15Pinout BB15Pinout::niclaVoiceDefaults() {
  BB15Pinout pinout;
  pinout.host.boardReset = 10u;
  pinout.host.akidaCs = 6u;
  pinout.host.ttModeCs = 0u;
  pinout.host.interrupt = 5u;
  pinout.akidaReset.route = BB15ResetRoute::Expander;
  pinout.akidaReset.pin = 4u;
  pinout.expander.bootMode = 0u;
  pinout.expander.akidaSleep = 3u;
  pinout.expander.akidaInterrupt = 2u;
  return pinout;
}

BB15Model::BB15Model(const uint8_t* data, size_t size)
    : data_(data), size_(size), external_address_(0x80000000u) {
  info_.serializedSize = size_;
}

bool BB15Model::valid() const { return data_ != nullptr && size_ > 0u; }

size_t BB15Model::size() const { return size_; }

const uint8_t* BB15Model::data() const { return data_; }

BB15ModelInfo BB15Model::info() const { return info_; }

const BB15TensorInfo& BB15Model::input() const { return info_.input; }

const BB15TensorInfo& BB15Model::output() const { return info_.output; }

BB15ModelStorage BB15Model::storage() const { return storage_; }

BB15Model& BB15Model::setStorage(BB15ModelStorage storage) {
  storage_ = storage;
  return *this;
}

uint32_t BB15Model::externalAddress() const { return external_address_; }

BB15Model& BB15Model::setExternalAddress(uint32_t address) {
  external_address_ = address;
  return *this;
}

size_t BB15RunResult::elementCount() const {
  if (dimensions.size() == 0u) {
    return 0u;
  }
  return akida::shape_size(dimensions);
}

BB15::BB15(const BB15Pinout& pinout, const BB15Config& config)
    : pinout_(pinout), config_(config) {
  last_error_ = make_error(BB15Status::Ok, "ok");
  initializeConstructorResetState();
}

BB15::~BB15() = default;

BB15Status BB15::setError(BB15Status status, const char* message,
                          uint32_t detail) {
  last_error_ = make_error(status, message, detail);
  return status;
}

bb15::internal::RuntimeOptions BB15::toRuntimeOptions() const {
  bb15::internal::RuntimeOptions options =
      bb15::internal::RuntimeOptions::niclaVisionDefaults();
  options.spiBus = config_.spi;
  options.akidaCsPin = pinout_.host.akidaCs;
  options.bridgeCsPin = pinout_.host.ttModeCs;
  options.spiClockHz = config_.spiClockHz;
  options.flashSpiClockHz = config_.flashSpiClockHz;
  options.expectedIpVersion = config_.expectedIpVersion;
  options.externalModelAddress = config_.defaultModelAddress;
  options.fetchTimeoutMs = config_.fetchTimeoutMs;
  options.fetchPollDelayMs = config_.fetchPollDelayMs;
  options.postBeginSettleMs = config_.postBeginSettleMs;
  options.postLinkSettleMs = config_.postLinkSettleMs;
  options.forcedFlashProfile = config_.forcedFlashProfile;
  options.assumeForcedFlashProfileReady = config_.assumeForcedFlashProfileReady;
  return options;
}

bool BB15::ensureLowLevelBoard() {
#if !AKD1500_PLATFORM_SUPPORTED
  setError(BB15Status::TransportStateError, "unsupported_platform_backend");
  return false;
#else
  if (low_level_board_) {
    return true;
  }

  bb15::internal::RuntimeBoardConfig config;
  config.spi_bus = config_.spi;
  config.pins.akida_cs = pinout_.host.akidaCs;
  config.pins.bridge_cs = pinout_.host.ttModeCs;
  config.spi_clock_hz = config_.spiClockHz;
  config.flash_spi_clock_hz = config_.flashSpiClockHz;
  config.external_program_data_address = config_.defaultModelAddress;
  config.expected_ip_version = config_.expectedIpVersion;
  config.forced_flash_profile = config_.forcedFlashProfile;
  config.assume_forced_flash_profile_ready =
      config_.assumeForcedFlashProfileReady;
  low_level_board_.reset(new bb15::internal::RuntimeBoard(config));
  return true;
#endif
}

uint32_t BB15::normalizeAddress(uint32_t address_or_offset) const {
  return bb15::internal::normalize_external_model_address(address_or_offset);
}

uint32_t BB15::logicalFlashOffset(uint32_t address_or_offset) const {
  if (address_or_offset < kExternalModelWindowSize) {
    return address_or_offset;
  }

  if (address_or_offset >= kExternalModelAliasBase &&
      address_or_offset <
          (kExternalModelAliasBase + kExternalModelWindowSize)) {
    return address_or_offset - kExternalModelAliasBase;
  }

  if (address_or_offset >= kExternalModelWindowBase &&
      address_or_offset <
          (kExternalModelWindowBase + kExternalModelWindowSize)) {
    return address_or_offset - kExternalModelWindowBase;
  }

  return address_or_offset;
}

bool BB15::ensureWireStarted() {
  if (wire_started_) {
    return true;
  }
  if (config_.wire == nullptr) {
    setError(BB15Status::InvalidConfig, "wire_required");
    return false;
  }
  config_.wire->begin();
  config_.wire->setClock(config_.i2cClockHz);
  wire_started_ = true;
  return true;
}

void BB15::prepareHostSpiPinsForAkidaAccess() {
  pinMode(pinout_.host.akidaCs, OUTPUT);
  digitalWrite(pinout_.host.akidaCs, HIGH);

  pinMode(pinout_.host.ttModeCs, OUTPUT);
  digitalWrite(pinout_.host.ttModeCs, HIGH);
}

void BB15::setBoardReset(bool asserted) {
  pinMode(pinout_.host.boardReset, OUTPUT);
  digitalWrite(pinout_.host.boardReset, asserted ? LOW : HIGH);
}

void BB15::initializeConstructorResetState() {
  // Release the active-low board reset before accessing its I2C expander.
  setBoardReset(false);

  if (pinout_.akidaReset.route == BB15ResetRoute::HostGpio) {
    pinMode(pinout_.akidaReset.pin, OUTPUT);
    digitalWrite(pinout_.akidaReset.pin, HIGH);
    return;
  }

  if (config_.wire == nullptr) {
    return;
  }

  config_.wire->begin();
  config_.wire->setClock(config_.i2cClockHz);
  wire_started_ = true;
  if (!expander_) {
    expander_.reset(
        new bb15::PioExpander6408(*config_.wire, config_.expanderAddress));
  }
  if (!expander_) {
    return;
  }

  (void)expander_->pinMode(pinout_.akidaReset.pin,
                           bb15::PioExpander6408::PinMode::Output);
  // Expander reset polarity is active-low at the BB15 side, so released is
  // represented by a HIGH output here.
  (void)expander_->digitalWrite(pinout_.akidaReset.pin, true);
}

bool BB15::ensureExpander() {
  if (expander_) {
    return true;
  }
  if (!ensureWireStarted()) {
    return false;
  }
  expander_.reset(
      new bb15::PioExpander6408(*config_.wire, config_.expanderAddress));
  return true;
}

bool BB15::probeExpander() {
  if (!ensureExpander()) {
    return false;
  }
  if (!expander_->probe()) {
    setError(BB15Status::ExpanderMissing, "expander_missing",
             config_.expanderAddress);
    return false;
  }
  last_error_ = make_error(BB15Status::Ok, "ok");
  return true;
}

bool BB15::configureExpanderDefaults() {
  if (!probeExpander()) {
    return false;
  }
  if (!expander_->configureBb15DefaultOutputs()) {
    setError(BB15Status::ExpanderConfigFailed, "expander_config_failed");
    return false;
  }
  expander_configured_ = true;
  last_error_ = make_error(BB15Status::Ok, "ok");
  return true;
}

bool BB15::setAkidaReset(bool asserted) {
  if (pinout_.akidaReset.route == BB15ResetRoute::HostGpio) {
    pinMode(pinout_.akidaReset.pin, OUTPUT);
    digitalWrite(pinout_.akidaReset.pin, asserted ? LOW : HIGH);
    last_error_ = make_error(BB15Status::Ok, "ok");
    return true;
  }

  if (!ensureExpander()) {
    return false;
  }
  if (!expander_->pinMode(pinout_.akidaReset.pin,
                          bb15::PioExpander6408::PinMode::Output) ||
      !expander_->digitalWrite(pinout_.akidaReset.pin, !asserted)) {
    setError(BB15Status::ExpanderConfigFailed, "akida_reset_control_failed");
    return false;
  }
  last_error_ = make_error(BB15Status::Ok, "ok");
  return true;
}

bool BB15::holdAkidaInReset() { return setAkidaReset(true); }

bool BB15::releaseAkidaReset() {
  if (!setAkidaReset(false)) {
    return false;
  }
  delay(config_.resetReleaseSettleMs);
  return true;
}

bool BB15::holdBoardInReset() {
  initialized_ = false;
  s2m_active_ = false;
  ip_version_ = 0u;
  detected_flash_jedec_ = 0u;
  detected_flash_name_ = "unknown";
  has_supported_flash_profile_ = false;
  setBoardReset(true);
  last_error_ = make_error(BB15Status::Ok, "ok");
  return true;
}

bool BB15::releaseBoardReset() {
  setBoardReset(false);
  delay(config_.resetReleaseSettleMs);
  last_error_ = make_error(BB15Status::Ok, "ok");
  return true;
}

bool BB15::powerDown() { return holdBoardInReset(); }

bool BB15::powerUp() { return releaseBoardReset(); }

bool BB15::sleep() {
  initialized_ = false;
  s2m_active_ = false;
  ip_version_ = 0u;
  if (!setAkidaSleep(true)) {
    return false;
  }
  delay(config_.sleepEnterSettleMs);
  return true;
}

bool BB15::wake() {
  initialized_ = false;
  s2m_active_ = false;
  ip_version_ = 0u;
  if (!setAkidaSleep(false)) {
    return false;
  }
  delay(config_.sleepExitSettleMs);
  return true;
}

bool BB15::setAkidaSleep(bool enabled) {
  if (!ensureExpander()) {
    return false;
  }
  if (!expander_->pinMode(pinout_.expander.akidaSleep,
                          bb15::PioExpander6408::PinMode::Output) ||
      !expander_->digitalWrite(pinout_.expander.akidaSleep, enabled)) {
    setError(BB15Status::ExpanderConfigFailed, "akida_sleep_gate_failed");
    return false;
  }
  last_error_ = make_error(BB15Status::Ok, "ok");
  return true;
}

bool BB15::enterExternalFlashBootMode() {
  if (!releaseBoardReset()) {
    return false;
  }
  if (!configureExpanderDefaults()) {
    return false;
  }

  prepareHostSpiPinsForAkidaAccess();
  setAkidaReset(true);
  delay(config_.resetAssertMs);

  if (!expander_->pinMode(pinout_.expander.bootMode,
                          bb15::PioExpander6408::PinMode::Output) ||
      !expander_->digitalWrite(pinout_.expander.bootMode, false) ||
      !setAkidaSleep(false)) {
    setError(BB15Status::ResetFailed, "akida_mode_strap_failed");
    return false;
  }

  if (!releaseAkidaReset()) {
    setError(BB15Status::ResetFailed, "akida_reset_release_failed");
    return false;
  }
  return true;
}

bool BB15::coldBootExternalFlashMode(uint32_t holdResetMs) {
  if (!configureExpanderDefaults()) {
    return false;
  }

  prepareHostSpiPinsForAkidaAccess();
  setAkidaReset(true);
  if (!expander_->pinMode(pinout_.expander.bootMode,
                          bb15::PioExpander6408::PinMode::Output) ||
      !expander_->digitalWrite(pinout_.expander.bootMode, false) ||
      !setAkidaSleep(false)) {
    setError(BB15Status::ResetFailed, "akida_mode_strap_failed");
    return false;
  }

  delay(holdResetMs);
  return releaseAkidaReset();
}

BB15Status BB15::begin() {
  if (!enterExternalFlashBootMode()) {
    return last_error_.status;
  }

#if !AKD1500_PLATFORM_SUPPORTED
  initialized_ = true;
  ip_version_ = 0u;
  last_error_ = make_error(BB15Status::Ok, "ok");
  return BB15Status::Ok;
#else
  if (!ensureLowLevelBoard()) {
    return setError(BB15Status::InvalidConfig, "board_create_failed");
  }

  low_level_board_->begin();
  ip_version_ = low_level_board_->read_ip_version();
  if (config_.expectedIpVersion != 0u &&
      ip_version_ != config_.expectedIpVersion) {
    return setError(BB15Status::LinkFailed, "unexpected_ip_version",
                    ip_version_);
  }

  initialized_ = true;
  last_error_ = make_error(BB15Status::Ok, "ok");
  return BB15Status::Ok;
#endif
}

bool BB15::detectFlash() {
#if !AKD1500_PLATFORM_SUPPORTED
  setError(BB15Status::FlashUnsupported, "unsupported_platform_flash_detect");
  return false;
#else
  if (!initialized_) {
    const BB15Status status = begin();
    if (status != BB15Status::Ok) {
      return false;
    }
  }

  if (!ensureLowLevelBoard()) {
    setError(BB15Status::InvalidConfig, "board_create_failed");
    return false;
  }

  const bool ok = low_level_board_->ensure_spi_flash_runtime_profile();
  detected_flash_jedec_ = low_level_board_->detected_flash_jedec();
  detected_flash_name_ = low_level_board_->detected_flash_name();
  has_supported_flash_profile_ =
      low_level_board_->has_supported_flash_profile();
  if (!ok) {
    setError(BB15Status::FlashUnsupported, "flash_detect_failed",
             detected_flash_jedec_);
    return false;
  }

  last_error_ = make_error(BB15Status::Ok, "ok");
  return true;
#endif
}

BB15FlashInfo BB15::flashInfo() const {
  BB15FlashInfo info;
  info.detected = detected_flash_jedec_ != 0u;
  info.jedec = detected_flash_jedec_;
  info.name = detected_flash_name_;
  info.supportedProfile = has_supported_flash_profile_;
  return info;
}

bool BB15::s2mEnter() {
#if !AKD1500_PLATFORM_SUPPORTED
  setError(BB15Status::TransportStateError, "unsupported_platform_s2m");
  s2m_active_ = false;
  return false;
#else
  if (!detectFlash()) {
    return false;
  }
  if (!ensureLowLevelBoard()) {
    setError(BB15Status::InvalidConfig, "board_create_failed");
    return false;
  }
  if (!low_level_board_->enter_s2m()) {
    setError(BB15Status::TransportStateError, "s2m_enter_failed");
    s2m_active_ = false;
    return false;
  }
  s2m_active_ = true;
  last_error_ = make_error(BB15Status::Ok, "ok");
  return true;
#endif
}

bool BB15::s2mExit() {
#if !AKD1500_PLATFORM_SUPPORTED
  setError(BB15Status::TransportStateError, "unsupported_platform_s2m");
  s2m_active_ = false;
  return false;
#else
  if (!ensureLowLevelBoard()) {
    setError(BB15Status::InvalidConfig, "board_create_failed");
    return false;
  }
  if (!low_level_board_->leave_s2m()) {
    setError(BB15Status::TransportStateError, "s2m_exit_failed");
    s2m_active_ = false;
    return false;
  }
  s2m_active_ = false;
  detected_flash_jedec_ = low_level_board_->detected_flash_jedec();
  detected_flash_name_ = low_level_board_->detected_flash_name();
  has_supported_flash_profile_ =
      low_level_board_->has_supported_flash_profile();
  last_error_ = make_error(BB15Status::Ok, "ok");
  return true;
#endif
}

bool BB15::programExternalData(const uint8_t* data, size_t size,
                               uint32_t address) {
#if !AKD1500_PLATFORM_SUPPORTED
  (void)data;
  (void)size;
  (void)address;
  setError(BB15Status::TransportStateError, "unsupported_platform_flash_write");
  return false;
#else
  if (data == nullptr || size == 0u) {
    setError(BB15Status::InvalidInput, "invalid_external_data");
    return false;
  }

  if (!detectFlash()) {
    return false;
  }
  if (!ensureLowLevelBoard()) {
    setError(BB15Status::InvalidConfig, "board_create_failed");
    return false;
  }

  const bool ok = low_level_board_->stage_program_data_to_bridge_flash(
      data, size, normalizeAddress(address));
  ip_version_ = low_level_board_->read_ip_version();
  detected_flash_jedec_ = low_level_board_->detected_flash_jedec();
  detected_flash_name_ = low_level_board_->detected_flash_name();
  has_supported_flash_profile_ =
      low_level_board_->has_supported_flash_profile();
  if (!ok) {
    setError(BB15Status::FlashStageFailed, "flash_stage_failed");
    return false;
  }
  last_error_ = make_error(BB15Status::Ok, "ok");
  return true;
#endif
}

bool BB15::verifyExternalData(const uint8_t* data, size_t size,
                              uint32_t address) {
#if !AKD1500_PLATFORM_SUPPORTED
  (void)data;
  (void)size;
  (void)address;
  setError(BB15Status::TransportStateError,
           "unsupported_platform_flash_verify");
  return false;
#else
  if (data == nullptr || size == 0u) {
    setError(BB15Status::InvalidInput, "invalid_external_data");
    return false;
  }

  if (!detectFlash()) {
    return false;
  }
  if (!ensureLowLevelBoard()) {
    setError(BB15Status::InvalidConfig, "board_create_failed");
    return false;
  }

  const bool ok = low_level_board_->verify_program_data_from_bridge_flash(
      data, size, normalizeAddress(address));
  ip_version_ = low_level_board_->read_ip_version();
  detected_flash_jedec_ = low_level_board_->detected_flash_jedec();
  detected_flash_name_ = low_level_board_->detected_flash_name();
  has_supported_flash_profile_ =
      low_level_board_->has_supported_flash_profile();
  if (!ok) {
    setError(BB15Status::FlashVerifyFailed, "flash_verify_failed");
    return false;
  }
  last_error_ = make_error(BB15Status::Ok, "ok");
  return true;
#endif
}

bool BB15::readExternalData(uint32_t address, uint8_t* out, size_t size) {
#if !AKD1500_PLATFORM_SUPPORTED
  (void)address;
  (void)out;
  (void)size;
  setError(BB15Status::TransportStateError, "unsupported_platform_flash_read");
  return false;
#else
  if (out == nullptr || size == 0u) {
    setError(BB15Status::InvalidInput, "invalid_external_read");
    return false;
  }
  if (!initialized_) {
    const BB15Status status = begin();
    if (status != BB15Status::Ok) {
      return false;
    }
  }
  if (!ensureLowLevelBoard()) {
    setError(BB15Status::InvalidConfig, "board_create_failed");
    return false;
  }

  const bool ok = low_level_board_->read_bridge_flash(
      logicalFlashOffset(address), out, size);
  if (!ok) {
    setError(BB15Status::TransportStateError, "read_external_data_failed",
             logicalFlashOffset(address));
    return false;
  }

  last_error_ = make_error(BB15Status::Ok, "ok");
  return true;
#endif
}

bool BB15::flashModel(const BB15Model& model) {
  if (!model.valid()) {
    setError(BB15Status::ModelInvalid, "invalid_model");
    return false;
  }
  return programExternalData(model.data(), model.size(),
                             model.externalAddress());
}

bool BB15::verifyModel(const BB15Model& model) {
  if (!model.valid()) {
    setError(BB15Status::ModelInvalid, "invalid_model");
    return false;
  }
  return verifyExternalData(model.data(), model.size(),
                            model.externalAddress());
}

BB15Runner BB15::createRunner() { return BB15Runner(*this); }

void BB15::printLastError(Print& out) const {
  out.print("status=");
  out.print(static_cast<int>(last_error_.status));
  out.print(" detail=");
  out.print(last_error_.detail);
  out.print(" reason=");
  out.println(last_error_.message);
}

void BB15::printSummary(Print& out) const {
  out.println("board=BB15");
  out.print("pins board_reset=");
  out.print(pinout_.host.boardReset);
  out.print(" ");
  out.print("expander=0x");
  out.println(config_.expanderAddress, HEX);
  out.print("pins akida_reset=");
  if (pinout_.akidaReset.route == BB15ResetRoute::HostGpio) {
    out.print("host:");
  } else {
    out.print("expander:");
  }
  out.print(pinout_.akidaReset.pin);
  out.print(" akida_cs=");
  out.print(pinout_.host.akidaCs);
  out.print(" tt_mode_cs=");
  out.print(pinout_.host.ttModeCs);
  out.print(" interrupt=");
  out.println(pinout_.host.interrupt);
}

BB15Runner::BB15Runner(BB15& board) : board_(&board) {
  last_error_ = make_error(BB15Status::Ok, "ok");
}

BB15Status BB15Runner::setError(BB15Status status, const char* message,
                                uint32_t detail) {
  last_error_ = make_error(status, message, detail);
  if (board_ != nullptr) {
    board_->last_error_ = last_error_;
  }
  return status;
}

BB15Status BB15Runner::begin() {
  if (board_ == nullptr) {
    return setError(BB15Status::InvalidConfig, "board_required");
  }

  if (!board_->initialized()) {
    const BB15Status status = board_->begin();
    if (status != BB15Status::Ok) {
      last_error_ = board_->lastError();
      return status;
    }
  }

  backend_.reset(new bb15::internal::RuntimeRunner(board_->toRuntimeOptions()));
  const bb15::internal::RuntimeStatus status = backend_->begin();
  board_->ip_version_ = backend_->ipVersion();
  if (status != bb15::internal::RuntimeStatus::Ok) {
    const BB15Error mapped = map_error(backend_->lastError());
    ready_ = false;
    return setError(mapped.status, mapped.message, mapped.detail);
  }

  ready_ = true;
  model_loaded_ = false;
  inference_in_flight_ = false;
  model_info_ = {};
  last_error_ = make_error(BB15Status::Ok, "ok");
  board_->last_error_ = last_error_;
  return BB15Status::Ok;
}

BB15Status BB15Runner::loadModel(const BB15Model& model) {
  if (!ready_ || backend_ == nullptr) {
    return setError(BB15Status::NotInitialized, "begin_required");
  }
  if (!model.valid()) {
    return setError(BB15Status::ModelInvalid, "invalid_model");
  }

  bb15::internal::RuntimeModel runtime_model;
  runtime_model.serializedProgram = model.data();
  runtime_model.size = model.size();
  runtime_model.storage = map_storage(model.storage());
  runtime_model.externalLocation = model.externalAddress();

  const bb15::internal::RuntimeStatus status = backend_->load(runtime_model);
  board_->ip_version_ = backend_->ipVersion();
  if (status != bb15::internal::RuntimeStatus::Ok) {
    const BB15Error mapped = map_error(backend_->lastError());
    model_loaded_ = false;
    inference_in_flight_ = false;
    model_info_ = {};
    return setError(mapped.status, mapped.message, mapped.detail);
  }

  model_loaded_ = backend_->modelLoaded();
  inference_in_flight_ = false;
  model_info_ = map_model_info(backend_->modelInfo(), model.size());
  last_error_ = make_error(BB15Status::Ok, "ok");
  board_->last_error_ = last_error_;
  return BB15Status::Ok;
}

BB15Status BB15Runner::enqueue(const BB15Input& input) {
  if (!model_loaded_ || backend_ == nullptr) {
    return setError(BB15Status::ModelNotLoaded, "model_not_loaded");
  }
  if (input.data == nullptr || input.dimensions.size() == 0u) {
    return setError(BB15Status::InvalidInput, "invalid_input");
  }

  if (!shapes_equal(input.dimensions, model_info_.input.dimensions)) {
    return setError(BB15Status::InvalidInput, "input_shape_mismatch");
  }

#if !AKD1500_PLATFORM_SUPPORTED
  return setError(BB15Status::TransportStateError,
                  "unsupported_platform_enqueue");
#else
  akida::HardwareDevice* device = backend_->hardwareDevice();
  akida::HardwareDriver* driver = backend_->hardwareDriver();
  const akida::ProgramInfo* program_info = backend_->programInfo();
  if (device == nullptr || driver == nullptr || program_info == nullptr) {
    return setError(BB15Status::TransportStateError,
                    "runtime_handles_unavailable");
  }

  auto input_tensor = akida::Dense::create_view(
      reinterpret_cast<const char*>(input.data), input.type, input.dimensions,
      akida::Dense::Layout::RowMajor);
  auto input_vector = akida::Dense::split(*input_tensor);
  if (input_vector.empty()) {
    return setError(BB15Status::InvalidInput, "no_input");
  }

  device->set_batch_size(1u, driver->akida_visible_memory() == 0u);

  std::vector<akida::TensorUniquePtr> sparse_inputs;
  const akida::Tensor* runtime_input = input_vector.front().get();
  if (!model_info_.input.dense) {
    sparse_inputs = dense_to_sparse_inputs(input_vector, *program_info);
    if (sparse_inputs.empty() || sparse_inputs.front() == nullptr) {
      return setError(BB15Status::InvalidInput, "sparse_conversion_failed");
    }
    runtime_input = sparse_inputs.front().get();
  }

  if (!device->enqueue(*runtime_input)) {
    return setError(BB15Status::EnqueueFailed, "enqueue_failed");
  }
#endif

  inference_in_flight_ = true;
  last_error_ = make_error(BB15Status::Ok, "ok");
  if (board_ != nullptr) {
    board_->last_error_ = last_error_;
  }
  return BB15Status::Ok;
}

BB15RunResult BB15Runner::fetch() {
  if (!model_loaded_ || backend_ == nullptr) {
    BB15RunResult result;
    result.status = BB15Status::ModelNotLoaded;
    result.error = make_error(BB15Status::ModelNotLoaded, "model_not_loaded");
    setError(result.status, result.error.message, result.error.detail);
    return result;
  }
  if (!inference_in_flight_) {
    BB15RunResult result;
    result.status = BB15Status::InvalidInput;
    result.error =
        make_error(BB15Status::InvalidInput, "no_inference_in_flight");
    setError(result.status, result.error.message, result.error.detail);
    return result;
  }

#if !AKD1500_PLATFORM_SUPPORTED
  BB15RunResult result;
  result.status = BB15Status::TransportStateError;
  result.error =
      make_error(BB15Status::TransportStateError, "unsupported_platform_fetch");
  setError(result.status, result.error.message, result.error.detail);
  return result;
#else
  akida::HardwareDevice* device = backend_->hardwareDevice();
  if (device == nullptr) {
    BB15RunResult result;
    result.status = BB15Status::TransportStateError;
    result.error = make_error(BB15Status::TransportStateError,
                              "runtime_handles_unavailable");
    setError(result.status, result.error.message, result.error.detail);
    return result;
  }

  akida::TensorUniquePtr output = device->fetch();
  if (!output) {
    BB15RunResult result;
    result.status = BB15Status::OutputNotReady;
    result.error = make_error(BB15Status::OutputNotReady, "output_not_ready");
    last_error_ = result.error;
    if (board_ != nullptr) {
      board_->last_error_ = last_error_;
      board_->ip_version_ = backend_->ipVersion();
    }
    return result;
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
    BB15RunResult result;
    result.status = BB15Status::OutputFormatMismatch;
    result.error =
        make_error(BB15Status::OutputFormatMismatch, "output_format_mismatch");
    inference_in_flight_ = false;
    setError(result.status, result.error.message, result.error.detail);
    return result;
  }

  BB15RunResult result = make_run_result(*dense_output);
  inference_in_flight_ = false;
  last_error_ = make_error(BB15Status::Ok, "ok");
  if (board_ != nullptr) {
    board_->last_error_ = last_error_;
    board_->ip_version_ = backend_->ipVersion();
  }
  return result;
#endif
}

BB15RunResult BB15Runner::infer(const BB15Input& input) {
  if (!model_loaded_ || backend_ == nullptr) {
    BB15RunResult result;
    result.status = BB15Status::ModelNotLoaded;
    result.error = make_error(BB15Status::ModelNotLoaded, "model_not_loaded");
    setError(result.status, result.error.message, result.error.detail);
    return result;
  }

  const bb15::internal::RuntimeRunResult runtime_result =
      backend_->infer(map_input(input));
  BB15RunResult result = map_run_result(runtime_result);
  inference_in_flight_ = false;
  last_error_ = result.error;
  if (result.ok()) {
    last_error_ = make_error(BB15Status::Ok, "ok");
  }
  if (board_ != nullptr) {
    board_->last_error_ = last_error_;
    board_->ip_version_ = backend_->ipVersion();
  }
  return result;
}

BB15RunResult BB15Runner::infer(const uint8_t* data, const akida::Shape& dims) {
  BB15Input input;
  input.data = data;
  input.type = akida::TensorType::uint8;
  input.dimensions = dims;
  return infer(input);
}

BB15ClassificationResult BB15Runner::classify(const BB15Input& input) {
  if (!model_loaded_ || backend_ == nullptr) {
    BB15ClassificationResult result;
    result.status = BB15Status::ModelNotLoaded;
    result.error = make_error(BB15Status::ModelNotLoaded, "model_not_loaded");
    setError(result.status, result.error.message, result.error.detail);
    return result;
  }

  const bb15::internal::RuntimeClassificationResult runtime_result =
      backend_->classify(map_input(input));
  BB15ClassificationResult result = map_classification_result(runtime_result);
  inference_in_flight_ = false;
  last_error_ = result.error;
  if (result.ok()) {
    last_error_ = make_error(BB15Status::Ok, "ok");
  }
  if (board_ != nullptr) {
    board_->last_error_ = last_error_;
    board_->ip_version_ = backend_->ipVersion();
  }
  return result;
}

bool BB15Runner::readRegister32(uint32_t address, uint32_t& value) {
#if !AKD1500_PLATFORM_SUPPORTED
  (void)address;
  (void)value;
  setError(BB15Status::TransportStateError,
           "unsupported_platform_register_read");
  return false;
#else
  if (!ready_ || backend_ == nullptr) {
    setError(BB15Status::NotInitialized, "begin_required");
    return false;
  }
  akida::HardwareDriver* driver = backend_->hardwareDriver();
  if (driver == nullptr) {
    setError(BB15Status::TransportStateError, "runtime_handles_unavailable");
    return false;
  }
  value = driver->read32(address);
  last_error_ = make_error(BB15Status::Ok, "ok");
  if (board_ != nullptr) {
    board_->last_error_ = last_error_;
  }
  return true;
#endif
}

bool BB15Runner::writeRegister32(uint32_t address, uint32_t value) {
#if !AKD1500_PLATFORM_SUPPORTED
  (void)address;
  (void)value;
  setError(BB15Status::TransportStateError,
           "unsupported_platform_register_write");
  return false;
#else
  if (!ready_ || backend_ == nullptr) {
    setError(BB15Status::NotInitialized, "begin_required");
    return false;
  }
  akida::HardwareDriver* driver = backend_->hardwareDriver();
  if (driver == nullptr) {
    setError(BB15Status::TransportStateError, "runtime_handles_unavailable");
    return false;
  }
  driver->write32(address, value);
  last_error_ = make_error(BB15Status::Ok, "ok");
  if (board_ != nullptr) {
    board_->last_error_ = last_error_;
  }
  return true;
#endif
}

BB15ClassificationResult BB15Runner::classify(const uint8_t* data,
                                              const akida::Shape& dims) {
  BB15Input input;
  input.data = data;
  input.type = akida::TensorType::uint8;
  input.dimensions = dims;
  return classify(input);
}

void BB15Runner::printModelInfo(Print& out) const {
  out.print("valid=");
  out.print(model_info_.valid ? "yes" : "no");
  out.print(" input_shape=[");
  for (size_t i = 0; i < model_info_.input.dimensions.size(); ++i) {
    if (i != 0u) {
      out.print(", ");
    }
    out.print(model_info_.input.dimensions[i]);
  }
  out.print("] output_shape=[");
  for (size_t i = 0; i < model_info_.output.dimensions.size(); ++i) {
    if (i != 0u) {
      out.print(", ");
    }
    out.print(model_info_.output.dimensions[i]);
  }
  out.println("]");
}
