#pragma once

#include <cstring>
#include <cstdint>
#include <memory>

#include <AKD1500.h>

#include "akd500/akd1500_spi_driver.h"
#include "akida_port/nicla_voice_akd1500_board.h"

namespace akd1500_fast_flash {

enum class FlashProfilePolicy {
  Auto = 0,
  Winbond6B,
  WinbondEB,
  Renesas6B,
};

struct Config {
  uint32_t akidaSpiClockHz = 25000000u;
  uint32_t flashSpiClockHz = 2000000u;
  uint32_t externalModelOffset = 0u;
  uint32_t expectedIpVersion = 0xBCA10309u;
  FlashProfilePolicy flashProfilePolicy = FlashProfilePolicy::Auto;
  bool verboseRuntimeDiagnostics = true;
};

struct ProbeSummary {
  bool attempted = false;
  bool ok = false;
  bool runtimeProfileSupported = false;
  bool forcedProfile = false;
  uint32_t elapsedMs = 0u;
  uint32_t detectedJedec = 0u;
  const char* policyName = "auto";
  const char* forcedProfileName = "auto";
  const char* detectedFlashName = "unknown";
  const char* laneModeInferred = "unknown";
  akida::SpiFlashRuntimeConfig runtimeConfig = {};
};

struct LoadSummary {
  bool ok = false;
  uint32_t runnerBeginMs = 0u;
  uint32_t modelLoadMs = 0u;
  uint32_t totalMs = 0u;
  uint32_t ipVersion = 0u;
};

struct SessionSummary {
  uint32_t requestedFlashSpiClockHz = 0u;
  ProbeSummary probe = {};
  LoadSummary load = {};
};

inline const char* flash_profile_policy_name(FlashProfilePolicy policy) {
  switch (policy) {
    case FlashProfilePolicy::Auto:
      return "auto";
    case FlashProfilePolicy::Winbond6B:
      return "winbond_6b";
    case FlashProfilePolicy::WinbondEB:
      return "winbond_eb";
    case FlashProfilePolicy::Renesas6B:
      return "renesas_6b";
  }
  return "unknown";
}

inline const char* forced_flash_profile_name(FlashProfilePolicy policy) {
  switch (policy) {
    case FlashProfilePolicy::Auto:
      return nullptr;
    case FlashProfilePolicy::Winbond6B:
      return "winbond";
    case FlashProfilePolicy::WinbondEB:
      return "winbond_eb";
    case FlashProfilePolicy::Renesas6B:
      return "renesas";
  }
  return nullptr;
}

inline const char* inferred_lane_mode(
    const akida::SpiFlashRuntimeConfig& runtime_config) {
  if (runtime_config.read_opcode == 0x6Bu &&
      runtime_config.transfer_type == 0u) {
    return "1-1-4_quad_output";
  }
  if (runtime_config.read_opcode == 0xEBu &&
      runtime_config.transfer_type == 1u) {
    return "1-4-4_quad_io";
  }
  return "unknown";
}

inline void print_jedec(Print& out, uint32_t jedec) {
  const uint8_t manufacturer_id = static_cast<uint8_t>((jedec >> 16) & 0xFFu);
  const uint8_t memory_type = static_cast<uint8_t>((jedec >> 8) & 0xFFu);
  const uint8_t capacity_id = static_cast<uint8_t>(jedec & 0xFFu);
  if (manufacturer_id < 0x10u) {
    out.print('0');
  }
  out.print(manufacturer_id, HEX);
  out.print(':');
  if (memory_type < 0x10u) {
    out.print('0');
  }
  out.print(memory_type, HEX);
  out.print(':');
  if (capacity_id < 0x10u) {
    out.print('0');
  }
  out.print(capacity_id, HEX);
}

inline AKD1500Options make_options(const Config& config) {
  AKD1500Options options = AKD1500Options::niclaVisionDefaults();
  options.spiClockHz = config.akidaSpiClockHz;
  options.flashSpiClockHz = config.flashSpiClockHz;
  options.expectedIpVersion = config.expectedIpVersion;
  options.postBeginSettleMs = 10u;
  options.postLinkSettleMs = 10u;
  options.externalModelAddress =
      AkidaNicla::externalModelAddressFromOffset(config.externalModelOffset);
  options.forcedFlashProfile =
      forced_flash_profile_name(config.flashProfilePolicy);
  return options;
}

inline akida_port::AKD1500BoardConfig make_board_config(
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

inline bool shapes_equal(const akida::Shape& lhs, const akida::Shape& rhs) {
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

class FastFlashModelRunner {
 public:
  explicit FastFlashModelRunner(const Config& config)
      : config_(config), options_(make_options(config)) {
    last_error_ = {};
  }

  const Config& config() const { return config_; }
  const AKD1500Options& options() const { return options_; }
  const AKD1500Error& lastError() const { return last_error_; }
  uint32_t ipVersion() const { return ip_version_; }
  bool modelLoaded() const { return model_info_.valid; }
  AKD1500ModelInfo modelInfo() const { return model_info_; }
  const SessionSummary& sessionSummary() const { return session_summary_; }

  AKD1500Status load(const AKD1500Model& model) {
    refresh_options_for_load();
    session_summary_ = {};
    session_summary_.requestedFlashSpiClockHz = options_.flashSpiClockHz;
    model_info_ = {};
    last_error_ = {};
    ip_version_ = 0u;
    runner_.reset();

    if (config_.verboseRuntimeDiagnostics) {
      probe_runtime();
    }

    if (model.serializedProgram == nullptr || model.size == 0u ||
        model.storage != AKD1500ModelStorage::ExternalFlash) {
      last_error_.status = AKD1500Status::InvalidInput;
      last_error_.message = "invalid_external_flash_model";
      return last_error_.status;
    }

    const uint32_t total_start_ms = millis();
    runner_.reset(new AKD1500FlashRunner(options_));

    const uint32_t begin_start_ms = millis();
    const AKD1500Status begin_status = runner_->begin();
    session_summary_.load.runnerBeginMs = millis() - begin_start_ms;
    ip_version_ = runner_->ipVersion();
    last_error_ = runner_->lastError();
    if (begin_status != AKD1500Status::Ok) {
      session_summary_.load.totalMs = millis() - total_start_ms;
      session_summary_.load.ipVersion = ip_version_;
      return begin_status;
    }

    const uint32_t load_start_ms = millis();
    const AKD1500Status load_status =
        runner_->loadExternalModel(model.serializedProgram, model.size);
    session_summary_.load.modelLoadMs = millis() - load_start_ms;
    session_summary_.load.totalMs = millis() - total_start_ms;
    session_summary_.load.ipVersion = runner_->ipVersion();
    ip_version_ = runner_->ipVersion();
    last_error_ = runner_->lastError();
    session_summary_.load.ok = load_status == AKD1500Status::Ok;
    if (load_status != AKD1500Status::Ok) {
      return load_status;
    }

    model_info_.valid = runner_->modelLoaded();
    model_info_.inputIsDense = runner_->inputIsDense();
    model_info_.outputIsDense = runner_->outputIsDense();
    model_info_.canLearn = runner_->canLearn();
    model_info_.inputDimensions = runner_->inputDimensions();
    model_info_.outputDimensions = runner_->outputDimensions();
    remember_detected_flash_profile();
    last_error_ = {};
    last_error_.status = AKD1500Status::Ok;
    last_error_.message = "ok";
    return AKD1500Status::Ok;
  }

  AKD1500ClassificationResult classifyUint8(const uint8_t* input_data) {
    if (!model_info_.valid || model_info_.inputDimensions.size() == 0u) {
      return make_failed_classification(AKD1500Status::ModelNotLoaded,
                                        "model_not_loaded");
    }
    return classifyUint8(input_data, model_info_.inputDimensions);
  }

  AKD1500ClassificationResult classifyUint8(const uint8_t* input_data,
                                            const akida::Shape& dimensions) {
    if (input_data == nullptr || dimensions.size() == 0u) {
      return make_failed_classification(AKD1500Status::InvalidInput,
                                        "invalid_input");
    }
    if (!runner_ || !model_info_.valid) {
      return make_failed_classification(AKD1500Status::ModelNotLoaded,
                                        "model_not_loaded");
    }
    if (!input_matches_model(dimensions)) {
      return make_failed_classification(AKD1500Status::InvalidInput,
                                        "input_shape_mismatch");
    }

    const AKD1500RunResult run_result = runner_->run(input_data, dimensions);
    ip_version_ = runner_->ipVersion();
    last_error_ = run_result.ok() ? AKD1500Error{} : run_result.error;
    if (run_result.ok()) {
      last_error_.status = AKD1500Status::Ok;
      last_error_.message = "ok";
    }
    return make_classification_result(run_result);
  }

  void printLastError(Print& out) const {
    out.print("status=");
    out.print(AkidaNicla::statusName(last_error_.status));
    out.print(" (");
    out.print(static_cast<int>(last_error_.status));
    out.print(")");
    out.print(" detail=");
    out.print(last_error_.detail);
    out.print(" reason=");
    out.println(last_error_.message);
  }

  void printModelInfo(Print& out) const {
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

  void printSessionSummary(Print& out, const char* prefix) const {
    out.print(prefix);
    out.print(" flash_policy policy=");
    out.print(flash_profile_policy_name(config_.flashProfilePolicy));
    out.print(" forced_profile=");
    out.print(forced_flash_profile_name(config_.flashProfilePolicy) != nullptr
                  ? forced_flash_profile_name(config_.flashProfilePolicy)
                  : "auto");
    out.print(" requested_flash_hz=");
    out.println(static_cast<unsigned long>(options_.flashSpiClockHz));

    out.print(prefix);
    out.print(" flash_probe result=");
    if (!session_summary_.probe.attempted) {
      out.println("SKIP");
    } else if (!session_summary_.probe.ok) {
      out.print("FAIL elapsed_ms=");
      out.print(static_cast<unsigned long>(session_summary_.probe.elapsedMs));
      out.print(" profile_supported=");
      out.println(session_summary_.probe.runtimeProfileSupported ? "yes"
                                                                 : "no");
    } else {
      out.print("PASS elapsed_ms=");
      out.print(static_cast<unsigned long>(session_summary_.probe.elapsedMs));
      out.print(" jedec=");
      print_jedec(out, session_summary_.probe.detectedJedec);
      out.print(" name=");
      out.print(session_summary_.probe.detectedFlashName);
      out.print(" read=0x");
      out.print(session_summary_.probe.runtimeConfig.read_opcode, HEX);
      out.print(" trans=");
      out.print(
          static_cast<unsigned long>(session_summary_.probe.runtimeConfig.transfer_type));
      out.print(" wait=");
      out.print(
          static_cast<unsigned long>(session_summary_.probe.runtimeConfig.wait_cycles));
      out.print(" mode_en=");
      out.print(session_summary_.probe.runtimeConfig.mode_bits_enabled ? 1u
                                                                       : 0u);
      out.print(" mode=0x");
      if (session_summary_.probe.runtimeConfig.mode_bits_value < 0x10u) {
        out.print('0');
      }
      out.print(session_summary_.probe.runtimeConfig.mode_bits_value, HEX);
      out.print(" lane_mode_inferred=");
      out.println(session_summary_.probe.laneModeInferred);
    }

    out.print(prefix);
    out.print(" flash_load_timing runner_begin_ms=");
    out.print(static_cast<unsigned long>(session_summary_.load.runnerBeginMs));
    out.print(" model_load_ms=");
    out.print(static_cast<unsigned long>(session_summary_.load.modelLoadMs));
    out.print(" total_ms=");
    out.print(static_cast<unsigned long>(session_summary_.load.totalMs));
    out.print(" ip_version=0x");
    out.println(session_summary_.load.ipVersion, HEX);
  }

 private:
  void refresh_options_for_load() {
    options_ = make_options(config_);
    if (cached_forced_flash_profile_ != nullptr) {
      options_.forcedFlashProfile = cached_forced_flash_profile_;
      options_.assumeForcedFlashProfileReady = true;
    }
  }

  void remember_detected_flash_profile() {
    if (!runner_) {
      return;
    }
    const char* detected_name = runner_->detectedFlashName();
    if (detected_name == nullptr || detected_name[0] == '\0' ||
        std::strcmp(detected_name, "unknown") == 0 ||
        std::strcmp(detected_name, "unsupported") == 0 ||
        std::strcmp(detected_name, "jedec_read_failed") == 0) {
      return;
    }
    cached_forced_flash_profile_ = detected_name;
  }

  static AKD1500ClassificationResult make_classification_result(
      const AKD1500RunResult& run_result) {
    AKD1500ClassificationResult result;
    result.status = run_result.status;
    result.error = run_result.error;
    result.predictedIndex = run_result.predictedIndex;
    result.scores = run_result;
    return result;
  }

  AKD1500ClassificationResult make_failed_classification(
      AKD1500Status status, const char* message) {
    last_error_.status = status;
    last_error_.detail = 0u;
    last_error_.message = message;
    AKD1500ClassificationResult result;
    result.status = status;
    result.error = last_error_;
    return result;
  }

  bool input_matches_model(const akida::Shape& input_dimensions) const {
    if (!model_info_.valid || model_info_.inputDimensions.size() == 0u) {
      return true;
    }
    if (shapes_equal(input_dimensions, model_info_.inputDimensions)) {
      return true;
    }
    if (model_info_.inputDimensions.size() == 4u &&
        input_dimensions.size() == 3u &&
        model_info_.inputDimensions[0] == 1u) {
      return input_dimensions[0] == model_info_.inputDimensions[1] &&
             input_dimensions[1] == model_info_.inputDimensions[2] &&
             input_dimensions[2] == model_info_.inputDimensions[3];
    }
    return false;
  }

  void probe_runtime() {
    session_summary_.probe = {};
    session_summary_.probe.attempted = true;
    session_summary_.probe.policyName =
        flash_profile_policy_name(config_.flashProfilePolicy);
    session_summary_.probe.forcedProfileName =
        forced_flash_profile_name(config_.flashProfilePolicy) != nullptr
            ? forced_flash_profile_name(config_.flashProfilePolicy)
            : "auto";
    session_summary_.probe.forcedProfile =
        config_.flashProfilePolicy != FlashProfilePolicy::Auto;

    const uint32_t start_ms = millis();
    akida_port::AKD1500Board board(make_board_config(options_));
    session_summary_.probe.ok = board.ensure_spi_flash_runtime_profile();
    session_summary_.probe.elapsedMs = millis() - start_ms;
    session_summary_.probe.detectedJedec = board.detected_flash_jedec();
    session_summary_.probe.detectedFlashName = board.detected_flash_name();
    session_summary_.probe.runtimeConfig = board.detected_flash_runtime_config();
    session_summary_.probe.runtimeProfileSupported =
        board.has_supported_flash_profile();
    session_summary_.probe.laneModeInferred =
        inferred_lane_mode(session_summary_.probe.runtimeConfig);
  }

  Config config_;
  AKD1500Options options_;
  const char* cached_forced_flash_profile_ = nullptr;
  std::unique_ptr<AKD1500FlashRunner> runner_;
  AKD1500ModelInfo model_info_ = {};
  AKD1500Error last_error_ = {};
  uint32_t ip_version_ = 0u;
  SessionSummary session_summary_ = {};
};

}  // namespace akd1500_fast_flash
