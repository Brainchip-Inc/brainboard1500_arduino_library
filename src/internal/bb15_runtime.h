#pragma once

// This header is the internal translation boundary between the public `BB15`
// API and the transplanted legacy AKD1500 runtime.
//
// Why this exists:
// - `BB15` should not directly expose the old runtime naming everywhere in its
//   own implementation
// - the legacy runtime still exists, but it should now look like an internal
//   dependency of the BB15 library rather than a peer public surface
// - future cleanup can replace or rename the legacy runtime behind this file
//   without forcing a broad search/replace across every `BB15` source file

#include "internal/legacy_akd1500/AKD1500.h"

namespace bb15 {
namespace internal {

using RuntimeStatus = AKD1500Status;
using RuntimeError = AKD1500Error;
using RuntimeOptions = AKD1500Options;
using RuntimeModelStorage = AKD1500ModelStorage;
using RuntimeModel = AKD1500Model;
using RuntimeModelInfo = AKD1500ModelInfo;
using RuntimeInput = AKD1500Input;
using RuntimeRunResult = AKD1500RunResult;
using RuntimeClassificationResult = AKD1500ClassificationResult;
using RuntimeRunner = AkidaNicla;

#if AKD1500_PLATFORM_SUPPORTED
using RuntimeBoard = akida_port::AKD1500Board;
using RuntimeBoardConfig = akida_port::AKD1500BoardConfig;
#endif

inline uint32_t normalize_external_model_address(uint32_t address_or_offset) {
  return RuntimeRunner::normalizeExternalModelAddress(address_or_offset);
}

inline uint32_t external_model_address_from_offset(uint32_t offset) {
  return RuntimeRunner::externalModelAddressFromOffset(offset);
}

inline bool stage_model_to_flash(const RuntimeOptions& options,
                                 const uint8_t* data, size_t size,
                                 uint32_t address) {
  return RuntimeRunner::stageModelToFlash(options, data, size, address);
}

inline bool verify_model_in_flash(const RuntimeOptions& options,
                                  const uint8_t* data, size_t size,
                                  uint32_t address) {
  return RuntimeRunner::verifyModelInFlash(options, data, size, address);
}

}  // namespace internal
}  // namespace bb15
