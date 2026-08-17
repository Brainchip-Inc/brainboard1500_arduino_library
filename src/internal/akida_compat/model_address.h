#pragma once

#include <cstdint>

namespace akida {
namespace compat {

constexpr uint32_t kExternalModelAliasBase = 0x80000000u;
constexpr uint32_t kExternalModelWindowBase = 0xFC000000u;
constexpr uint32_t kExternalModelWindowSize = 0x00800000u;

inline uint32_t normalize_external_model_address(uint32_t address_or_offset) {
  if (address_or_offset >= kExternalModelAliasBase &&
      address_or_offset <
          (kExternalModelAliasBase + kExternalModelWindowSize)) {
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

inline uint32_t external_model_address_from_offset(uint32_t offset) {
  return normalize_external_model_address(offset);
}

}  // namespace compat
}  // namespace akida
