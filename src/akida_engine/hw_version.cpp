#include "akida/hw_version.h"
#include "akida/registers_top_level.h"

#include <cstring>

namespace akida {

static inline HwVersion decode_hw_version_from_reg(uint32_t reg) {
  const auto vendor_id = static_cast<uint8_t>(get_field(reg, VENDOR_ID));
  const auto minor_rev = static_cast<uint8_t>(get_field(reg, MINOR_REV));
  const auto major_rev = static_cast<uint8_t>(get_field(reg, MAJOR_REV));
  const auto prod_id = static_cast<uint8_t>(get_field(reg, PROD_ID));
  return {vendor_id, prod_id, major_rev, minor_rev};
}

static inline bool is_plausible_akida_version(const HwVersion& v) {
  // AKD/NSoC vendor is BrainChip (0xBC). Keep product heuristic broad enough
  // to cover known variants while rejecting obviously shifted fields.
  const bool vendor_ok = (v.vendor_id == 0xBC);
  const bool product_ok =
      (v.product_id == 0x00) || (v.product_id == 0xA1) || (v.product_id == 0xA2);
  return vendor_ok && product_ok;
}

HwVersion read_hw_version(const HardwareDriver& driver) {
  HwVersion version{0, 0, 0, 0};
  // Try first to read IP revision from device
  const auto top_level_reg_offset = driver.top_level_reg();
  const auto reg = driver.read32(top_level_reg_offset + REG_IP_VERSION);
  if (reg != 0) {
    // Decode raw first.
    version = decode_hw_version_from_reg(reg);

    // Some SPI bridge configurations can shift byte lanes. If raw decode does
    // not look like an AKIDA signature, try all byte rotations and keep the
    // first plausible match.
    if (!is_plausible_akida_version(version)) {
      const uint32_t reg_ror8 = (reg >> 8) | (reg << 24);
      const uint32_t reg_ror16 = (reg >> 16) | (reg << 16);
      const uint32_t reg_ror24 = (reg >> 24) | (reg << 8);
      const HwVersion candidates[] = {
          decode_hw_version_from_reg(reg_ror8),
          decode_hw_version_from_reg(reg_ror16),
          decode_hw_version_from_reg(reg_ror24),
      };
      for (const auto& candidate : candidates) {
        if (is_plausible_akida_version(candidate)) {
          version = candidate;
          break;
        }
      }
    }

  } else {
    // Legacy device: rely instead on the information provided by the driver
    auto driver_desc = driver.desc();
    if (strstr(driver_desc, "NSoC_v2") != nullptr) {
      version = NSoC_v2;

    } else if (strstr(driver_desc, "NSoC_v1") != nullptr) {
      version = NSoC_v1;
    }
  }
  return version;
}

}  // namespace akida
