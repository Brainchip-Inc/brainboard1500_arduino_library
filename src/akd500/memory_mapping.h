#pragma once

#include <cstdint>

namespace akida {
namespace soc {
namespace akd500 {
// NSoC top level address
constexpr uint32_t kTopLevelRegBase = 0xFCC00000;
// registers region size
constexpr uint32_t kRegistersRegionSize = 1 * 1024 * 1024;

// Main memory offset in AKD500
constexpr uint32_t kMainMemoryBase = 0x20000000;
// Main memory size is 1MB
constexpr uint32_t kMainMemorySize = 1 * 1024 * 1024;

}  // namespace akd500

}  // namespace soc

}  // namespace akida
