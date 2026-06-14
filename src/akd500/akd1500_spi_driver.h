#pragma once

#include <cstddef>
#include <cstdint>

#include "abstract_spi_driver.h"

#include "akd500/memory_mapping.h"
#include "infra/hardware_driver.h"

namespace akida {

struct SpiFlashRuntimeConfig {
  uint8_t read_opcode = 0x6Bu;
  uint8_t transfer_type = 0x0u;
  uint8_t wait_cycles = 0x8u;
  bool mode_bits_enabled = false;
  uint8_t mode_bits_value = 0x00u;
};

class Akd1500SpiDriver : public HardwareDriver {
 public:
  explicit Akd1500SpiDriver(AbstractSpiDriver* spi_driver,
                            uint32_t akida_visible_memory_base,
                            uint32_t akida_visible_memory_size);
  const char* desc() const override { return "SPI/AKD1500"; }

  uint32_t scratch_memory() const override {
    static constexpr uint32_t kSpiMemoryBase = 0xfc800000;
    return kSpiMemoryBase;
  }

  uint32_t scratch_size() const override {
    return soc::akd500::kMainMemorySize;
  }

  uint32_t top_level_reg() const override {
    return soc::akd500::kTopLevelRegBase;
  }

  uint32_t akida_visible_memory() const override {
    return akida_visible_memory_base_;
  }

  uint32_t akida_visible_memory_size() const override {
    return akida_visible_memory_size_;
  }

  void set_spi_flash_runtime_config(const SpiFlashRuntimeConfig& config);
  void reinit_spi_flash_runtime();

  void read(uint32_t address, void* data, size_t size) const override;
  void write(uint32_t address, const void* data, size_t size) override;

 protected:
  AbstractSpiDriver* spi_driver_;
  uint32_t akida_visible_memory_base_;
  uint32_t akida_visible_memory_size_;
  SpiFlashRuntimeConfig flash_runtime_config_;
};

}  // namespace akida
