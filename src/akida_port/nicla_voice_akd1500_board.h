#pragma once

#include <cstdint>
#include <memory>

#include <Arduino.h>
#include <SPI.h>

#include "akd500/abstract_spi_driver.h"
#include "akd500/akd1500_spi_driver.h"
#include "akida/hardware_device.h"

namespace akida_port {

struct AKD1500Pins {
  uint8_t akida_cs = 6u;
  uint8_t bridge_cs = 10u;
};

struct AKD1500BoardConfig {
  SPIClass* spi_bus = &SPI;
  AKD1500Pins pins;
  uint32_t spi_clock_hz = 2000000u;
  uint32_t flash_spi_clock_hz = 0u;
  uint32_t external_program_data_address = 0xFC000000u;
  uint32_t expected_ip_version = 0xBCA10309u;
  uint32_t visible_memory_base = 0u;
  uint32_t visible_memory_size = 0u;
  const char* forced_flash_profile = nullptr;
};

bool stage_program_data_to_bridge_flash(const AKD1500BoardConfig& config,
                                        const uint8_t* serialized_program,
                                        size_t serialized_program_size,
                                        uint32_t external_program_data_address);

bool verify_program_data_from_bridge_flash(
    const AKD1500BoardConfig& config, const uint8_t* serialized_program,
    size_t serialized_program_size, uint32_t external_program_data_address);

class ArduinoSpiDriver final : public akida::AbstractSpiDriver {
 public:
  ArduinoSpiDriver() = default;

  void configure(SPIClass* spi_bus, uint8_t akida_cs_pin,
                 uint32_t spi_clock_hz);
  uint32_t clock_hz() const { return spi_clock_hz_; }
  void set_clock_hz(uint32_t spi_clock_hz) { spi_clock_hz_ = spi_clock_hz; }
  void begin();

  void read(uint8_t* data, size_t size) override;
  void write(const uint8_t* data, size_t size) override;
  void transfer(const uint8_t* tx, uint8_t* rx, size_t size) override;
  void chip_select(uint32_t slave_id, bool active) override;

 private:
  SPIClass* spi_bus_ = &SPI;
  uint8_t akida_cs_pin_ = 6u;
  uint32_t spi_clock_hz_ = 2000000u;
  bool initialized_ = false;
  bool active_ = false;

  SPIClass& spi_bus();
  SPISettings make_spi_settings() const;
  bool begin_transfer();
  void end_transfer(bool temporary);
};

class AKD1500Board {
 public:
  explicit AKD1500Board(const AKD1500BoardConfig& config);

  void begin();
  bool ensure_spi_flash_runtime_profile();
  uint32_t read_ip_version();
  void dump_spi_master_state(const char* prefix = "[AKD1500][spim]");
  void dump_runtime_state(const char* prefix = "[AKD1500][state]");
  bool reinit_spi_flash_runtime();
  bool read_bridge_flash(uint32_t flash_offset, uint8_t* data, size_t size);
  bool stage_program_data_to_bridge_flash(const uint8_t* serialized_program,
                                          size_t serialized_program_size,
                                          uint32_t external_program_data_address);
  bool verify_program_data_from_bridge_flash(const uint8_t* serialized_program,
                                             size_t serialized_program_size,
                                             uint32_t external_program_data_address);
  uint32_t detected_flash_jedec() const { return detected_flash_jedec_; }
  const char* detected_flash_name() const { return detected_flash_name_; }
  akida::SpiFlashRuntimeConfig detected_flash_runtime_config() const {
    return detected_flash_runtime_config_;
  }
  bool has_supported_flash_profile() const {
    return detected_flash_profile_supported_;
  }

  const AKD1500BoardConfig& config() const { return config_; }
  akida::HardwareDriver& hardware_driver();

 private:
  AKD1500BoardConfig config_;
  ArduinoSpiDriver spi_driver_;
  std::unique_ptr<akida::Akd1500SpiDriver> akida_driver_;
  uint32_t detected_flash_jedec_ = 0u;
  const char* detected_flash_name_ = "unknown";
  akida::SpiFlashRuntimeConfig detected_flash_runtime_config_{};
  bool detected_flash_profile_attempted_ = false;
  bool detected_flash_profile_supported_ = false;

  void ensure_started();
};

using NiclaVoiceAkd1500Pins = AKD1500Pins;
using NiclaVoiceAkd1500Config = AKD1500BoardConfig;
using NiclaVoiceAkd1500Board = AKD1500Board;

}  // namespace akida_port
