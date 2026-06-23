#include "akida_port/nicla_voice_akd1500_board.h"

#include <array>
#include <algorithm>
#include <cstdio>
#include <cstring>
#include <memory>
#include <vector>

#if __has_include(<SPIF/SPIFBlockDevice.h>)
#include <SPIF/SPIFBlockDevice.h>
#define AKD1500_HAS_BRIDGE_FLASH 1
#else
#define AKD1500_HAS_BRIDGE_FLASH 0
#endif

#include "flatbuffers/base.h"
#include "akida_engine/registers_dma_engine.h"
#include "akida/registers_top_level.h"

namespace akida_port {

#ifndef AKD1500_LIBRARY_ENABLE_LOGS
#define AKD1500_LIBRARY_ENABLE_LOGS 0
#endif

#if AKD1500_LIBRARY_ENABLE_LOGS
#define AKD1500_LIBRARY_LOG(...) std::printf(__VA_ARGS__)
#else
#define AKD1500_LIBRARY_LOG(...) ((void)0)
#endif

namespace {

constexpr uint32_t kFlashWindowBase = 0xFC000000u;
constexpr uint32_t kFlashAliasBase = 0x80000000u;
constexpr uint32_t kFlashWindowSize = 0x00800000u;
constexpr size_t kSpiChunkSize = 1024u;
constexpr uint32_t kSysCfgBase = 0xFCE00000u;
constexpr uint32_t kControlSignalsReg = kSysCfgBase + 0x18u;
constexpr uint32_t kSpiMCfgBase = 0xFCF20000u;
constexpr uint32_t kSpiMCtrlr0 = kSpiMCfgBase + 0x00u;
constexpr uint32_t kSpiMSsienr = kSpiMCfgBase + 0x08u;
constexpr uint32_t kSpiMSer = kSpiMCfgBase + 0x10u;
constexpr uint32_t kSpiMBaudr = kSpiMCfgBase + 0x14u;
constexpr uint32_t kSpiMSpiCtrlr0 = kSpiMCfgBase + 0xF4u;
constexpr uint32_t kSpiMDdrDriveEdge = kSpiMCfgBase + 0xF8u;
constexpr uint32_t kSpiMXipModeBits = kSpiMCfgBase + 0xFCu;
constexpr uint32_t kSpiMXipIncrInst = kSpiMCfgBase + 0x100u;

static inline void dump_spim_snapshot(akida::HardwareDriver& driver,
                                      const char* where) {
  const auto top = driver.top_level_reg();
  std::printf(
      "[AKIDA][SPIM_SNAP] platform=nicla where=%s ctrl_signals=0x%08lX general_control=0x%08lX spi_m.ctrlr0=0x%08lX spi_m.ssienr=0x%08lX spi_m.ser=0x%08lX spi_m.baudr=0x%08lX spi_m.spi_ctrlr0=0x%08lX spi_m.ddr_drive_edge=0x%08lX spi_m.xip_mode_bits=0x%08lX spi_m.xip_incr_inst=0x%08lX\r\n",
      where ? where : "unknown",
      static_cast<unsigned long>(driver.read32(kControlSignalsReg)),
      static_cast<unsigned long>(driver.read32(top + akida::REG_GENERAL_CONTROL)),
      static_cast<unsigned long>(driver.read32(kSpiMCtrlr0)),
      static_cast<unsigned long>(driver.read32(kSpiMSsienr)),
      static_cast<unsigned long>(driver.read32(kSpiMSer)),
      static_cast<unsigned long>(driver.read32(kSpiMBaudr)),
      static_cast<unsigned long>(driver.read32(kSpiMSpiCtrlr0)),
      static_cast<unsigned long>(driver.read32(kSpiMDdrDriveEdge)),
      static_cast<unsigned long>(driver.read32(kSpiMXipModeBits)),
      static_cast<unsigned long>(driver.read32(kSpiMXipIncrInst)));
}
constexpr uint32_t kCtrlEnSpiS2mMask = (1u << 16);
// The original 250 ms guard band is far larger than the observed settle time
// needed on the Nicla Vision + BB15 path and dominates cold-start latency.
// Keep a conservative non-zero pause while removing the extra ~1 s of startup
// overhead paid across repeated S2M enter/leave transitions.
constexpr uint32_t kS2mPhaseGapMs = 25u;
constexpr uint8_t kFlashCmdResetEnable = 0x66u;
constexpr uint8_t kFlashCmdResetMemory = 0x99u;
constexpr uint8_t kFlashCmdReleasePowerDown = 0xABu;
constexpr uint8_t kFlashCmdReadJedecId = 0x9Fu;
constexpr uint8_t kFlashCmdWriteEnable = 0x06u;
constexpr uint8_t kFlashCmdWriteVolatileCfg = 0x81u;
constexpr uint8_t kFlashCmdReadStatus = 0x05u;
constexpr uint8_t kFlashCmdReadStatus2 = 0x35u;
constexpr uint8_t kFlashCmdReadData = 0x03u;
constexpr uint8_t kFlashCmdWriteStatus2 = 0x31u;
constexpr uint8_t kFlashCmdPageProgram = 0x02u;
constexpr uint8_t kFlashCmdSectorErase4K = 0x20u;
constexpr uint8_t kFlashStatus2QuadEnable = 0x02u;
constexpr uint32_t kFlashSectorSize = 4096u;
constexpr size_t kFlashPageSize = 256u;
constexpr size_t kFlashVerifyChunkSize = 64u;
constexpr bool kEnableSpiTrace = false;
constexpr size_t kSpiTracePreviewBytes = 32u;
constexpr size_t kSpiTransactionPreviewBytes = 8u;
uint32_t g_spi_trace_sequence = 0u;

struct FlashProfile {
  const char* name;
  uint8_t manufacturer_id;
  uint8_t memory_type;
  uint8_t capacity_id;
  uint32_t capacity_bytes;
  akida::SpiFlashRuntimeConfig runtime_config;
  uint8_t qe_read_cmd;
  uint8_t qe_write_cmd;
  uint8_t qe_mask;
};

constexpr uint32_t pack_jedec(uint8_t manufacturer_id, uint8_t memory_type,
                              uint8_t capacity_id) {
  return (static_cast<uint32_t>(manufacturer_id) << 16) |
         (static_cast<uint32_t>(memory_type) << 8) |
         static_cast<uint32_t>(capacity_id);
}

constexpr uint32_t kWinbondW25Q64JWCapacityBytes = 0x00800000u;
constexpr uint32_t kRenesasAt25Sl321CapacityBytes = 0x00400000u;

constexpr FlashProfile kWinbondW25Q64JWProfile = {
    "winbond_w25q64jw",
    0xEFu,
    0x60u,
    0x17u,
    kWinbondW25Q64JWCapacityBytes,
    akida::SpiFlashRuntimeConfig{0x6Bu, 0x0u, 0x8u, false, 0x00u},
    kFlashCmdReadStatus2,
    kFlashCmdWriteStatus2,
    kFlashStatus2QuadEnable,
};

constexpr FlashProfile kRenesasAt25Sl321Profile = {
    "renesas_at25sl321",
    0x1Fu,
    0x42u,
    0x16u,
    kRenesasAt25Sl321CapacityBytes,
    akida::SpiFlashRuntimeConfig{0x6Bu, 0x0u, 0x8u, false, 0x00u},
    kFlashCmdReadStatus2,
    kFlashCmdWriteStatus2,
    kFlashStatus2QuadEnable,
};

const FlashProfile kFlashProfiles[] = {
    kWinbondW25Q64JWProfile,
    kRenesasAt25Sl321Profile,
};

const FlashProfile* winbond_w25q64jw_eb_profile() {
  static const FlashProfile profile = []() {
    FlashProfile variant = kWinbondW25Q64JWProfile;
    variant.name = "winbond_w25q64jw_eb";
    variant.runtime_config =
        akida::SpiFlashRuntimeConfig{0xEBu, 0x1u, 0xAu, false, 0x00u};
    return variant;
  }();
  return &profile;
}

void spi_delay_us(uint32_t delay_us) {
  volatile uint32_t cycles = delay_us * 100u;
  while (cycles-- != 0u) {
    __asm__ volatile("nop");
  }
}

void dump_spi_trace_bytes(const char* label, const uint8_t* data, size_t size) {
  if (!kEnableSpiTrace) {
    return;
  }
  std::printf("[AKIDA][SPI] ctx=%s %s len=%lu data=",
              akida::spi_trace_context(),
              label,
              static_cast<unsigned long>(size));
  const size_t preview = (size < kSpiTracePreviewBytes) ? size : kSpiTracePreviewBytes;
  for (size_t i = 0; i < preview; ++i) {
    std::printf("%s%02X", (i == 0u) ? "" : " ",
                static_cast<unsigned>(data[i]));
  }
  if (preview < size) {
    std::printf(" ...( +%lu bytes )",
                static_cast<unsigned long>(size - preview));
  }
  std::printf("\r\n");
}

void trace_spi_cs(bool active) {
  if (!kEnableSpiTrace) {
    return;
  }
  std::printf("[AKIDA][SPI] ctx=%s cs=%s\r\n",
              akida::spi_trace_context(),
              active ? "assert" : "deassert");
}

uint32_t crc32_ieee(const uint8_t* data, size_t size) {
  uint32_t crc = 0xFFFFFFFFu;
  for (size_t i = 0; i < size; ++i) {
    crc ^= static_cast<uint32_t>(data[i]);
    for (uint32_t bit = 0; bit < 8u; ++bit) {
      const uint32_t mask = 0u - (crc & 1u);
      crc = (crc >> 1u) ^ (0xEDB88320u & mask);
    }
  }
  return ~crc;
}

void print_preview_hex(const uint8_t* data, size_t size) {
  if (data == nullptr) {
    std::printf("-");
    return;
  }
  const size_t preview =
      (size < kSpiTransactionPreviewBytes) ? size : kSpiTransactionPreviewBytes;
  for (size_t i = 0; i < preview; ++i) {
    std::printf("%02X", static_cast<unsigned>(data[i]));
  }
  if (preview < size) {
    std::printf("...");
  }
}

void trace_spi_transaction(const char* op, size_t size,
                           const uint8_t* tx, const uint8_t* rx) {
#if AKIDA_NICLA_SPI_TRANSACTION_TRACE
  const unsigned long seq = static_cast<unsigned long>(++g_spi_trace_sequence);
  std::printf("[AKIDA][SPI_TRACE] platform=nicla seq=%lu ctx=%s op=%s len=%lu",
              seq,
              akida::spi_trace_context(),
              op,
              static_cast<unsigned long>(size));
  if (tx != nullptr) {
    std::printf(" tx_crc=0x%08lX tx=",
                static_cast<unsigned long>(crc32_ieee(tx, size)));
    print_preview_hex(tx, size);
  } else {
    std::printf(" tx_crc=- tx=-");
  }
  if (rx != nullptr) {
    std::printf(" rx_crc=0x%08lX rx=",
                static_cast<unsigned long>(crc32_ieee(rx, size)));
    print_preview_hex(rx, size);
  } else {
    std::printf(" rx_crc=- rx=-");
  }
  std::printf("\r\n");
#else
  (void)op;
  (void)size;
  (void)tx;
  (void)rx;
#endif
}

void trace_spi_cs_transaction(uint32_t slave_id, bool active) {
#if AKIDA_NICLA_SPI_TRANSACTION_TRACE
  const unsigned long seq = static_cast<unsigned long>(++g_spi_trace_sequence);
  std::printf("[AKIDA][SPI_TRACE] platform=nicla seq=%lu ctx=%s op=cs slave=%lu active=%u\r\n",
              seq,
              akida::spi_trace_context(),
              static_cast<unsigned long>(slave_id),
              active ? 1u : 0u);
#else
  (void)slave_id;
  (void)active;
#endif
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

size_t round_up_to(size_t value, size_t granularity) {
  if (granularity == 0u) {
    return value;
  }
  return ((value + granularity - 1u) / granularity) * granularity;
}

size_t logical_flash_offset(uint32_t external_program_data_address) {
  if (external_program_data_address < kFlashWindowSize) {
    return static_cast<size_t>(external_program_data_address);
  }
  if (external_program_data_address >= kFlashAliasBase &&
      external_program_data_address < (kFlashAliasBase + kFlashWindowSize)) {
    return static_cast<size_t>(external_program_data_address - kFlashAliasBase);
  }
  if (external_program_data_address >= kFlashWindowBase &&
      external_program_data_address < (kFlashWindowBase + kFlashWindowSize)) {
    return static_cast<size_t>(external_program_data_address - kFlashWindowBase);
  }
  return static_cast<size_t>(-1);
}

#if AKD1500_HAS_BRIDGE_FLASH
SPIFBlockDevice& bridge_flash() {
  static SPIFBlockDevice flash;
  return flash;
}

bool ensure_flash_ready() {
  static bool initialized = false;
  if (initialized) {
    return true;
  }

  const int status = bridge_flash().init();
  if (status != 0) {
    AKD1500_LIBRARY_LOG("[AKD1500][flash] init failed status=%d\r\n", status);
    return false;
  }

  initialized = true;
  return true;
}

bool write_flash_payload(const uint8_t* payload, size_t payload_size,
                         uint32_t external_program_data_address) {
  if (payload == nullptr || payload_size == 0u) {
    return false;
  }

  if (!ensure_flash_ready()) {
    return false;
  }

  const size_t offset = logical_flash_offset(external_program_data_address);
  if (offset == static_cast<size_t>(-1)) {
    AKD1500_LIBRARY_LOG("[AKD1500][flash] invalid logical address 0x%08lX\r\n",
                        static_cast<unsigned long>(external_program_data_address));
    return false;
  }
  const size_t erase_size = bridge_flash().get_erase_size(offset);
  const size_t program_size = bridge_flash().get_program_size();
  if (erase_size == 0u || program_size == 0u) {
    AKD1500_LIBRARY_LOG("[AKD1500][flash] invalid erase/program size\r\n");
    return false;
  }
  const size_t erase_length = round_up_to(payload_size, erase_size);

  if (bridge_flash().erase(offset, erase_length) != 0) {
    AKD1500_LIBRARY_LOG(
        "[AKD1500][flash] erase failed offset=0x%08lX size=%lu\r\n",
        static_cast<unsigned long>(offset),
        static_cast<unsigned long>(erase_length));
    return false;
  }

  std::vector<uint8_t> chunk(program_size, 0xFFu);
  for (size_t written = 0u; written < erase_length; written += program_size) {
    const size_t copy_size =
        ((written + program_size) <= payload_size) ? program_size
                                                   : (payload_size - written);
    std::fill(chunk.begin(), chunk.end(), 0xFFu);
    if (written < payload_size && copy_size > 0u) {
      std::memcpy(chunk.data(), payload + written, copy_size);
    }

    const int status = bridge_flash().program(
        chunk.data(), offset + written, static_cast<mbed::bd_size_t>(program_size));
    if (status != 0) {
      AKD1500_LIBRARY_LOG(
          "[AKD1500][flash] program failed offset=0x%08lX size=%lu status=%d\r\n",
          static_cast<unsigned long>(offset + written),
          static_cast<unsigned long>(program_size), status);
      return false;
    }
  }

  return true;
}

bool verify_flash_payload(const uint8_t* payload, size_t payload_size,
                          uint32_t external_program_data_address) {
  if (payload == nullptr || payload_size == 0u) {
    return false;
  }

  if (!ensure_flash_ready()) {
    return false;
  }

  const size_t offset = logical_flash_offset(external_program_data_address);
  if (offset == static_cast<size_t>(-1)) {
    AKD1500_LIBRARY_LOG("[AKD1500][flash] invalid logical address 0x%08lX\r\n",
                        static_cast<unsigned long>(external_program_data_address));
    return false;
  }
  const size_t program_size = bridge_flash().get_program_size();
  if (program_size == 0u) {
    AKD1500_LIBRARY_LOG("[AKD1500][flash] invalid program size\r\n");
    return false;
  }
  const size_t read_length = round_up_to(payload_size, program_size);

  std::vector<uint8_t> chunk(program_size, 0xFFu);
  for (size_t read_offset = 0u; read_offset < read_length; read_offset += program_size) {
    const size_t compare_size =
        ((read_offset + program_size) <= payload_size) ? program_size
                                                       : (payload_size - read_offset);
    std::fill(chunk.begin(), chunk.end(), 0xFFu);
    const int status = bridge_flash().read(
        chunk.data(), offset + read_offset, static_cast<mbed::bd_size_t>(program_size));
    if (status != 0) {
      AKD1500_LIBRARY_LOG(
          "[AKD1500][flash] read failed offset=0x%08lX size=%lu status=%d\r\n",
          static_cast<unsigned long>(offset + read_offset),
          static_cast<unsigned long>(program_size), status);
      return false;
    }

    if (std::memcmp(chunk.data(), payload + read_offset, compare_size) != 0) {
      AKD1500_LIBRARY_LOG(
          "[AKD1500][flash] verify mismatch offset=0x%08lX size=%lu\r\n",
          static_cast<unsigned long>(offset + read_offset),
          static_cast<unsigned long>(compare_size));
      return false;
    }
  }

  return true;
}

bool read_flash_bytes(uint32_t flash_offset, uint8_t* data, size_t size) {
  if (data == nullptr || size == 0u) {
    return false;
  }

  if (!ensure_flash_ready()) {
    return false;
  }

  const int status = bridge_flash().read(
      data, static_cast<mbed::bd_addr_t>(flash_offset),
      static_cast<mbed::bd_size_t>(size));
  if (status != 0) {
    AKD1500_LIBRARY_LOG(
        "[AKD1500][flash] read bytes failed offset=0x%08lX size=%lu status=%d\r\n",
        static_cast<unsigned long>(flash_offset),
        static_cast<unsigned long>(size), status);
    return false;
  }

  return true;
}
#else
bool write_flash_payload(const uint8_t* payload, size_t payload_size,
                         uint32_t external_program_data_address) {
  (void)payload;
  (void)payload_size;
  (void)external_program_data_address;
  return false;
}

bool verify_flash_payload(const uint8_t* payload, size_t payload_size,
                          uint32_t external_program_data_address) {
  (void)payload;
  (void)payload_size;
  (void)external_program_data_address;
  return false;
}

bool read_flash_bytes(uint32_t flash_offset, uint8_t* data, size_t size) {
  (void)flash_offset;
  (void)data;
  (void)size;
  return false;
}
#endif

bool bridge_transfer(ArduinoSpiDriver& spi_driver, uint8_t bridge_cs_pin,
                     const uint8_t* tx, uint8_t* rx, size_t size) {
  if (tx == nullptr || rx == nullptr || size == 0u) {
    return false;
  }

  spi_driver.chip_select(0u, false);
  digitalWrite(bridge_cs_pin, LOW);
  spi_driver.transfer(tx, rx, size);
  digitalWrite(bridge_cs_pin, HIGH);
  return true;
}

bool read_flash_jedec(ArduinoSpiDriver& spi_driver, uint8_t bridge_cs_pin,
                      uint8_t* manufacturer_id, uint8_t* memory_type,
                      uint8_t* capacity_id) {
  if (manufacturer_id == nullptr || memory_type == nullptr ||
      capacity_id == nullptr) {
    return false;
  }

  uint8_t tx[4] = {kFlashCmdReadJedecId, 0x00u, 0x00u, 0x00u};
  uint8_t rx[4] = {0u, 0u, 0u, 0u};
  if (!bridge_transfer(spi_driver, bridge_cs_pin, tx, rx, sizeof(tx))) {
    return false;
  }

  *manufacturer_id = rx[1];
  *memory_type = rx[2];
  *capacity_id = rx[3];
  return true;
}

bool flash_bridge_cmd1(ArduinoSpiDriver& spi_driver, uint8_t bridge_cs_pin,
                       uint8_t cmd) {
  uint8_t tx[1] = {cmd};
  uint8_t rx[1] = {0u};
  return bridge_transfer(spi_driver, bridge_cs_pin, tx, rx, sizeof(tx));
}

uint8_t flash_bridge_read_status(ArduinoSpiDriver& spi_driver,
                                 uint8_t bridge_cs_pin) {
  uint8_t tx[2] = {kFlashCmdReadStatus, 0x00u};
  uint8_t rx[2] = {0u, 0u};
  if (!bridge_transfer(spi_driver, bridge_cs_pin, tx, rx, sizeof(tx))) {
    return 0xFFu;
  }
  return rx[1];
}

uint8_t flash_bridge_read_status2(ArduinoSpiDriver& spi_driver,
                                  uint8_t bridge_cs_pin) {
  uint8_t tx[2] = {kFlashCmdReadStatus2, 0x00u};
  uint8_t rx[2] = {0u, 0u};
  if (!bridge_transfer(spi_driver, bridge_cs_pin, tx, rx, sizeof(tx))) {
    return 0xFFu;
  }
  return rx[1];
}

uint8_t flash_bridge_read_register(ArduinoSpiDriver& spi_driver,
                                   uint8_t bridge_cs_pin, uint8_t cmd) {
  uint8_t tx[2] = {cmd, 0x00u};
  uint8_t rx[2] = {0u, 0u};
  if (!bridge_transfer(spi_driver, bridge_cs_pin, tx, rx, sizeof(tx))) {
    return 0xFFu;
  }
  return rx[1];
}

bool flash_bridge_wait_ready(ArduinoSpiDriver& spi_driver,
                             uint8_t bridge_cs_pin, uint32_t timeout_ms) {
  const uint32_t start_ms = millis();
  while ((millis() - start_ms) <= timeout_ms) {
    if ((flash_bridge_read_status(spi_driver, bridge_cs_pin) & 0x01u) == 0u) {
      return true;
    }
    delay(1);
  }
  return false;
}

bool flash_bridge_wait_wel(ArduinoSpiDriver& spi_driver, uint8_t bridge_cs_pin,
                           bool expect_set) {
  const uint32_t start_ms = millis();
  while ((millis() - start_ms) <= 100u) {
    const bool wel_set =
        (flash_bridge_read_status(spi_driver, bridge_cs_pin) & 0x02u) != 0u;
    if (wel_set == expect_set) {
      return true;
    }
    delay(1);
  }
  return false;
}

bool flash_bridge_write_status2(ArduinoSpiDriver& spi_driver,
                                uint8_t bridge_cs_pin, uint8_t status2) {
  uint8_t tx[2] = {kFlashCmdWriteStatus2, status2};
  uint8_t rx[2] = {0u, 0u};
  return bridge_transfer(spi_driver, bridge_cs_pin, tx, rx, sizeof(tx));
}

bool flash_bridge_write_register(ArduinoSpiDriver& spi_driver,
                                 uint8_t bridge_cs_pin, uint8_t cmd,
                                 uint8_t value) {
  uint8_t tx[2] = {cmd, value};
  uint8_t rx[2] = {0u, 0u};
  return bridge_transfer(spi_driver, bridge_cs_pin, tx, rx, sizeof(tx));
}

bool flash_bridge_write_enable(ArduinoSpiDriver& spi_driver,
                               uint8_t bridge_cs_pin) {
  if (!flash_bridge_cmd1(spi_driver, bridge_cs_pin, kFlashCmdWriteEnable)) {
    return false;
  }
  return flash_bridge_wait_wel(spi_driver, bridge_cs_pin, true);
}

bool flash_bridge_reset_preamble(ArduinoSpiDriver& spi_driver,
                                 uint8_t bridge_cs_pin) {
  if (!flash_bridge_cmd1(spi_driver, bridge_cs_pin,
                         kFlashCmdReleasePowerDown)) {
    return false;
  }
  delayMicroseconds(10);
  if (!flash_bridge_cmd1(spi_driver, bridge_cs_pin, kFlashCmdResetEnable)) {
    return false;
  }
  delay(1);
  if (!flash_bridge_cmd1(spi_driver, bridge_cs_pin, kFlashCmdResetMemory)) {
    return false;
  }
  delay(5);
  return true;
}

bool flash_bridge_sector_erase(ArduinoSpiDriver& spi_driver,
                               uint8_t bridge_cs_pin, uint32_t address) {
  if (!flash_bridge_write_enable(spi_driver, bridge_cs_pin)) {
    return false;
  }

  uint8_t tx[4] = {
      kFlashCmdSectorErase4K,
      static_cast<uint8_t>((address >> 16) & 0xFFu),
      static_cast<uint8_t>((address >> 8) & 0xFFu),
      static_cast<uint8_t>(address & 0xFFu),
  };
  uint8_t rx[4] = {0u, 0u, 0u, 0u};
  if (!bridge_transfer(spi_driver, bridge_cs_pin, tx, rx, sizeof(tx))) {
    return false;
  }

  return flash_bridge_wait_ready(spi_driver, bridge_cs_pin, 2000u);
}

bool flash_bridge_page_program(ArduinoSpiDriver& spi_driver,
                               uint8_t bridge_cs_pin, uint32_t address,
                               const uint8_t* data, size_t size) {
  if (data == nullptr || size == 0u || size > kFlashPageSize) {
    return false;
  }
  if (((address & (kFlashPageSize - 1u)) + size) > kFlashPageSize) {
    return false;
  }
  if (!flash_bridge_write_enable(spi_driver, bridge_cs_pin)) {
    return false;
  }

  std::array<uint8_t, 4u + kFlashPageSize> tx{};
  std::array<uint8_t, 4u + kFlashPageSize> rx{};
  tx[0] = kFlashCmdPageProgram;
  tx[1] = static_cast<uint8_t>((address >> 16) & 0xFFu);
  tx[2] = static_cast<uint8_t>((address >> 8) & 0xFFu);
  tx[3] = static_cast<uint8_t>(address & 0xFFu);
  std::memcpy(&tx[4], data, size);

  if (!bridge_transfer(spi_driver, bridge_cs_pin, tx.data(), rx.data(),
                       4u + size)) {
    return false;
  }

  return flash_bridge_wait_ready(spi_driver, bridge_cs_pin, 200u);
}

bool flash_bridge_read_data(ArduinoSpiDriver& spi_driver,
                            uint8_t bridge_cs_pin, uint32_t address,
                            uint8_t* data, size_t size) {
  if (data == nullptr || size == 0u || size > kFlashVerifyChunkSize) {
    return false;
  }

  std::array<uint8_t, 4u + kFlashVerifyChunkSize> tx{};
  std::array<uint8_t, 4u + kFlashVerifyChunkSize> rx{};
  tx[0] = kFlashCmdReadData;
  tx[1] = static_cast<uint8_t>((address >> 16) & 0xFFu);
  tx[2] = static_cast<uint8_t>((address >> 8) & 0xFFu);
  tx[3] = static_cast<uint8_t>(address & 0xFFu);

  if (!bridge_transfer(spi_driver, bridge_cs_pin, tx.data(), rx.data(),
                       4u + size)) {
    return false;
  }

  std::memcpy(data, &rx[4], size);
  return true;
}

bool s2m_enter(akida::Akd1500SpiDriver& driver, uint32_t* ctrl_before) {
  if (ctrl_before == nullptr) {
    return false;
  }

  *ctrl_before = driver.read32(kControlSignalsReg);
  const uint32_t ctrl_after = *ctrl_before | kCtrlEnSpiS2mMask;
  driver.write32(kControlSignalsReg, ctrl_after);
  uint32_t ctrl_latched = 0u;
  for (uint32_t attempt = 0u; attempt < 3u; ++attempt) {
    ctrl_latched = driver.read32(kControlSignalsReg);
    if ((ctrl_latched & kCtrlEnSpiS2mMask) != 0u) {
      break;
    }
    delay(1);
  }
  AKD1500_LIBRARY_LOG(
      "[AKD1500][s2m] ctrl before=0x%08lX after=0x%08lX latched=0x%08lX\r\n",
      static_cast<unsigned long>(*ctrl_before),
      static_cast<unsigned long>(ctrl_after),
      static_cast<unsigned long>(ctrl_latched));
  delay(kS2mPhaseGapMs);
  return (ctrl_latched & kCtrlEnSpiS2mMask) != 0u;
}

void s2m_leave(akida::Akd1500SpiDriver& driver, ArduinoSpiDriver& spi_driver,
               uint8_t bridge_cs_pin, uint32_t ctrl_before) {
  driver.write32(kControlSignalsReg, ctrl_before);
  digitalWrite(bridge_cs_pin, HIGH);
  spi_driver.chip_select(0u, false);
  const uint32_t ctrl_after = driver.read32(kControlSignalsReg);
  AKD1500_LIBRARY_LOG("[AKD1500][s2m] ctrl restored=0x%08lX\r\n",
                      static_cast<unsigned long>(ctrl_after));
  delay(kS2mPhaseGapMs);
}

const FlashProfile* find_flash_profile(uint8_t manufacturer_id,
                                       uint8_t memory_type,
                                       uint8_t capacity_id) {
  for (const auto& profile : kFlashProfiles) {
    if (profile.manufacturer_id == manufacturer_id &&
        profile.memory_type == memory_type &&
        profile.capacity_id == capacity_id) {
      return &profile;
    }
  }
  return nullptr;
}

const FlashProfile* find_flash_profile_by_name(const char* name) {
  if (name == nullptr || name[0] == '\0' || std::strcmp(name, "auto") == 0) {
    return nullptr;
  }
  for (const auto& profile : kFlashProfiles) {
    if (std::strcmp(profile.name, name) == 0) {
      return &profile;
    }
  }
  if (std::strcmp(name, "winbond") == 0) {
    return find_flash_profile(0xEFu, 0x60u, 0x17u);
  }
  if (std::strcmp(name, "winbond_eb") == 0) {
    return winbond_w25q64jw_eb_profile();
  }
  if (std::strcmp(name, "winbond_w25q64jw_eb") == 0) {
    return winbond_w25q64jw_eb_profile();
  }
  if (std::strcmp(name, "renesas") == 0) {
    return find_flash_profile(0x1Fu, 0x42u, 0x16u);
  }
  return nullptr;
}

bool apply_flash_profile(akida::Akd1500SpiDriver& driver,
                         ArduinoSpiDriver& spi_driver, uint8_t bridge_cs_pin,
                         const FlashProfile& profile, uint32_t* jedec_out,
                         bool validate_jedec = true) {
  uint32_t ctrl_before = 0u;
  if (!s2m_enter(driver, &ctrl_before)) {
    AKD1500_LIBRARY_LOG("[AKD1500][s2m] enter failed\r\n");
    return false;
  }

  bool ok = true;
  ok &= flash_bridge_cmd1(spi_driver, bridge_cs_pin, kFlashCmdReleasePowerDown);
  delayMicroseconds(10);
  ok &= flash_bridge_cmd1(spi_driver, bridge_cs_pin, kFlashCmdResetEnable);
  ok &= flash_bridge_cmd1(spi_driver, bridge_cs_pin, kFlashCmdResetMemory);
  delay(1);

  uint8_t manufacturer_id = 0u;
  uint8_t memory_type = 0u;
  uint8_t capacity_id = 0u;
  if (validate_jedec) {
    ok &= read_flash_jedec(spi_driver, bridge_cs_pin, &manufacturer_id,
                           &memory_type, &capacity_id);
    const uint32_t detected_jedec =
        pack_jedec(manufacturer_id, memory_type, capacity_id);
    if (jedec_out != nullptr) {
      *jedec_out = detected_jedec;
    }
    if (!ok) {
      AKD1500_LIBRARY_LOG("[AKD1500][s2m] flash JEDEC read failed\r\n");
      s2m_leave(driver, spi_driver, bridge_cs_pin, ctrl_before);
      return false;
    }
    if (detected_jedec !=
        pack_jedec(profile.manufacturer_id, profile.memory_type,
                   profile.capacity_id)) {
      AKD1500_LIBRARY_LOG(
          "[AKD1500][s2m] flash profile mismatch expected=%02X:%02X:%02X got=%02X:%02X:%02X\r\n",
          static_cast<unsigned>(profile.manufacturer_id),
          static_cast<unsigned>(profile.memory_type),
          static_cast<unsigned>(profile.capacity_id),
          static_cast<unsigned>(manufacturer_id),
          static_cast<unsigned>(memory_type),
          static_cast<unsigned>(capacity_id));
      s2m_leave(driver, spi_driver, bridge_cs_pin, ctrl_before);
      return false;
    }
  } else {
    manufacturer_id = profile.manufacturer_id;
    memory_type = profile.memory_type;
    capacity_id = profile.capacity_id;
    if (jedec_out != nullptr) {
      *jedec_out = pack_jedec(manufacturer_id, memory_type, capacity_id);
    }
  }

  const uint8_t status = flash_bridge_read_status(spi_driver, bridge_cs_pin);
  const uint8_t qe_before = flash_bridge_read_register(
      spi_driver, bridge_cs_pin, profile.qe_read_cmd);
  if ((qe_before & profile.qe_mask) == 0u) {
    ok &= flash_bridge_cmd1(spi_driver, bridge_cs_pin, kFlashCmdWriteEnable);
    ok &= flash_bridge_wait_wel(spi_driver, bridge_cs_pin, true);
    ok &= flash_bridge_write_register(
        spi_driver, bridge_cs_pin, profile.qe_write_cmd,
        static_cast<uint8_t>(qe_before | profile.qe_mask));
    ok &= flash_bridge_wait_ready(spi_driver, bridge_cs_pin, 200u);
  }
  const uint8_t qe_after = flash_bridge_read_register(
      spi_driver, bridge_cs_pin, profile.qe_read_cmd);
  const bool qe_ok = (qe_after & profile.qe_mask) != 0u;
  ok &= qe_ok;
  if (ok) {
    driver.set_spi_flash_runtime_config(profile.runtime_config);
  }

  AKD1500_LIBRARY_LOG(
      "[AKD1500][s2m] flash profile=%s jedec=%02X:%02X:%02X sr1=0x%02X qe_before=0x%02X qe_after=0x%02X qe=%u read=0x%02X trans=%u wait=%u mode_en=%u mode=0x%02X result=%s\r\n",
      profile.name, static_cast<unsigned>(manufacturer_id),
      static_cast<unsigned>(memory_type), static_cast<unsigned>(capacity_id),
      static_cast<unsigned>(status), static_cast<unsigned>(qe_before),
      static_cast<unsigned>(qe_after), qe_ok ? 1u : 0u,
      static_cast<unsigned>(profile.runtime_config.read_opcode),
      static_cast<unsigned>(profile.runtime_config.transfer_type),
      static_cast<unsigned>(profile.runtime_config.wait_cycles),
      profile.runtime_config.mode_bits_enabled ? 1u : 0u,
      static_cast<unsigned>(profile.runtime_config.mode_bits_value),
      ok ? "PASS" : "FAIL");

  s2m_leave(driver, spi_driver, bridge_cs_pin, ctrl_before);
  return ok;
}

}  // namespace

bool stage_program_data_to_bridge_flash(const AKD1500BoardConfig& config,
                                        const uint8_t* serialized_program,
                                        size_t serialized_program_size,
                                        uint32_t external_program_data_address) {
  AKD1500Board board(config);
  return board.stage_program_data_to_bridge_flash(
      serialized_program, serialized_program_size,
      external_program_data_address);
}

bool verify_program_data_from_bridge_flash(
    const AKD1500BoardConfig& config, const uint8_t* serialized_program,
    size_t serialized_program_size, uint32_t external_program_data_address) {
  AKD1500Board board(config);
  return board.verify_program_data_from_bridge_flash(
      serialized_program, serialized_program_size,
      external_program_data_address);
}

void ArduinoSpiDriver::configure(SPIClass* spi_bus, uint8_t akida_cs_pin,
                                 uint32_t spi_clock_hz) {
  spi_bus_ = (spi_bus != nullptr) ? spi_bus : &SPI;
  akida_cs_pin_ = akida_cs_pin;
  spi_clock_hz_ = spi_clock_hz;
}

SPIClass& ArduinoSpiDriver::spi_bus() {
  return (spi_bus_ != nullptr) ? *spi_bus_ : SPI;
}

SPISettings ArduinoSpiDriver::make_spi_settings() const {
  return SPISettings(spi_clock_hz_, MSBFIRST, SPI_MODE0);
}

bool ArduinoSpiDriver::begin_transfer() {
  if (!initialized_) {
    return false;
  }
  if (!active_) {
    spi_bus().beginTransaction(make_spi_settings());
    return true;
  }
  return false;
}

void ArduinoSpiDriver::end_transfer(bool temporary) {
  if (temporary) {
    spi_bus().endTransaction();
  }
}

void ArduinoSpiDriver::begin() {
  pinMode(akida_cs_pin_, OUTPUT);
  digitalWrite(akida_cs_pin_, HIGH);
  if (!initialized_) {
    spi_bus().begin();
    initialized_ = true;
  }
  active_ = false;
}

void ArduinoSpiDriver::read(uint8_t* data, size_t size) {
  if (!initialized_ || data == nullptr || size == 0u) {
    return;
  }

  const bool temporary = begin_transfer();
  std::array<uint8_t, kSpiChunkSize> tx_dummy = {};
  size_t offset = 0u;
  while (offset < size) {
    const size_t chunk = ((size - offset) > kSpiChunkSize) ? kSpiChunkSize
                                                           : (size - offset);
    std::fill(tx_dummy.begin(), tx_dummy.begin() + chunk, 0u);
    spi_bus().transfer(tx_dummy.data(), chunk);
    std::memcpy(data + offset, tx_dummy.data(), chunk);
    dump_spi_trace_bytes("read-rx", data + offset, chunk);
    trace_spi_transaction("read", chunk, nullptr, data + offset);
    offset += chunk;
  }
  end_transfer(temporary);
}

void ArduinoSpiDriver::write(const uint8_t* data, size_t size) {
  if (!initialized_ || data == nullptr || size == 0u) {
    return;
  }

  const bool temporary = begin_transfer();
  std::array<uint8_t, kSpiChunkSize> tx_buffer = {};
  size_t offset = 0u;
  while (offset < size) {
    const size_t chunk = ((size - offset) > kSpiChunkSize) ? kSpiChunkSize
                                                           : (size - offset);
    std::memcpy(tx_buffer.data(), data + offset, chunk);
    spi_bus().transfer(tx_buffer.data(), chunk);
    dump_spi_trace_bytes("write-tx", data + offset, chunk);
    trace_spi_transaction("write", chunk, data + offset, nullptr);
    offset += chunk;
  }
  end_transfer(temporary);
}

void ArduinoSpiDriver::transfer(const uint8_t* tx, uint8_t* rx, size_t size) {
  if (!initialized_ || tx == nullptr || rx == nullptr || size == 0u) {
    return;
  }

  const bool temporary = begin_transfer();
  std::array<uint8_t, kSpiChunkSize> txrx_buffer = {};
  size_t offset = 0u;
  while (offset < size) {
    const size_t chunk = ((size - offset) > kSpiChunkSize) ? kSpiChunkSize
                                                           : (size - offset);
    std::memcpy(txrx_buffer.data(), tx + offset, chunk);
    spi_bus().transfer(txrx_buffer.data(), chunk);
    std::memcpy(rx + offset, txrx_buffer.data(), chunk);
    dump_spi_trace_bytes("transfer-tx", tx + offset, chunk);
    dump_spi_trace_bytes("transfer-rx", rx + offset, chunk);
    trace_spi_transaction("transfer", chunk, tx + offset, rx + offset);
    offset += chunk;
  }
  end_transfer(temporary);
}

void ArduinoSpiDriver::chip_select(uint32_t slave_id, bool active) {
  (void)slave_id;
  if (active) {
    if (!active_) {
      spi_bus().beginTransaction(make_spi_settings());
      digitalWrite(akida_cs_pin_, LOW);
      active_ = true;
      trace_spi_cs(true);
      trace_spi_cs_transaction(slave_id, true);
      if (AKIDA_NICLA_SPI_CS_ASSERT_DELAY_US > 0) {
        spi_delay_us(AKIDA_NICLA_SPI_CS_ASSERT_DELAY_US);
      }
    }
    return;
  }

  if (active_) {
    if (AKIDA_NICLA_SPI_CS_DEASSERT_DELAY_US > 0) {
      spi_delay_us(AKIDA_NICLA_SPI_CS_DEASSERT_DELAY_US);
    }
    digitalWrite(akida_cs_pin_, HIGH);
    active_ = false;
    trace_spi_cs(false);
    trace_spi_cs_transaction(slave_id, false);
    spi_bus().endTransaction();
  }
}

AKD1500Board::AKD1500Board(const AKD1500BoardConfig& config)
    : config_(config) {
  spi_driver_.configure(config_.spi_bus, config_.pins.akida_cs,
                        config_.spi_clock_hz);
}

void AKD1500Board::begin() {
  if (akida_driver_) {
    return;
  }

  pinMode(config_.pins.bridge_cs, OUTPUT);
  digitalWrite(config_.pins.bridge_cs, HIGH);
  spi_driver_.begin();

  akida_driver_.reset(new akida::Akd1500SpiDriver(
      &spi_driver_, config_.visible_memory_base, config_.visible_memory_size));
}

void AKD1500Board::ensure_started() {
  if (!akida_driver_) {
    begin();
  }
}

bool AKD1500Board::ensure_spi_flash_runtime_profile() {
  ensure_started();
  if (detected_flash_profile_attempted_) {
    if (detected_flash_profile_supported_) {
      akida_driver_->reinit_spi_flash_runtime();
    }
    return detected_flash_profile_supported_;
  }

  detected_flash_profile_attempted_ = true;
  const bool forced_profile =
      config_.forced_flash_profile != nullptr &&
      config_.forced_flash_profile[0] != '\0' &&
      std::strcmp(config_.forced_flash_profile, "auto") != 0;
  if (forced_profile && config_.assume_forced_flash_profile_ready) {
    const FlashProfile* profile =
        find_flash_profile_by_name(config_.forced_flash_profile);
    detected_flash_name_ = (profile != nullptr) ? profile->name : "unsupported";
    if (profile == nullptr) {
      AKD1500_LIBRARY_LOG(
          "[AKD1500][s2m] assumed flash profile not found name=%s\r\n",
          config_.forced_flash_profile);
      return false;
    }
    detected_flash_jedec_ = pack_jedec(profile->manufacturer_id,
                                       profile->memory_type,
                                       profile->capacity_id);
    detected_flash_runtime_config_ = profile->runtime_config;
    detected_flash_profile_supported_ = true;
    akida_driver_->set_spi_flash_runtime_config(profile->runtime_config);
    akida_driver_->reinit_spi_flash_runtime();
    AKD1500_LIBRARY_LOG(
        "[AKD1500][s2m] flash profile assumed ready name=%s jedec=%02X:%02X:%02X read=0x%02X trans=%u wait=%u mode_en=%u mode=0x%02X\r\n",
        profile->name, static_cast<unsigned>(profile->manufacturer_id),
        static_cast<unsigned>(profile->memory_type),
        static_cast<unsigned>(profile->capacity_id),
        static_cast<unsigned>(profile->runtime_config.read_opcode),
        static_cast<unsigned>(profile->runtime_config.transfer_type),
        static_cast<unsigned>(profile->runtime_config.wait_cycles),
        profile->runtime_config.mode_bits_enabled ? 1u : 0u,
        static_cast<unsigned>(profile->runtime_config.mode_bits_value));
    return true;
  }

  uint32_t detected_jedec = 0u;
  const uint32_t original_spi_clock_hz = spi_driver_.clock_hz();
  const uint32_t flash_spi_clock_hz =
      (config_.flash_spi_clock_hz != 0u) ? config_.flash_spi_clock_hz
                                         : original_spi_clock_hz;
  const auto restore_spi_clock = [this, original_spi_clock_hz]() {
    spi_driver_.set_clock_hz(original_spi_clock_hz);
  };
  spi_driver_.set_clock_hz(flash_spi_clock_hz);

  uint32_t ctrl_before = 0u;
  if (!s2m_enter(*akida_driver_, &ctrl_before)) {
    AKD1500_LIBRARY_LOG("[AKD1500][s2m] flash detect failed: enter\r\n");
    restore_spi_clock();
    return false;
  }
  bool ok = flash_bridge_cmd1(spi_driver_, config_.pins.bridge_cs,
                              kFlashCmdReleasePowerDown);
  delayMicroseconds(10);
  ok &= flash_bridge_cmd1(spi_driver_, config_.pins.bridge_cs,
                          kFlashCmdResetEnable);
  ok &= flash_bridge_cmd1(spi_driver_, config_.pins.bridge_cs,
                          kFlashCmdResetMemory);
  delay(1);
  uint8_t manufacturer_id = 0u;
  uint8_t memory_type = 0u;
  uint8_t capacity_id = 0u;
  ok &= read_flash_jedec(spi_driver_, config_.pins.bridge_cs, &manufacturer_id,
                         &memory_type, &capacity_id);
  s2m_leave(*akida_driver_, spi_driver_, config_.pins.bridge_cs, ctrl_before);

  detected_jedec = pack_jedec(manufacturer_id, memory_type, capacity_id);
  detected_flash_jedec_ = detected_jedec;

  if (!ok) {
    detected_flash_name_ = "jedec_read_failed";
    AKD1500_LIBRARY_LOG("[AKD1500][s2m] flash detect failed: JEDEC read\r\n");
    restore_spi_clock();
    return false;
  }

  const FlashProfile* profile = nullptr;
  if (forced_profile) {
    profile = find_flash_profile_by_name(config_.forced_flash_profile);
    if (profile != nullptr) {
      manufacturer_id = profile->manufacturer_id;
      memory_type = profile->memory_type;
      capacity_id = profile->capacity_id;
      detected_jedec = pack_jedec(manufacturer_id, memory_type, capacity_id);
      detected_flash_jedec_ = detected_jedec;
    }
  } else {
    profile = find_flash_profile(manufacturer_id, memory_type, capacity_id);
  }
  detected_flash_name_ = (profile != nullptr) ? profile->name : "unsupported";
  if (profile == nullptr) {
    if (forced_profile) {
      AKD1500_LIBRARY_LOG(
          "[AKD1500][s2m] unsupported forced flash profile=%s jedec=%02X:%02X:%02X\r\n",
          config_.forced_flash_profile,
          static_cast<unsigned>(manufacturer_id),
          static_cast<unsigned>(memory_type),
          static_cast<unsigned>(capacity_id));
    } else {
      AKD1500_LIBRARY_LOG(
          "[AKD1500][s2m] unsupported flash jedec=%02X:%02X:%02X\r\n",
          static_cast<unsigned>(manufacturer_id),
          static_cast<unsigned>(memory_type),
          static_cast<unsigned>(capacity_id));
    }
    restore_spi_clock();
    return false;
  }
  detected_flash_runtime_config_ = profile->runtime_config;

  detected_flash_profile_supported_ = apply_flash_profile(
      *akida_driver_, spi_driver_, config_.pins.bridge_cs, *profile,
      &detected_flash_jedec_, !forced_profile);
  restore_spi_clock();
  if (!detected_flash_profile_supported_) {
    AKD1500_LIBRARY_LOG(
        "[AKD1500][s2m] flash runtime profile apply failed name=%s jedec=%02X:%02X:%02X\r\n",
        profile->name, static_cast<unsigned>(manufacturer_id),
        static_cast<unsigned>(memory_type),
        static_cast<unsigned>(capacity_id));
    return false;
  }

  akida_driver_->reinit_spi_flash_runtime();
  AKD1500_LIBRARY_LOG(
      "[AKD1500][s2m] flash profile selected mode=%s name=%s jedec=%02X:%02X:%02X capacity=0x%08lX read=0x%02X trans=%u wait=%u mode_en=%u mode=0x%02X\r\n",
      forced_profile ? "forced" : "auto", profile->name,
      static_cast<unsigned>(manufacturer_id),
      static_cast<unsigned>(memory_type),
      static_cast<unsigned>(capacity_id),
      static_cast<unsigned long>(profile->capacity_bytes),
      static_cast<unsigned>(profile->runtime_config.read_opcode),
      static_cast<unsigned>(profile->runtime_config.transfer_type),
      static_cast<unsigned>(profile->runtime_config.wait_cycles),
      profile->runtime_config.mode_bits_enabled ? 1u : 0u,
      static_cast<unsigned>(profile->runtime_config.mode_bits_value));
  return true;
}

uint32_t AKD1500Board::read_ip_version() {
  ensure_started();
  return akida_driver_->read32(akida_driver_->top_level_reg() +
                               akida::REG_IP_VERSION);
}

void AKD1500Board::dump_spi_master_state(const char* prefix) {
  ensure_started();
  if (prefix == nullptr) {
    prefix = "[AKD1500][spim]";
  }

  const auto read_reg = [this](uint32_t address) {
    return akida_driver_->read32(address);
  };

  std::printf(
      "%s ctrl_signals=0x%08lX spi_m.ctrlr0=0x%08lX spi_m.ssienr=0x%08lX spi_m.ser=0x%08lX spi_m.baudr=0x%08lX\r\n",
      prefix, static_cast<unsigned long>(read_reg(kControlSignalsReg)),
      static_cast<unsigned long>(read_reg(kSpiMCtrlr0)),
      static_cast<unsigned long>(read_reg(kSpiMSsienr)),
      static_cast<unsigned long>(read_reg(kSpiMSer)),
      static_cast<unsigned long>(read_reg(kSpiMBaudr)));
  std::printf(
      "%s spi_m.spi_ctrlr0=0x%08lX spi_m.ddr_drive_edge=0x%08lX spi_m.xip_mode_bits=0x%08lX spi_m.xip_incr_inst=0x%08lX\r\n",
      prefix, static_cast<unsigned long>(read_reg(kSpiMSpiCtrlr0)),
      static_cast<unsigned long>(read_reg(kSpiMDdrDriveEdge)),
      static_cast<unsigned long>(read_reg(kSpiMXipModeBits)),
      static_cast<unsigned long>(read_reg(kSpiMXipIncrInst)));
}

void AKD1500Board::dump_runtime_state(const char* prefix) {
  ensure_started();
  if (prefix == nullptr) {
    prefix = "[AKD1500][state]";
  }

  const auto top = akida_driver_->top_level_reg();
  const auto read_reg = [this](uint32_t address) {
    return akida_driver_->read32(address);
  };
  const auto dump_dma = [&](const char* dma_name, uint32_t base) {
    std::printf(
        "%s dma=%s base=0x%08lX ctrl=0x%08lX desc_cont=0x%08lX desc_status=0x%08lX mon_status=0x%08lX job_fifo=0x%08lX timer=0x%08lX in_payload=0x%08lX out_payload=0x%08lX in_words=0x%08lX out_words=0x%08lX\r\n",
        prefix, dma_name, static_cast<unsigned long>(base),
        static_cast<unsigned long>(read_reg(base + akida::DMA_CTRL_REG)),
        static_cast<unsigned long>(read_reg(base + akida::DMA_DESC_CONT_REG)),
        static_cast<unsigned long>(read_reg(base + akida::DMA_DESC_STATUS_REG)),
        static_cast<unsigned long>(read_reg(base + akida::DMA_BUF_MON_STATUS_REG)),
        static_cast<unsigned long>(read_reg(base + akida::DMA_JOB_ID_FIFO_REG)),
        static_cast<unsigned long>(
            read_reg(base + akida::DMA_BUFFER_TIMER_STATUS_REG)),
        static_cast<unsigned long>(read_reg(base + akida::DMA_INPUT_PAYLOAD_REG)),
        static_cast<unsigned long>(read_reg(base + akida::DMA_OUTPUT_PAYLOAD_REG)),
        static_cast<unsigned long>(
            read_reg(base + akida::DMA_INPUT_WORD_COUNT_REG)),
        static_cast<unsigned long>(
            read_reg(base + akida::DMA_OUTPUT_WORD_COUNT_REG)));
  };

  std::printf(
      "%s ip_version=0x%08lX general_control=0x%08lX int_ctrl=0x%08lX int_mask=0x%08lX int_src=0x%08lX\r\n",
      prefix,
      static_cast<unsigned long>(read_reg(top + akida::REG_IP_VERSION)),
      static_cast<unsigned long>(read_reg(top + akida::REG_GENERAL_CONTROL)),
      static_cast<unsigned long>(
          read_reg(top + akida::REG_INTERRUPT_CONTROLLER_GENERAL_CONTROL)),
      static_cast<unsigned long>(
          read_reg(top + akida::REG_INTERRUPT_CONTROLLER_SOURCE_MASK)),
      static_cast<unsigned long>(
          read_reg(top + akida::REG_INTERRUPT_CONTROLLER_SOURCE)));
  dump_spi_master_state(prefix);
  dump_dma("event", akida::dma_event_reg_base(top));
  dump_dma("hrc", akida::dma_hrc_reg_base(top));
  dump_dma("config", akida::dma_config_reg_base(top));
}

bool AKD1500Board::reinit_spi_flash_runtime() {
  return ensure_spi_flash_runtime_profile();
}

bool AKD1500Board::read_bridge_flash(uint32_t flash_offset, uint8_t* data,
                                     size_t size) {
  return read_flash_bytes(flash_offset, data, size);
}

bool AKD1500Board::stage_program_data_to_bridge_flash(
    const uint8_t* serialized_program, size_t serialized_program_size,
    uint32_t external_program_data_address) {
  const size_t program_info_size = serialized_program_info_size(
      serialized_program, serialized_program_size);
  if (program_info_size == 0u ||
      program_info_size > serialized_program_size) {
    AKD1500_LIBRARY_LOG("[AKD1500][flash] invalid serialized program split\r\n");
    return false;
  }

  begin();
  if (read_ip_version() != config_.expected_ip_version) {
    AKD1500_LIBRARY_LOG(
        "[AKD1500][flash] unexpected ip version when staging expected=0x%08lX got=0x%08lX\r\n",
        static_cast<unsigned long>(config_.expected_ip_version),
        static_cast<unsigned long>(read_ip_version()));
    return false;
  }

  const size_t flash_offset = logical_flash_offset(external_program_data_address);
  if (flash_offset == static_cast<size_t>(-1)) {
    AKD1500_LIBRARY_LOG("[AKD1500][flash] invalid logical address 0x%08lX\r\n",
                        static_cast<unsigned long>(external_program_data_address));
    return false;
  }

  const uint8_t* program_data = serialized_program + program_info_size;
  const size_t program_data_size = serialized_program_size - program_info_size;
  AKD1500_LIBRARY_LOG(
      "[AKD1500][flash] stage via bridge bytes=%lu addr=0x%08lX offset=0x%08lX\r\n",
      static_cast<unsigned long>(program_data_size),
      static_cast<unsigned long>(external_program_data_address),
      static_cast<unsigned long>(flash_offset));

  uint32_t ctrl_before = 0u;
  if (!s2m_enter(*akida_driver_, &ctrl_before)) {
    AKD1500_LIBRARY_LOG("[AKD1500][flash] stage failed: s2m enter\r\n");
    return false;
  }

  bool ok = flash_bridge_reset_preamble(spi_driver_, config_.pins.bridge_cs);

  const uint32_t erase_start =
      static_cast<uint32_t>(flash_offset) & ~(kFlashSectorSize - 1u);
  const uint32_t erase_end =
      (static_cast<uint32_t>(flash_offset) +
       static_cast<uint32_t>(program_data_size) +
       (kFlashSectorSize - 1u)) &
      ~(kFlashSectorSize - 1u);

  for (uint32_t addr = erase_start; ok && addr < erase_end;
       addr += kFlashSectorSize) {
    ok = flash_bridge_sector_erase(spi_driver_, config_.pins.bridge_cs, addr);
  }

  for (size_t offset = 0u; ok && offset < program_data_size;
       offset += kFlashPageSize) {
    const size_t chunk_size =
        std::min(kFlashPageSize, program_data_size - offset);
    const uint32_t chunk_addr =
        static_cast<uint32_t>(flash_offset + offset);
    ok = flash_bridge_page_program(spi_driver_, config_.pins.bridge_cs,
                                   chunk_addr, program_data + offset,
                                   chunk_size);
  }

  std::array<uint8_t, kFlashVerifyChunkSize> verify{};
  for (size_t offset = 0u; ok && offset < program_data_size;
       offset += kFlashVerifyChunkSize) {
    const size_t chunk_size =
        std::min(kFlashVerifyChunkSize, program_data_size - offset);
    const uint32_t chunk_addr =
        static_cast<uint32_t>(flash_offset + offset);
    verify.fill(0u);
    ok = flash_bridge_read_data(spi_driver_, config_.pins.bridge_cs, chunk_addr,
                                verify.data(), chunk_size);
    if (ok &&
        std::memcmp(program_data + offset, verify.data(), chunk_size) != 0) {
      ok = false;
    }
  }

  s2m_leave(*akida_driver_, spi_driver_, config_.pins.bridge_cs, ctrl_before);
  if (!ok) {
    AKD1500_LIBRARY_LOG("[AKD1500][flash] stage failed during erase/program/verify\r\n");
    return false;
  }

  if (!ensure_spi_flash_runtime_profile()) {
    AKD1500_LIBRARY_LOG("[AKD1500][flash] stage failed: runtime profile\r\n");
    return false;
  }

  return true;
}

bool AKD1500Board::verify_program_data_from_bridge_flash(
    const uint8_t* serialized_program, size_t serialized_program_size,
    uint32_t external_program_data_address) {
  const size_t program_info_size = serialized_program_info_size(
      serialized_program, serialized_program_size);
  if (program_info_size == 0u ||
      program_info_size > serialized_program_size) {
    AKD1500_LIBRARY_LOG("[AKD1500][flash] invalid serialized program split\r\n");
    return false;
  }

  begin();
  if (read_ip_version() != config_.expected_ip_version) {
    AKD1500_LIBRARY_LOG(
        "[AKD1500][flash] unexpected ip version when verifying expected=0x%08lX got=0x%08lX\r\n",
        static_cast<unsigned long>(config_.expected_ip_version),
        static_cast<unsigned long>(read_ip_version()));
    return false;
  }

  const size_t flash_offset = logical_flash_offset(external_program_data_address);
  if (flash_offset == static_cast<size_t>(-1)) {
    AKD1500_LIBRARY_LOG("[AKD1500][flash] invalid logical address 0x%08lX\r\n",
                        static_cast<unsigned long>(external_program_data_address));
    return false;
  }

  const uint8_t* program_data = serialized_program + program_info_size;
  const size_t program_data_size = serialized_program_size - program_info_size;
  uint32_t ctrl_before = 0u;
  if (!s2m_enter(*akida_driver_, &ctrl_before)) {
    AKD1500_LIBRARY_LOG("[AKD1500][flash] verify failed: s2m enter\r\n");
    return false;
  }

  bool ok = flash_bridge_reset_preamble(spi_driver_, config_.pins.bridge_cs);
  std::array<uint8_t, kFlashVerifyChunkSize> verify{};
  for (size_t offset = 0u; ok && offset < program_data_size;
       offset += kFlashVerifyChunkSize) {
    const size_t chunk_size =
        std::min(kFlashVerifyChunkSize, program_data_size - offset);
    const uint32_t chunk_addr =
        static_cast<uint32_t>(flash_offset + offset);
    verify.fill(0u);
    ok = flash_bridge_read_data(spi_driver_, config_.pins.bridge_cs, chunk_addr,
                                verify.data(), chunk_size);
    if (ok &&
        std::memcmp(program_data + offset, verify.data(), chunk_size) != 0) {
      ok = false;
    }
  }

  s2m_leave(*akida_driver_, spi_driver_, config_.pins.bridge_cs, ctrl_before);
  return ok;
}

akida::HardwareDriver& AKD1500Board::hardware_driver() {
  ensure_started();
  return *akida_driver_;
}

}  // namespace akida_port
