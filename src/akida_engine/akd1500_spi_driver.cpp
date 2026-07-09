#include "akd500/akd1500_spi_driver.h"

#include <array>
#include <cassert>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <type_traits>

#ifdef ARDUINO
#include <Arduino.h>
#endif

#include "infra/registers_common.h"

namespace akida {

#ifndef AKD1500_LIBRARY_ENABLE_LOGS
#define AKD1500_LIBRARY_ENABLE_LOGS 0
#endif

#if AKD1500_LIBRARY_ENABLE_LOGS
#define AKD1500_LIBRARY_LOG(...) std::printf(__VA_ARGS__)
#else
#define AKD1500_LIBRARY_LOG(...) ((void)0)
#endif

namespace {
const char* g_spi_trace_context = "idle";
constexpr uint32_t kSpisStatusPollDelayUs = 10u;
constexpr uint32_t kSpisRequestSettleDelayUs = 20u;
constexpr uint32_t kHrcDmaDescContReg = 0xFCC28004u;
inline void spis_wait_us(uint32_t delay_us) {
#ifdef ARDUINO
    delayMicroseconds(delay_us);
#else
    volatile uint32_t cycles = delay_us * 100u;
    while (cycles-- != 0u) {
        __asm__ volatile("nop");
    }
#endif
}

inline uint64_t trace_now_us() {
#ifdef ARDUINO
    return static_cast<uint64_t>(micros());
#else
    return 0u;
#endif
}

inline bool is_hrc_launch_reg_trace_target(uint32_t address, size_t size) {
    return address == kHrcDmaDescContReg && size == sizeof(uint32_t);
}

void print_trace_hex(const uint8_t* data, size_t size) {
    if (data == nullptr) {
        std::printf("-");
        return;
    }
    for (size_t i = 0; i < size; ++i) {
        std::printf("%02X", static_cast<unsigned>(data[i]));
    }
}

void trace_launch_reg_access(const char* op, const char* caller_context,
                             uint32_t address, const uint8_t* tx, size_t tx_size,
                             const uint8_t* rx, size_t rx_size, uint64_t t0,
                             uint64_t t1, uint64_t t2, uint64_t t3,
                             uint64_t t4) {
#if AKIDA_NICLA_HRC_LAUNCH_REG_TRACE
    std::printf(
        "[AKIDA][HRC_LAUNCH_TRACE] platform=nicla caller=%s local=%s op=%s addr=0x%08lX t_pre_cs=%llu t_post_cs=%llu t_post_io1=%llu t_post_io2=%llu t_post_deassert=%llu tx=",
        (caller_context != nullptr) ? caller_context : "unset",
        spi_trace_context(),
        op,
        static_cast<unsigned long>(address),
        static_cast<unsigned long long>(t0),
        static_cast<unsigned long long>(t1),
        static_cast<unsigned long long>(t2),
        static_cast<unsigned long long>(t3),
        static_cast<unsigned long long>(t4));
    print_trace_hex(tx, tx_size);
    std::printf(" rx=");
    print_trace_hex(rx, rx_size);
    std::printf("\r\n");
#else
    (void)op;
    (void)caller_context;
    (void)address;
    (void)tx;
    (void)tx_size;
    (void)rx;
    (void)rx_size;
    (void)t0;
    (void)t1;
    (void)t2;
    (void)t3;
    (void)t4;
#endif
}
}

const char* spi_trace_context() { return g_spi_trace_context; }

void spi_trace_set_context(const char* context) {
    g_spi_trace_context = (context != nullptr) ? context : "unset";
}

namespace akd1500::spi {

static constexpr uint32_t kSlaveID = 0;

enum class Commands : uint8_t {
    Read = 0x60,
    Write = 0x80,
};

enum class BurstWordSize : uint8_t {
    x1 = 0,
    x4 = 1,
    x8 = 2,
    x16 = 3,
    x32 = 4,
};

} // namespace akd1500::spi

namespace akd1500 {

static SpiFlashRuntimeConfig default_spi_flash_runtime_config() {
    SpiFlashRuntimeConfig config;
    config.read_opcode = 0x6Bu;
    config.transfer_type = 0x0u;
    config.wait_cycles = 0x8u;
    config.mode_bits_enabled = false;
    config.mode_bits_value = 0x00u;
    return config;
}

static void prime_spi_flash_control_signals(HardwareDriver* driver) {
    ScopedSpiTraceContext trace_scope("akd1500::prime_control_signals");
    constexpr uint32_t kSysConfigControlSignalsReg = 0xFCE00018u;
    // Datasheet defaults relevant to the SPI flash runtime path:
    // - EN_SPI_S2M = 0
    // - SPIM_DQS_PD = 1
    // - SPIM_DI_SWAP = 1
    constexpr uint32_t kExpectedControlSignals = 0x00220000u;
    driver->write32(kSysConfigControlSignalsReg, kExpectedControlSignals);
}

static void init_spi_flash(HardwareDriver* driver,
                           const SpiFlashRuntimeConfig& config) {
    ScopedSpiTraceContext trace_scope("akd1500::init_spi_flash");
    constexpr uint32_t kSysConfigControlSignalsReg = 0xFCE00018u;
    constexpr uint32_t kSpiMasterCfgBase = 0xFCF20000u;
    constexpr uint32_t kSpiMasterCtrlr0 = kSpiMasterCfgBase + 0x00u;
    constexpr uint32_t kSpiMasterSsienr = kSpiMasterCfgBase + 0x08u;
    constexpr uint32_t kSpiMasterSer = kSpiMasterCfgBase + 0x10u;
    constexpr uint32_t kSpiMasterBaudr = kSpiMasterCfgBase + 0x14u;
    constexpr uint32_t kSpiMasterSpiCtrlr0 = kSpiMasterCfgBase + 0xF4u;
    constexpr uint32_t kSpiMasterDdrDriveEdge = kSpiMasterCfgBase + 0xF8u;
    constexpr uint32_t kSpiMasterXipModeBits = kSpiMasterCfgBase + 0xFCu;
    constexpr uint32_t kSpiMasterXipIncrInst = kSpiMasterCfgBase + 0x100u;

    constexpr RegDetail kCtrlEnSpiS2m(16);
    constexpr RegDetail kCtrlSpimDiSwap(21);
    constexpr RegDetail kCtrlr0Dfs(0, 4);
    constexpr RegDetail kCtrlr0SpiFrf(22, 23);
    constexpr RegDetail kCtrlr0SsiIsMst(31);
    constexpr RegDetail kSsienrEnable(0);
    constexpr RegDetail kBaudrSckdv(1, 15);
    constexpr RegDetail kSpiCtrlr0TransType(0, 1);
    constexpr RegDetail kSpiCtrlr0AddrL(2, 5);
    constexpr RegDetail kSpiCtrlr0XipMdBitEn(7);
    constexpr RegDetail kSpiCtrlr0InstL(8, 9);
    constexpr RegDetail kSpiCtrlr0WaitCycles(11, 15);
    constexpr RegDetail kSpiCtrlr0SpiDdrEn(16);
    constexpr RegDetail kSpiCtrlr0InstDdrEn(17);
    constexpr RegDetail kSpiCtrlr0XipInstEn(20);
    constexpr RegDetail kDdrDriveEdgeTde(0, 7);
    constexpr RegDetail kXipIncrInst(0, 15);

    uint32_t reg = driver->read32(kSysConfigControlSignalsReg);
    set_field(&reg, kCtrlSpimDiSwap, 1);
    set_field(&reg, kCtrlEnSpiS2m, 0);
    driver->write32(kSysConfigControlSignalsReg, reg);

    reg = 0;
    set_field(&reg, kSsienrEnable, 0);
    driver->write32(kSpiMasterSsienr, reg);

    reg = 0;
    set_field(&reg, kCtrlr0SsiIsMst, 1);
    set_field(&reg, kCtrlr0SpiFrf, 0x2);
    set_field(&reg, kCtrlr0Dfs, 0x1f);
    driver->write32(kSpiMasterCtrlr0, reg);

    driver->write32(kSpiMasterSer, 1u << 0);

    reg = 0;
    set_field(&reg, kBaudrSckdv, 0x02);
    driver->write32(kSpiMasterBaudr, reg);

    reg = 0;
    set_field(&reg, kSpiCtrlr0TransType, config.transfer_type & 0x3u);
    set_field(&reg, kSpiCtrlr0AddrL, 0x6);
    set_field(&reg, kSpiCtrlr0XipMdBitEn, config.mode_bits_enabled ? 1u : 0u);
    set_field(&reg, kSpiCtrlr0InstL, 0x2);
    set_field(&reg, kSpiCtrlr0WaitCycles, config.wait_cycles & 0x1Fu);
    set_field(&reg, kSpiCtrlr0SpiDdrEn, 0);
    set_field(&reg, kSpiCtrlr0InstDdrEn, 0);
    set_field(&reg, kSpiCtrlr0XipInstEn, 1);
    driver->write32(kSpiMasterSpiCtrlr0, reg);

    reg = 0;
    set_field(&reg, kDdrDriveEdgeTde, 0);
    driver->write32(kSpiMasterDdrDriveEdge, reg);

    reg = 0;
    driver->write32(kSpiMasterXipModeBits,
                    static_cast<uint32_t>(config.mode_bits_value));

    reg = 0;
    set_field(&reg, kXipIncrInst, config.read_opcode);
    driver->write32(kSpiMasterXipIncrInst, reg);

    reg = 0;
    set_field(&reg, kSsienrEnable, 1);
    driver->write32(kSpiMasterSsienr, reg);
}

} // namespace akd1500

Akd1500SpiDriver::Akd1500SpiDriver(AbstractSpiDriver *spi_driver,
                                   uint32_t akida_visible_memory_base,
                                   uint32_t akida_visible_memory_size)
    : spi_driver_(spi_driver),
      akida_visible_memory_base_(akida_visible_memory_base),
      akida_visible_memory_size_(akida_visible_memory_size),
      flash_runtime_config_(akd1500::default_spi_flash_runtime_config()) {
    // make sure we deassert line, or this could cause issues
    spi_driver_->chip_select(akd1500::spi::kSlaveID, false);
    akd1500::prime_spi_flash_control_signals(this);
    akd1500::init_spi_flash(this, flash_runtime_config_);
}

void Akd1500SpiDriver::set_spi_flash_runtime_config(
    const SpiFlashRuntimeConfig& config) {
    flash_runtime_config_ = config;
}

void Akd1500SpiDriver::reinit_spi_flash_runtime() {
    ScopedSpiTraceContext trace_scope(
        "Akd1500SpiDriver::reinit_spi_flash_runtime");
    akd1500::prime_spi_flash_control_signals(this);
    akd1500::init_spi_flash(this, flash_runtime_config_);
}

static inline uint32_t to_spi_address(const uint32_t address) {
    // SPI addresses are 24 bits: it can only access 32 bits aligned addresses
    // in 0xfcxxxxxx range. To convert a 24 bits address to a 32 bits one,
    // AKD1500 SPI slave left shifts by 2 then adds 0xfc000000. To do the
    // opposite action we can substract 0xfc000000 then right shift by 2. This
    // is equivalent to right shift by 2, then discard the most significant byte
    assert((address >> 24) == 0xfc &&
           "SPI Slave can only access 0xfcxxxxxx memory space");
    assert((address & 0b11) == 0 &&
           "SPI Slave can only access 32 bits aligned addresses");
    return (address >> 2u) & 0x00ffffff;
}

static inline bool is_spi_master_memory_window(uint32_t address) {
    return address >= 0xFC000000u && address <= 0xFC7FFFFFu;
}

template <int burst_word_size>
struct BurstTraits;

template <>
struct BurstTraits<1> {
    static constexpr uint8_t size_code = 0u;
    static constexpr akd1500::spi::BurstWordSize word_count =
        akd1500::spi::BurstWordSize::x1;
};

template <>
struct BurstTraits<4> {
    static constexpr uint8_t size_code = 1u;
    static constexpr akd1500::spi::BurstWordSize word_count =
        akd1500::spi::BurstWordSize::x4;
};

template <>
struct BurstTraits<8> {
    static constexpr uint8_t size_code = 2u;
    static constexpr akd1500::spi::BurstWordSize word_count =
        akd1500::spi::BurstWordSize::x8;
};

template <>
struct BurstTraits<16> {
    static constexpr uint8_t size_code = 3u;
    static constexpr akd1500::spi::BurstWordSize word_count =
        akd1500::spi::BurstWordSize::x16;
};

template <>
struct BurstTraits<32> {
    static constexpr uint8_t size_code = 4u;
    static constexpr akd1500::spi::BurstWordSize word_count =
        akd1500::spi::BurstWordSize::x32;
};

template <int burst_word_size>
constexpr uint8_t burst_size_code() {
    return BurstTraits<burst_word_size>::size_code;
}

static inline void spis_write_then_read(AbstractSpiDriver* driver,
                                        const uint8_t* tx, size_t tx_size,
                                        uint8_t* rx, size_t rx_size) {
    driver->chip_select(akd1500::spi::kSlaveID, true);
    if (tx != nullptr && tx_size > 0) {
        driver->write(tx, tx_size);
    }
    if (rx != nullptr && rx_size > 0) {
        driver->read(rx, rx_size);
    }
    driver->chip_select(akd1500::spi::kSlaveID, false);
}

static bool spis_poll_status(AbstractSpiDriver* driver, uint8_t cmd_status) {
    ScopedSpiTraceContext trace_scope("spis_poll_status");
    constexpr uint32_t kMaxPoll = 2000u;
    uint8_t last_status = 0u;
    for (uint32_t i = 0; i < kMaxPoll; ++i) {
        uint8_t cmd = cmd_status;
        // Datasheet sequence uses one dummy byte before status payload.
        std::array<uint8_t, 2> rx = {0, 0};
        spis_write_then_read(driver, &cmd, 1u, rx.data(), rx.size());
        last_status = rx[1];
        if (rx[1] == 0x80u) {
            return true;
        }
        spis_wait_us(kSpisStatusPollDelayUs);
    }
    AKD1500_LIBRARY_LOG(
        "[AKD1500][spis] status timeout cmd=0x%02X last=0x%02X polls=%lu\r\n",
        static_cast<unsigned>(cmd_status),
        static_cast<unsigned>(last_status),
        static_cast<unsigned long>(kMaxPoll));
    return false;
}

template <int burst_word_size>
static bool spis_read_spim_window_burst(AbstractSpiDriver* driver,
                                        uint32_t address, uint32_t* data) {
    ScopedSpiTraceContext trace_scope("spis_read_spim_window_burst");
    static_assert(burst_word_size == 1 || burst_word_size == 4 ||
                      burst_word_size == 8 || burst_word_size == 16 ||
                      burst_word_size == 32,
                  "invalid burst size");
    const uint8_t code = burst_size_code<burst_word_size>();
    const uint32_t spi_address = to_spi_address(address);

    // Read request command (datasheet 3.7.4.4): 0x20 + size_code + addr[23:0]
    const std::array<uint8_t, 4> req = {
        static_cast<uint8_t>(0x20u + code),
        static_cast<uint8_t>((spi_address >> 16) & 0xFFu),
        static_cast<uint8_t>((spi_address >> 8) & 0xFFu),
        static_cast<uint8_t>(spi_address & 0xFFu),
    };
    spis_write_then_read(driver, req.data(), req.size(), nullptr, 0u);
    spis_wait_us(kSpisRequestSettleDelayUs);

    // Poll read-status command (0x48) until 0x80.
    if (!spis_poll_status(driver, 0x48u)) {
        AKD1500_LIBRARY_LOG(
            "[AKD1500][spis] read-window status failed addr=0x%08lX burst=%u\r\n",
            static_cast<unsigned long>(address),
            static_cast<unsigned>(burst_word_size));
        return false;
    }

    // Read1n command (datasheet example: 0x06 for 8 words => 0x04 + size_code)
    const uint8_t read1n_cmd = static_cast<uint8_t>(0x04u + code);
    std::array<uint8_t, burst_word_size * sizeof(uint32_t) + 1> raw = {};
    spis_write_then_read(driver, &read1n_cmd, 1u, raw.data(), raw.size());

    // Skip first dummy byte.
    const uint8_t* payload = raw.data() + 1u;
    for (size_t i = 0; i < burst_word_size; ++i) {
        uint32_t w = 0;
        std::memcpy(&w, payload + i * sizeof(uint32_t), sizeof(uint32_t));
        data[i] = __builtin_bswap32(w);
    }
    return true;
}

template <int burst_word_size>
static bool spis_write_spim_window_burst(AbstractSpiDriver* driver,
                                         uint32_t address,
                                         const uint32_t* data) {
    ScopedSpiTraceContext trace_scope("spis_write_spim_window_burst");
    static_assert(burst_word_size == 1 || burst_word_size == 4 ||
                      burst_word_size == 8 || burst_word_size == 16 ||
                      burst_word_size == 32,
                  "invalid burst size");
    const uint8_t code = burst_size_code<burst_word_size>();
    const uint32_t spi_address = to_spi_address(address);

    // Write command (datasheet 3.7.4.2): 0x80 + size_code + addr[23:0] + payload
    std::array<uint32_t, burst_word_size + 1> packet = {};
    uint8_t* p = reinterpret_cast<uint8_t*>(packet.data());
    p[0] = static_cast<uint8_t>(0x80u + code);
    p[1] = static_cast<uint8_t>((spi_address >> 16) & 0xFFu);
    p[2] = static_cast<uint8_t>((spi_address >> 8) & 0xFFu);
    p[3] = static_cast<uint8_t>(spi_address & 0xFFu);
    for (size_t i = 0; i < burst_word_size; ++i) {
        packet[i + 1] = __builtin_bswap32(data[i]);
    }
    spis_write_then_read(driver, p, packet.size() * sizeof(uint32_t), nullptr, 0u);
    spis_wait_us(kSpisRequestSettleDelayUs);

    // Poll write-status command (0xC8) until 0x80.
    if (!spis_poll_status(driver, 0xC8u)) {
        AKD1500_LIBRARY_LOG(
            "[AKD1500][spis] write-window status failed addr=0x%08lX burst=%u\r\n",
            static_cast<unsigned long>(address),
            static_cast<unsigned>(burst_word_size));
        return false;
    }
    return true;
}

template <int burst_word_size>
constexpr akd1500::spi::BurstWordSize data_word_count_from_burst() {
    return BurstTraits<burst_word_size>::word_count;
}

template <int burst_word_size>
static void spi_write_burst(AbstractSpiDriver *driver, uint32_t address,
                            const uint32_t *data);

template <int burst_word_size>
static void spi_read_burst(AbstractSpiDriver *driver, uint32_t address,
                           uint32_t *data);

template <akd1500::spi::Commands command, int burst_word_size>
struct BurstIo;

template <int burst_word_size>
struct BurstIo<akd1500::spi::Commands::Write, burst_word_size> {
    template <typename buffer>
    static inline void transfer(AbstractSpiDriver* driver, uint32_t address,
                                buffer data) {
        spi_write_burst<burst_word_size>(driver, address, data);
    }
};

template <int burst_word_size>
struct BurstIo<akd1500::spi::Commands::Read, burst_word_size> {
    template <typename buffer>
    static inline void transfer(AbstractSpiDriver* driver, uint32_t address,
                                buffer data) {
        spi_read_burst<burst_word_size>(driver, address, data);
    }
};
void spi_header(akd1500::spi::Commands command,
                akd1500::spi::BurstWordSize word_count, const uint32_t address,
                uint8_t *buffer) {
    // 1st byte is command + burst size
    buffer[0] =
        static_cast<uint8_t>(command) | static_cast<uint8_t>(word_count);
    // next 3 bytes are address, MSB first
    const auto spi_address = to_spi_address(address);
    buffer[1] = (spi_address >> 16) & 0xff;
    buffer[2] = (spi_address >> 8) & 0xff;
    buffer[3] = spi_address & 0xff;
}

template <int word_size> inline constexpr size_t words_to_bytes_size() {
    return word_size * sizeof(uint32_t); // SPI uses 32 bits words unit
}

template <int burst_word_size>
static void spi_write_burst(AbstractSpiDriver *driver, uint32_t address,
                            const uint32_t *data) {
    ScopedSpiTraceContext trace_scope("spi_write_burst");
    // toggle slave line ON
    driver->chip_select(akd1500::spi::kSlaveID, true);

    // when writing, we put the header in the same buffer as the data to perform
    // a single write, so we have 1 more word in the burst data
    std::array<uint32_t, burst_word_size + 1> burst_data;
    auto *u8_data = reinterpret_cast<uint8_t *>(burst_data.data());
    spi_header(akd1500::spi::Commands::Write,
               data_word_count_from_burst<burst_word_size>(), address, u8_data);

    // now we perform the bytes swap
    for (size_t i = 0; i < burst_word_size; ++i) {
        burst_data[i + 1] = __builtin_bswap32(data[i]);
    }

    // then write all at once
    driver->write(u8_data, burst_data.size() * sizeof(uint32_t));

    // toggle slave line OFF
    driver->chip_select(akd1500::spi::kSlaveID, false);
}

static inline bool is_single_word_fc_register_write(uint32_t address,
                                                    size_t size) {
    return size == sizeof(uint32_t) &&
           !is_spi_master_memory_window(address) &&
           ((address & 0x3u) == 0u);
}

static void spi_write_single_word_stm32_like(AbstractSpiDriver* driver,
                                             uint32_t address,
                                             uint32_t word,
                                             const char* caller_context) {
    ScopedSpiTraceContext trace_scope("spi_write_single_word_stm32_like");
    std::array<uint32_t, 2> burst_data = {0u, 0u};
    auto* u8_data = reinterpret_cast<uint8_t*>(burst_data.data());
    spi_header(akd1500::spi::Commands::Write,
               akd1500::spi::BurstWordSize::x1, address, u8_data);
    burst_data[1] = __builtin_bswap32(word);
#if AKIDA_NICLA_HRC_LAUNCH_REG_TRACE
    const uint64_t t0 = trace_now_us();
#endif
    driver->chip_select(akd1500::spi::kSlaveID, true);
#if AKIDA_NICLA_HRC_LAUNCH_REG_TRACE
    const uint64_t t1 = trace_now_us();
#endif
    driver->write(u8_data, burst_data.size() * sizeof(uint32_t));
#if AKIDA_NICLA_HRC_LAUNCH_REG_TRACE
    const uint64_t t2 = trace_now_us();
#endif
    driver->chip_select(akd1500::spi::kSlaveID, false);
#if AKIDA_NICLA_HRC_LAUNCH_REG_TRACE
    const uint64_t t3 = trace_now_us();
    trace_launch_reg_access("write32", caller_context, address, u8_data,
                            burst_data.size() * sizeof(uint32_t), nullptr, 0u,
                            t0, t1, t2, t2, t3);
#else
    (void)caller_context;
#endif
}

static void spi_read_single_word_traced(AbstractSpiDriver* driver,
                                        uint32_t address, uint32_t* data,
                                        const char* caller_context) {
    ScopedSpiTraceContext trace_scope("spi_read_single_word_traced");
    std::array<uint8_t, 5> header = {};
    spi_header(akd1500::spi::Commands::Read,
               akd1500::spi::BurstWordSize::x1, address, header.data());
    header[4] = 0u;
    uint32_t burst_word = 0u;
    uint8_t* raw = reinterpret_cast<uint8_t*>(&burst_word);
#if AKIDA_NICLA_HRC_LAUNCH_REG_TRACE
    const uint64_t t0 = trace_now_us();
#endif
    driver->chip_select(akd1500::spi::kSlaveID, true);
#if AKIDA_NICLA_HRC_LAUNCH_REG_TRACE
    const uint64_t t1 = trace_now_us();
#endif
    driver->write(header.data(), header.size());
#if AKIDA_NICLA_HRC_LAUNCH_REG_TRACE
    const uint64_t t2 = trace_now_us();
#endif
    driver->read(raw, sizeof(uint32_t));
#if AKIDA_NICLA_HRC_LAUNCH_REG_TRACE
    const uint64_t t3 = trace_now_us();
#endif
    driver->chip_select(akd1500::spi::kSlaveID, false);
#if AKIDA_NICLA_HRC_LAUNCH_REG_TRACE
    const uint64_t t4 = trace_now_us();
#endif
    *data = __builtin_bswap32(burst_word);
#if AKIDA_NICLA_HRC_LAUNCH_REG_TRACE
    trace_launch_reg_access("read32", caller_context, address, header.data(),
                            header.size(), raw, sizeof(uint32_t), t0, t1, t2,
                            t3, t4);
#else
    (void)caller_context;
#endif
}

template <int burst_word_size>
static void spi_read_burst(AbstractSpiDriver *driver, uint32_t address,
                           uint32_t *data) {
    ScopedSpiTraceContext trace_scope("spi_read_burst");
    // toggle slave line ON
    driver->chip_select(akd1500::spi::kSlaveID, true);

    // write header
    std::array<uint8_t, 5> header;
    spi_header(akd1500::spi::Commands::Read,
               data_word_count_from_burst<burst_word_size>(), address,
               header.data());
    header[4] = 0; // read requires to wait 8 spi clocks before response, so we
                   // insert a dummy byte that will delay the read accordingly
    driver->write(header.data(), header.size());
    std::array<uint32_t, burst_word_size> burst_data;
    driver->read(reinterpret_cast<uint8_t *>(burst_data.data()),
                 burst_data.size() * sizeof(uint32_t));

    // now we perform the bytes swap
    for (size_t i = 0; i < burst_word_size; ++i) {
        data[i] = __builtin_bswap32(burst_data[i]);
    }

    // toggle slave line OFF
    driver->chip_select(akd1500::spi::kSlaveID, false);
}

template <akd1500::spi::Commands command, int burst_word_size, typename buffer>
static inline void loop_bursts(AbstractSpiDriver *driver, uint32_t *address,
                               buffer *data, size_t *word_size) {
    // just loop until we cannot burst with the remaining size
    while (*word_size >= burst_word_size) {
        BurstIo<command, burst_word_size>::transfer(driver, *address, *data);
        *word_size -= burst_word_size;
        *address +=
            static_cast<uint32_t>(words_to_bytes_size<burst_word_size>());
        *data += burst_word_size;
    }
}

template <akd1500::spi::Commands command, typename buffer>
static inline void spi_op(AbstractSpiDriver *driver, uint32_t address,
                          buffer data, size_t word_size) {
    // buffer is template, but it is used only to have the same code to both
    // const uint32_t* and uint32_t* variants
    static_assert(
        std::is_same<typename std::remove_const<
                         typename std::remove_pointer<buffer>::type>::type,
                     uint32_t>::value,
        "buffer type should be uint32_t");
    // read or write using the highest burst size until there is no data
    loop_bursts<command, 32>(driver, &address, &data, &word_size);
    loop_bursts<command, 16>(driver, &address, &data, &word_size);
    loop_bursts<command, 8>(driver, &address, &data, &word_size);
    loop_bursts<command, 4>(driver, &address, &data, &word_size);
    loop_bursts<command, 1>(driver, &address, &data, &word_size);
}

template <int burst_word_size, typename buffer>
static inline bool loop_bursts_spim_window_read(AbstractSpiDriver* driver,
                                                uint32_t* address, buffer* data,
                                                size_t* word_size) {
    while (*word_size >= burst_word_size) {
        if (!spis_read_spim_window_burst<burst_word_size>(driver, *address, *data)) {
            return false;
        }
        *word_size -= burst_word_size;
        *address += static_cast<uint32_t>(words_to_bytes_size<burst_word_size>());
        *data += burst_word_size;
    }
    return true;
}

template <int burst_word_size, typename buffer>
static inline bool loop_bursts_spim_window_write(AbstractSpiDriver* driver,
                                                 uint32_t* address, buffer* data,
                                                 size_t* word_size) {
    while (*word_size >= burst_word_size) {
        if (!spis_write_spim_window_burst<burst_word_size>(driver, *address, *data)) {
            return false;
        }
        *word_size -= burst_word_size;
        *address += static_cast<uint32_t>(words_to_bytes_size<burst_word_size>());
        *data += burst_word_size;
    }
    return true;
}

template <typename buffer>
static inline bool spi_spim_window_read(AbstractSpiDriver* driver, uint32_t address,
                                        buffer data, size_t word_size) {
    return loop_bursts_spim_window_read<32>(driver, &address, &data, &word_size) &&
           loop_bursts_spim_window_read<16>(driver, &address, &data, &word_size) &&
           loop_bursts_spim_window_read<8>(driver, &address, &data, &word_size) &&
           loop_bursts_spim_window_read<4>(driver, &address, &data, &word_size) &&
           loop_bursts_spim_window_read<1>(driver, &address, &data, &word_size);
}

template <typename buffer>
static inline bool spi_spim_window_write(AbstractSpiDriver* driver, uint32_t address,
                                         buffer data, size_t word_size) {
    return loop_bursts_spim_window_write<32>(driver, &address, &data, &word_size) &&
           loop_bursts_spim_window_write<16>(driver, &address, &data, &word_size) &&
           loop_bursts_spim_window_write<8>(driver, &address, &data, &word_size) &&
           loop_bursts_spim_window_write<4>(driver, &address, &data, &word_size) &&
           loop_bursts_spim_window_write<1>(driver, &address, &data, &word_size);
}

void Akd1500SpiDriver::read(uint32_t address, void *data, size_t size) const {
    const char* caller_context = spi_trace_context();
    ScopedSpiTraceContext trace_scope("Akd1500SpiDriver::read");
    if (size == 0u || data == nullptr) {
        return;
    }

    if (is_hrc_launch_reg_trace_target(address, size)) {
        spi_read_single_word_traced(spi_driver_, address,
                                    reinterpret_cast<uint32_t*>(data),
                                    caller_context);
        return;
    }

    // AKD1500 SPI have transfers aligned to 32 bits
    const auto nb_32b_words = size / sizeof(uint32_t);
    const auto unaligned_bytes = size % sizeof(uint32_t);

    // Read full words
    if (is_spi_master_memory_window(address) &&
        is_spi_master_memory_window(address + static_cast<uint32_t>(size - 1))) {
        if (!spi_spim_window_read(spi_driver_, address,
                                  reinterpret_cast<uint32_t*>(data), nb_32b_words)) {
            AKD1500_LIBRARY_LOG(
                "[AKD1500][spis] read aborted addr=0x%08lX size=%lu\r\n",
                static_cast<unsigned long>(address),
                static_cast<unsigned long>(size));
            return;
        }
    } else {
        spi_op<akd1500::spi::Commands::Read>(
            spi_driver_, address, reinterpret_cast<uint32_t *>(data), nb_32b_words);
    }

    // then handle non aligned bytes
    if (unaligned_bytes > 0) {
        uint32_t word;
        // read a full word
        const uint32_t tail_addr =
            address + static_cast<uint32_t>(nb_32b_words * sizeof(uint32_t));
        if (is_spi_master_memory_window(tail_addr)) {
            if (!spis_read_spim_window_burst<1>(spi_driver_, tail_addr, &word)) {
                AKD1500_LIBRARY_LOG(
                    "[AKD1500][spis] read tail aborted addr=0x%08lX size=%lu\r\n",
                    static_cast<unsigned long>(tail_addr),
                    static_cast<unsigned long>(size));
                return;
            }
        } else {
            spi_read_burst<1>(spi_driver_, tail_addr, &word);
        }

        // then put relevant data at the correct location in the bytes buffer
        auto *u8_data = reinterpret_cast<uint8_t *>(data);
        if (unaligned_bytes == 1) {
            u8_data[size - 1] = static_cast<uint8_t>(word & 0xFF);
        } else if (unaligned_bytes == 2) {
            u8_data[size - 1] = static_cast<uint8_t>((word >> 8) & 0xFF);
            u8_data[size - 2] = static_cast<uint8_t>(word & 0xFF);
        } else if (unaligned_bytes == 3) {
            u8_data[size - 1] = static_cast<uint8_t>((word >> 16) & 0xFF);
            u8_data[size - 2] = static_cast<uint8_t>((word >> 8) & 0xFF);
            u8_data[size - 3] = static_cast<uint8_t>(word & 0xFF);
        }
    }
}

void Akd1500SpiDriver::write(uint32_t address, const void *data, size_t size) {
    const char* caller_context = spi_trace_context();
    ScopedSpiTraceContext trace_scope("Akd1500SpiDriver::write");
    if (size == 0u || data == nullptr) {
        return;
    }

    if (is_single_word_fc_register_write(address, size)) {
        spi_write_single_word_stm32_like(
            spi_driver_, address, *reinterpret_cast<const uint32_t*>(data),
            caller_context);
        return;
    }

    // AKD1500 SPI have transfers aligned to 32 bits
    const auto nb_32b_words = size / sizeof(uint32_t);
    const auto unaligned_bytes = size % sizeof(uint32_t);

    // transfer full 32 bits words (they will get bytes swapped during spi
    // burst)
    if (is_spi_master_memory_window(address) &&
        is_spi_master_memory_window(address + static_cast<uint32_t>(size - 1))) {
        if (!spi_spim_window_write(spi_driver_, address,
                                   reinterpret_cast<const uint32_t*>(data), nb_32b_words)) {
            AKD1500_LIBRARY_LOG(
                "[AKD1500][spis] write aborted addr=0x%08lX size=%lu\r\n",
                static_cast<unsigned long>(address),
                static_cast<unsigned long>(size));
            return;
        }
    } else {
        spi_op<akd1500::spi::Commands::Write>(
            spi_driver_, address, reinterpret_cast<const uint32_t *>(data),
            nb_32b_words);
    }

    // then handle non aligned bytes
    if (unaligned_bytes > 0) {
        uint32_t word;
        // write a full word from unaligned data
        const auto *u8_data = reinterpret_cast<const uint8_t *>(data);
        if (unaligned_bytes == 1) {
            word = static_cast<uint32_t>(u8_data[size - 1]);
        } else if (unaligned_bytes == 2) {
            word = static_cast<uint32_t>(u8_data[size - 2] |
                                         (u8_data[size - 1] << 8));
        } else if (unaligned_bytes == 3) {
            word = static_cast<uint32_t>(u8_data[size - 3] |
                                         (u8_data[size - 2] << 8) |
                                         (u8_data[size - 1] << 16));
        }
        const uint32_t tail_addr =
            address + static_cast<uint32_t>(nb_32b_words * sizeof(uint32_t));
        if (is_spi_master_memory_window(tail_addr)) {
            if (!spis_write_spim_window_burst<1>(spi_driver_, tail_addr, &word)) {
                AKD1500_LIBRARY_LOG(
                    "[AKD1500][spis] write tail aborted addr=0x%08lX size=%lu\r\n",
                    static_cast<unsigned long>(tail_addr),
                    static_cast<unsigned long>(size));
                return;
            }
        } else {
            spi_write_burst<1>(spi_driver_, tail_addr, &word);
        }
    }
    // debug
}
} // namespace akida
