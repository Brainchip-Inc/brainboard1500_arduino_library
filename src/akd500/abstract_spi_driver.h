#pragma once

#include <cstddef>
#include <cstdint>

#ifndef AKIDA_NICLA_SPI_TRANSACTION_TRACE
#define AKIDA_NICLA_SPI_TRANSACTION_TRACE 0
#endif

#ifndef AKIDA_NICLA_SPI_CS_ASSERT_DELAY_US
#define AKIDA_NICLA_SPI_CS_ASSERT_DELAY_US 0
#endif

#ifndef AKIDA_NICLA_SPI_CS_DEASSERT_DELAY_US
#define AKIDA_NICLA_SPI_CS_DEASSERT_DELAY_US 0
#endif

#ifndef AKIDA_NICLA_HRC_LAUNCH_REG_TRACE
#define AKIDA_NICLA_HRC_LAUNCH_REG_TRACE 0
#endif

namespace akida {
const char* spi_trace_context();
void spi_trace_set_context(const char* context);

class ScopedSpiTraceContext {
 public:
#if AKIDA_NICLA_SPI_TRANSACTION_TRACE || AKIDA_NICLA_HRC_LAUNCH_REG_TRACE
  explicit ScopedSpiTraceContext(const char* context)
      : previous_(spi_trace_context()) {
    spi_trace_set_context(context);
  }

  ~ScopedSpiTraceContext() { spi_trace_set_context(previous_); }

 private:
  const char* previous_;
#else
  explicit ScopedSpiTraceContext(const char*) {}
#endif
};

class AbstractSpiDriver {
 public:
  virtual ~AbstractSpiDriver() {}
  virtual void read(uint8_t* data, size_t size) = 0;
  virtual void write(const uint8_t* data, size_t size) = 0;
  virtual void transfer(const uint8_t* tx, uint8_t* rx, size_t size) = 0;
  virtual void chip_select(uint32_t slave_ID, bool active) = 0;
};

class Stm32SpiDriver : public AbstractSpiDriver {
public:
    ~Stm32SpiDriver() {};
    void read(uint8_t* data, size_t size);
    void write(const uint8_t* data, size_t size);
    void transfer(const uint8_t* tx, uint8_t* rx, size_t size);
    void chip_select(uint32_t slave_ID, bool active);
    void set_chip_select_active_low(bool active_low);
};
}  // namespace akida
