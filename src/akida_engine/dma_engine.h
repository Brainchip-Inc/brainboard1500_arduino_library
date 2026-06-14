#pragma once

#include <cstdint>

#include "engine/dma.h"

namespace akida {

namespace dma {

struct Engine {
 public:
  explicit Engine(uint32_t reg_base_addr, uint32_t desc_bytes_size);

  dma::addr descriptor_base_addr;
  const uint32_t descriptor_bytes_size;
  const uint32_t reg_base_addr;
};

struct Config {
  Engine engine;
};

struct Inputs {
  Engine engine;
};

}  // namespace dma

}  // namespace akida
