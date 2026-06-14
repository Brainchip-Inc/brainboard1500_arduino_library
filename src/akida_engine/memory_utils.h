#pragma once

#include <cassert>
#include <cstddef>

#include "engine/dma.h"
#include "infra/hardware_driver.h"

namespace akida {

inline dma::addr to_dma_addr(const void* id) {
  assert(sizeof(id) == sizeof(dma::addr));
  return static_cast<dma::addr>(reinterpret_cast<size_t>((id)));
}

bool accessible_from_akida(const void* id, const HardwareDriver& driver);

}  // namespace akida
