#pragma once

#include <cstdint>

#include "akida/np.h"
#include "engine/dma.h"

namespace akida {

namespace dma {

enum class Target {
  CnpFilter,
  CnpLearnThres,
  CnpFireThres,
  FnpWeights,
  NpRegisters,
  HrcRegisters,
  HrcSram,
};

// Return vector of 2 elements containing the DMA header
wbuffer format_config_header(const struct np::Ident& np, Target target,
                             uint32_t size, uint16_t dest_addr,
                             bool xl = false);

uint32_t parse_config_read_size(const wbuffer& read_header);

bool config_block_size_needs_xl(uint32_t block_size);

// Size of address
constexpr uint32_t kXlIncrementSz = 16;
constexpr uint32_t kConfigReadPacketHdrSz = 1;
constexpr uint32_t kConfigReadPacketOffset = 32;
constexpr uint32_t kConfigWriteHdrWordLen = 2;
constexpr uint32_t kConfigWritePacketOffset =
    kConfigWriteHdrWordLen * sizeof(dma::w32);

}  // namespace dma

}  // namespace akida
