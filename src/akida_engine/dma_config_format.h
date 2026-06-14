#pragma once

#include <cstdint>
#include "infra/registers_common.h"

namespace akida {
namespace dma {

// DMA header format
static constexpr uint32_t HDR_WORD1 = 0x0;
static constexpr RegDetail HDR_NP_COL(24, 31);
static constexpr RegDetail HDR_NP_ROW(16, 23);
static constexpr RegDetail HDR_NP_DST(8, 11);
static constexpr RegDetail HDR_HRC_EN(7);
static constexpr RegDetail HDR_UID(0, 3);

static constexpr uint32_t HDR_WORD2 = 0x1;
static constexpr RegDetail HDR_XL(31);
static constexpr RegDetail HDR_BLOCK_LEN(16, 29);
static constexpr RegDetail HDR_START_ADDR(0, 15);

static constexpr uint8_t HDR_UID_CNP_FILTER = 0;
static constexpr uint8_t HDR_UID_CNP_LEARN_THRES = 2;
static constexpr uint8_t HDR_UID_CNP_THRES_FIRE = 4;
static constexpr uint8_t HDR_UID_FNP_WEIGHT = 6;
static constexpr uint8_t HDR_UID_NP_REGS = 8;
static constexpr uint8_t HDR_UID_HRC_SRAM = 0;
static constexpr uint8_t HDR_UID_HRC_REGS = 8;

// Read word
static constexpr uint32_t HDR_READ_WORD1 = 0x0;
static constexpr RegDetail HDR_READ_PACKET_SZ(0, 15);

}  // namespace dma
}  // namespace akida
