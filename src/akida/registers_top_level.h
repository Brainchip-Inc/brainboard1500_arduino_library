#pragma once

#include <cstdint>
#include "infra/registers_common.h"

namespace akida {

static constexpr uint32_t REG_IP_VERSION = 0x0;
static constexpr RegDetail MINOR_REV(0, 7);
static constexpr RegDetail MAJOR_REV(8, 15);
static constexpr RegDetail PROD_ID(16, 23);
static constexpr RegDetail VENDOR_ID(24, 31);

static constexpr uint32_t REG_GENERAL_CONTROL = 0x4;
static constexpr RegDetail REWIND_MODE(0);
static constexpr RegDetail PR_MESH_RST_END(1);
static constexpr RegDetail AK_LOGIC_RST(8);
static constexpr RegDetail AK_CORE_RST(9);
static constexpr RegDetail AK_MESH_RST(10);
static constexpr RegDetail SCC_CORE_RESET(12);
static constexpr RegDetail AK_CORE_CLKPD(16);
static constexpr RegDetail AK_C2C_USP_CLKPD(17);
static constexpr RegDetail AK_C2C_DSP_CLKPD(18);
static constexpr RegDetail SCC_CORE_CLKPD(20);

// Mesh info registers definition
static constexpr uint32_t REG_MESH_INFO1 = 0x50;
static constexpr RegDetail MESH_ROWS(0, 7);
static constexpr RegDetail MESH_COLS(8, 15);
static constexpr RegDetail R1_START_COL(16, 23);
static constexpr RegDetail R2_START_COL(24, 31);
static constexpr uint32_t REG_MESH_INFO2 = 0x54;
static constexpr RegDetail NP_PER_NODE(0, 2);
static constexpr RegDetail DMA_NODE_EMPTY(4);
static constexpr RegDetail DMA_NODE_ROW(8, 15);
static constexpr RegDetail DMA_NODE_COL(16, 23);
static constexpr RegDetail DMA_AE_NP(24, 27);
static constexpr RegDetail DMA_CFG_NP(28, 31);
static constexpr uint32_t REG_MESH_INFO3 = 0x58;
static constexpr RegDetail FNP2_ROW(0, 7);
static constexpr RegDetail FNP2_COL(8, 15);
static constexpr RegDetail FNP2_NUM(16, 17);
static constexpr RegDetail COL_NUM_LAST_NP(24, 31);

// Interrupt controller registers
static constexpr uint32_t INTERRUPT_CONTROLLER_OFFSET = 0x70000;
static constexpr uint32_t REG_INTERRUPT_CONTROLLER_GENERAL_CONTROL =
    INTERRUPT_CONTROLLER_OFFSET + 0x0;
static constexpr RegDetail INTERRUPT_CONTROLLER_GENERAL_CONTROL_GLB_INT_EN(0);
static constexpr uint32_t REG_INTERRUPT_CONTROLLER_SOURCE_MASK =
    INTERRUPT_CONTROLLER_OFFSET + 0x4;
static constexpr RegDetail REG_INTERRUPT_CONTROLLER_SOURCE_MASK_AEDMA(0);
static constexpr RegDetail REG_INTERRUPT_CONTROLLER_SOURCE_MASK_AEIF(1);
static constexpr RegDetail REG_INTERRUPT_CONTROLLER_SOURCE_MASK_CFGDMA(2);
static constexpr RegDetail REG_INTERRUPT_CONTROLLER_SOURCE_MASK_CFGIF(3);
static constexpr RegDetail REG_INTERRUPT_CONTROLLER_SOURCE_MASK_SCC_HRC(5);
static constexpr uint32_t REG_INTERRUPT_CONTROLLER_SOURCE =
    INTERRUPT_CONTROLLER_OFFSET + 0x8;
static constexpr RegDetail REG_INTERRUPT_CONTROLLER_SOURCE_AEDMA(0);
static constexpr RegDetail REG_INTERRUPT_CONTROLLER_SOURCE_AEIF(1);
static constexpr RegDetail REG_INTERRUPT_CONTROLLER_SOURCE_CFGDMA(2);
static constexpr RegDetail REG_INTERRUPT_CONTROLLER_SOURCE_CFGIF(3);
static constexpr RegDetail REG_INTERRUPT_CONTROLLER_SOURCESCC_HRC(5);

}  // namespace akida
