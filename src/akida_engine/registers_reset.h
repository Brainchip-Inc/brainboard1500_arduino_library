#pragma once

#include <cstdint>
#include "infra/registers_common.h"

namespace akida {

// Reset control registers
static constexpr uint32_t REG_CLOCK_RESET_CRG11_BASE = 0xf0001000;

// Akida IP reset register (this is not the SoC system reset register, for that
// look at 0x1020 and 0x1024)
static constexpr uint32_t REG_AKIDA_CLOCK_RESET_CTRL =
    REG_CLOCK_RESET_CRG11_BASE + 0x210;
static constexpr RegDetail AKIDA_NP_RESET(0);
static constexpr RegDetail AKIDA_NP_LOGIC_RESET(1);
static constexpr RegDetail AKIDA_SCC_RESET(4);
static constexpr RegDetail AKIDA_CORE_CLKPD(16);
static constexpr RegDetail AKIDA_APB_CLKPD(17);
static constexpr RegDetail AKIDA_SCC_CLKPD(18);
static constexpr RegDetail AKIDA_RTC_CLKPD(20);
static constexpr RegDetail AKIDA_LPDDR_CLKPD(21);

}  // namespace akida
