#pragma once

#include <cstdint>

#include "infra/registers_common.h"

namespace akida {

// fields for word 1 cnp
static constexpr RegDetail CONV_X(0, 11);
static constexpr RegDetail CONV_Y(16, 27);
static constexpr RegDetail CONV_POTENTIAL_MSB(28, 31);
// fields for word 2 cnp
static constexpr RegDetail CONV_F(0, 10);
static constexpr RegDetail CONV_ACTIVATION(16, 23);
static constexpr RegDetail CONV_POTENTIAL_LSB(12, 31);
// fields for word 1 fnp
static constexpr RegDetail FC_F(0, 19);
// fields for word 2 fnp
static constexpr RegDetail FC_ACTIVATION(0, 19);  // potential is the same
static constexpr RegDetail FC_POLARITY(31);       // should be set to 1

// field for size
static constexpr RegDetail OUTPUT_WORD_SIZE(0, 27);

}  // namespace akida
