#pragma once

#include "engine/dma.h"

namespace akida {

struct InferenceMemory {
  dma::addr inputs_base_address = 0;
  dma::addr outputs_base_address = 0;
};

}  // namespace akida
