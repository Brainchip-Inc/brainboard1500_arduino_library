#ifndef PRESENCE_CLASSIFIER_INPUTS_H_
#define PRESENCE_CLASSIFIER_INPUTS_H_

#include <cstdint>
#include "akida/shape.h"
#include "akida/tensor.h"

extern const unsigned char presence_inputs[];
extern const int64_t presence_inputs_len;
extern const akida::Shape presence_inputs_shape;
extern const akida::TensorType presence_inputs_type;

#endif
