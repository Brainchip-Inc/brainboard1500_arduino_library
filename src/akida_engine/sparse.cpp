#include "akida/sparse.h"

#include <algorithm>
#include <cstdint>

#include "akida/shape.h"
#include "akida/tensor.h"

namespace akida {

bool Sparse::operator==(const Tensor& ref) const {
  // We cannot compare Sparse easily, so we first to convert the ref to a dense
  if (ref.tensor_class() == Tensor::Class::Dense) {
    // We can use the Dense operator directly
    return static_cast<const Dense&>(ref) == *this;
  }
  // As a fallback, we create a ColMajor Dense clone
  auto dense_clone = Dense::from_sparse(*this, Dense::Layout::ColMajor);
  // return Dense comparison
  return *dense_clone == ref;
}

}  // namespace akida
