#include "akida/tensor.h"

#include <memory>

#include "akida/dense.h"
#include "akida/sparse.h"

#include "infra/system.h"

namespace akida {

DenseConstPtr Tensor::as_dense(TensorConstPtr tensor) {
  if (tensor && tensor->tensor_class() == Tensor::Class::Dense) {
    return std::static_pointer_cast<const Dense>(tensor);
  }
  return nullptr;
}

SparseConstPtr Tensor::as_sparse(TensorConstPtr tensor) {
  if (tensor && tensor->tensor_class() == Tensor::Class::Sparse) {
    return std::static_pointer_cast<const Sparse>(tensor);
  }
  return nullptr;
}

DenseConstPtr Tensor::ensure_dense(TensorConstPtr tensor) {
  // Assume this is already a Dense
  auto dense = Tensor::as_dense(tensor);
  if (dense) {
    return dense;
  }
  // If we were passed a Sparse, convert it to a Dense
  auto sparse = Tensor::as_sparse(tensor);
  if (sparse) {
    return Dense::from_sparse(*sparse, Dense::Layout::RowMajor);
  }
  return nullptr;
}

}  // namespace akida
