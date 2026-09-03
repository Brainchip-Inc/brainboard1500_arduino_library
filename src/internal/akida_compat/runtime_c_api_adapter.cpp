#include "internal/akida_compat/runtime_c_api_adapter.h"

#include <climits>
#include <cstring>
#include <utility>
#include <vector>

#include "akida/dense.h"
#include "akida/input_conversion.h"
#include "akida/tensor.h"
#include "internal/akida_compat/model_address.h"

namespace akida {
namespace compat {

RuntimeCAPIAdapter::RuntimeCAPIAdapter(HardwareDevice& device,
                                       const RuntimeCAPIConfig& config)
    : device_(device), config_(config) {}

void RuntimeCAPIAdapter::toggle_clock_counter(bool enable) {
  device_.toggle_clock_counter(enable);
}

uint32_t RuntimeCAPIAdapter::get_clock_counter() {
  return device_.read_clock_counter();
}

int RuntimeCAPIAdapter::program(uint8_t* buffer, size_t size, bool learn_en) {
  if (buffer == nullptr || size == 0u) {
    return failure();
  }

  if (current_program_ != nullptr) {
    device_.unprogram();
  }

  current_program_ = buffer;
  program_info_ = device_.program(buffer, size);
  if (!program_info_.is_valid()) {
    return failure();
  }

  if (current_learn_en_ != learn_en) {
    device_.toggle_learn(learn_en);
    current_learn_en_ = learn_en;
  }
  return success();
}

int RuntimeCAPIAdapter::program_only(uint8_t* buffer, size_t size) {
  if (buffer == nullptr || size == 0u) {
    return failure();
  }

  current_program_ = buffer;
  program_info_ = device_.program(buffer, size);
  return program_info_.is_valid() ? success() : failure();
}

int RuntimeCAPIAdapter::program_flash(uint8_t* program_info_buffer, size_t len,
                                      uint32_t flash_address,
                                      uint8_t* is_el_model) {
  if (program_info_buffer == nullptr || len == 0u) {
    if (is_el_model != nullptr) {
      *is_el_model = 0u;
    }
    return failure();
  }

  current_program_ = program_info_buffer;
  uint32_t program_data_address = flash_address;
  if (flash_address < kExternalModelWindowSize) {
    program_data_address = config_.flash_base_address + flash_address;
  } else {
    program_data_address = normalize_external_model_address(flash_address);
  }
  program_info_ = device_.program_external_data(program_info_buffer, len,
                                                program_data_address);
  if (!program_info_.is_valid()) {
    if (is_el_model != nullptr) {
      *is_el_model = 0u;
    }
    return failure();
  }

  if (is_el_model != nullptr) {
    *is_el_model = program_info_.can_learn() ? 1u : 0u;
  }
  return success();
}

int RuntimeCAPIAdapter::batch_size(int size, bool allocate_inputs) {
  return static_cast<int>(device_.set_batch_size(size, allocate_inputs));
}

int RuntimeCAPIAdapter::learn_mode(bool enable) {
  if (current_learn_en_ != enable) {
    device_.toggle_learn(enable);
    current_learn_en_ = enable;
    if (current_learn_en_ != device_.learn_enabled()) {
      return failure();
    }
  }
  return success();
}

int RuntimeCAPIAdapter::forward(uint8_t* input, const uint32_t* input_dims,
                                uint8_t* output, int output_size) {
  if (input == nullptr || input_dims == nullptr || output == nullptr ||
      output_size < 0) {
    return failure();
  }

  TensorConstPtr in = Dense::create_view(
      reinterpret_cast<const char*>(input), TensorType::uint8,
      hwc_shape(input_dims), Dense::Layout::RowMajor);
  auto ret = device_.forward({in});
  if (ret.empty()) {
    return failure();
  }

  auto out = Tensor::ensure_dense(std::move(ret.front()));
  if (out == nullptr ||
      out->size() * sizeof(int) != static_cast<size_t>(output_size)) {
    return failure();
  }

  std::memcpy(output, out->buffer()->data(), static_cast<size_t>(output_size));
  return success();
}

int RuntimeCAPIAdapter::predict(uint8_t* input, const uint32_t* input_dims,
                                float* output, int output_size_bytes) {
  if (input == nullptr || input_dims == nullptr || output == nullptr ||
      output_size_bytes < 0) {
    return failure();
  }

  TensorConstPtr in = Dense::create_view(
      reinterpret_cast<const char*>(input), TensorType::uint8,
      hwc_shape(input_dims), Dense::Layout::RowMajor);
  auto ret = device_.predict({in});
  if (ret.empty()) {
    return failure();
  }

  auto out = Tensor::ensure_dense(std::move(ret.front()));
  if (out == nullptr ||
      out->size() * sizeof(float) != static_cast<size_t>(output_size_bytes)) {
    return failure();
  }

  std::memcpy(output, out->buffer()->data(),
              static_cast<size_t>(output_size_bytes));
  return success();
}

void RuntimeCAPIAdapter::fit(uint8_t* input, const uint32_t* input_dims,
                             int32_t* input_label) {
  if (input == nullptr || input_dims == nullptr || input_label == nullptr) {
    return;
  }

  auto input_tensor = Dense::create_view(
      reinterpret_cast<const char*>(input), TensorType::uint8,
      bhwc_shape(input_dims), Dense::Layout::RowMajor);
  auto input_vector = Dense::split(*input_tensor);
  std::vector<int32_t> labels = {*input_label};
  (void)device_.fit(input_vector, labels);
}

int RuntimeCAPIAdapter::enqueue(uint8_t* input, const uint32_t* input_dims,
                                int32_t* input_label) {
  if (input == nullptr || input_dims == nullptr) {
    return failure();
  }

  auto input_tensor = Dense::create_view(
      reinterpret_cast<const char*>(input), TensorType::uint8,
      hwc_shape(input_dims), Dense::Layout::RowMajor);
  const bool queued = (input_label != nullptr)
                          ? device_.enqueue(*input_tensor, input_label)
                          : device_.enqueue(*input_tensor);
  return queued ? success() : failure();
}

int RuntimeCAPIAdapter::fetch(uint8_t* output, int output_size,
                              bool dequantize) {
  if (output == nullptr || output_size < 0) {
    return failure();
  }

  auto output_ptr = device_.fetch();
  if (!output_ptr) {
    return failure();
  }

  if (dequantize) {
    const Dense* dense_output = conversion::as_dense(*output_ptr);
    if (dense_output == nullptr) {
      return failure();
    }
    auto dequantized_output = device_.dequantize(*dense_output);
    if (!dequantized_output) {
      return failure();
    }
    std::memcpy(output, dequantized_output->data<float>(),
                static_cast<size_t>(output_size));
    return success();
  }

  auto out = Tensor::ensure_dense(std::move(output_ptr));
  if (out == nullptr) {
    return failure();
  }
  std::memcpy(output, out->buffer()->data(), static_cast<size_t>(output_size));
  return success();
}

int RuntimeCAPIAdapter::save_learn_weights(uint32_t* weights_ptr,
                                           uint32_t size) {
  if (weights_ptr == nullptr) {
    return failure();
  }

  const uint32_t layer_size =
      static_cast<uint32_t>(device_.learn_mem_size() * 4u);
  if (layer_size > size) {
    return failure();
  }

  device_.learn_mem(weights_ptr);
  return static_cast<int>(layer_size);
}

uint32_t RuntimeCAPIAdapter::learn_mem_size() const {
  return static_cast<uint32_t>(device_.learn_mem_size() * 4u);
}

int RuntimeCAPIAdapter::update_learn_weights(const uint32_t* weights_ptr,
                                             uint32_t size) {
  if (weights_ptr == nullptr) {
    return failure();
  }

  const uint32_t layer_size =
      static_cast<uint32_t>(device_.learn_mem_size() * 4u);
  if (layer_size != size) {
    return failure();
  }

  device_.update_learn_mem(weights_ptr);
  return static_cast<int>(layer_size);
}

int32_t RuntimeCAPIAdapter::inferred_class(const int32_t* result,
                                           int num_classes, int num_neurons) {
  if (result == nullptr || num_classes <= 0 || num_neurons <= 0) {
    return -1;
  }

  const int n_activations = num_classes * num_neurons;
  int32_t max_val = INT32_MIN;
  int max_index = 0;
  for (int i = 0; i < n_activations; ++i) {
    if (result[i] > max_val) {
      max_val = result[i];
      max_index = i;
    }
  }
  return max_index / num_neurons;
}

Shape RuntimeCAPIAdapter::hwc_shape(const uint32_t* input_dims) {
  return Shape{input_dims[0], input_dims[1], input_dims[2]};
}

Shape RuntimeCAPIAdapter::bhwc_shape(const uint32_t* input_dims) {
  return Shape{1u, input_dims[0], input_dims[1], input_dims[2]};
}

}  // namespace compat
}  // namespace akida
