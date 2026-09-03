#pragma once

#include <cstddef>
#include <cstdint>

#include "akida/hardware_device.h"
#include "akida/program_info.h"
#include "internal/akida_compat/model_address.h"

namespace akida {
namespace compat {

struct RuntimeCAPIConfig {
  int success_code = 0;
  int failure_code = -1;
  uint32_t flash_base_address = kExternalModelAliasBase;
};

class RuntimeCAPIAdapter {
 public:
  explicit RuntimeCAPIAdapter(HardwareDevice& device,
                              const RuntimeCAPIConfig& config = {});

  void toggle_clock_counter(bool enable);
  uint32_t get_clock_counter();

  int program(uint8_t* buffer, size_t size, bool learn_en);
  int program_only(uint8_t* buffer, size_t size);
  int program_flash(uint8_t* program_info_buffer, size_t len,
                    uint32_t flash_address, uint8_t* is_el_model);
  int batch_size(int size, bool allocate_inputs);
  int learn_mode(bool enable);
  int forward(uint8_t* input, const uint32_t* input_dims, uint8_t* output,
              int output_size);
  int predict(uint8_t* input, const uint32_t* input_dims, float* output,
              int output_size_bytes);
  void fit(uint8_t* input, const uint32_t* input_dims, int32_t* input_label);
  int enqueue(uint8_t* input, const uint32_t* input_dims, int32_t* input_label);
  int fetch(uint8_t* output, int output_size, bool dequantize);
  int save_learn_weights(uint32_t* weights_ptr, uint32_t size);
  uint32_t learn_mem_size() const;
  int update_learn_weights(const uint32_t* weights_ptr, uint32_t size);

  const ProgramInfo& program_info() const { return program_info_; }
  static int32_t inferred_class(const int32_t* result, int num_classes,
                                int num_neurons);

 private:
  static Shape hwc_shape(const uint32_t* input_dims);
  static Shape bhwc_shape(const uint32_t* input_dims);

  int success() const { return config_.success_code; }
  int failure() const { return config_.failure_code; }

  HardwareDevice& device_;
  RuntimeCAPIConfig config_;
  uint8_t* current_program_ = nullptr;
  bool current_learn_en_ = false;
  ProgramInfo program_info_;
};

}  // namespace compat
}  // namespace akida
