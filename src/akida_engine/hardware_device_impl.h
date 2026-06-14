#pragma once

#include <cstddef>
#include <cstdint>
#include <memory>
#include <queue>
#include <vector>

#include "akida/hardware_device.h"
#include "akida/hw_version.h"
#include "akida/np.h"
#include "akida/program_info.h"
#include "akida/tensor.h"
#include "engine/dma.h"
#include "infra/hardware_driver.h"

#include "device_memory.h"
#include "dma_engine.h"
#include "external_mem_mgr.h"
#include "memory_mgr.h"
#include "multipass_memory.h"
#include "pipeline_state.h"

namespace akida {

namespace dma {
// forward declarations
enum class Target;
}  // namespace dma

class HardwareDeviceImpl final : public HardwareDevice {
 public:
  HardwareDeviceImpl(HardwareDriver* driver);

  ~HardwareDeviceImpl();

  HwVersion version() const override;

  const char* desc() const override { return driver_->desc(); }

  void pipeline(bool enable);

  void toggle_clock_counter(bool enable) override;

  uint32_t read_clock_counter() override;

  uint32_t read_config_clock_counter() override;

  void dma_config_write(const dma::w32* buffer, size_t buffer_size);

  void dma_config_read(dma::w32* buffer, const struct np::Ident& np,
                       dma::Target target, uint16_t addr_target_word,
                       uint32_t nb_words);

  void read_np_registers(uint32_t* output, const struct np::Ident& np,
                         uint32_t nb_registers) override;

  // Device fit
  std::vector<TensorUniquePtr> fit(
      const std::vector<TensorConstPtr>& inputs,
      const std::vector<int32_t>& input_labels) override;

  // Device forward
  std::vector<TensorUniquePtr> forward(
      const std::vector<TensorConstPtr>& inputs) override;

  // Device predict
  std::vector<TensorUniquePtr> predict(
      const std::vector<TensorConstPtr>& inputs) override;

  // Queue input
  bool enqueue(const Tensor& input, const int32_t* label = nullptr) override;

  // check for output
  TensorUniquePtr fetch() override;

  // apply rescale
  DenseUniquePtr dequantize(const Dense& potentials) override;

  // perform hardware device programming
  ProgramInfo program(const uint8_t* program, size_t size) override;
  ProgramInfo program_external_data(const uint8_t* program_info_buffer,
                                    size_t program_info_size,
                                    uint32_t program_data_address) override;

  size_t set_batch_size(size_t requested_batch_size,
                        bool allocate_inputs) override;

  // enable/disable learning mode
  void toggle_learn(bool learn_en) override;

  // unprogram current program
  void unprogram() override;

  // Return the memory used currently in the device
  MemoryInfo memory() const override { return mem_mgr_.report(); }

  void reset_top_memory() override { mem_mgr_.reset_top_usage(); }

  const akida::span<uint8_t>& program() const override {
    return current_program_buffer_;
  }

  bool learn_enabled() const override { return current_program_learn_en_; }

  size_t learn_mem_size() const override;

  void learn_mem(uint32_t* output_buffer) override;

  void update_learn_mem(const uint32_t* input_buffer) override;

  HardwareDriver* driver() const override { return driver_; }

  MemoryMgr* mem() { return &mem_mgr_; }

  ExternalMemoryMgr* external_mem() { return &external_mem_; }

  const dma::Config& dma_config() const { return dma_config_; }

 private:
  HardwareDriver* driver_;
  HwVersion version_;
  dma::Config dma_config_;
  dma::Inputs dma_event_;
  dma::Inputs dma_hrc_;
  MemoryMgr mem_mgr_;
  akida::span<uint8_t> current_program_buffer_;
  ProgramInfo current_program_info_;
  bool current_program_learn_en_;
  ExternalMemoryMgr external_mem_;

  // infos on memory that need to be allocated for multi pass program
  MultiPassMemory multi_pass_memory_;
  // infos on memory that need to be allocated for inference
  InferenceMemory inference_memory_;

  PipelineState pipeline_state_;

  // Initialization helpers
  void init();

  // pipeline helper
  std::vector<TensorUniquePtr> forward_loop(
      const std::vector<TensorConstPtr>& inputs,
      const std::vector<int32_t>* labels);

  // DMA helpers
  const dma::Inputs& select_dma_engine(bool is_hrc);
  bool clock_counter_enabled();
};

}  // namespace akida
