#pragma once

#include <cstdint>

#include "akida/hw_version.h"
#include "akida/shape.h"
#include "akida/span.h"

namespace akida {
class HardwareDeviceImpl;
class ProgramInfo;
struct MultiPassMemory;

namespace program {
void rewind(HardwareDeviceImpl*, const ProgramInfo&);
void play_single_pass(HardwareDeviceImpl*, const ProgramInfo&);
void play_multi_pass(HardwareDeviceImpl*, const ProgramInfo&, MultiPassMemory*);
void configure_learning_mode_single_pass(HardwareDeviceImpl*,
                                         const ProgramInfo&, bool);
void configure_learning_mode_multi_pass(HardwareDeviceImpl*, const ProgramInfo&,
                                        const MultiPassMemory&, bool learn_en);
}  // namespace program

namespace fb {
struct Program;
struct ProgramInfo;
enum IoType : int8_t;
}  // namespace fb

// Helper class to extract information about a program
class AKIDASHAREDLIB_EXPORT ProgramInfo {
 public:
  ProgramInfo();
  ProgramInfo(const uint8_t* serialized_program_info,
              size_t program_info_size,
              uint32_t program_data_address);
  explicit ProgramInfo(const uint8_t* serialized_program_buffer,
                       size_t program_size);

  /**
   * @brief Return the Hardware version the program was generated for
   */
  HwVersion device_version() const;

  /**
   * @brief Return the input dimensions of the program as a 3 elements array
   */
  const uint32_t* input_dims() const;

  /**
   * @brief Return the output dimensions of the program as a 3D Shape
   */
  Shape output_dims() const;

  /**
   * @brief Return true if inputs are expected to be dense, false otherwise
   */
  bool input_is_dense() const;

  /**
   * @brief Return true if outputs are dense, false otherwise
   */
  bool output_is_dense() const;

  /**
   * @brief Return true if outputs are activations, false if they are potentials
   */
  bool activation_enabled() const;

  /**
   * @brief Return the width of dense inputs window (area sent to HRC)
   */
  uint32_t dense_input_window_width() const;

  /**
   * @brief Return the height of dense inputs window (area sent to HRC)
   */
  uint32_t dense_input_window_height() const;

  /**
   * @brief Return true if the program can toggle learn
   */
  bool can_learn() const;

  /**
   * @brief Return the number of 32b words needed to store weights of learning
   * layer
   */
  uint32_t learn_weights_word_size() const;

  /**
   * @brief In case of a multipass program, return the number of descriptors per
   * pass (it is the same for every pass)
   */
  uint8_t number_of_descriptors_per_pass() const;

  /**
   * @brief Return the number of passes of the program
   */
  uint32_t number_of_passes() const;

  /**
   * @brief Return the number of descriptors required to program akida (it does
   * not include extra descriptor for learning if any)
   */
  uint32_t number_of_program_descriptors_required() const;

  /**
   * @brief Return the number of extra descriptors for a multipass program
   */
  uint32_t number_of_extra_program_descriptors_required() const;

  /**
   * @brief Return true if learning is executing on FNP3, false otherwise
   */
  bool learning_on_fnp3() const;

  /**
   * @brief Return the number of bytes required to program (it can be lower than
   * the total program size)
   */
  size_t program_data_required_memory() const;

  /**
   * @brief Return the number of bytes required by weights of FNP2s
   */
  size_t fnp2_required_memory() const;

  /**
   * @brief Return the serialized shifts vector, used to dequantize outputs
   */
  akida::span<int32_t> shifts() const;

  /**
   * @brief Return the serialized scales vector, used to dequantize outputs
   */
  akida::span<float> scales() const;

  /**
   * @brief Return the expected type of program inputs
   */
  fb::IoType inputs_type() const;

  /**
   * @brief Return the type of program outputs
   */
  fb::IoType outputs_type() const;

  /**
   * @brief Return true if a serialized program info buffer has been parsed,
   * false otherwise
   */
  bool is_valid() const;

  /**
   * @brief Return the external program data device address, or 0 if the
   * program data are stored locally in the combined serialized buffer.
   */
  uint32_t program_data_address() const { return program_data_address_; }

  // This will be removed once we have implemented all helper methods
  const akida::span<uint8_t>& program() const { return program_data_; }

  friend void program::rewind(HardwareDeviceImpl* device,
                              const ProgramInfo& program_info);

  friend void program::play_single_pass(HardwareDeviceImpl* device,
                                        const ProgramInfo& program_info);

  friend void program::play_multi_pass(HardwareDeviceImpl* device,
                                       const ProgramInfo& program_info,
                                       MultiPassMemory* multipass_memory);

  friend void program::configure_learning_mode_single_pass(
      HardwareDeviceImpl* device, const ProgramInfo& program_info,
      bool learn_en);

  friend void program::configure_learning_mode_multi_pass(
      HardwareDeviceImpl* device, const ProgramInfo& program_info,
      const MultiPassMemory& multipass_memory, bool learn_en);

 protected:
  akida::span<uint8_t> program_data_;
  uint32_t program_data_address_;
  const fb::Program* program_;
  const fb::ProgramInfo* program_info_;
};

}  // namespace akida
