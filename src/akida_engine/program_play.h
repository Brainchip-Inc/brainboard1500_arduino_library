#pragma once

#include <cstddef>
#include <cstdint>

#include "akida/program_info.h"

#include "hardware_device_impl.h"
#include "multipass_memory.h"

namespace akida {
namespace program {

using ExternalTrackReader = bool (*)(uint32_t offset, uint8_t* data,
                                     size_t size);
using ProgressHook = void (*)(uint32_t timestamp_ms);

void set_external_track_reader(ExternalTrackReader reader);
void set_progress_hook(ProgressHook hook);

void rewind(HardwareDeviceImpl* device, const ProgramInfo& program_info);

void play_single_pass(HardwareDeviceImpl* device,
                      const ProgramInfo& program_info);
void play_multi_pass(HardwareDeviceImpl* device,
                     const ProgramInfo& program_info,
                     MultiPassMemory* multipass_memory);

void configure_learning_mode_single_pass(HardwareDeviceImpl* device,
                                         const ProgramInfo& program_info,
                                         bool learn_en);
void configure_learning_mode_multi_pass(HardwareDeviceImpl* device,
                                        const ProgramInfo& program_info,
                                        const MultiPassMemory& multipass_memory,
                                        bool learn_en);

// Utility functions to get informations from program
void learn_mem(HardwareDeviceImpl* device, const uint8_t* program_data,
               uint32_t* ram_dump);
void update_learn_mem(HardwareDeviceImpl* device, const uint8_t* program_data,
                      const uint32_t* ram_dump);

}  // namespace program
}  // namespace akida
