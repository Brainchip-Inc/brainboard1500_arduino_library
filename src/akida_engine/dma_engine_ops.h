#pragma once

#include "dma_engine.h"

#include <cstdint>
#include <vector>

#include "akida/hw_version.h"
#include "engine/dma.h"
#include "infra/hardware_driver.h"
#include "infra/registers_common.h"

#include "dma_desc_ops.h"
#include "hardware_device_impl.h"

namespace akida {

namespace dma {
static constexpr uint32_t MAX_PIPELINE_SIZE = (kMaxNbDescriptors - 1);

// Configure DMA descriptors buffer and number of descriptors.
// It tells the DMA where to look at for descriptors, and how many descriptors
// it will loop on
void configure_descriptors_buffer(HardwareDriver* driver, const Engine& dma,
                                  uint32_t num_descriptors);

// Configure control register and enable/disable DMA engine
void toggle_engine(HardwareDriver* driver, uint32_t reg_base_addr,
                   bool enabled);

// Enqueue a descriptor to be processed by DMA. It does the following:
// - Copies descriptor to descriptors buffer in scratch buffer at the next
// available index
// - Programs DMA to process it without waiting for completion, by incrementing
// DMA_LAST_DESC_CONT field from register DMA_DESC_CONT_REG
// DMA_LAST_DESC_CONT is a circular counter from 0 to the max number of
// descriptors - 1
// Return the address where the descriptor was written
dma::addr enqueue_descriptor(HardwareDriver* driver, const Engine& dma,
                             const dma::Descriptor& descriptor);

// Tell config DMA engine to process a given descriptor. It does the following:
// - Turns DMA on
// - Calls enqueue_descriptor function (see its comment to know what this
// function is doing).
// - Waits for descriptor to be processed
// - Turns DMA off
void process(HardwareDriver* driver, const Config& dma,
             const Descriptor& descriptor);

// Used in single pass: return ID of last processed job
uint16_t get_last_job_id_processed(HardwareDriver* driver, const Inputs& dma);

// Turn clock counter measures on or off
void toggle_buffer_timer(HardwareDriver* driver, const Engine& dma,
                         bool enabled);

// Retrieve clock counter measures
uint32_t read_buffer_timer(HardwareDriver* driver, const Engine& dma);

// Tell if clock counter is enabled
bool is_buffer_timer_enabled(const HardwareDriver& driver, const Inputs& dma);

// Enable or disable pipeline. When enabled, it will be kept enable on best
// effort, disabled in multi pass and when learning is enabled.
void toggle_pipeline(HardwareDriver* driver, const Inputs& dma, bool enabled);

// Configure inputs controller to generate descriptors and process the multiple
// passes necessary to process events.
void prepare_engine_multi_pass(HardwareDriver* driver, const Inputs& dma,
                               dma::addr hw_desc_addr,
                               dma::addr hw_payload_addr, uint32_t num_loops);

// Configure output buffer clearing policy
void set_output_buffer_clear(HardwareDriver* driver, const Inputs& dma,
                             uint32_t clear_size);

// Check if the given interrupt (flag) is set on the DMA
bool check_for_interrupt(HardwareDriver* driver, const Engine& dma,
                         const RegDetail& flag);

// Clear all interrupts from the DMA
void clear_interrupts(HardwareDriver* driver, const Engine& dma);

// Configure config dma depending on the given program. It does the following:
// - Soft reset dma
// - Configures descriptors buffer & number by calling
// configure_descriptors_buffer function
// - Masks some interrupts depending on multipass mode or not
// - Toggles multipass mode, if on, set the number of descriptors per pass
// - Toggles output header (on for single pass, off for multipass)
// If the program is multi pass, it also:
// - Configures the number of extra descriptor required (extra descriptors are
// needed to learn using FNP3, to update weights from NP to program after each
// input)
// - Configures outbound container size
void init_config_dma(HardwareDriver* driver, const Config& dma,
                     const ProgramInfo& program_info);

// Toggle extra descriptors. Extra descriptors are used by learning using FNP3,
// to update weights from NP to program after each input
void toggle_extra_descriptors(HardwareDriver* driver, const Config& dma,
                              bool enable);

// Configure DMA with default values
// FIXME: this should not be called on config DMA, unless to scan mesh. When
// scanning mesh will be out of the engine, this could be renamed to
// init_input_dma and take const dma::Input& as argument
// It does the following:
// - Soft reset dma
// - Configures descriptors buffer & number by calling
// configure_descriptors_buffer function
// - Turns output header on
// - Toggles dma off
void init_default_dma(HardwareDriver* driver, const Engine& dma,
                      uint32_t number_of_descriptors);

// Turn config DMA on and running, and wait for config DMA to configure NPs
void enable_config_dma_multipass(HardwareDriver* driver, const Config& dma);

// wait for config DMA to process a descriptor
void wait_config_dma_descriptor_complete(HardwareDriver* driver,
                                         const Config& dma);

// Enqueue descriptor at "extra descriptors" location.
// This works if we have only 1 total extra descriptor, which is our use case,
// because it is only used by learn multipass, and learning can't be split
// accross NPs so there will be a single extra descriptor in a program
void enqueue_extra_descriptor(HardwareDriver* driver, const Config& dma,
                              const Descriptor& descriptor);

}  // namespace dma

}  // namespace akida
