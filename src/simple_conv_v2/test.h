#include "akida/hardware_device.h"

enum EngineEvent {
  EngineProgramStart,
  EngineProgramSuccess,
  EngineEnqueueStart,
  EngineEnqueueSuccess,
  EngineEnqueueFailed,
  EngineFetchStart,
  EngineFetchSuccess,
  EngineFetchFailed,
};

using on_engine_event_t = void (*)(EngineEvent);

/**
 * @brief Akida engine unit test based on TESTNAME configuration
 *
 * This method takes a pointer to a target-specifc HardwareDriver
 * implementation.
 *
 * The driver would typically be instantiated by a target-specific main
 * application calling the method.
 *
 * The implementation of the method relies on three serialized arrays:
 * - the program,
 * - the predefined test inputs,
 * - the expected test outputs.
 *
 * The method first instantiates a device corresponding to the specified driver,
 * then loads the serialized program from the test configuration.
 *
 * It then performs an inference on the predefined inputs and compares the
 * result with the expected outputs.
 *
 * @param driver : an implementation of an akida::HardwareDriver
 * @param on_engine_event : a callback that will be called on several Engine
 * events
 * @return true if the hardware outputs match the expected outputs.
 */
bool simple_conv_v2(
    akida::HardwareDriver* driver,
    on_engine_event_t on_engine_event = [](EngineEvent) {});

bool external_simple_conv_v2(
    akida::HardwareDriver* driver,
    uint32_t external_program_data_address,
    on_engine_event_t on_engine_event = [](EngineEvent) {});
