#ifndef PRESENCE_CLASSIFIER_TEST_H_
#define PRESENCE_CLASSIFIER_TEST_H_

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

bool external_presence_classifier(
    akida::HardwareDriver* driver,
    uint32_t external_program_data_address,
    on_engine_event_t on_engine_event = [](EngineEvent) {});

#endif  // PRESENCE_CLASSIFIER_TEST_H_
