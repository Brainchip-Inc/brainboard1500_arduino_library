# BB15 Arduino Library

`BB15` is the board-centric Arduino library for BrainBoard15.

The public API is centered on the physical BB15 board rather than the older
`AKD1500`-named surface:

- `BB15` handles board bring-up, reset routing, expander access, flash access,
  and transport state.
- `BB15Model` describes a serialized Akida model and where it lives.
- `BB15Runner` loads models and runs inference.

The constructor requires a `BB15Pinout`, so host-board wiring is always
explicit:

```cpp
#include <BB15.h>

BB15Pinout pinout = BB15Pinout::niclaSenseMeDefaults();
BB15Config config = BB15Config::niclaVisionDefaults();

BB15 bb15(pinout, config);
```

## Supported Direction

This library is intentionally organized around a few boundaries:

- library code owns BB15 board control and runtime setup
- examples own demo policy such as camera capture and interrupt handling
- board differences are carried by `BB15Pinout`
- host/runtime tuning is carried by `BB15Config`

The current examples keep interrupt completion local to the sketch on purpose.
That keeps the library synchronous and simple unless a broader async API is
actually needed later.

## Supported Hosts

The current target hosts are:

- Arduino Nicla Sense ME
- Arduino Nicla Vision

Validation status as of August 17, 2026:

- `bb15_model_flasher` compiled for Nicla Sense ME and Nicla Vision
- `bb15_inference` compiled for Nicla Sense ME and Nicla Vision
- `bb15_inference` was hardware-tested on Nicla Sense ME with BB15 using the
  interrupt-driven completion path
- Nicla Vision support is compile-validated in this repository, but not
  hardware-validated in this workspace yet

Classic AVR boards are not a target for this library.

## Included Examples

Two examples are treated as the primary entry points:

1. `examples/bb15_model_flasher`
   This flashes one exported Akida model into BB15 external flash and verifies
   that it can be loaded back by the runtime.
2. `examples/bb15_inference`
   This loads the flashed model and runs interrupt-driven inference. On Nicla
   Sense ME it uses synthetic input. On Nicla Vision it follows the same BB15
   runtime path and swaps in camera input.

Both sketches are heavily commented and meant to be modified by users.

## Getting Started

1. Export your model into the example asset files:
   - `program.h`
   - `program.cpp`
   - `model_metadata.h`
   - `model_metadata.cpp`
2. Adjust `make_pinout()` if your BB15 stack wiring differs from the defaults.
3. Upload `examples/bb15_model_flasher`.
4. Confirm the sketch reports a successful flash and verify pass.
5. Upload `examples/bb15_inference`.
6. Confirm inference completes and prints scores.

## Arduino CLI Build Checks

The validated compile commands are:

```bash
arduino-cli compile --clean --fqbn arduino:mbed_nicla:nicla_sense --library . examples/bb15_model_flasher
arduino-cli compile --clean --fqbn arduino:mbed_nicla:nicla_sense --library . examples/bb15_inference
arduino-cli compile --clean --fqbn arduino:mbed_nicla:nicla_vision --library . examples/bb15_model_flasher
arduino-cli compile --clean --fqbn arduino:mbed_nicla:nicla_vision --library . examples/bb15_inference
```

Run these sequentially. Parallel `arduino-cli` compiles can race in the shared
Arduino cache and produce misleading failures.

## Repository Layout

- `src/`
  Public library code and internal runtime support owned by this package.
- `examples/`
  The two supported starting-point sketches.
- `docs/API_DEFINITION.md`
  The intended public API shape and user-facing review notes.
- `docs/IMPLEMENTATION_CHECKLIST.md`
  Remaining work before the library is fully release-hardened.

## Current Gaps

The library is now buildable as a standalone Arduino package in this
repository, but some cleanup still remains:

- internal runtime naming still carries some legacy `AKD1500` identifiers
- Nicla Vision still needs hardware validation in this repository
- CI and release automation are not added yet
- a project license still needs to be added
