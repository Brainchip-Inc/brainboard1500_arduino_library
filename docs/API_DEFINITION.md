# BB15 API Definition

## Goal

Define a public Arduino API for BrainBoard15 that matches how the hardware is
actually used:

- the library is centered on the BB15 board, not only the AKD1500 runtime
- board bring-up, expander control, reset strapping, and flash access are
  first-class responsibilities
- model handling is explicit and inspectable
- inference is simple for users while still exposing lower-level steps when
  needed

## Design Principles

### 1. One main entry point

The top-level object is `BB15`.

```cpp
BB15Pinout pinout = BB15Pinout::niclaVisionDefaults();
BB15Config config = BB15Config::niclaVisionDefaults();

BB15 bb15(pinout, config);
bb15.begin();
```

The public experience should feel board-centric instead of requiring users to
reason in terms of internal runtime classes.

### 2. Separate board, model, and runtime responsibilities

Use these conceptual layers:

- `BB15`: board control, buses, pins, expander, reset, flash path, and S2M
- `BB15Model`: serialized model blob plus storage description
- `BB15Runner`: model loading, enqueue, fetch, infer, and classify flow

### 3. Prefer explicit configuration

Physical wiring belongs in `BB15Pinout`.

Runtime tuning belongs in `BB15Config`.

The constructor requires the pinout so the physical board mapping is always
explicit.

### 4. Use descriptive methods

Prefer:

- `setAkidaReset(bool asserted)`
- `setAkidaSleep(bool enabled)`
- `holdAkidaInReset()`
- `releaseAkidaReset()`

over raw polarity-driven helpers.

### 5. High-level first, low-level available

Common tasks should stay short:

- bring up the board
- flash a model
- load a model
- run inference

Advanced flows still need an escape hatch:

- manual S2M enter and exit
- explicit flash verification
- split enqueue and fetch
- register reads and writes for bring-up and diagnostics

## Public Types

### `BB15Pinout`

```cpp
struct BB15Pins {
  uint8_t akidaCs;
  uint8_t bridgeCs;
  uint8_t proceed;
  uint8_t interrupt;
};

enum class BB15ResetRoute {
  HostGpio = 0,
  Expander,
};

struct BB15ResetPin {
  BB15ResetRoute route = BB15ResetRoute::HostGpio;
  uint8_t pin = 3;
};

struct BB15ExpanderPins {
  uint8_t bootMode;
  uint8_t akidaSleep;
  uint8_t akidaInterrupt;
};

struct BB15Pinout {
  BB15Pins host = {};
  BB15ResetPin akidaReset = {};
  BB15ExpanderPins expander = {};

  static BB15Pinout niclaVisionDefaults();
  static BB15Pinout niclaSenseMeDefaults();
};
```

`BB15Pinout` carries the physical host-to-BB15 mapping. The interrupt pin
belongs here even though interrupt handling remains example-local, because it
is still part of the wiring contract.

### `BB15Config`

```cpp
struct BB15Config {
  SPIClass* spi = &SPI;
  TwoWire* wire = &Wire;

  uint8_t expanderAddress = 0x43;
  uint32_t spiClockHz = 25000000u;
  uint32_t flashSpiClockHz = 2000000u;
  uint32_t expectedIpVersion = 0xBCA10309u;

  uint32_t defaultModelAddress = 0x80000000u;
  uint32_t fetchTimeoutMs = 20000u;
  uint32_t fetchPollDelayMs = 1u;

  uint32_t postBeginSettleMs = 50u;
  uint32_t postLinkSettleMs = 50u;
  uint32_t i2cClockHz = 100000u;
  uint32_t resetAssertMs = 5u;
  uint32_t resetReleaseSettleMs = 10u;

  const char* forcedFlashProfile = nullptr;
  bool assumeForcedFlashProfileReady = false;
};
```

`BB15Config` carries runtime tuning, transport settings, and expected device
parameters. It does not carry host pin names.

### `BB15Status`

```cpp
enum class BB15Status {
  Ok = 0,
  NotInitialized,
  InvalidConfig,
  ExpanderMissing,
  ExpanderConfigFailed,
  ResetFailed,
  LinkFailed,
  FlashUnsupported,
  FlashStageFailed,
  FlashVerifyFailed,
  ModelInvalid,
  ModelNotLoaded,
  InvalidInput,
  EnqueueFailed,
  OutputNotReady,
  FetchTimeout,
  OutputFormatMismatch,
  TransportStateError,
};
```

`OutputNotReady` is intentional. Split-phase inference should be able to
distinguish "not done yet" from an actual error.

## User-Facing Review

As a user, the constructor shape is now reasonable:

```cpp
BB15Pinout pinout = BB15Pinout::niclaSenseMeDefaults();
BB15Config config = BB15Config::niclaVisionDefaults();
BB15 bb15(pinout, config);
```

This makes board wiring explicit while keeping runtime knobs separate.

The remaining important user-level rule is:

- interrupt behavior belongs in examples unless multiple real applications
  demonstrate that the library itself needs a first-class async runtime API

That boundary is currently appropriate for this repository.
