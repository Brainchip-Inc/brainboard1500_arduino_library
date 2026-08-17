# Implementation Checklist

## Release Snapshot

This is the current state of the standalone `BB15` library as prepared on
August 17, 2026.

Completed:

- `BB15` is the public package identity.
- `#include <BB15.h>` is the primary include.
- the new API is extracted into its own standalone Arduino library layout
- the constructor requires an explicit `BB15Pinout`
- both main examples live in `examples/`
- Nicla Sense ME and Nicla Vision builds are passing with `arduino-cli`
- the Nicla Sense ME interrupt-driven inference path was hardware-validated

Still open:

- a project license must be added
- CI for the supported build matrix must be added
- Nicla Vision still needs hardware validation
- some internal runtime names still carry legacy `AKD1500` terminology

## Next Work Items

### 1. Release Hardening

- add a license file
- add GitHub Actions or equivalent CI for sequential `arduino-cli` builds
- write a short release checklist for:
  - compile
  - flash model
  - run inference
  - smoke-test serial output

### 2. API Cleanup

- decide whether `BB15Config::niclaVisionDefaults()` should become a more
  generic board-independent default helper
- review whether `BB15Error` and `BB15Status` need more field-diagnostic detail
- decide whether any remaining `AkidaNicla`-named internals should be wrapped
  behind more neutral BB15-owned internal naming

### 3. Example Polish

- hardware-validate `bb15_inference` on Nicla Vision
- confirm the camera-input path matches the same interrupt completion flow used
  on Nicla Sense ME
- keep example comments strong, but avoid reintroducing excessive serial noise

### 4. Model Workflow

- decide whether `BB15Model` should parse more exported metadata directly
- decide whether multi-model flash layouts need a first-class helper
- decide whether host-memory model loading needs a dedicated example

### 5. Documentation

- add a wiring section with the validated Nicla Sense ME and Nicla Vision
  pinouts
- add a migration note for users coming from the legacy `AKD1500` API
- keep `docs/API_DEFINITION.md` in sync with the real public surface
