# Repository Scope

## Purpose

This repository exists to isolate the new `BB15` API from the legacy library
layout and naming.

The core question it answers is:

"What is the smallest clean repository that can own the new board-centric API?"

## What Belongs Here

- public `BB15` headers and implementation
- BB15-local support code such as the PIO expander helper
- user-facing examples for flashing, loading, and inference
- build metadata for the standalone Arduino library
- documentation for supported host boards and wiring

## What Does Not Automatically Belong Here

- every legacy `AKD1500` example
- camera- or microphone-specific application policy
- one-off jig code
- audit artifacts
- model training code
- generated release artifacts

## Boundary Rule

Examples may depend on sensors or UI flows.

The library itself should only absorb a responsibility when that responsibility
is intrinsic to:

- BB15 board control
- model transport and storage
- runtime session setup
- inference execution

If a feature only exists to make one demo easier, it should remain in the
example layer unless that demo exposes an actual API flaw.

## Decisions Already Made

- the public package identity is `BB15`
- the primary include remains `#include <BB15.h>`
- interrupt handling remains example-local unless a stronger cross-example need
  appears later

## Current Extraction Boundary

The current `src/BB15.*` seed still depends on legacy runtime pieces outside
this repository seed:

- `AKD1500.h`
- `AkidaNicla`
- `akida_port::AKD1500Board`

That dependency is acceptable for the seed phase, but it is the main blocker to
standalone publication.

What "extract from the old AKD1500-named runtime/layout" means in practice is:

- this repository should eventually build without requiring sibling source
  trees from the old repo
- the new `BB15` public API should not be implemented by reaching back into a
  differently named library as if it were the real owner
- any runtime pieces that remain necessary should be moved here, renamed or
  wrapped as internal implementation details, and organized as part of this
  library's own structure
