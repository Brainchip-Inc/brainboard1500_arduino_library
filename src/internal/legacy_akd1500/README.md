# Legacy Runtime Boundary

This subtree contains the transplanted `AKD1500` runtime that the new `BB15`
API currently depends on internally.

It is placed under `src/internal/legacy_akd1500/` on purpose:

- to make it clear that `BB15` is the public library surface
- to stop presenting `AKD1500.h` as if it were a new top-level public entry
  point of this repository
- to give the extraction work a clear home while the runtime is still being
  renamed, wrapped, or reduced

Current policy:

- examples should include `#include <BB15.h>`
- new public code should not reach for this subtree directly unless there is a
  deliberate internal reason
- future cleanup can rename or split these files further without changing the
  public `BB15` include surface
