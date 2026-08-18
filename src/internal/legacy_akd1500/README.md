# Internal Runtime Boundary

This subtree contains the low-level device runtime that `BB15` depends on
internally.

It lives under `src/internal/legacy_akd1500/` to keep the public library
surface clean and focused:

- `BB15` remains the intended public include surface
- examples should include `#include <BB15.h>`
- internal runtime files stay behind the library boundary

Current policy:

- new public code should not reach for this subtree directly unless there is a
  deliberate internal reason
- internal reorganization in this subtree should not change the public `BB15`
  include surface
