# Examples

This library currently ships with two primary examples:

- `bb15_model_flasher`
- `bb15_inference`

`bb15_model_flasher` is the first-step sketch. It flashes one exported Akida
model into BB15 external flash and verifies that the runtime can load it.

`bb15_inference` is the follow-up sketch. It loads the flashed model and runs
interrupt-driven inference:

- on Nicla Sense ME it uses synthetic input
- on Nicla Vision it follows the same BB15 runtime path and swaps in camera
  input

Both sketches are intended to be user-facing examples, not internal test
programs. They should stay readable, heavily commented where needed, and free
of unnecessary internal clutter.
