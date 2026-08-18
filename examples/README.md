# Examples

This library currently ships with three primary examples:

- `bb15_model_flasher`
- `bb15_inference`
- `bb15_sleep_wake`

`bb15_model_flasher` is the first-step sketch. It flashes one exported Akida
model into BB15 external flash and verifies that the runtime can load it.

`bb15_inference` is the follow-up sketch. It loads the flashed model and runs
interrupt-driven inference:

- on Nicla Sense ME it uses synthetic input
- on Nicla Vision it follows the same BB15 runtime path and swaps in camera
  input

All three sketches are intended to be user-facing examples, not internal test
programs. They should stay readable, heavily commented where needed, and free
of unnecessary internal clutter.

`bb15_sleep_wake` is the lifecycle example. It loads the flashed model, runs a
simple synthetic inference, enters Akida sleep through the expander-driven
sleep control, wakes the device, and re-runs bring-up so users can see the
intended `sleep()` / `wake()` flow directly. It is optional and sits after the
main flasher and inference path.
