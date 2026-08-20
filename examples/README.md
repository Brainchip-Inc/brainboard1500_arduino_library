# Examples

This library now ships with four explicit board-specific sketches:

- `bb15_model_flasher_nicla_sense`
- `bb15_dummy_inference_nicla_sense`
- `bb15_model_flasher_nicla_vision`
- `bb15_dummy_inference_nicla_vision`

The two flasher sketches are the first-step examples. They flash one exported
Akida model into BB15 external flash, verify it, and confirm that the runtime
can load the model back.

The two dummy inference sketches are the follow-up examples. They load the
flashed model from BB15 external flash and run interrupt-driven synthetic
inference so users can validate the runtime path before adding a real sensor or
camera source.

All four sketches are intended to be user-facing examples. They should stay
readable, self-contained, and explicit about which host board they target.
