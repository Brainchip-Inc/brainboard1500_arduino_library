# Examples

This library ships with four board bring-up/runtime sketches and one Nicla
Vision camera demo:

- `bb15_model_flasher_nicla_sense`
- `bb15_dummy_inference_nicla_sense`
- `bb15_model_flasher_nicla_vision`
- `bb15_dummy_inference_nicla_vision`
- `bb15_nicla_vision_human_detection`

The two flasher sketches are the first-step examples. They flash one exported
Akida model into BB15 external flash, verify it, and confirm that the runtime
can load the model back.

The two dummy inference sketches are the follow-up examples. They load the
flashed model from BB15 external flash and run interrupt-driven synthetic
inference so users can validate the runtime path before adding a real sensor or
camera source.

All five sketches are intended to be user-facing examples. They should stay
readable, self-contained, and explicit about which host board they target.

`bb15_nicla_vision_human_detection` is the live-camera follow-up for Nicla
Vision. First upload `bb15_model_flasher_nicla_vision`, which flashes the
bundled VWW person-detection model. Then upload the camera sketch and run:

```bash
python3 -m pip install -r tools/requirements.txt
python3 tools/bb15_nicla_vision_preview.py --port /dev/ttyACM0
```

Replace `/dev/ttyACM0` with the serial port assigned by your operating system.
The camera demo loads the existing flash model and never modifies it. It keeps
camera capture, preprocessing, USB streaming, and completion interrupts local
to the sketch; the BB15 library API remains hardware-focused.
