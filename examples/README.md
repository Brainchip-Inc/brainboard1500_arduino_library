# Examples

This library includes examples for Arduino Nicla Sense ME and Arduino Nicla
Vision.

- `bb15_model_flasher_nicla_sense`
  Flashes the bundled model to BB15 external flash from Nicla Sense ME.

- `bb15_dummy_inference_nicla_sense`
  Loads the flashed model and runs synthetic interrupt-driven inference on Nicla
  Sense ME.

- `bb15_model_flasher_nicla_vision`
  Flashes the bundled Visual Wake Words person-detection model to BB15 external
  flash from Nicla Vision.

- `bb15_dummy_inference_nicla_vision`
  Loads the flashed model and runs synthetic interrupt-driven inference on Nicla
  Vision.

- `bb15_nicla_vision_human_detection`
  Captures live frames from the Nicla Vision camera, runs person detection on
  BB15, and streams a grayscale preview and inference results over USB.

## Nicla Vision Human Detection

Upload `bb15_model_flasher_nicla_vision` first to install the bundled VWW
person-detection model. Then upload `bb15_nicla_vision_human_detection`.

Start the desktop preview:

```bash
python3 -m pip install -r tools/requirements.txt
python3 tools/bb15_nicla_vision_preview.py --port /dev/ttyACM0
```

Replace `/dev/ttyACM0` with the serial port used by your board.

The camera demo captures RGB565 video at `320x240`, sends a grayscale preview
over USB CDC, and prepares a rotated `96x96x3` RGB input for the flashed VWW
model.
