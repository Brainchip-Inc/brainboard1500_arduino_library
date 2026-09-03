# Examples

This library includes examples for Arduino Nicla Sense ME, Arduino Nicla
Vision and Arduino Nicla Voice.

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

- `bb15_nicla_voice_keyword_spotting`
  Captures live audio from the Nicla Voice microphone, computes MFCC features
  on the host, runs ten-keyword spotting on BB15, and streams the waveform, the
  features and the class scores over USB.

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


## Nicla Voice Keyword Spotting

No flasher sketch is needed. The keyword model is 22 KB and is compiled into
the sketch, so it lives in nRF52832 program flash rather than BB15 external
flash.

```bash
arduino-cli compile --clean --fqbn arduino:mbed_nicla:nicla_voice --library . examples/bb15_nicla_voice_keyword_spotting
arduino-cli upload --fqbn arduino:mbed_nicla:nicla_voice --port /dev/cu.usbmodem9AD4C4763 examples/bb15_nicla_voice_keyword_spotting
PY=/path/to/a/python/with/tk/8.6/or/later/bin/python3
"$PY" -m pip install --target ~/.kws-libs -r tools/requirements.txt
PYTHONPATH=~/.kws-libs "$PY" tools/bb15_nicla_voice_kws_gui.py --port /dev/cu.usbmodem9AD4C4763
```

Replace the port with the one your operating system assigned. This demo streams
at **921600 baud**, not the 115200 the other examples use, and the sketch needs
about ten seconds after reset before it answers, because it loads the NDP120
firmware packages from the board's QSPI flash first.

The desktop tool needs a Python with **both Tk 8.6 or later and `pyserial` in
the same environment**. See
`bb15_nicla_voice_keyword_spotting/README.md` for why a virtual environment
over a standalone CPython build is not enough, and for what to check when the
board enumerates on USB but sends nothing.
