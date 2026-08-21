# Nicla Vision Human Detection

This Nicla Vision-only example captures the onboard camera, runs the VWW
person-detection model already stored in BB15 external flash, and streams a
grayscale preview plus inference results over USB CDC.

It does not write BB15 flash. First upload
`../bb15_model_flasher_nicla_vision` and confirm its flash, verify, and model
load steps pass. That flasher installs the exact VWW model expected here.

Compile and upload this sketch from the repository root:

```bash
arduino-cli compile --clean --fqbn arduino:mbed_nicla:nicla_vision --library . examples/bb15_nicla_vision_human_detection
arduino-cli upload --fqbn arduino:mbed_nicla:nicla_vision --port /dev/ttyACM0 examples/bb15_nicla_vision_human_detection
```

Then install the desktop dependency and start the preview tool:

```bash
python3 -m pip install -r tools/requirements.txt
python3 tools/bb15_nicla_vision_preview.py --port /dev/ttyACM0
```

Replace `/dev/ttyACM0` with your serial port. The sketch uses the onboard
GC2145 at `320x240` RGB565, sends a grayscale preview, center-crops and resizes
to `96x96`, expands RGB values, then rotates the input 180 degrees for the
USB-bottom model orientation. The binary USB protocol is for the bundled
preview tool; do not attach a text serial monitor while streaming.
