# BB15 Arduino Library

`BB15` is the board-centric Arduino library for BrainBoard15.

The public API is centered on the physical BB15 board:

- `BB15` handles board bring-up, reset routing, expander access, flash access,
  and transport state.
- `BB15Model` describes a serialized Akida model and where it lives.
- `BB15Runner` loads models and runs inference.

The constructor requires a `BB15Pinout`, so host-board wiring is always
explicit:

```cpp
#include <BB15.h>

BB15Pinout pinout = BB15Pinout::niclaSenseMeDefaults();
BB15Config config = BB15Config::defaults();

void setup() {
  static BB15 bb15(pinout, config);
}
```

Construct `BB15` once `setup()` is running. Do not create it as a global
object, because the constructor touches board hardware state.

## Reset And Sleep Control

The standard pinouts define separate controls for the complete BB15 board and
the Akida device:

- `powerDown()` holds the full BB15 board reset through the host reset pin.
- `powerUp()` releases the full BB15 board reset.
- `holdAkidaInReset()` and `releaseAkidaReset()` control only the Akida reset
  through expander P4.
- `sleep()` and `wake()` control Akida sleep through expander P3.

After `powerUp()` or `wake()`, call `begin()` and `BB15Runner::begin()` before
running inference.

## Supported Hosts

The current target hosts are:

- Arduino Nicla Sense ME
- Arduino Nicla Vision
- Arduino Nicla Voice

Classic AVR boards are not a target for this library.

## Included Examples

The example set is intentionally small and board-specific:

1. `examples/bb15_model_flasher_nicla_sense`
   Flashes one exported Akida model into BB15 external flash from a Nicla Sense
   ME host and verifies that the runtime can load it back.
2. `examples/bb15_dummy_inference_nicla_sense`
   Loads the flashed model and runs interrupt-driven synthetic inference on a
   Nicla Sense ME host.
3. `examples/bb15_model_flasher_nicla_vision`
   Flashes one exported Akida model into BB15 external flash from a Nicla
   Vision host and verifies that the runtime can load it back.
4. `examples/bb15_dummy_inference_nicla_vision`
   Loads the flashed model and runs interrupt-driven synthetic inference on a
   Nicla Vision host.
5. `examples/bb15_nicla_vision_human_detection`
   Captures live Nicla Vision camera frames, preprocesses them for the bundled
   VWW person-detection model, runs interrupt-completed inference, and streams
   a grayscale preview plus results over USB CDC.
6. `examples/bb15_nicla_voice_keyword_spotting`
   Captures live Nicla Voice microphone audio, computes MFCC features on the
   host, runs ten-keyword spotting on BB15, and streams the waveform, the
   features and the class scores over USB CDC.

All sketches are heavily commented and meant to be modified by users.

## Getting Started

1. Export your model into the example asset files:
   - `program.h`
   - `program.cpp`
   - `model_metadata.h`
   - `model_metadata.cpp`
2. Pick the example pair that matches your host board.
3. Upload the matching `bb15_model_flasher_*` sketch.
4. Confirm the sketch reports a successful flash and verify pass.
5. Upload the matching `bb15_dummy_inference_*` sketch.
6. Confirm inference completes and prints scores.

### Nicla Vision Human Detection

The Vision flasher installs
`NiclaV_VWW_PersonDet_EN_USBbottom_2026-06-14`, a `96x96x3` Visual Wake Words
person-detection model. The matching live-camera workflow is:

```bash
arduino-cli compile --clean --fqbn arduino:mbed_nicla:nicla_vision --library . examples/bb15_model_flasher_nicla_vision
arduino-cli upload --fqbn arduino:mbed_nicla:nicla_vision --port /dev/ttyACM0 examples/bb15_model_flasher_nicla_vision
arduino-cli compile --clean --fqbn arduino:mbed_nicla:nicla_vision --library . examples/bb15_nicla_vision_human_detection
arduino-cli upload --fqbn arduino:mbed_nicla:nicla_vision --port /dev/ttyACM0 examples/bb15_nicla_vision_human_detection
python3 -m pip install -r tools/requirements.txt
python3 tools/bb15_nicla_vision_preview.py --port /dev/ttyACM0
```

Upload the flasher first, confirm flash/verify/model-load success, then upload
the human-detection sketch. Replace `/dev/ttyACM0` with the serial port assigned
by your operating system. The camera sketch never writes flash. It captures
RGB565 at `320x240`, sends a grayscale USB preview, and converts each frame to
the VWW model's rotated `96x96x3` input locally. The desktop tool requires
Python 3.7 or later, `pyserial`, and the system Tk package used by `tkinter`.

### Nicla Voice Keyword Spotting

This demo needs no flasher sketch. Its keyword model is 22,112 bytes and is
compiled into the sketch, so it lives in nRF52832 program flash and costs no
RAM. The Nicla Vision demo makes the opposite choice because its VWW program is
183,884 bytes.

```bash
arduino-cli compile --clean --fqbn arduino:mbed_nicla:nicla_voice --library . examples/bb15_nicla_voice_keyword_spotting
arduino-cli upload --fqbn arduino:mbed_nicla:nicla_voice --port /dev/cu.usbmodem9AD4C4763 examples/bb15_nicla_voice_keyword_spotting
PY=/path/to/a/python/with/tk/8.6/or/later/bin/python3
"$PY" -m pip install --target ~/.kws-libs -r tools/requirements.txt
PYTHONPATH=~/.kws-libs "$PY" tools/bb15_nicla_voice_kws_gui.py --port /dev/cu.usbmodem9AD4C4763
```

`PY` has to be a Python with Tk 8.6 or later, which is often not the `python3`
on your PATH; see "The Desktop Tools And Python" below.
Replace the port with the one your operating system assigned. It recognises
`down`, `go`, `left`, `no`, `off`, `on`, `right`, `stop`, `up` and `yes`.

Three things about this demo differ from the others, and all three will look
like faults if they are not expected:

- **It streams at 921600 baud**, not 115200. `Serial` on Nicla Voice is a UART
  bridged to USB by the onboard SAMD11, and at 115200 one result packet takes
  longer to drain than the NDP120's 24 ms audio chunk period, which breaks the
  audio stream.
- **The sketch needs about ten seconds after reset** before it answers
  anything, because it loads the NDP120 firmware packages from the board's QSPI
  flash first. The desktop tool retries across that window.
- **USB enumeration proves nothing about the sketch.** The Nicla Voice's USB
  device is provided by the onboard SAMD11 bridge, not the nRF52832, so the
  board can enumerate and hand you a serial port while the nRF52832 is running
  a different sketch or none. If the tool sits on `CONNECTING`, re-upload this
  example first. A genuine bring-up failure now reports itself instead of going
  quiet, and the tool prints the reason.

The audio front end matches BrainChip's spark firmware value for value: 16 kHz,
a 320-sample hop, a 640-point real FFT, 40 mel bins from 20 Hz to 4000 Hz, and
49 frames of 10 coefficients, with spark's voice-activity gating, smoothing,
threshold, debounce and chiming defaults hard-coded.

## The Desktop Tools And Python

Both `tools/bb15_nicla_vision_preview.py` and
`tools/bb15_nicla_voice_kws_gui.py` need one Python environment carrying **both
Tk 8.6 or later and `pyserial`**.

Apple's system Tcl/Tk 8.5, which `/usr/bin/python3` uses on macOS, renders
nothing: the window opens at the right size and stays blank, with no error
message. A plain virtual environment over a standalone CPython build does not
fix it and breaks Tk instead, because Tcl resolves `init.tcl` relative to
`sys.prefix`.

The arrangement that works whatever the interpreter is to install `pyserial`
beside it and put it on `PYTHONPATH`, launching the base interpreter rather
than a virtual environment's `python`. Installing straight into the interpreter
also works, but standalone builds, including the ones `uv` manages, are marked
externally managed and refuse with a PEP 668 error.
`examples/bb15_nicla_voice_keyword_spotting/README.md` gives both commands and
says which is which.

## Arduino CLI Build Checks

The current validated compile commands are:

```bash
arduino-cli compile --clean --fqbn arduino:mbed_nicla:nicla_sense --library . examples/bb15_model_flasher_nicla_sense
arduino-cli compile --clean --fqbn arduino:mbed_nicla:nicla_sense --library . examples/bb15_dummy_inference_nicla_sense
arduino-cli compile --clean --fqbn arduino:mbed_nicla:nicla_vision --library . examples/bb15_model_flasher_nicla_vision
arduino-cli compile --clean --fqbn arduino:mbed_nicla:nicla_vision --library . examples/bb15_dummy_inference_nicla_vision
arduino-cli compile --clean --fqbn arduino:mbed_nicla:nicla_vision --library . examples/bb15_nicla_vision_human_detection
arduino-cli compile --clean --fqbn arduino:mbed_nicla:nicla_voice --library . examples/bb15_nicla_voice_keyword_spotting
```

Run these sequentially. Parallel `arduino-cli` compiles can race in the shared
Arduino cache and produce misleading failures.

## Validation Status

Validation status as of August 21, 2026:

- the four existing primary examples compile successfully in this repository
- the Nicla Sense ME flasher and dummy inference examples compile for
  `arduino:mbed_nicla:nicla_sense`
- the Nicla Vision flasher and dummy inference examples compile for
  `arduino:mbed_nicla:nicla_vision`
- the Nicla Vision flasher has been hardware-validated in this workspace and
  now completes flash, verify, and model load successfully
- the human-detection camera path has been hardware-validated on Nicla Vision:
  VWW model load, camera capture, preprocessing, interrupt-completed inference,
  and a complete `320x240` USB frame-result packet all succeeded

Nicla Voice keyword spotting, as of September 2, 2026:

- the example compiles for `arduino:mbed_nicla:nicla_voice` and has been
  hardware-validated end to end on a Nicla Voice with BB15 attached
- `BB15Pinout::niclaVoiceDefaults()` was confirmed on hardware: `begin()`
  reports IP version `0xBCA10309`, the model loads from host memory, and
  inference returns results
- the ported MFCC front end is bit identical to spark's own C on the same
  toolchain, over ten test signals covering silence, tones, an impulse and
  noise; on the nRF52832 it agrees with the host to within 1.5e-4 of one
  model-input quantization step, and none of the quantized bytes differ
- the inference path was proved against spark's reference input before any
  microphone audio was trusted: the twelve output potentials match its known
  values exactly, giving `go`
- the audio stream is continuous under load. With the speech gate forced open
  so every 60 ms block runs three MFCC frames and inference runs every third
  block, 665 consecutive blocks streamed with no dropped audio chunks and no
  sequence gaps, at a measured block period of 59.99 ms
- timings measured on device: 2.2 ms per MFCC frame, 6 to 7 ms per block for
  the three frames, and 5.8 ms per inference
- resource use: 210,512 of 527,616 bytes of flash and 39,584 of 64,288 bytes of
  RAM, leaving 24,704 bytes free
- classification behaves correctly in both directions: room noise classifies as
  `unknown` and triggers nothing, and a spoken keyword triggers a detection and
  is named

## Repository Layout

- `src/`
  Public library code and internal runtime support owned by this package.
- `examples/`
  The supported user-facing sketches.

## Licensing

This project is licensed under the MIT License. See `LICENSE`.

This repository also bundles third-party components that are distributed
under the Apache License, Version 2.0, including the vendored FlatBuffers
headers under `src/flatbuffers/`. A copy of that license is in
`LICENSE-APACHE-2.0`.

`NOTICE` lists every copyright and license statement found in this tree,
the paths each one covers, and the files that carry no such statement.
