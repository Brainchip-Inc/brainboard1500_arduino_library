# BrainBoard15 Arduino Library

Arduino library for running an AKD1500 on a Nicla Vision with the BB15 wiring
used by the bundled examples. The public flow is intentionally small:

- flash the bundled human-classifier model into BB15 external flash
- run the camera classification demo
- optionally open the live preview tool

## Supported Hardware

- Arduino Nicla Vision
- BrainBoard15 / BB15 wiring used by the included examples
- AKD1500 external-flash execution path

Example wiring assumed by the shipped sketches:

- `BB15 proceed / board-level enable`: `D2` (same Nicla Vision pin; required and driven high during bring-up)
- `AKD` chip select: `D7`
- bridge / flash chip select: `D1`
- `AKD1500 RESET_N`: `D3` (required; software drives this line during bring-up)
- SPI `COPI`: `D8`
- SPI `CIPO`: `D10`
- SPI `SCK`: `D9`
- BB15 expander: `I2C 0x43` on `Wire` (`SDA=D11`, `SCL=D12`)

## Wiring Notes

The shipped examples actively drive two different classes of control lines
during board bring-up:

- board-level proceed / enable line: `D2`, driven high
- device-level AKD reset line: `D3`, driven as `AKD1500 RESET_N`

The software sequence used by both examples is:

1. drive `D2` high so the BB15 board is in the expected proceed /
   enabled state
2. configure the BB15 expander on `0x43`
3. assert `AKD1500 RESET_N` low on `D3`
4. strap the AKD1500 boot mode through the expander (`P0` low for the
   external-flash execution path)
5. release `AKD1500 RESET_N` high on `D3`
6. link to the AKD1500 and continue with flash/model operations

That means the `D2 / PA_10` board-level control connection and the
`D3 -> AKD1500 RESET_N` connection are both required for the bundled software
flow. If either is missing from your wiring, the examples may compile and
upload but board bring-up will fail.

## Install

### Option 1: Clone or Download This Repo

Clone the repository, or download it as a ZIP from GitHub.

To use it as a local Arduino library, place the repository folder under your
Arduino sketchbook `libraries/` directory, for example:

```text
~/Arduino/libraries/BrainBoard15_arduino_library
```

Then restart Arduino IDE so it rescans local libraries.

### Option 2: Add ZIP Library

If you downloaded a ZIP from GitHub, use Arduino IDE:

- `Sketch` -> `Include Library` -> `Add .ZIP Library...`

Point it at the downloaded repository ZIP.

## Included Examples

- `NiclaVisionModelFlasher`
  First-time bring-up sketch. Configures BB15, stages the bundled model into
  external flash, verifies the flashed bytes, links to the AKD1500, and checks
  that the flashed model can be loaded.
- `NiclaVisionCameraFlashClassify`
  Flagship demo. Captures frames from the Nicla Vision camera, preprocesses
  them for the bundled human-classifier model, runs repeated classification
  from external flash, and optionally serves frames to the preview tool.

## Quick Start

### 1. Flash the bundled model

Open:

```text
File -> Examples -> AKD1500 -> NiclaVisionModelFlasher
```

Build and upload it to the connected Nicla Vision.

This first sketch depends on the `D2 / PA_10` board-level proceed / enable line and
the `D3 -> AKD1500 RESET_N` connection described above. The flasher first
drives the board-level control lines high, then asserts AKD reset, straps the
boot mode through the expander, and finally releases reset before attempting
flash access and model load.

Expected serial output includes:

- BB15 board setup pass
- flash stage pass
- flash verify pass
- AKD IP version
- model load pass

If you use `arduino-cli`, the equivalent build/upload flow is:

```bash
arduino-cli compile --fqbn arduino:mbed_nicla:nicla_vision \
  /path/to/BrainBoard15_arduino_library/examples/NiclaVisionModelFlasher

arduino-cli upload -p /dev/ttyACM0 --fqbn arduino:mbed_nicla:nicla_vision \
  /path/to/BrainBoard15_arduino_library/examples/NiclaVisionModelFlasher
```

### 2. Run the camera classification demo

Open:

```text
File -> Examples -> AKD1500 -> NiclaVisionCameraFlashClassify
```

Build and upload it to the same board.

Open Serial Monitor at `115200` baud and confirm:

- camera bring-up pass
- flash stage + verify pass
- AKD link pass
- model info printout
- repeated inference lines with `human` / `no_human`

The camera demo exposes sketch-level SPI speed knobs near the top of the file:

- `kAkidaSpiClockHz`
- `kFlashSpiClockHz`

Edit those in the IDE if you want to tune the data path.

### 3. Optional: run the preview tool

The repo includes:

```text
tools/nicla_vision_preview.py
```

Requirements:

- Python 3
- `pyserial`
- `tkinter` available in your Python install

Install `pyserial` if needed:

```bash
pip install pyserial
```

Run the preview app against the camera demo:

```bash
python3 tools/nicla_vision_preview.py \
  --port /dev/ttyACM0 \
  --fps 3 \
  --assume-demo-config
```

The preview app requests frames from `NiclaVisionCameraFlashClassify` and shows
the camera image plus inference metadata.

## Updating the Model

Both published examples keep the model local to the example folder on purpose.
That makes model replacement explicit and easy to review.

To switch to a new model:

1. Export the new serialized model in the same Arduino-friendly form used here.
2. Replace these files in both example folders:
   - `program.h`
   - `program.cpp`
   - `program_header_only.h`
3. If you want a different flash slot, update `kFlashModelOffset` in both
   sketches.
4. Upload `NiclaVisionModelFlasher` again to restage and verify the new model.
5. Upload `NiclaVisionCameraFlashClassify` and confirm the new model runs.

Keep the same `AKD1500Model` external-flash flow:

```cpp
model.storage = AKD1500ModelStorage::ExternalFlash;
model.externalLocation = AkidaNicla::externalModelAddressFromOffset(offset);
```

## Troubleshooting

- If your wiring was copied from an older connection list or diagram, verify
  that `D2 / PA_10` is connected to the BB15 board-level proceed / enable
  input and that `D3` is connected to `AKD1500 RESET_N`. The shipped examples
  drive `D2 / PA_10` high first, then assert and release `RESET_N` so the
  AKD1500 can be strapped into external-flash mode before linking.
- If Arduino IDE opens the example but compiles against a different `AKD1500`
  library copy, remove stale duplicates from your sketchbook `libraries/`
  directory and restart the IDE.
- If the preview tool sees no frames after a fresh USB reconnect, start the
  preview tool first and tap reset on the Nicla once.
- If flash stage or verify fails, rerun `NiclaVisionModelFlasher` before trying
  the camera demo again.
- If the camera demo prints stable boot logs but no classification loop, verify
  the model flasher completed successfully and that the BB15 wiring matches the
  pinout listed above.
