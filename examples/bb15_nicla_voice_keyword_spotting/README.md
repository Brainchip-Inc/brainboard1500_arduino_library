# Nicla Voice Keyword Spotting

This Nicla Voice-only example listens on the onboard microphone, computes MFCC
features on the nRF52832, classifies them on BB15, and streams the waveform,
the features and the twelve class scores over USB CDC.

It recognises ten keywords: `down`, `go`, `left`, `no`, `off`, `on`, `right`,
`stop`, `up`, `yes`. The model also reports `silence` and `unknown`, which are
transmitted but can never trigger a detection.

It does not write BB15 flash, and needs no flasher sketch. The model is 22 KB
and is compiled into the sketch, so it lives in nRF52832 program flash and
costs no RAM.

## Build and run

Compile and upload from the repository root:

```bash
arduino-cli compile --clean --fqbn arduino:mbed_nicla:nicla_voice --library . examples/bb15_nicla_voice_keyword_spotting
arduino-cli upload --fqbn arduino:mbed_nicla:nicla_voice --port /dev/cu.usbmodem9AD4C4763 examples/bb15_nicla_voice_keyword_spotting
```

Then start the desktop tool. `PY` has to be a Python with Tk 8.6 or later, not
necessarily the `python3` on your PATH, and "The Python the desktop tool needs"
below explains why and how to check:

```bash
PY=/path/to/a/python/with/tk/8.6/or/later/bin/python3
"$PY" -m pip install --target ~/.kws-libs -r tools/requirements.txt
PYTHONPATH=~/.kws-libs "$PY" tools/bb15_nicla_voice_kws_gui.py --port /dev/cu.usbmodem9AD4C4763
```

Replace the port with the one your operating system assigned. Read that section
before the first run: a Python that cannot draw shows an empty window rather
than an error.

**The sketch takes about ten seconds after reset before it answers anything.**
It loads 516 KB of NDP120 firmware packages from the board's QSPI flash first.
The tool retries across that window and shows `CONNECTING` while it waits.

The USB link runs at **921600 baud**, not the 115200 the other examples use.
`Serial` on this board is a UART bridged to USB by the onboard SAMD11, and at
115200 one result packet takes 37 ms to drain, longer than the NDP120's 24 ms
audio chunk period, which breaks the audio stream. The sketch and the tool both
default to 921600; do not attach a text serial monitor while streaming.

## What runs where

The Nicla Voice microphone reaches the NDP120 only, never the nRF52832
(Nicla Voice datasheet section 4.7), so audio is pulled back out of the NDP120
over SPI using the Arduino core's bundled `NDP` library. That needs all three
firmware packages already on the board's flash: with only the MCU and DSP
packages loaded the audio holding tank never advances. This demo uses none of
the neural-network package's own classes; it is loaded because it carries the
DSP audio flow configuration.

From there everything runs on the nRF52832 and BB15:

| stage | where | cost |
| --- | --- | --- |
| 16 kHz mono capture, 24 ms chunks | NDP120 over SPI | 6 to 9 ms per 60 ms block |
| DC blocking and block RMS | nRF52832 | negligible |
| MFCC, 3 frames per block | nRF52832 | 2.2 ms per frame |
| classification, every third block | BB15 AKD1500 | 5.8 ms |

The front end matches BrainChip's spark firmware value for value: 16 kHz, a
320-sample hop, a 640-sample frame, a 640-point real FFT, 40 mel bins from
20 Hz to 4000 Hz, and 49 frames of 10 coefficients. The gating, smoothing,
threshold, debounce and chiming defaults are spark's, hard-coded rather than
configurable.

## Reading the display

- **Result card**: the detected keyword, and the ten keyword classes as bars
  against the 0.50 score threshold. `silence` and `unknown` are hidden here
  because they cannot trigger; the device still sends them.
- **Waveform**: the DC-blocked microphone signal over the last 3.6 seconds.
- **SPEECH / IDLE badge**, at the top right of the waveform: spark's voice
  activity state, which decides whether a block reaches the MFCC front end at
  all. It stays on `SPEECH` through short pauses, because the gate holds open
  for 1300 ms after the last block above the RMS threshold, and the `features`
  time in the telemetry row drops to `0 ms` whenever it reads `IDLE`.
- **Level meter**, under the waveform: block RMS in dBFS, with a tick at the
  RMS threshold the gate uses.

The MFCC features themselves are not drawn. The device still computes and
transmits them, and the packet still carries them, so a tool that wants them
has them.

Say a keyword close to the board. A detection needs the class to hold above
0.50 for three consecutive inferences, so a single ambiguous frame will not
fire.

## The Python the desktop tool needs

The tool needs one environment that has **both Tk 8.6 or later and `pyserial`**.

Apple's system Tcl/Tk 8.5, which `/usr/bin/python3` uses on macOS, draws
nothing: the window opens at the right size and stays blank, with no error.
`tools/bb15_nicla_vision_preview.py` has the same exposure.

A plain virtual environment over a standalone CPython build does not solve it
and breaks Tk instead, because Tcl looks for `init.tcl` relative to
`sys.prefix`:

```
_tkinter.TclError: Cannot find a usable init.tcl in the following directories: ...
```

What works whatever the interpreter is to install `pyserial` beside it and put
it on `PYTHONPATH`, launching the base interpreter rather than any virtual
environment's `python`. That leaves `sys.prefix` alone, so Tcl still finds its
scripts:

```bash
PY=/path/to/a/python/with/tk/8.6/or/later/bin/python3
"$PY" -m pip install --target ~/.kws-libs -r tools/requirements.txt
PYTHONPATH=~/.kws-libs "$PY" tools/bb15_nicla_voice_kws_gui.py --port /dev/cu.usbmodem9AD4C4763
```

Installing straight into the interpreter also works, but only if that
interpreter is not marked externally managed. Standalone builds, including the
ones `uv` manages, are, and refuse with a PEP 668 error unless you add
`--break-system-packages`:

```bash
"$PY" -m pip install -r tools/requirements.txt   # may refuse, see PEP 668
```

## When the board looks dead

**USB enumeration proves nothing about the sketch.** The Nicla Voice appears on
USB as a composite CMSIS-DAP and serial device provided by the onboard SAMD11
bridge, not by the nRF52832 (datasheet section 4.8). The board can therefore
enumerate, list correctly under `arduino-cli board list`, and hand you a serial
port while the nRF52832 is running something else entirely or nothing at all.

**Listen before concluding anything.** While it is not streaming, the sketch
reports itself every two seconds, so a plain terminal is enough to tell a
healthy board from a wedged one:

```bash
PORT=/dev/cu.usbmodem9AD4C4763
stty -f $PORT 921600 raw && head -c 400 $PORT
```

Three outcomes, and they mean different things:

| what you see | what it means |
| --- | --- |
| `idle ready=1 waiting_for_start_stream` | The board is fine. It is waiting to be asked, and the tool simply needs to ask again. |
| `idle result=FAIL status=0x..` | Bring-up failed and the sketch is saying which stage. `0x81` is the MFCC front end, `0x82` the NDP120 microphone, `0x83` BB15. A power cycle is the first thing to try. |
| nothing at all | Either it is still inside its ten-second boot, or something is genuinely wrong. Wait fifteen seconds and look again before treating it as broken. |

**Do not diagnose this board by sending one command once.** Opening the port
reconfigures the USB bridge's UART, and a request written immediately afterwards
is lost more often than not: measured on a healthy, streaming board, a single
open-write-read attempt came back empty nine times in fifteen. Retrying the
request for a few seconds, which the bundled tool does, brings that down to
roughly one in twelve. A board that fails a single-shot probe is usually a
healthy board that was asked at the wrong moment.

If the terminal shows nothing after a fifteen-second look, then check what is
actually on the board: any other sketch, including a debugging build, leaves
this demo silent in a way that looks exactly like a broken tool. Re-upload this
example.

The board is its own debug probe, so `arduino-cli upload` works over the same
USB cable with no external hardware. openocd finds the onboard CMSIS-DAP at
`0x2341:0x0065` and programs only the sketch's own flash sectors; the QSPI
flash holding the NDP120 packages is never touched.
