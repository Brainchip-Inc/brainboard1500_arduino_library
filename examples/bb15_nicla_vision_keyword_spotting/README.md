# Nicla Vision Keyword Spotting

This Nicla Vision-only example listens on the onboard PDM microphone, computes
MFCC features on the STM32H747, classifies them on BB15, and streams the
waveform, the features and the twelve class scores over USB CDC.

It recognises ten keywords: `down`, `go`, `left`, `no`, `off`, `on`, `right`,
`stop`, `up`, `yes`. The model also reports `silence` and `unknown`, which are
transmitted but can never trigger a detection.

It does not write BB15 flash, and needs no flasher sketch. The model is 22 KB
and is compiled into the sketch, so it lives in STM32H747 program flash and
costs no RAM.

Everything downstream of capture is the Nicla Voice demo's pipeline value for
value: the same DC blocker, 960-sample blocks, RMS speech gate, MFCC front end,
spectrogram ring, inference period, smoothing, threshold, debounce and chiming.
Both demos speak the same USB protocol and are drawn by the same desktop tool.

## Build and run

Compile and upload from the repository root:

```bash
arduino-cli compile --clean --fqbn arduino:mbed_nicla:nicla_vision --library . examples/bb15_nicla_vision_keyword_spotting
arduino-cli upload --fqbn arduino:mbed_nicla:nicla_vision --port /dev/cu.usbmodem101 examples/bb15_nicla_vision_keyword_spotting
```

Then start the desktop tool. `PY` has to be a Python with Tk 8.6 or later, not
necessarily the `python3` on your PATH; "The Python the desktop tool needs" in
the Nicla Voice example's `README.md` explains why and how to check:

```bash
PY=/path/to/a/python/with/tk/8.6/or/later/bin/python3
"$PY" -m pip install --target ~/.kws-libs -r tools/requirements.txt
PYTHONPATH=~/.kws-libs "$PY" tools/bb15_kws_gui.py --port /dev/cu.usbmodem101
```

Replace the port with the one your operating system assigned. The tool reads the
board out of the config packet, so the same command line serves either demo and
the window names whichever board answered.

**There is no boot wait.** Unlike the Nicla Voice demo there is no NDP120
firmware to load, so the sketch answers as soon as the host opens the port.

`Serial` here is native USB CDC on the STM32H747, so the baud value is not
load-bearing: the link runs at USB speed whatever is asked for. The sketch and
the tool both say 921600 so that one command line works for either board.

## What runs where

The Nicla Vision's MP34DT06JTR microphone is wired to the STM32H747's DFSDM
peripheral, so audio arrives in the application processor directly and the core's
bundled `PDM` library is the whole capture path. The DFSDM delivers 256 samples
per half transfer, and `py_audio_init()` will not accept a double buffer smaller
than 1024 bytes, so the demo asks for exactly that: 512 samples, 32 ms of audio
per callback.

Two details of that library shape the sketch:

- `setBufferSize()` must be called before `begin()`. `begin()` captures the
  buffer pointer before `py_audio_init()` runs, and resizing reallocates the
  buffer, which would leave the library filling freed memory.
- `onReceive()` runs in interrupt context, so it only raises a flag. The read,
  the filtering, the MFCC and the inference all happen in `loop()`.

From there everything runs on the STM32H747 and BB15:

| stage | where | cost |
| --- | --- | --- |
| 16 kHz mono capture, 32 ms buffers | STM32H747 DFSDM | under 1 ms per 60 ms block |
| DC blocking and block RMS | STM32H747 | negligible |
| MFCC, 3 frames per block | STM32H747 | under 1 ms per block |
| classification, every third block | BB15 AKD1500 | 2 ms |

The front end matches BrainChip's spark firmware value for value: 16 kHz, a
320-sample hop, a 640-sample frame, a 640-point real FFT, 40 mel bins from
20 Hz to 4000 Hz, and 49 frames of 10 coefficients. The gating, smoothing,
threshold, debounce and chiming defaults are spark's, hard-coded rather than
configurable.

`mfcc.{h,cpp}`, `kiss_fft*.{c,h}`, `model_metadata.{h,cpp}` and `program.{h,cpp}`
are byte-identical copies of the Nicla Voice example's. They are board
independent, and keeping them identical rather than shared keeps each example a
self-contained Arduino sketch folder.

## Microphone gain

`PDM.setGain()` takes 0 to 8 on this board, and the library turns that into a
right shift of the DFSDM output, so only every third step changes anything. This
demo asks for 8, the loudest the library exposes.

That is deliberately not a tuning knob for the pipeline: `kRmsThreshold` is
spark's 550 and stays there. On the validated board speech at arm's length
reaches a block RMS of 1400 to 2600 against a quiet-room floor near 280, so the
gate opens on speech and stays shut on room noise. If a quieter room or a
further microphone leaves the SPEECH badge dark, move the board closer before
reaching for the threshold: lowering it lets room noise into the MFCC front end,
which is what the gate exists to prevent.

## Packets are built whole before they are sent

Every `Serial.write()` on native USB CDC is its own blocking USB transfer.
Writing a result packet field by field costs around 250 of them, sixteen times a
second, and a host that closes the port mid-packet can leave the sketch blocked
in that path for good: it goes silent and stays silent until it is reset.

So the sketch builds each packet in `g_packet` and hands it over in one
`Serial.write(buffer, size)`. This is the one place where it deliberately
departs from the Nicla Voice demo's shape, whose `Serial` is a UART bridged to
USB by a SAMD11 and never blocks that way.

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
- **dropped**, in the telemetry row: PDM buffers the sketch never saw, counted
  when the capture interrupt lands on a buffer the previous one has not been
  read out of yet. It carries the meaning the Nicla Voice demo's NDP120 chunk
  counter carries there.

Say a keyword close to the board. A detection needs the class to hold above
0.50 for three consecutive inferences, so a single ambiguous frame will not
fire.

## When the board looks dead

While it is not streaming, the sketch reports itself every two seconds, so a
plain terminal is enough to tell a healthy board from a wedged one:

```bash
PORT=/dev/cu.usbmodem101
stty -f $PORT 921600 raw && head -c 400 $PORT
```

| what you see | what it means |
| --- | --- |
| `idle ready=1 waiting_for_start_stream` | The board is fine, and the tool simply needs to ask again. |
| `idle result=FAIL status=0x..` | Bring-up failed and the sketch is saying which stage. `0x81` is the MFCC front end, `0x82` the PDM microphone, `0x83` BB15. |
| nothing at all | Nothing is running the sketch. Re-upload it. |

A `0x83` is most often mechanical: the BB15 header pins seat shallow in the
Nicla Vision sockets. `examples/bb15_dummy_inference_nicla_vision` is the
shortest way to confirm the link on its own, and it prints
`ip_version=0xBCA10309` once BB15 answers.

## Validation

Validated on a Nicla Vision with BB15 attached, Arduino mbed_nicla core 4.6.0:

- **BB15 SPI clock 25 MHz**, the value the Nicla Vision human-detection example
  uses. `begin()`, model load and inference were all reliable at it, so the
  8 MHz the Nicla Voice demo runs at was not needed here.
- Block cadence held at 16.6 blocks per second with `dropped` at zero across a
  two-minute continuous stream and 37 connect and disconnect cycles.
- All ten keywords were driven through the host's speakers with `say`, and all
  ten triggered detections. Reliability differs by word: of the four re-tested
  over four repeats each, `down`, `left` and `no` fired every time and `go`
  fired twice, `go` being the one this model most often confuses.
