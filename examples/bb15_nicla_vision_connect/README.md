# Nicla Vision Keyword Spotting and Human Detection over Bluetooth

This Nicla Vision-only example runs two demos driven entirely from the
BrainChip Connect phone app over Bluetooth Low Energy, instead of from the
desktop tool over USB. It advertises as `BrainBoard1500`, answers the app's
commands, receives both models from the app, and reports detections back.

**Keyword Spotting** recognises ten keywords: `down`, `go`, `left`, `no`,
`off`, `on`, `right`, `stop`, `up`, `yes`. The model also reports `silence` and
`unknown`, which are scored but can never trigger a detection. The audio
pipeline is the USB keyword spotting example's, value for value: the same DC
blocker, 960-sample blocks, RMS speech gate, MFCC front end, spectrogram ring,
inference period, smoothing, debounce and chiming.

**Vision Human Detection** scores a 96x96 crop of the camera against the visual
wake words model and reports `person` or `no_person` for every frame, at about
15 frames a second. The crop is the widest centred square of a 320x240 capture,
sampled down and rotated half a turn, because the model was trained with the
camera USB-side down.

What differs from the USB examples is the transport and where the models come
from.

**This example is not finished.** The camera preview frame is built and its
format is defined below, but the board does not send it yet: the format has
still to be measured against what reads as live over this link, and the app has
no renderer for it. Only detections go out today.

The BrainChip Connect app is currently private and will be released soon.

## The models this example needs

**No model is compiled into this sketch and none is committed here.** The app
sends each one over Bluetooth as a `.zip`, and the board writes it into BB15
external flash and loads it from there. A model survives a power cycle and a
firmware update, so it only has to be sent once.

The archive must hold `info.yaml` and a `<name>_program_info.bin` /
`<name>_program_data.bin` pair, which is the shape the app already reads.

Each application has its own 512 kB slot in external flash, so installing one
model leaves the other alone. **Which slot a transfer lands in is decided by
the input shape in `info.yaml`**, which the app writes before it sends any
bytes:

| `input_shape` | application | slot |
| --- | --- | --- |
| `[49, 10, 1]` | Keyword Spotting | 0 |
| `[96, 96, 3]` | Vision Human Detection | 1 |

Nothing else the app sends names the application, and the shape is the property
that settles which pipeline can feed the model. A model whose shape matches
neither is refused as soon as its program info has arrived, before any of the
model data is sent. `flash_address` is an offset within the slot, so both
applications use the same `0x1000`.

Only one program fits in the Akida fabric, so nothing is loaded until the app
asks for an application to run. Starting one is therefore a model swap, which
takes about 1.9 seconds for either model and is inside the 3 second window the
app allows.

**The model must be built for the Akida engine this library carries, which is
2.5.0.** A model built for another engine version is refused: the serialized
program format differs between engine versions, and both the engine and this
example check the version before loading. In particular **an AkidaTag model
will not load on this board**, because that firmware is built against Akida
2.17.0. The board reports the refusal over its serial port and keeps running;
it does not load a model it cannot trust.

## How the model arrives

The board speaks the same model transfer protocol as AkidaTag, so one path in
the app serves both boards. It is specified in `docs/ble-model-transfer.md` in
the AkidaTag firmware repository, which is the contract both sides are built
from; what follows is what a reader of this sketch needs.

A session is the metadata characteristics, then the program info, then the
model data, in that order on one connection. Each half is sent a block at a
time: every write to `f000aa01` opens with the four byte little-endian offset
of the bytes behind it, and at each block boundary the board answers on
`f000aa02` with a fourteen byte status record saying what it did and which byte
it expects next.

**The phone never assumes a block size.** The board reports its own in every
status, and here that is one flash sector, 4,096 bytes, because a sector is the
least the BrainBoard's flash can commit and therefore all of a model the board
ever has to hold. A block is erased, programmed and read back before the board
acknowledges it, so an acknowledged block is one that is genuinely in flash.

`f000aa03` carries the two control messages, START and ABORT. START names the
half and its length, so the separate file size and transfer type
characteristics the earlier protocol used are gone. There is one path and no
fallback, and an interrupted transfer is started again from zero rather than
resumed.

The `flash_address` in the package's `info.yaml` has to be a sector boundary
inside the 512 kB slot this example owns, and clear of the sector the record
sits in, so anything from 0x1000 up. An address outside that is refused at
START with `ERR_PARAM`; AkidaTag's own packages say 0x101000, which this board
will not take.

The board answers `DONE` when the whole file is stored and checked, and
`READY` only once it has programmed the BrainBoard with the model and scored
one inference with it. Those mean different things and the app shows them
differently: `DONE` is the progress bar reaching 100%, `READY` is the update
having worked.

Two things follow from committing a block at a time, and both are deliberate:

- **From the moment the data half starts, the board has no model.** The record
  naming the stored model is taken away before the first sector is erased,
  because a record describing bytes that are being overwritten is a lie. The
  board says so on its serial port, empties its application list, and stops
  scoring. A transfer that is then interrupted leaves the board with nothing to
  run, which is the truth, and the next boot reports exactly that rather than a
  corrupted model.
- **Nothing wedges.** A phone that disconnects part way, or simply stops
  writing, costs the board a staging block and its counters. It goes back to
  advertising and takes a fresh transfer.

ArduinoBLE gives a write handler no way to answer at ATT level, so the three
ATT errors the specification names are reported differently here: a malformed
control frame comes back as `ERR_PARAM` on the status characteristic, which is
more use to the app than an error code anyway, and a START that arrives before
the phone has subscribed to the status characteristic is refused in silence,
because a board that cannot report on a transfer cannot report that either.
The serial log says which happened.

## What a loaded model costs the sketch

Only the program info stays in memory. The engine reads the model data out of
BrainBoard flash as it runs, so the host holds 504 bytes for this keyword
model rather than the whole 22,112 byte program. Measured on the bench, the
heap with the keyword model loaded is 19,278 bytes, where holding the whole
program cost 40,980.

What that costs barely depends on the model. The same sketch was given the
183,884 byte human detection program, more than eight times the size, and the
heap went to 19,438 bytes: 160 bytes more, which is the difference between a
640 byte program info and a 504 byte one. A model's data half costs the host
nothing.

The same is true while a transfer is running: the board holds one 4,096 byte
block, not the model.

## Build and run

Compile and upload from the repository root:

```bash
arduino-cli compile --clean --fqbn arduino:mbed_nicla:nicla_vision --library . examples/bb15_nicla_vision_connect
arduino-cli upload --fqbn arduino:mbed_nicla:nicla_vision --port /dev/cu.usbmodem101 examples/bb15_nicla_vision_connect
```

Then open the app, pick `BrainBoard1500` from the device list, connect, and
send a model from the Model Update screen. Each model that arrives adds its
application to the home screen, so send both to get both; Run Application
starts whichever one you pick.

The serial port carries a short log of what the board is doing, at 115200 baud.
It is not needed to run the demo. Each boot opens with `previous_run=`, which
says whether the last run ended by being asked to restart (`commanded`), was
cut short without asking (`unexpected`), or had no predecessor because the
board lost power (`cold`). It is there to answer, without guessing, whether a
board that went quiet actually restarted.

## What the LED says

The RGB LED is driven directly and is active low.

| Colour | Meaning |
| --- | --- |
| Blue, one short flash every two seconds | advertising, no phone connected |
| Green, steady | a phone is connected |
| Blue, fast blink | a model transfer is running |
| Red, one short flash | a keyword was detected, or a person was seen |
| Red, slow blink | setup failed; the serial log says at which stage |

## Battery

The percentage comes from the board's MAX1726x fuel gauge and the charging
state from its PF1550 PMIC. `kBatteryCapacityMah` in `device_status.cpp` has to
match the cell actually fitted, or the percentage will be wrong.

Whether a cell is fitted at all cannot be read on this board while it runs from
USB: the gauge sits on the system rail, so with no cell it reports that rail as
a full, healthy battery and its battery-present bit agrees. A board on USB with
no cell therefore reads as a charged one.

## The camera preview frame

Defined here so the app can be built against it, though the board does not send
it yet. Preview frames ride the same notify characteristic as the microphone
waveform, `6e400003-b5a3-f393-e0a9-e50e24dcca9e`, and are told apart from the
text frames by the first byte and from the waveform by the second.

| offset | size | field |
| --- | --- | --- |
| 0 | 1 | magic, `0x42` |
| 1 | 1 | command, `0x0D` for a preview |
| 2 | 2 | uint16 LE image sequence, one per image |
| 4 | 2 | uint16 LE byte offset of this chunk within the image |
| 6 | 1 | image width in pixels |
| 7 | 1 | image height in pixels |
| 8 | 1 | pixel format, `0` for 8-bit grayscale |
| 9 | 1 | reserved, zero |
| 10 | .. | pixels, up to 229 of them |

Every chunk carries the geometry, so a reader needs no state beyond the image
it is assembling: start a new image of `width * height` bytes when the sequence
changes, write each chunk at its own offset, and render when it is full. An
image whose sequence is superseded before it fills should be dropped rather
than held, because the board always sends the newest frame and never queues.

## What the app cannot do against this board

The app's edge learning and firmware update screens will not work here, because
this example exposes neither service. Everything else the app offers is
supported: discovery, connection, device information, battery, the application
list, running and stopping the detector, the live microphone waveform, and the
six keyword spotting controls.

Leaving the device screen does not drop the Bluetooth link, so the board stays
connected and, as a peripheral serving one central, stops advertising. If the
app then reaches its "Device Disconnected" notice, its RECONNECT button cannot
find the board and restarting the app is what recovers. The board needs nothing
doing to it: its boot log will show no restart, and the LED stays green because
the link really is still up.
