# Nicla Vision Keyword Spotting over Bluetooth

This Nicla Vision-only example runs keyword spotting driven entirely from the
BrainChip Connect phone app over Bluetooth Low Energy, instead of from the
desktop tool over USB. It advertises as `BrainBoard1500`, answers the app's
commands, receives its model from the app, and reports detections back.

It recognises ten keywords: `down`, `go`, `left`, `no`, `off`, `on`, `right`,
`stop`, `up`, `yes`. The model also reports `silence` and `unknown`, which are
scored but can never trigger a detection.

The audio pipeline is the USB keyword spotting example's, value for value: the
same DC blocker, 960-sample blocks, RMS speech gate, MFCC front end,
spectrogram ring, inference period, smoothing, debounce and chiming. What
differs is the transport and where the model comes from.

The BrainChip Connect app is currently private and will be released soon.

## The model this example needs

**No model is compiled into this sketch and none is committed here.** The app
sends one over Bluetooth as a `.zip`, and the board writes it into BB15
external flash and loads it from there. The model survives a power cycle and a
firmware update, so it only has to be sent once.

The archive must hold `info.yaml` and a `<name>_program_info.bin` /
`<name>_program_data.bin` pair, which is the shape the app already reads.

**The model must be built for the Akida engine this library carries, which is
2.5.0.** A model built for another engine version is refused: the serialized
program format differs between engine versions, and both the engine and this
example check the version before loading. In particular **an AkidaTag model
will not load on this board**, because that firmware is built against Akida
2.17.0. The board reports the refusal over its serial port and keeps running;
it does not load a model it cannot trust.

## Build and run

Compile and upload from the repository root:

```bash
arduino-cli compile --clean --fqbn arduino:mbed_nicla:nicla_vision --library . examples/bb15_nicla_vision_connect
arduino-cli upload --fqbn arduino:mbed_nicla:nicla_vision --port /dev/cu.usbmodem101 examples/bb15_nicla_vision_connect
```

Then open the app, pick `BrainBoard1500` from the device list, connect, and
send a model from the Model Update screen. Once the model is installed the
Keyword Spotting application appears on the home screen; Run Application starts
the detector.

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
| Red, one short flash | a keyword was detected |
| Red, slow blink | setup failed; the serial log says at which stage |

## Battery

The percentage comes from the board's MAX1726x fuel gauge and the charging
state from its PF1550 PMIC. `kBatteryCapacityMah` in `device_status.cpp` has to
match the cell actually fitted, or the percentage will be wrong.

Whether a cell is fitted at all cannot be read on this board while it runs from
USB: the gauge sits on the system rail, so with no cell it reports that rail as
a full, healthy battery and its battery-present bit agrees. A board on USB with
no cell therefore reads as a charged one.

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
