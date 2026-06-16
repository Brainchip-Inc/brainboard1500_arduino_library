# NiclaVisionInaTimerMonitor

Timer-driven INA current monitor for the Nicla Vision + BB15 stack.

This example is intended for power characterization of the shipped
external-flash camera classification flow. It brings up BB15, INA, camera, and
Akida, then automatically runs two phases:

1. `idle_ready`
   Akida is initialized and the model is loaded, but no inference is running.
2. `infer_loop`
   The camera capture, preprocess, and classification loop runs continuously.

The example uses an `mbed::Ticker` only to schedule sampling deadlines. INA
reads happen in the main loop, and only outside the synchronous Akida
classification call. That keeps the CSV trace informative without forcing I2C
polling into the inference critical path.

## What It Prints

The serial output begins with `# ...` metadata lines and then emits one CSV row
per serviced INA sample.

Important fields include:

- `phase`
- `ticks_due`
- `service_lag_us`
- `grab_ms`
- `prep_ms`
- `infer_ms`
- `ch1_raw_shunt`
- `ch1_shunt_uv`
- `ch1_current_a`
- `ch1_current_ma`
- `ch1_power_w`
- the matching `ch2_*` fields

## How To Run

Open:

```text
File -> Examples -> AKD1500 -> NiclaVisionInaTimerMonitor
```

Build and upload it to a Nicla Vision connected to the BB15 board.

Open Serial Monitor at `115200` baud. The example starts by itself and will
transition from `idle_ready` into `infer_loop` automatically.

If you use `arduino-cli`:

```bash
arduino-cli compile --fqbn arduino:mbed_nicla:nicla_vision \
  --library /path/to/BrainBoard15_arduino_library \
  /path/to/BrainBoard15_arduino_library/examples/NiclaVisionInaTimerMonitor

arduino-cli upload -p /dev/ttyACM0 --fqbn arduino:mbed_nicla:nicla_vision \
  /path/to/BrainBoard15_arduino_library/examples/NiclaVisionInaTimerMonitor
```

## Logging And Plotting

The recommended host-side helpers live in the library `tools/` folder:

- `tools/log_ina_csv.sh`
- `tools/plot_ina_csv.py`

See [`tools/README.md`](../../tools/README.md) for the capture and plotting
workflow.
