# INA Measurement Tools

These helpers are intended for use with the
`examples/NiclaVisionInaTimerMonitor` sketch.

## Files

- `log_ina_csv.sh`
  Capture the example's serial output into a timestamped log file on the host.
- `plot_ina_csv.py`
  Turn a captured log into a multi-page PDF report with current, power, and
  timing plots.

## Capture Workflow

1. Upload `NiclaVisionInaTimerMonitor` to the connected Nicla Vision.
2. Start the logger script on the host.
3. Reset the board if you want the full boot metadata and full
   `idle_ready -> infer_loop` transition in the captured file.
4. Stop the logger with `Ctrl+C` after collecting enough data.
5. Run the plotter on the saved log file.

## Logger Usage

Default port:

```bash
./tools/log_ina_csv.sh
```

Specific serial port:

```bash
./tools/log_ina_csv.sh /dev/ttyACM1
```

By default the logger writes timestamped files under:

```text
~/bb15_ina_logs/
```

You can override the destination directory:

```bash
LOG_DIR=/tmp/bb15_logs ./tools/log_ina_csv.sh
```

## Plotter Usage

Dependencies:

- Python 3
- `matplotlib`
- `pandas`
- `seaborn`

Install them if needed:

```bash
pip install matplotlib pandas seaborn
```

Generate a PDF next to the captured log:

```bash
python3 tools/plot_ina_csv.py ~/bb15_ina_logs/bb15_ina_YYYYMMDD_HHMMSS.csv
```

Choose an explicit output file:

```bash
python3 tools/plot_ina_csv.py \
  ~/bb15_ina_logs/bb15_ina_YYYYMMDD_HHMMSS.csv \
  -o /tmp/bb15_ina_report.pdf
```

## Notes

- The plotter reads the mixed log directly, including the leading `# ...`
  metadata lines.
- The measurement example reports both converted values and raw INA fields so
  the capture can be audited if scaling questions come up later.
