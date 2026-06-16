#!/usr/bin/env bash
set -euo pipefail

PORT="${1:-/dev/ttyACM0}"
BAUD="${BAUD:-115200}"
LOG_DIR="${LOG_DIR:-$HOME/bb15_ina_logs}"

mkdir -p "$LOG_DIR"
STAMP="$(date +%Y%m%d_%H%M%S)"
LOG_FILE="$LOG_DIR/bb15_ina_${STAMP}.csv"

echo "Waiting for serial port: $PORT"
while [[ ! -e "$PORT" ]]; do
  sleep 0.2
done

echo "Logging to: $LOG_FILE"
echo "Press Ctrl+C to stop."
echo "Start this script first, then reset the Nicla to capture the full boot log."

python - "$PORT" "$BAUD" "$LOG_FILE" <<'PY'
import sys
import serial

port = sys.argv[1]
baud = int(sys.argv[2])
log_file = sys.argv[3]

ser = serial.Serial(port, baud, timeout=1)
try:
    with open(log_file, "w", buffering=1) as f:
        while True:
            line = ser.readline()
            if not line:
                continue
            text = line.decode("utf-8", errors="replace")
            print(text, end="")
            f.write(text)
finally:
    ser.close()
PY
