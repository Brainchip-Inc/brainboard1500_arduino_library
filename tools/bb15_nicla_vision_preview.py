#!/usr/bin/env python3
"""Live USB preview for `bb15_nicla_vision_human_detection`.

Requirements:
  python3 -m pip install pyserial
  tkinter from the operating system's Python package

Usage:
  python3 tools/bb15_nicla_vision_preview.py --port /dev/ttyACM0
"""

from __future__ import annotations

import argparse
import struct
import sys
import time
import tkinter as tk
from dataclasses import dataclass
from typing import Optional, Tuple

import serial


MAGIC = b"BB15"
VERSION = 1
HEADER = struct.Struct("<4sBBI")
FRAME_METADATA = struct.Struct("<IHHBBBBiiHHH")
MAX_PAYLOAD_BYTES = 100_000

COMMAND_START_STREAM = 1
COMMAND_STOP_STREAM = 2
COMMAND_REQUEST_CONFIG = 3
MESSAGE_CONFIG = 0x81
MESSAGE_FRAME_RESULT = 0x82
MESSAGE_ERROR = 0x83
PIXEL_FORMAT_GRAY8 = 1


@dataclass
class CameraConfig:
    width: int
    height: int
    pixel_format: int
    model_width: int
    model_height: int


@dataclass
class FrameResult:
    sequence: int
    width: int
    height: int
    pixel_format: int
    predicted_index: int
    status: int
    score_count: int
    score0: int
    score1: int
    grab_ms: int
    prep_ms: int
    infer_ms: int
    pixels: bytes


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", required=True, help="Serial device, e.g. /dev/ttyACM0")
    parser.add_argument("--baud", type=int, default=115200, help="USB CDC baud setting")
    return parser.parse_args()


def read_exact(port: serial.Serial, count: int) -> bytes:
    data = bytearray()
    while len(data) < count:
        chunk = port.read(count - len(data))
        if not chunk:
            raise TimeoutError(f"timed out while reading {count} bytes")
        data.extend(chunk)
    return bytes(data)


def read_packet(port: serial.Serial) -> Tuple[int, bytes]:
    """Scan through any boot text until a complete framed protocol message."""
    window = bytearray()
    while True:
        byte = port.read(1)
        if not byte:
            raise TimeoutError("timed out waiting for BB15 protocol header")
        window.extend(byte)
        if len(window) > len(MAGIC):
            del window[0]
        if bytes(window) != MAGIC:
            continue
        remainder = read_exact(port, HEADER.size - len(MAGIC))
        version, message_type, payload_size = struct.unpack("<BBI", remainder)
        if version != VERSION:
            window.clear()
            continue
        if payload_size > MAX_PAYLOAD_BYTES:
            raise ValueError(f"invalid payload size: {payload_size}")
        return message_type, read_exact(port, payload_size)


def write_command(port: serial.Serial, command: int) -> None:
    port.write(HEADER.pack(MAGIC, VERSION, command, 0))
    port.flush()


def parse_config(payload: bytes) -> CameraConfig:
    if len(payload) != 9:
        raise ValueError(f"invalid config payload size: {len(payload)}")
    width, height, pixel_format, model_width, model_height = struct.unpack(
        "<HHBHH", payload
    )
    if pixel_format != PIXEL_FORMAT_GRAY8:
        raise ValueError(f"unsupported preview pixel format: {pixel_format}")
    return CameraConfig(width, height, pixel_format, model_width, model_height)


def parse_frame_result(payload: bytes) -> FrameResult:
    if len(payload) < FRAME_METADATA.size:
        raise ValueError("truncated frame-result payload")
    metadata = FRAME_METADATA.unpack(payload[: FRAME_METADATA.size])
    (
        sequence,
        width,
        height,
        pixel_format,
        predicted_index,
        status,
        score_count,
        score0,
        score1,
        grab_ms,
        prep_ms,
        infer_ms,
    ) = metadata
    pixels = payload[FRAME_METADATA.size :]
    if pixel_format != PIXEL_FORMAT_GRAY8:
        raise ValueError(f"unsupported preview pixel format: {pixel_format}")
    if len(pixels) != width * height:
        raise ValueError(f"invalid grayscale payload: {len(pixels)} bytes for {width}x{height}")
    return FrameResult(
        sequence,
        width,
        height,
        pixel_format,
        predicted_index,
        status,
        score_count,
        score0,
        score1,
        grab_ms,
        prep_ms,
        infer_ms,
        pixels,
    )


def label_for_prediction(index: int) -> str:
    if index == 0:
        return "No person"
    if index == 1:
        return "Person"
    return f"Class {index}"


def ppm(frame: FrameResult) -> bytes:
    header = f"P5\n{frame.width} {frame.height}\n255\n".encode("ascii")
    # PhotoImage accepts bytes directly. Do not decode raw pixel data to a
    # Unicode string: Tcl may UTF-8 encode values above 0x7F and shift the PGM
    # raster, producing the bright-region tearing seen in the preview.
    return header + frame.pixels


def connect(port_name: str, baud: int) -> Tuple[serial.Serial, CameraConfig]:
    port = serial.Serial(port_name, baud, timeout=3.0)
    # Opening USB CDC often resets the board. Let it print its boot banner,
    # then remove that text before negotiating the binary stream protocol.
    time.sleep(1.0)
    port.reset_input_buffer()
    write_command(port, COMMAND_REQUEST_CONFIG)
    while True:
        message_type, payload = read_packet(port)
        if message_type == MESSAGE_CONFIG:
            config = parse_config(payload)
            write_command(port, COMMAND_START_STREAM)
            return port, config
        if message_type == MESSAGE_ERROR:
            raise RuntimeError(f"device setup error: status={payload.hex()}")


def main() -> int:
    args = parse_args()
    root = tk.Tk()
    root.title("BB15 Nicla Vision Human Detection")
    root.geometry("920x760")
    root.minsize(700, 600)
    root.configure(bg="#e9edf0")

    header = tk.Frame(root, bg="#15232d", padx=24, pady=16)
    header.pack(fill="x")
    tk.Label(
        header,
        text="BB15  /  NICLA VISION",
        bg="#15232d",
        fg="#94c9cf",
        font=("TkDefaultFont", 10, "bold"),
    ).pack(anchor="w")
    tk.Label(
        header,
        text="Live human detection",
        bg="#15232d",
        fg="#f7fbfc",
        font=("TkDefaultFont", 20, "bold"),
    ).pack(anchor="w", pady=(2, 0))

    content = tk.Frame(root, bg="#e9edf0", padx=20, pady=18)
    content.pack(fill="both", expand=True)

    result_card = tk.Frame(content, bg="#ffffff", padx=16, pady=12)
    result_card.pack(fill="x", pady=(0, 14))
    result_title = tk.Label(
        result_card,
        text="CONNECTING",
        bg="#ffffff",
        fg="#62727b",
        font=("TkDefaultFont", 10, "bold"),
    )
    result_title.grid(row=0, column=0, sticky="w")
    prediction_label = tk.Label(
        result_card,
        text="Waiting for camera",
        bg="#ffffff",
        fg="#1f2d35",
        font=("TkDefaultFont", 18, "bold"),
    )
    prediction_label.grid(row=1, column=0, sticky="w", pady=(2, 0))
    score_label = tk.Label(
        result_card,
        text="Scores: --",
        bg="#ffffff",
        fg="#62727b",
        font=("TkDefaultFont", 10),
    )
    score_label.grid(row=1, column=1, sticky="e", padx=(24, 0))
    result_card.columnconfigure(0, weight=1)

    preview_card = tk.Frame(content, bg="#11191e", padx=8, pady=8)
    preview_card.pack(fill="both", expand=True)
    image_label = tk.Label(preview_card, bg="#11191e")
    image_label.pack(fill="both", expand=True)

    telemetry = tk.Frame(content, bg="#e9edf0", pady=12)
    telemetry.pack(fill="x")
    status_label = tk.Label(
        telemetry,
        anchor="w",
        bg="#e9edf0",
        fg="#52616a",
        font=("TkDefaultFont", 10),
        text="Connecting to camera...",
    )
    status_label.pack(side="left")
    connection_label = tk.Label(
        telemetry,
        anchor="e",
        bg="#e9edf0",
        fg="#287c68",
        font=("TkDefaultFont", 10, "bold"),
        text="USB CDC",
    )
    connection_label.pack(side="right")
    running = True

    def close_window() -> None:
        nonlocal running
        running = False

    root.protocol("WM_DELETE_WINDOW", close_window)
    port: Optional[serial.Serial] = None
    stream_started = False
    frame_count = 0
    last_frame_at = time.monotonic()

    try:
        while running:
            try:
                if port is None:
                    status_label.configure(text=f"Connecting to {args.port}...")
                    result_title.configure(text="CONNECTING", fg="#a16a1c")
                    prediction_label.configure(text="Waiting for camera", fg="#1f2d35")
                    score_label.configure(text="Scores: --")
                    connection_label.configure(text="USB CDC", fg="#a16a1c")
                    root.update()
                    port, config = connect(args.port, args.baud)
                    stream_started = True
                    status_label.configure(
                        text=(
                            f"Preview {config.width}x{config.height} grayscale  |  "
                            f"Model {config.model_width}x{config.model_height} RGB"
                        )
                    )
                    connection_label.configure(text="STREAMING", fg="#287c68")

                message_type, payload = read_packet(port)
                if message_type == MESSAGE_CONFIG:
                    continue
                if message_type == MESSAGE_ERROR:
                    raise RuntimeError(f"device runtime error: status={payload.hex()}")
                if message_type != MESSAGE_FRAME_RESULT:
                    continue
                frame = parse_frame_result(payload)
            except (serial.SerialException, TimeoutError, ValueError, RuntimeError) as exc:
                status_label.configure(text=f"Reconnecting: {exc}")
                result_title.configure(text="CONNECTION LOST", fg="#a84038")
                prediction_label.configure(text="Trying to reconnect", fg="#a84038")
                score_label.configure(text="Scores: --")
                connection_label.configure(text="RECONNECTING", fg="#a84038")
                root.update()
                if port is not None:
                    try:
                        port.close()
                    except serial.SerialException:
                        pass
                port = None
                stream_started = False
                time.sleep(0.5)
                continue

            image = tk.PhotoImage(data=ppm(frame), format="PPM")
            available_width = max(frame.width, root.winfo_width() - 20)
            available_height = max(frame.height, root.winfo_height() - 70)
            scale = max(1, min(available_width // frame.width, available_height // frame.height))
            if scale > 1:
                image = image.zoom(scale, scale)
            image_label.configure(image=image)
            image_label.image = image

            now = time.monotonic()
            fps = 1.0 / max(now - last_frame_at, 0.001)
            last_frame_at = now
            frame_count += 1
            is_person = frame.predicted_index == 1
            result_title.configure(
                text="PERSON DETECTED" if is_person else "NO PERSON DETECTED",
                fg="#1d8568" if is_person else "#62727b",
            )
            prediction_label.configure(
                text=label_for_prediction(frame.predicted_index),
                fg="#14765d" if is_person else "#1f2d35",
            )
            score_label.configure(text=f"Scores  no person {frame.score0}   person {frame.score1}")
            status = (
                f"Frame {frame.sequence}  |  Capture {frame.grab_ms} ms  |  "
                f"Preprocess {frame.prep_ms} ms  |  Inference {frame.infer_ms} ms  |  "
                f"Receive {fps:.1f} fps"
            )
            if frame.status != 0:
                status += f"  BB15 status={frame.status}"
            status_label.configure(text=status)
            root.update_idletasks()
            root.update()
    except tk.TclError:
        pass
    except KeyboardInterrupt:
        pass
    finally:
        if port is not None:
            try:
                if stream_started:
                    write_command(port, COMMAND_STOP_STREAM)
            except serial.SerialException:
                pass
            port.close()
        root.destroy()
    return 0


if __name__ == "__main__":
    sys.exit(main())
