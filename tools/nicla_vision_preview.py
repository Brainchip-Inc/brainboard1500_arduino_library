#!/usr/bin/env python3
"""
Desktop preview tool for the Nicla Vision serial/WebSerial frame protocol.

Requirements:
- Python 3
- pyserial

Usage:
  python3 nicla_vision_preview.py --port /dev/ttyACM0

Protocol:
- send byte 2 to request image mode + resolution
- send byte 1 to request one frame
- send byte 3 to start continuous streaming
- send byte 4 to stop continuous streaming
- frame payload is wrapped as:
    START = FA CE FE ED
    STOP  = DA BB AD 00
"""

from __future__ import annotations

import argparse
import sys
import time
import tkinter as tk
from dataclasses import dataclass

import serial


START_SEQUENCE = b"\xFA\xCE\xFE\xED"
STOP_SEQUENCE = b"\xDA\xBB\xAD\x00"
INFERENCE_RESULT_SEQUENCE = b"\xAC\x1D\x1A\xDA"
REQUEST_FRAME = b"\x01"
REQUEST_CONFIG = b"\x02"
REQUEST_STREAM_START = b"\x03"
REQUEST_STREAM_STOP = b"\x04"
CONFIG_RETRIES = 8
RECONNECT_DELAY_S = 1.0
OPEN_SETTLE_DELAY_S = 1.0
CONFIG_SCAN_TIMEOUT_S = 3.0

CAMERA_GRAYSCALE = 0
CAMERA_BAYER = 1
CAMERA_RGB565 = 2

CAMERA_R160x120 = 0
CAMERA_R320x240 = 1
CAMERA_R320x320 = 2
CAMERA_R640x480 = 3
CAMERA_R800x600 = 5
CAMERA_R1600x1200 = 6

RESOLUTION_MAP = {
    CAMERA_R160x120: (160, 120),
    CAMERA_R320x240: (320, 240),
    CAMERA_R320x320: (320, 320),
    CAMERA_R640x480: (640, 480),
    CAMERA_R800x600: (800, 600),
    CAMERA_R1600x1200: (1600, 1200),
}


@dataclass
class CameraConfig:
    image_mode: int
    resolution: int
    width: int
    height: int

    @property
    def bytes_per_pixel(self) -> int:
        if self.image_mode == CAMERA_RGB565:
            return 2
        return 1

    @property
    def frame_size(self) -> int:
        return self.width * self.height * self.bytes_per_pixel


@dataclass
class InferenceResult:
    ok: bool
    predicted_index: int
    akida_status: int
    score_count: int
    score0: int
    score1: int
    prep_ms: int | None
    inference_ms: int
    grab_ms: int | None


def label_for_prediction(predicted_index: int) -> str:
    if predicted_index == 1:
        return "Human present"
    if predicted_index == 0:
        return "No human"
    return f"Class {predicted_index}"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", required=True, help="Serial port, e.g. /dev/ttyACM0")
    parser.add_argument("--baud", type=int, default=115200, help="Serial baud rate")
    parser.add_argument(
        "--fps",
        type=float,
        default=30.0,
        help="Requested preview rate for legacy single-frame mode",
    )
    parser.add_argument(
        "--no-stream",
        action="store_true",
        help="Disable continuous stream mode and use single-frame polling",
    )
    parser.add_argument(
        "--assume-demo-config",
        action="store_true",
        help="Fall back to grayscale 320x240 when config negotiation fails",
    )
    return parser.parse_args()


def read_exact(ser: serial.Serial, size: int) -> bytes:
    data = bytearray()
    while len(data) < size:
        chunk = ser.read(size - len(data))
        if not chunk:
          raise TimeoutError(f"Timed out while reading {size} bytes")
        data.extend(chunk)
    return bytes(data)


def wait_for_sequence(ser: serial.Serial, sequence: bytes) -> None:
    window = bytearray()
    while True:
        byte = ser.read(1)
        if not byte:
            raise TimeoutError("Timed out while waiting for frame start")
        window.extend(byte)
        if len(window) > len(sequence):
            del window[0]
        if bytes(window) == sequence:
            return


def get_camera_config(ser: serial.Serial) -> CameraConfig:
    last_error: Exception | None = None
    for _ in range(CONFIG_RETRIES):
        try:
            ser.reset_input_buffer()
            ser.write(REQUEST_CONFIG)
            deadline = time.monotonic() + CONFIG_SCAN_TIMEOUT_S
            window = bytearray()
            image_mode = None
            resolution = None
            while time.monotonic() < deadline:
                chunk = ser.read(1)
                if not chunk:
                    continue
                window.extend(chunk)
                if len(window) > 2:
                    del window[0]
                if len(window) < 2:
                    continue
                candidate_mode = window[0]
                candidate_resolution = window[1]
                if candidate_mode not in (CAMERA_GRAYSCALE, CAMERA_BAYER, CAMERA_RGB565):
                    continue
                if candidate_resolution not in RESOLUTION_MAP:
                    continue
                image_mode = candidate_mode
                resolution = candidate_resolution
                break
            if image_mode is None or resolution is None:
                raise TimeoutError("Timed out while scanning for camera config bytes")
            width, height = RESOLUTION_MAP[resolution]
            return CameraConfig(
                image_mode=image_mode,
                resolution=resolution,
                width=width,
                height=height,
            )
        except (TimeoutError, ValueError, serial.SerialException) as exc:
            last_error = exc
            time.sleep(0.1)
    if last_error is None:
        raise TimeoutError("Timed out while reading camera config")
    raise last_error


def rgb565_to_rgb888(frame: bytes, width: int, height: int) -> bytes:
    out = bytearray(width * height * 3)
    j = 0
    for i in range(0, len(frame), 2):
        value = (frame[i] << 8) | frame[i + 1]
        r5 = (value >> 11) & 0x1F
        g6 = (value >> 5) & 0x3F
        b5 = value & 0x1F
        out[j] = (r5 * 255) // 31
        out[j + 1] = (g6 * 255) // 63
        out[j + 2] = (b5 * 255) // 31
        j += 3
    return bytes(out)


def frame_to_ppm_bytes(config: CameraConfig, frame: bytes) -> bytes:
    if config.image_mode == CAMERA_RGB565:
        rgb = rgb565_to_rgb888(frame, config.width, config.height)
        header = f"P6\n{config.width} {config.height}\n255\n".encode("ascii")
        return header + rgb
    if config.image_mode == CAMERA_GRAYSCALE:
        header = f"P5\n{config.width} {config.height}\n255\n".encode("ascii")
        return header + frame
    raise ValueError(f"Unsupported image mode for preview: {config.image_mode}")


def compute_integer_scale(
    root: tk.Tk, status_label: tk.Label, config: CameraConfig
) -> int:
    available_width = max(config.width, root.winfo_width() - 24)
    available_height = max(
        config.height,
        root.winfo_height()
        - status_label.winfo_height()
        - 24,
    )
    scale_x = max(1, available_width // config.width)
    scale_y = max(1, available_height // config.height)
    return max(1, min(scale_x, scale_y))


def read_framed_frame(ser: serial.Serial, config: CameraConfig) -> tuple[bytes, float]:
    read_start = time.monotonic()
    wait_for_sequence(ser, START_SEQUENCE)
    frame = read_exact(ser, config.frame_size)
    stop = read_exact(ser, len(STOP_SEQUENCE))
    if stop != STOP_SEQUENCE:
        raise ValueError(f"Unexpected frame terminator: {stop.hex()}")
    return frame, (time.monotonic() - read_start) * 1000.0


def request_frame(ser: serial.Serial, config: CameraConfig) -> tuple[bytes, float]:
    last_error: Exception | None = None
    for _ in range(CONFIG_RETRIES):
        try:
            ser.reset_input_buffer()
            request_start = time.monotonic()
            ser.write(REQUEST_FRAME)
            frame, _ = read_framed_frame(ser, config)
            return frame, (time.monotonic() - request_start) * 1000.0
        except (TimeoutError, ValueError, serial.SerialException) as exc:
            last_error = exc
            time.sleep(0.1)
    if last_error is None:
        raise TimeoutError("Timed out while requesting frame")
    raise last_error


def read_stream_frame(ser: serial.Serial, config: CameraConfig) -> tuple[bytes, float]:
    return read_framed_frame(ser, config)


def try_enable_stream_mode(ser: serial.Serial, config: CameraConfig) -> bool:
    try:
        ser.reset_input_buffer()
        ser.write(REQUEST_STREAM_START)
        read_stream_frame(ser, config)
        return True
    except (TimeoutError, ValueError, serial.SerialException):
        try:
            ser.write(REQUEST_STREAM_STOP)
        except serial.SerialException:
            pass
        ser.reset_input_buffer()
        return False


def try_read_inference_result(ser: serial.Serial) -> InferenceResult | None:
    previous_timeout = ser.timeout
    deadline = time.monotonic() + 0.25
    window = bytearray()
    ser.timeout = 0.05
    try:
        while time.monotonic() < deadline:
            byte = ser.read(1)
            if not byte:
                return None
            window.extend(byte)
            if len(window) > len(INFERENCE_RESULT_SEQUENCE):
                del window[0]
            if bytes(window) == INFERENCE_RESULT_SEQUENCE:
                break
        else:
            return None

        payload = bytearray()
        while len(payload) < 14:
            chunk = ser.read(14 - len(payload))
            if not chunk:
                raise TimeoutError("Timed out while reading inference payload")
            payload.extend(chunk)

        prep_ms: int | None = None
        inference_ms = int.from_bytes(payload[12:14], byteorder="little", signed=False)
        grab_ms: int | None = None

        extra = bytearray()
        while len(extra) < 4:
            chunk = ser.read(4 - len(extra))
            if not chunk:
                break
            extra.extend(chunk)

        if len(extra) == 2:
            grab_ms = int.from_bytes(extra, byteorder="little", signed=False)
        elif len(extra) == 4:
            prep_ms = inference_ms
            inference_ms = int.from_bytes(extra[0:2], byteorder="little", signed=False)
            grab_ms = int.from_bytes(extra[2:4], byteorder="little", signed=False)
    finally:
        ser.timeout = previous_timeout

    return InferenceResult(
        ok=payload[0] != 0,
        predicted_index=payload[1],
        akida_status=payload[2],
        score_count=payload[3],
        score0=int.from_bytes(payload[4:8], byteorder="little", signed=True),
        score1=int.from_bytes(payload[8:12], byteorder="little", signed=True),
        prep_ms=prep_ms,
        inference_ms=inference_ms,
        grab_ms=grab_ms,
    )


def open_serial(port: str, baud: int) -> serial.Serial:
    ser = serial.Serial(port, baud, timeout=2.0)
    time.sleep(OPEN_SETTLE_DELAY_S)
    ser.reset_input_buffer()
    return ser


def demo_fallback_config() -> CameraConfig:
    return CameraConfig(
        image_mode=CAMERA_GRAYSCALE,
        resolution=CAMERA_R320x240,
        width=320,
        height=240,
    )


def connect_with_config(
    port: str, baud: int, assume_demo_config: bool
) -> tuple[serial.Serial, CameraConfig]:
    while True:
        ser: serial.Serial | None = None
        try:
            ser = open_serial(port, baud)
            try:
                config = get_camera_config(ser)
            except (TimeoutError, ValueError) as exc:
                if not assume_demo_config:
                    raise
                config = demo_fallback_config()
                print(f"[preview] config fallback after negotiation failure: {exc}")
            print(
                f"[preview] config mode={config.image_mode} resolution={config.resolution} "
                f"width={config.width} height={config.height} frame_size={config.frame_size}"
            )
            return ser, config
        except (TimeoutError, ValueError, serial.SerialException) as exc:
            print(f"[preview] connect retry: {exc}")
            if ser is not None:
                ser.close()
            time.sleep(RECONNECT_DELAY_S)


def main() -> int:
    args = parse_args()

    root = tk.Tk()
    root.title("Nicla Vision Preview")
    root.geometry("960x720")

    label = tk.Label(root, text="Connecting...")
    label.pack(fill="x")

    image_label = tk.Label(root)
    image_label.pack(fill="both", expand=True)

    frame_count = 0
    frame_delay_s = 1.0 / args.fps if args.fps > 0 else 0.0
    ser, config = connect_with_config(args.port, args.baud, args.assume_demo_config)
    stream_mode = False
    if not args.no_stream:
        stream_mode = try_enable_stream_mode(ser, config)
        print(f"[preview] stream_mode={'on' if stream_mode else 'off'}")

    try:
        while True:
            try:
                loop_start = time.monotonic()
                if stream_mode:
                    frame, rx_ms = read_stream_frame(ser, config)
                else:
                    frame, rx_ms = request_frame(ser, config)
                inference = try_read_inference_result(ser)
            except (TimeoutError, ValueError, serial.SerialException) as exc:
                label.configure(text=f"Reconnecting after serial error: {exc}")
                root.update_idletasks()
                root.update()
                try:
                    ser.close()
                except serial.SerialException:
                    pass
                ser, config = connect_with_config(
                    args.port, args.baud, args.assume_demo_config
                )
                stream_mode = False
                if not args.no_stream:
                    stream_mode = try_enable_stream_mode(ser, config)
                    print(f"[preview] stream_mode={'on' if stream_mode else 'off'}")
                continue

            decode_start = time.monotonic()
            ppm = frame_to_ppm_bytes(config, frame)
            image = tk.PhotoImage(data=ppm.decode("latin1"), format="PPM")
            scale = compute_integer_scale(root, label, config)
            if scale > 1:
                image = image.zoom(scale, scale)
            decode_ms = (time.monotonic() - decode_start) * 1000.0

            ui_start = time.monotonic()
            image_label.configure(image=image)
            image_label.image = image

            frame_count += 1
            status = f"Frames: {frame_count}  Size: {config.width}x{config.height}"
            if inference is not None:
                if inference.ok and inference.score_count >= 2:
                    status += (
                        f"  {label_for_prediction(inference.predicted_index)}"
                        f"  Scores: [{inference.score0}, {inference.score1}]"
                    )
                    if inference.prep_ms is not None:
                        status += f"  Prep: {inference.prep_ms} ms"
                    status += f"  Infer: {inference.inference_ms} ms"
                    if inference.grab_ms is not None:
                        status += f"  Grab: {inference.grab_ms} ms"
                else:
                    status += f"  Infer: FAIL  status={inference.akida_status}"
                    if inference.prep_ms is not None:
                        status += f"  Prep: {inference.prep_ms} ms"
                    status += f"  Infer: {inference.inference_ms} ms"
                    if inference.grab_ms is not None:
                        status += f"  Grab: {inference.grab_ms} ms"
            ui_ms = (time.monotonic() - ui_start) * 1000.0
            host_ms = decode_ms + ui_ms
            total_ms = (time.monotonic() - loop_start) * 1000.0
            status += f"  RX: {rx_ms:.0f} ms  Host: {host_ms:.0f} ms"
            if stream_mode:
                status += f"  Loop: {total_ms:.0f} ms"
            label.configure(text=status)
            root.update_idletasks()
            root.update()
            if not stream_mode and frame_delay_s > 0.0:
                time.sleep(frame_delay_s)
    except tk.TclError as exc:
        print(f"[preview] tkinter error: {exc}")
        return 0
    except KeyboardInterrupt:
        print("[preview] interrupted")
        return 0
    finally:
        try:
            if stream_mode:
                ser.write(REQUEST_STREAM_STOP)
        except serial.SerialException:
            pass
        ser.close()


if __name__ == "__main__":
    sys.exit(main())
