#!/usr/bin/env python3
"""Live USB view for `bb15_nicla_voice_keyword_spotting`.

Requirements:
  python3 -m pip install pyserial
  tkinter from the operating system's Python package

Usage:
  python3 tools/bb15_nicla_voice_kws_gui.py --port /dev/cu.usbmodem9AD4C4763
"""

from __future__ import annotations

import argparse
import math
import struct
import sys
import time
import tkinter as tk
from collections import deque
from dataclasses import dataclass
from typing import Deque, List, Optional, Tuple

import serial


MAGIC = b"BB15"
VERSION = 1
HEADER = struct.Struct("<4sBBI")
AUDIO_CONFIG = struct.Struct("<IHHHBBHHHHHBB")
AUDIO_RESULT = struct.Struct("<IIHHHHHHBBBBBBBB")
COMMAND_START_STREAM = 1
COMMAND_STOP_STREAM = 2
COMMAND_REQUEST_CONFIG = 3
MESSAGE_ERROR = 0x83
MESSAGE_AUDIO_CONFIG = 0x84
MESSAGE_AUDIO_RESULT = 0x85
# Status values the device reports in an error packet. Anything else is the
# Syntiant interface library's own code from a failed chunk read.
DEVICE_ERRORS = {
    0x81: "the MFCC front end could not start",
    0x82: "the NDP120 microphone did not start, try a power cycle",
    0x83: "BB15 did not come up, check the board is seated",
}

# From spark's kws_new_tags[], matching the silence and unknown class indices
# in the model's info.yaml. Neither of those two can trigger a detection.
CLASS_LABELS = (
    "down", "go", "left", "no", "off", "on",
    "right", "stop", "up", "yes", "silence", "unknown",
)
SILENCE_CLASS = 10
UNKNOWN_CLASS = 11
# The device computes and transmits all twelve class scores, and the headless
# decoder still reads them, which is what makes "unknown dominates" a usable
# diagnostic. Silence and unknown are hidden here at the display only: neither
# can ever trigger a detection, so nothing this card reports depends on them.
DISPLAY_CLASSES = tuple(
    index
    for index in range(len(CLASS_LABELS))
    if index not in (SILENCE_CLASS, UNKNOWN_CLASS)
)
NO_PREDICTION = 0xFF
SCORE_FULL_SCALE = 32767.0
# How long a detection stays highlighted before the card goes back to waiting.
DETECTION_HOLD_S = 2.5
ERROR_PAYLOAD_BYTES = 1
MAX_WAVEFORM_POINTS = 255
MAX_MFCC_FRAMES = 255

FULL_SCALE = 32768.0
# The sketch spends about 10 s loading the NDP120 firmware packages from QSPI
# flash before it answers anything, and its receive ring buffer is only 256
# bytes and drops silently once full. Asking slowly keeps the backlog well under
# that, and re-arming the stream recovers a command dropped anyway.
CONFIG_REQUEST_INTERVAL_S = 1.5
CONFIG_TIMEOUT_S = 40.0
STREAM_ARM_INTERVAL_S = 2.5
# Blocks arrive every 60 ms, so a gap this long already means the device stopped
# and the view on screen is stale. Saying so beats leaving it reading as live.
STREAM_STALE_S = 0.6
STREAM_STALL_TIMEOUT_S = 4.0

# Live view geometry. Each 60 ms block contributes COLUMNS_PER_BLOCK columns, so
# the window spans WINDOW_COLUMNS / COLUMNS_PER_BLOCK blocks of audio.
COLUMNS_PER_BLOCK = 8
WINDOW_COLUMNS = 480

# The waveform scale follows the largest excursion currently on screen, so a
# burst that has scrolled away stops holding the trace small. Smoothing the
# change over several blocks keeps it from jolting, and the printed value is
# rounded to a 1-2-5 step so the caption does not churn every frame.
# The floor keeps a quiet room looking quiet. Without it the scale relaxes
# until the microphone's own noise floor, around 600 peak on this board, fills
# the panel and silence reads as a loud signal.
SCALE_FLOOR = 4000.0
SCALE_HEADROOM = 1.15
SCALE_ATTACK = 0.35
SCALE_RELEASE = 0.08
# A single click or the DC blocker settling can be an order of magnitude above
# speech, so the scale follows a high percentile of the visible columns instead
# of their maximum. Excursions past it are clamped to the panel edge.
SCALE_PERCENTILE = 0.98
SCALE_CAPTION_DIGITS = 2

METER_FLOOR_DBFS = -60.0

# Every label whose text changes gets a fixed width. A label that resizes when
# its text changes leaves fragments of the old string behind on macOS Tk, and a
# stable width also stops the row shuffling as numbers grow and shrink.
WIDTH_RESULT_TITLE = 18
WIDTH_PREDICTION = 26
WIDTH_SCORES = 16
WIDTH_VIEW_CAPTION = 52
WIDTH_SCALE_CAPTION = 20
WIDTH_LEVEL = 26
WIDTH_BADGE = 8
WIDTH_STATUS = 124
WIDTH_CONNECTION = 14

COLOR_PAGE = "#e9edf0"
COLOR_HEADER = "#15232d"
COLOR_HEADER_ACCENT = "#94c9cf"
COLOR_HEADER_TITLE = "#f7fbfc"
COLOR_CARD = "#ffffff"
COLOR_TEXT = "#1f2d35"
COLOR_MUTED = "#62727b"
COLOR_LIVE = "#11191e"
COLOR_LIVE_CAPTION = "#7f97a3"
COLOR_GRID = "#1e2c33"
COLOR_ZERO_LINE = "#31474f"
COLOR_WAVE_FILL = "#2f7d8a"
COLOR_WAVE_EDGE = "#6fd3de"
COLOR_WAVE_FILL_STALE = "#25373d"
COLOR_WAVE_EDGE_STALE = "#41565d"
COLOR_METER_TRACK = "#1b262b"
COLOR_METER_QUIET = "#3d7c88"
COLOR_METER_LOUD = "#35a37f"
COLOR_METER_TICK = "#c9d6db"
COLOR_BADGE_IDLE_BG = "#1e2c33"
COLOR_BADGE_IDLE_FG = "#7f97a3"
COLOR_BADGE_SPEECH_BG = "#1d7a5f"
COLOR_BADGE_SPEECH_FG = "#eafaf4"
COLOR_SCORE_BAR_KEYWORD = "#2f7d8a"
COLOR_SCORE_BAR_TRIGGERED = "#1d8568"
COLOR_SCORE_TRACK = "#e3e9ec"
COLOR_SCORE_THRESHOLD = "#93a6af"
METER_FLOOR_DBFS = -60.0

# Every label whose text changes gets a fixed width. A label that resizes when
# its text changes leaves fragments of the old string behind on macOS Tk, and a
# stable width also stops the row shuffling as numbers grow and shrink.
WIDTH_RESULT_TITLE = 18
WIDTH_PREDICTION = 26
WIDTH_SCORES = 16
WIDTH_VIEW_CAPTION = 52
WIDTH_SCALE_CAPTION = 20
WIDTH_LEVEL = 26
WIDTH_BADGE = 8
WIDTH_STATUS = 124
WIDTH_CONNECTION = 14

COLOR_PAGE = "#e9edf0"
COLOR_HEADER = "#15232d"
COLOR_HEADER_ACCENT = "#94c9cf"
COLOR_HEADER_TITLE = "#f7fbfc"
COLOR_CARD = "#ffffff"
COLOR_TEXT = "#1f2d35"
COLOR_MUTED = "#62727b"
COLOR_LIVE = "#11191e"
COLOR_LIVE_CAPTION = "#7f97a3"
COLOR_GRID = "#1e2c33"
COLOR_ZERO_LINE = "#31474f"
COLOR_WAVE_FILL = "#2f7d8a"
COLOR_WAVE_EDGE = "#6fd3de"
COLOR_WAVE_FILL_STALE = "#25373d"
COLOR_WAVE_EDGE_STALE = "#41565d"
COLOR_METER_TRACK = "#1b262b"
COLOR_METER_QUIET = "#3d7c88"
COLOR_METER_LOUD = "#35a37f"
COLOR_METER_TICK = "#c9d6db"
COLOR_BADGE_IDLE_BG = "#1e2c33"
COLOR_BADGE_IDLE_FG = "#7f97a3"
COLOR_BADGE_SPEECH_BG = "#1d7a5f"
COLOR_BADGE_SPEECH_FG = "#eafaf4"
COLOR_SCORE_BAR_KEYWORD = "#2f7d8a"
COLOR_SCORE_BAR_TRIGGERED = "#1d8568"
COLOR_SCORE_TRACK = "#e3e9ec"
COLOR_SCORE_THRESHOLD = "#93a6af"
COLOR_OK = "#287c68"
COLOR_WARN = "#a16a1c"
COLOR_ERROR = "#a84038"


@dataclass
class AudioConfig:
    """Pipeline description the device sends once a stream is negotiated."""

    sample_rate_hz: int
    block_samples: int
    waveform_points: int
    spectrogram_frames: int
    mfcc_coefficients: int
    class_count: int
    rms_threshold: int
    speech_active_time_ms: int
    smoothing_alpha_q15: int
    score_threshold_q15: int
    debounce_ms: int
    chiming_threshold: int

    @property
    def block_ms(self) -> float:
        """Duration of one audio block in milliseconds."""
        return 1000.0 * self.block_samples / self.sample_rate_hz


@dataclass
class AudioResult:
    """One completed audio block with its waveform envelope."""

    sequence: int
    device_ms: int
    rms: int
    peak: int
    dropped_chunks: int
    capture_ms: int
    feature_ms: int
    infer_ms: int
    speech_active: int
    status: int
    predicted_index: int
    score_count: int
    waveform_points: int
    mfcc_frames: int
    chiming_count: int
    detections: int
    envelope: Tuple[int, ...]
    # Kept because the parser has to walk past these bytes to reach the scores,
    # and because the headless decoder in the repository's development notes
    # reads them. The window no longer draws them.
    features: bytes
    scores: Tuple[float, ...]


def parse_args() -> argparse.Namespace:
    """Read the serial port and baud rate from the command line."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--port", required=True, help="Serial device, e.g. /dev/cu.usbmodem9AD4C4763"
    )
    parser.add_argument(
        "--baud", type=int, default=921600, help="USB CDC baud setting used by the sketch"
    )
    return parser.parse_args()


class PacketReader:
    """Reassemble framed BB15 protocol messages from a byte stream.

    Scanning a buffer rather than blocking on the port keeps the window
    responsive while the device is quiet, and skips the sketch's boot text and
    the NDP library's own progress output without treating it as an error.
    """

    def __init__(self) -> None:
        self._buffer = bytearray()

    def feed(self, data: bytes) -> None:
        """Add newly received bytes to the reassembly buffer."""
        self._buffer.extend(data)

    def next_packet(self) -> Optional[Tuple[int, bytes]]:
        """Return the next complete message, or None if one is not ready yet."""
        while True:
            start = self._buffer.find(MAGIC)
            if start < 0:
                # Keep the last few bytes in case they are a split magic value.
                del self._buffer[: max(0, len(self._buffer) - (len(MAGIC) - 1))]
                return None
            del self._buffer[:start]
            if len(self._buffer) < HEADER.size:
                return None
            version, message_type, payload_size = struct.unpack(
                "<BBI", self._buffer[len(MAGIC) : HEADER.size]
            )
            if version != VERSION or not plausible_header(message_type, payload_size):
                del self._buffer[:1]
                continue
            total = HEADER.size + payload_size
            if len(self._buffer) < total:
                return None
            payload = bytes(self._buffer[HEADER.size : total])
            del self._buffer[:total]
            return message_type, payload


def plausible_header(message_type: int, payload_size: int) -> bool:
    """Say whether a header could really start one of the device's messages.

    The four-byte magic occurs by chance inside waveform data, so a scan that
    starts mid-stream can lock onto a false header. Checking the type against
    the message set, and the size against what that message must be, keeps the
    scan hunting instead of consuming the rest of a real packet as a payload.
    """
    if message_type == MESSAGE_ERROR:
        return payload_size == ERROR_PAYLOAD_BYTES
    if message_type == MESSAGE_AUDIO_CONFIG:
        return payload_size == AUDIO_CONFIG.size
    if message_type == MESSAGE_AUDIO_RESULT:
        trailer = payload_size - AUDIO_RESULT.size
        return 0 < trailer <= 4 * MAX_WAVEFORM_POINTS + 10 * MAX_MFCC_FRAMES + 2 * 255
    return False


def describe_error(payload: bytes) -> str:
    """Turn an error payload into something worth putting on screen."""
    if len(payload) != ERROR_PAYLOAD_BYTES:
        return f"malformed error payload {payload.hex()}"
    status = payload[0]
    return DEVICE_ERRORS.get(status, f"device status {status}")


def write_command(port: serial.Serial, command: int) -> None:
    """Send a zero-payload command packet to the device."""
    port.write(HEADER.pack(MAGIC, VERSION, command, 0))
    port.flush()


def parse_audio_config(payload: bytes) -> AudioConfig:
    """Decode an audio-config payload.

    Raises:
        ValueError: If the payload size does not match the protocol.
    """
    if len(payload) != AUDIO_CONFIG.size:
        raise ValueError(f"invalid config payload size: {len(payload)}")
    fields = AUDIO_CONFIG.unpack(payload)
    return AudioConfig(*fields[:12])


def parse_audio_result(payload: bytes) -> AudioResult:
    """Decode an audio-result payload and its min/max waveform envelope.

    Raises:
        ValueError: If the payload is truncated or the envelope size is wrong.
    """
    if len(payload) < AUDIO_RESULT.size:
        raise ValueError("truncated audio-result payload")
    metadata = AUDIO_RESULT.unpack(payload[: AUDIO_RESULT.size])
    trailer = payload[AUDIO_RESULT.size :]
    score_count, waveform_points, mfcc_frames = metadata[11], metadata[12], metadata[13]
    envelope_bytes = 4 * waveform_points
    feature_bytes = 10 * mfcc_frames
    expected = envelope_bytes + feature_bytes + 2 * score_count
    if len(trailer) != expected:
        raise ValueError(
            f"invalid result trailer: {len(trailer)} bytes, expected {expected}"
        )
    envelope = struct.unpack(f"<{2 * waveform_points}h", trailer[:envelope_bytes])
    raw_scores = struct.unpack(
        f"<{score_count}h", trailer[envelope_bytes + feature_bytes :]
    )
    return AudioResult(
        *metadata[:15],
        detections=metadata[15],
        envelope=envelope,
        features=trailer[envelope_bytes : envelope_bytes + feature_bytes],
        scores=tuple(value / SCORE_FULL_SCALE for value in raw_scores),
    )


def dbfs(amplitude: float) -> float:
    """Convert a linear amplitude to dB relative to 16-bit full scale."""
    return 20.0 * math.log10(max(amplitude, 1.0) / FULL_SCALE)


def set_text(widget: tk.Label, text: str) -> None:
    """Update a label only when its text actually changes, avoiding redraws."""
    if widget.cget("text") != text:
        widget.configure(text=text)


def aggregate_columns(
    envelope: Tuple[int, ...], columns: int
) -> List[Tuple[int, int]]:
    """Reduce a block's min/max envelope to a fixed number of display columns.

    Args:
        envelope: Interleaved (min, max) pairs for the block.
        columns: Number of display columns to produce.

    Returns:
        One (min, max) tuple per column, in time order.
    """
    pairs = len(envelope) // 2
    per_column = max(1, pairs // columns)
    result = []
    for column in range(columns):
        first = column * per_column
        last = pairs if column == columns - 1 else min(pairs, first + per_column)
        lowest = min(envelope[2 * index] for index in range(first, last))
        highest = max(envelope[2 * index + 1] for index in range(first, last))
        result.append((lowest, highest))
    return result


class KeywordSpottingWindow:
    """The demo window: header, result card, live audio view and telemetry."""

    def __init__(self, root: tk.Tk, port_name: str) -> None:
        self._root = root
        self._port_name = port_name
        self._columns: Deque[Tuple[int, int]] = deque(
            [(0, 0)] * WINDOW_COLUMNS, maxlen=WINDOW_COLUMNS
        )
        self._scores_shown: Tuple[float, ...] = ()
        self._triggered_class = -1
        self._last_detections = -1
        self._detected_at = 0.0
        self._scale = SCALE_FLOOR
        self._level_dbfs = METER_FLOOR_DBFS
        self._live = True
        self._config: Optional[AudioConfig] = None
        self._canvas_width = 1
        self._canvas_height = 1

        root.title("BB15 Nicla Voice Keyword Spotting")
        root.geometry("980x720")
        root.minsize(880, 640)
        root.configure(bg=COLOR_PAGE)

        self._build_header(root)
        content = tk.Frame(root, bg=COLOR_PAGE, padx=20, pady=18)
        content.pack(fill="both", expand=True)
        self._build_result_card(content)
        self._build_live_view(content)
        self._build_telemetry(content)

    def _build_header(self, root: tk.Tk) -> None:
        """Create the dark banner naming the board and the demo."""
        header = tk.Frame(root, bg=COLOR_HEADER, padx=24, pady=16)
        header.pack(fill="x")
        tk.Label(
            header,
            text="BB15  /  NICLA VOICE",
            bg=COLOR_HEADER,
            fg=COLOR_HEADER_ACCENT,
            font=("TkDefaultFont", 10, "bold"),
        ).pack(anchor="w")
        tk.Label(
            header,
            text="Live keyword spotting",
            bg=COLOR_HEADER,
            fg=COLOR_HEADER_TITLE,
            font=("TkDefaultFont", 20, "bold"),
        ).pack(anchor="w", pady=(2, 0))

    def _build_result_card(self, parent: tk.Frame) -> None:
        """Create the card that shows the debounced prediction."""
        card = tk.Frame(parent, bg=COLOR_CARD, padx=16, pady=12)
        card.pack(fill="x", pady=(0, 14))
        card.columnconfigure(0, weight=1)
        card.columnconfigure(1, minsize=210)

        self._result_title = tk.Label(
            card,
            text="CONNECTING",
            bg=COLOR_CARD,
            fg=COLOR_WARN,
            font=("TkDefaultFont", 10, "bold"),
            anchor="w",
            width=WIDTH_RESULT_TITLE,
        )
        self._result_title.grid(row=0, column=0, sticky="w")
        tk.Label(
            card,
            text="CLASSIFIER",
            bg=COLOR_CARD,
            fg=COLOR_MUTED,
            font=("TkDefaultFont", 10, "bold"),
        ).grid(row=0, column=1, sticky="e")

        self._prediction_label = tk.Label(
            card,
            text="Waiting for microphone",
            bg=COLOR_CARD,
            fg=COLOR_TEXT,
            font=("TkDefaultFont", 18, "bold"),
            anchor="w",
            width=WIDTH_PREDICTION,
        )
        self._prediction_label.grid(row=1, column=0, sticky="w", pady=(2, 0))
        self._score_label = tk.Label(
            card,
            text="not loaded",
            bg=COLOR_CARD,
            fg=COLOR_MUTED,
            font=("TkFixedFont", 10),
            anchor="e",
            justify="right",
            width=WIDTH_SCORES,
        )
        self._score_label.grid(row=1, column=1, sticky="e", pady=(2, 0))

        self._scores = tk.Canvas(
            card, bg=COLOR_CARD, highlightthickness=0, bd=0, height=62
        )
        self._scores.grid(row=2, column=0, columnspan=2, sticky="ew", pady=(12, 0))
        self._scores.bind("<Configure>", self._on_scores_resize)
        self._score_threshold_line = self._scores.create_line(
            0, 0, 0, 0, fill=COLOR_SCORE_THRESHOLD
        )
        self._score_tracks = []
        self._score_bars = []
        self._score_names = []
        for class_index in DISPLAY_CLASSES:
            self._score_tracks.append(
                self._scores.create_rectangle(0, 0, 0, 0, fill=COLOR_SCORE_TRACK, width=0)
            )
            self._score_bars.append(
                self._scores.create_rectangle(
                    0, 0, 0, 0, fill=COLOR_SCORE_BAR_KEYWORD, width=0
                )
            )
            self._score_names.append(
                self._scores.create_text(
                    0, 0, text=CLASS_LABELS[class_index], fill=COLOR_MUTED,
                    anchor="n", font=("TkDefaultFont", 8),
                )
            )

    def _build_live_view(self, parent: tk.Frame) -> None:
        """Create the scrolling waveform and the signal level meter."""
        card = tk.Frame(parent, bg=COLOR_LIVE, padx=16, pady=12)
        card.pack(fill="both", expand=True)
        card.columnconfigure(0, weight=1)
        card.columnconfigure(1, minsize=170)
        card.columnconfigure(2, minsize=70)
        card.rowconfigure(1, weight=1)

        self._view_caption = tk.Label(
            card,
            text="MICROPHONE WAVEFORM",
            bg=COLOR_LIVE,
            fg=COLOR_LIVE_CAPTION,
            font=("TkDefaultFont", 9, "bold"),
            anchor="w",
            width=WIDTH_VIEW_CAPTION,
        )
        self._view_caption.grid(row=0, column=0, sticky="w")
        self._scale_label = tk.Label(
            card,
            text="full scale \u00b1512",
            bg=COLOR_LIVE,
            fg=COLOR_LIVE_CAPTION,
            font=("TkFixedFont", 9),
            anchor="e",
            width=WIDTH_SCALE_CAPTION,
        )
        self._scale_label.grid(row=0, column=1, sticky="e")
        # The speech gate used to be visible as the gap in the feature strip.
        # With the strip gone this badge is the only thing showing it, so it
        # sits with the waveform it gates rather than in the result card, where
        # a detection would hide it.
        self._speech_badge = tk.Label(
            card,
            text="IDLE",
            bg=COLOR_BADGE_IDLE_BG,
            fg=COLOR_BADGE_IDLE_FG,
            font=("TkDefaultFont", 9, "bold"),
            width=WIDTH_BADGE,
            padx=6,
            pady=2,
        )
        self._speech_badge.grid(row=0, column=2, sticky="e", padx=(12, 0))

        self._canvas = tk.Canvas(
            card,
            bg=COLOR_LIVE,
            highlightthickness=0,
            bd=0,
            height=120,
        )
        self._canvas.grid(row=1, column=0, columnspan=3, sticky="nsew", pady=(10, 12))
        self._canvas.bind("<Configure>", self._on_canvas_resize)
        self._grid_lines = [
            self._canvas.create_line(0, 0, 0, 0, fill=COLOR_GRID) for _ in range(2)
        ]
        self._zero_line = self._canvas.create_line(0, 0, 0, 0, fill=COLOR_ZERO_LINE)
        self._wave = self._canvas.create_polygon(
            0, 0, 0, 0, 0, 0, fill=COLOR_WAVE_FILL, outline=COLOR_WAVE_EDGE, width=1
        )

        meter_row = tk.Frame(card, bg=COLOR_LIVE)
        meter_row.grid(row=2, column=0, columnspan=3, sticky="ew")
        meter_row.columnconfigure(0, weight=1)
        self._meter = tk.Canvas(
            meter_row, bg=COLOR_LIVE, highlightthickness=0, bd=0, height=18
        )
        self._meter.grid(row=0, column=0, sticky="ew")
        self._meter.bind("<Configure>", self._on_meter_resize)
        self._meter_track = self._meter.create_rectangle(
            0, 0, 0, 0, fill=COLOR_METER_TRACK, width=0
        )
        self._meter_fill = self._meter.create_rectangle(
            0, 0, 0, 0, fill=COLOR_METER_QUIET, width=0
        )
        self._meter_tick = self._meter.create_line(0, 0, 0, 0, fill=COLOR_METER_TICK)
        self._level_label = tk.Label(
            meter_row,
            text="rms    --       -- dBFS",
            bg=COLOR_LIVE,
            fg=COLOR_LIVE_CAPTION,
            font=("TkFixedFont", 9),
            anchor="e",
            width=WIDTH_LEVEL,
        )
        self._level_label.grid(row=0, column=1, sticky="e", padx=(12, 0))

    def _build_telemetry(self, parent: tk.Frame) -> None:
        """Create the bottom row of live numbers and the connection state."""
        telemetry = tk.Frame(parent, bg=COLOR_PAGE, padx=16, pady=12)
        telemetry.pack(fill="x")
        telemetry.columnconfigure(0, weight=1)
        self._status_label = tk.Label(
            telemetry,
            anchor="w",
            bg=COLOR_PAGE,
            fg=COLOR_MUTED,
            font=("TkFixedFont", 10),
            text=f"Connecting to {self._port_name}",
            width=WIDTH_STATUS,
        )
        self._status_label.grid(row=0, column=0, sticky="w")
        self._connection_label = tk.Label(
            telemetry,
            anchor="e",
            bg=COLOR_PAGE,
            fg=COLOR_WARN,
            font=("TkDefaultFont", 10, "bold"),
            text="USB CDC",
            width=WIDTH_CONNECTION,
        )
        self._connection_label.grid(row=0, column=1, sticky="e")

    def _on_scores_resize(self, event: tk.Event) -> None:
        """Lay the twelve class bars across the width available."""
        width = max(event.width, 1)
        self._score_bar_height = max(event.height - 16, 1)
        slot = width / len(DISPLAY_CLASSES)
        bar_width = max(slot - 8.0, 2.0)
        for index in range(len(DISPLAY_CLASSES)):
            left = index * slot + (slot - bar_width) / 2.0
            self._scores.coords(
                self._score_tracks[index], left, 0, left + bar_width,
                self._score_bar_height,
            )
            self._scores.coords(
                self._score_names[index], left + bar_width / 2.0,
                self._score_bar_height + 3,
            )
        threshold_y = self._score_bar_height * (1.0 - 0.5)
        self._scores.coords(self._score_threshold_line, 0, threshold_y, width, threshold_y)
        self._redraw_scores()

    def _redraw_scores(self) -> None:
        """Repaint the class bars from the last scores received."""
        height = getattr(self, "_score_bar_height", 1)
        for slot, class_index in enumerate(DISPLAY_CLASSES):
            score = (
                self._scores_shown[class_index]
                if class_index < len(self._scores_shown)
                else 0.0
            )
            box = self._scores.coords(self._score_tracks[slot])
            if len(box) != 4:
                continue
            left, _, right, bottom = box
            top = bottom - height * max(0.0, min(1.0, score))
            self._scores.coords(self._score_bars[slot], left, top, right, bottom)
            fill = (
                COLOR_SCORE_BAR_TRIGGERED
                if class_index == self._triggered_class
                else COLOR_SCORE_BAR_KEYWORD
            )
            if self._scores.itemcget(self._score_bars[slot], "fill") != fill:
                self._scores.itemconfigure(self._score_bars[slot], fill=fill)

    def _on_canvas_resize(self, event: tk.Event) -> None:
        """Remember the waveform canvas size and lay out its static items."""
        self._canvas_width = max(event.width, 1)
        self._canvas_height = max(event.height, 1)
        middle = self._canvas_height / 2.0
        self._canvas.coords(self._zero_line, 0, middle, self._canvas_width, middle)
        for index, line in enumerate(self._grid_lines):
            offset = self._canvas_height * (1 + 2 * index) / 4.0
            self._canvas.coords(line, 0, offset, self._canvas_width, offset)
        self._redraw_waveform()

    def _on_meter_resize(self, event: tk.Event) -> None:
        """Lay out the level meter track for a new width."""
        self._meter.coords(self._meter_track, 0, 4, max(event.width, 1), 14)
        self._redraw_meter()

    def _mark_stale(self) -> None:
        """Dim the trace, so a view that is not being fed cannot read as live."""
        if not self._live:
            return
        self._live = False
        self._canvas.itemconfigure(
            self._wave, fill=COLOR_WAVE_FILL_STALE, outline=COLOR_WAVE_EDGE_STALE
        )

    def _clear_live_view(self) -> None:
        """Blank the waveform and meter so a dead link cannot look live."""
        self._mark_stale()
        self._columns = deque([(0, 0)] * WINDOW_COLUMNS, maxlen=WINDOW_COLUMNS)
        self._scale = SCALE_FLOOR
        self._level_dbfs = METER_FLOOR_DBFS
        self._set_speech_badge(False)
        self._redraw_waveform()
        self._redraw_meter()
        set_text(self._scale_label, self._scale_caption())
        set_text(self._level_label, "rms    --       -- dBFS")

    def set_connecting(self) -> None:
        """Show that the tool is waiting for the device to answer."""
        self._clear_live_view()
        self._result_title.configure(text="CONNECTING", fg=COLOR_WARN)
        self._prediction_label.configure(
            text="Waiting for microphone", fg=COLOR_TEXT
        )
        self._score_label.configure(text="not loaded")
        self._connection_label.configure(text="CONNECTING", fg=COLOR_WARN)
        set_text(
            self._status_label,
            f"Connecting to {self._port_name}, about 10 s while the sketch boots",
        )

    def set_reconnecting(self, reason: str) -> None:
        """Show that the link dropped and the tool is retrying."""
        self._clear_live_view()
        self._result_title.configure(text="CONNECTION LOST", fg=COLOR_ERROR)
        self._prediction_label.configure(text="Trying to reconnect", fg=COLOR_ERROR)
        self._score_label.configure(text="no link")
        self._connection_label.configure(text="RECONNECTING", fg=COLOR_ERROR)
        set_text(self._status_label, f"Reconnecting: {reason}"[:WIDTH_STATUS])

    def set_waiting(self) -> None:
        """Say that blocks stopped arriving, so the view on screen is stale."""
        if not self._live:
            return
        self._mark_stale()
        self._result_title.configure(text="NO AUDIO", fg=COLOR_WARN)
        self._connection_label.configure(text="WAITING", fg=COLOR_WARN)

    def _show_prediction(self, result: AudioResult) -> None:
        """Render the debounced prediction and the twelve class scores."""
        self._scores_shown = result.scores
        if result.detections != self._last_detections:
            self._last_detections = result.detections
            if result.predicted_index != NO_PREDICTION:
                self._detected_at = time.monotonic()
        fresh = time.monotonic() - self._detected_at < DETECTION_HOLD_S
        detected = result.predicted_index != NO_PREDICTION and fresh
        self._triggered_class = result.predicted_index if detected else -1
        self._redraw_scores()

        if not result.scores:
            return
        top = max(DISPLAY_CLASSES, key=lambda index: result.scores[index])
        if result.scores[top] <= 0.0:
            # Nothing has been classified yet, so naming a class would be a lie.
            set_text(self._score_label, f"{'--':>8s}  ----")
        else:
            set_text(
                self._score_label,
                f"{CLASS_LABELS[top]:>8s} {result.scores[top]:5.2f}",
            )
        if detected:
            self._result_title.configure(text="KEYWORD DETECTED", fg=COLOR_OK)
            self._prediction_label.configure(
                text=CLASS_LABELS[result.predicted_index], fg=COLOR_OK
            )
        else:
            self._prediction_label.configure(text="Say a keyword", fg=COLOR_MUTED)

    def _mark_live(self) -> None:
        """Restore the live look after blocks start arriving again."""
        if self._live:
            return
        self._live = True
        self._canvas.itemconfigure(
            self._wave, fill=COLOR_WAVE_FILL, outline=COLOR_WAVE_EDGE
        )
        self._connection_label.configure(text="STREAMING", fg=COLOR_OK)

    def set_streaming(self, config: AudioConfig) -> None:
        """Adopt the device's pipeline description and show the listening state."""
        self._config = config
        self._clear_live_view()
        self._mark_live()
        self._scores_shown = ()
        self._triggered_class = -1
        self._last_detections = -1
        self._redraw_scores()
        set_text(
            self._view_caption,
            f"MICROPHONE WAVEFORM   {self._window_seconds():.1f} s window   "
            f"{config.sample_rate_hz} Hz mono",
        )
        if config.class_count == 0:
            self._prediction_label.configure(text="Awaiting model", fg=COLOR_MUTED)
            self._score_label.configure(text="12 classes pending")
        else:
            self._prediction_label.configure(text="Say a keyword", fg=COLOR_MUTED)
        self._redraw_meter()

    def _window_seconds(self) -> float:
        """Length of audio the live view holds, in seconds."""
        assert self._config is not None
        return WINDOW_COLUMNS / COLUMNS_PER_BLOCK * self._config.block_ms / 1000.0

    def _update_scale(self) -> None:
        """Fit the vertical scale to the largest excursion now on screen."""
        magnitudes = sorted(
            max(abs(lowest), abs(highest)) for lowest, highest in self._columns
        )
        index = min(len(magnitudes) - 1, int(SCALE_PERCENTILE * len(magnitudes)))
        target = max(SCALE_FLOOR, magnitudes[index] * SCALE_HEADROOM)
        weight = SCALE_ATTACK if target > self._scale else SCALE_RELEASE
        self._scale += weight * (target - self._scale)

    def _scale_caption(self) -> str:
        """State the current vertical scale, rounded so the caption stays calm."""
        unit = 10 ** max(
            0, int(math.log10(self._scale)) - (SCALE_CAPTION_DIGITS - 1)
        )
        return f"full scale \u00b1{int(round(self._scale / unit) * unit)}"

    def _redraw_waveform(self) -> None:
        """Repaint the envelope polygon for the current column history."""
        width = self._canvas_width
        height = self._canvas_height
        middle = height / 2.0
        span = height / 2.0 - 2.0
        columns = len(self._columns)
        if columns < 2 or width < 2:
            return
        upper: List[float] = []
        lower: List[float] = []
        for index, (lowest, highest) in enumerate(self._columns):
            x = index * (width - 1) / (columns - 1)
            upper.append(x)
            upper.append(middle - span * max(-1.0, min(1.0, highest / self._scale)))
            lower.append(x)
            lower.append(middle - span * max(-1.0, min(1.0, lowest / self._scale)))
        # Walk the upper envelope forward and the lower one back so the polygon
        # closes into a filled waveform body.
        coordinates = upper + [
            value
            for index in range(columns - 1, -1, -1)
            for value in (lower[2 * index], lower[2 * index + 1])
        ]
        self._canvas.coords(self._wave, *coordinates)

    def _set_speech_badge(self, active: bool) -> None:
        """Show whether spark's speech gate is passing audio to the front end."""
        wanted = "SPEECH" if active else "IDLE"
        if self._speech_badge.cget("text") == wanted:
            return
        self._speech_badge.configure(
            text=wanted,
            bg=COLOR_BADGE_SPEECH_BG if active else COLOR_BADGE_IDLE_BG,
            fg=COLOR_BADGE_SPEECH_FG if active else COLOR_BADGE_IDLE_FG,
        )

    def _redraw_meter(self) -> None:
        """Repaint the level meter bar and its threshold tick."""
        width = max(self._meter.winfo_width(), 1)
        if width < 2:
            return
        self._meter.coords(self._meter_track, 0, 4, width, 14)
        level = self._level_dbfs
        fraction = max(0.0, min(1.0, (level - METER_FLOOR_DBFS) / -METER_FLOOR_DBFS))
        self._meter.coords(self._meter_fill, 0, 4, width * fraction, 14)
        if self._config is None:
            self._meter.coords(self._meter_tick, 0, 0, 0, 0)
            return
        threshold = dbfs(self._config.rms_threshold)
        tick = max(
            0.0, min(1.0, (threshold - METER_FLOOR_DBFS) / -METER_FLOOR_DBFS)
        )
        self._meter.coords(self._meter_tick, width * tick, 1, width * tick, 17)
        wanted = COLOR_METER_LOUD if level >= threshold else COLOR_METER_QUIET
        if self._meter.itemcget(self._meter_fill, "fill") != wanted:
            self._meter.itemconfigure(self._meter_fill, fill=wanted)

    def show_result(self, result: AudioResult, block_rate: float) -> None:
        """Render one audio block: waveform, level meter and telemetry."""
        assert self._config is not None
        self._mark_live()
        self._show_prediction(result)
        if self._triggered_class < 0:
            self._result_title.configure(
                text="SPEECH" if result.speech_active else "LISTENING",
                fg=COLOR_OK if result.speech_active else COLOR_MUTED,
            )
        self._columns.extend(aggregate_columns(result.envelope, COLUMNS_PER_BLOCK))
        self._update_scale()
        self._level_dbfs = dbfs(result.rms)
        self._set_speech_badge(bool(result.speech_active))
        self._redraw_waveform()
        self._redraw_meter()

        set_text(self._scale_label, self._scale_caption())
        set_text(
            self._level_label, f"rms {result.rms:5d}   {self._level_dbfs:6.1f} dBFS"
        )
        status = (
            f"block {result.sequence:<8d}peak {result.peak:5d}   "
            f"dropped {result.dropped_chunks:<5d}capture {result.capture_ms:3d} ms   "
            f"features {result.feature_ms:3d} ms   "
            f"inference {result.infer_ms:3d} ms   {block_rate:4.1f} blk/s"
        )
        if result.status != 0:
            status += f"   BB15 status={result.status}"
        set_text(self._status_label, status)


def read_available(port: serial.Serial) -> bytes:
    """Read whatever the port has buffered without stalling the window."""
    pending = port.in_waiting
    return port.read(pending if pending else 1)


def negotiate_stream(
    port: serial.Serial, reader: PacketReader, window: KeywordSpottingWindow,
    root: tk.Tk
) -> AudioConfig:
    """Ask the device for its pipeline description and start its stream.

    Keeps the window repainting while it waits, because the sketch does not
    answer until it has loaded the NDP120 firmware packages.

    Returns:
        The configuration the device reported.

    Raises:
        TimeoutError: If the device never answers.
        RuntimeError: If the device reports an error instead.
    """
    deadline = time.monotonic() + CONFIG_TIMEOUT_S
    next_request_at = 0.0
    while True:
        now = time.monotonic()
        if now > deadline:
            raise TimeoutError("device never answered a config request")
        if now >= next_request_at:
            write_command(port, COMMAND_REQUEST_CONFIG)
            next_request_at = now + CONFIG_REQUEST_INTERVAL_S
        reader.feed(read_available(port))
        while True:
            packet = reader.next_packet()
            if packet is None:
                break
            message_type, payload = packet
            if message_type == MESSAGE_AUDIO_CONFIG:
                config = parse_audio_config(payload)
                write_command(port, COMMAND_START_STREAM)
                return config
            if message_type == MESSAGE_ERROR:
                raise RuntimeError(f"device setup error: status={payload.hex()}")
        root.update()


def main() -> int:
    """Open the device, then stream and draw audio blocks until the window closes."""
    args = parse_args()
    root = tk.Tk()
    window = KeywordSpottingWindow(root, args.port)
    running = True

    def close_window() -> None:
        nonlocal running
        running = False

    root.protocol("WM_DELETE_WINDOW", close_window)
    port: Optional[serial.Serial] = None
    reader = PacketReader()
    stream_started = False
    last_result_at = time.monotonic()
    last_arm_at = 0.0
    block_interval = 0.06

    try:
        while running:
            try:
                if port is None:
                    window.set_connecting()
                    root.update()
                    port = serial.Serial(args.port, args.baud, timeout=0.05)
                    port.reset_input_buffer()
                    reader = PacketReader()
                    config = negotiate_stream(port, reader, window, root)
                    window.set_streaming(config)
                    stream_started = True
                    last_result_at = time.monotonic()
                    last_arm_at = time.monotonic()

                reader.feed(read_available(port))
                packet = reader.next_packet()
                if packet is None:
                    now = time.monotonic()
                    if now - last_result_at > STREAM_STALL_TIMEOUT_S:
                        raise TimeoutError("no audio blocks arrived")
                    if now - last_result_at > STREAM_STALE_S:
                        window.set_waiting()
                    # A start command lost to the device's small receive buffer
                    # looks exactly like silence, so re-arm until blocks appear.
                    if now - last_arm_at > STREAM_ARM_INTERVAL_S:
                        write_command(port, COMMAND_START_STREAM)
                        last_arm_at = now
                    root.update()
                    continue

                message_type, payload = packet
                if message_type == MESSAGE_ERROR:
                    raise RuntimeError(f"device reports {describe_error(payload)}")
                if message_type != MESSAGE_AUDIO_RESULT:
                    continue
                result = parse_audio_result(payload)
            except (
                serial.SerialException,
                TimeoutError,
                ValueError,
                RuntimeError,
                OSError,
            ) as exc:
                window.set_reconnecting(str(exc))
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

            now = time.monotonic()
            block_interval = 0.8 * block_interval + 0.2 * max(
                now - last_result_at, 0.001
            )
            last_result_at = now
            last_arm_at = now
            window.show_result(result, 1.0 / block_interval)
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
        try:
            root.destroy()
        except tk.TclError:
            pass
    return 0


if __name__ == "__main__":
    sys.exit(main())
