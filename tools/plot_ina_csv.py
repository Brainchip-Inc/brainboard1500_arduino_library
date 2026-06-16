#!/usr/bin/env python3
"""Render BB15 INA CSV captures into a polished PDF report.

This script reads the mixed output produced by `log_ina_csv.sh`:
- `# ...` metadata lines
- one CSV header line
- repeated `bb15_ina_csv,...` rows

It generates a multi-page PDF intended for reports, bench notes, or sharing.
Dependencies: `matplotlib`, `pandas`, and `seaborn`.
"""

from __future__ import annotations

import argparse
import csv
import math
import os
import statistics
import sys
from dataclasses import dataclass
from typing import Dict, Iterable, List, Sequence, Tuple

import matplotlib.pyplot as plt
from matplotlib.backends.backend_pdf import PdfPages
from matplotlib.gridspec import GridSpec
from matplotlib.lines import Line2D
from matplotlib.patches import Patch
from matplotlib.ticker import MultipleLocator
import pandas as pd
import seaborn as sns


CSV_TAG = "bb15_ina_csv"


@dataclass
class Dataset:
    metadata: Dict[str, str]
    rows: List[Dict[str, object]]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Generate a PDF report from BB15 INA CSV capture output."
    )
    parser.add_argument("input", help="Path to the captured CSV/log file.")
    parser.add_argument(
        "-o",
        "--output",
        help="Output PDF path. Defaults to the input path with .pdf appended.",
    )
    return parser.parse_args()


def output_path_for(input_path: str, explicit: str | None) -> str:
    if explicit:
        return explicit
    stem, _ = os.path.splitext(input_path)
    return stem + ".pdf"


def read_capture(path: str) -> Dataset:
    metadata: Dict[str, str] = {}
    header: List[str] | None = None
    rows: List[Dict[str, object]] = []

    with open(path, "r", encoding="utf-8", errors="replace") as f:
        for raw_line in f:
            line = raw_line.strip()
            if not line:
                continue

            if line.startswith("# "):
                key_value = line[2:]
                if "=" in key_value:
                    key, value = key_value.split("=", 1)
                    metadata[key.strip()] = value.strip()
                continue

            if line.startswith("tag,"):
                header = [cell.strip() for cell in line.split(",")]
                continue

            if not line.startswith(CSV_TAG + ","):
                continue

            if header is None:
                raise ValueError("CSV data appeared before the header line.")

            cells = next(csv.reader([line]))
            if len(cells) != len(header):
                continue
            row = convert_row(dict(zip(header, cells)))
            rows.append(row)

    if not rows:
        raise ValueError(f"No {CSV_TAG} rows found in {path}")

    return Dataset(metadata=metadata, rows=rows)


def convert_row(row: Dict[str, str]) -> Dict[str, object]:
    ints = {
        "ms",
        "sample_seq",
        "ticks_due",
        "service_lag_us",
        "inference_active",
        "frame",
        "grab_ms",
        "prep_ms",
        "infer_ms",
        "predicted_index",
        "score0",
        "score1",
        "ch1_raw_shunt",
        "ch1_raw_bus",
        "ch2_raw_shunt",
        "ch2_raw_bus",
    }
    floats = {
        "ch1_shunt_uv",
        "ch1_bus_v",
        "ch1_current_a",
        "ch1_current_ma",
        "ch1_power_w",
        "ch2_shunt_uv",
        "ch2_bus_v",
        "ch2_current_a",
        "ch2_current_ma",
        "ch2_power_w",
    }
    converted: Dict[str, object] = {}
    for key, value in row.items():
        if key in ints:
            converted[key] = int(value)
        elif key in floats:
            converted[key] = float(value)
        else:
            converted[key] = value
    return converted


def series(rows: Sequence[Dict[str, object]], key: str) -> List[float]:
    return [float(row[key]) for row in rows]


def value_list(rows: Sequence[Dict[str, object]], key: str) -> List[object]:
    return [row[key] for row in rows]


def phase_segments(rows: Sequence[Dict[str, object]]) -> List[Tuple[str, float, float]]:
    segments: List[Tuple[str, float, float]] = []
    if not rows:
        return segments

    start_idx = 0
    current_phase = str(rows[0]["phase"])
    for idx in range(1, len(rows)):
        phase = str(rows[idx]["phase"])
        if phase != current_phase:
            segments.append(
                (
                    current_phase,
                    float(rows[start_idx]["t_s"]),
                    float(rows[idx - 1]["t_s"]),
                )
            )
            start_idx = idx
            current_phase = phase
    segments.append(
        (
            current_phase,
            float(rows[start_idx]["t_s"]),
            float(rows[-1]["t_s"]),
        )
    )
    return segments


def add_phase_bands(ax, segments: Sequence[Tuple[str, float, float]], alpha: float = 0.08):
    colors = {
        "idle_ready": "#2a9d8f",
        "infer_loop": "#e76f51",
        "boot": "#577590",
    }
    for phase, x0, x1 in segments:
        if x1 <= x0:
            x1 = x0 + 1e-6
        ax.axvspan(x0, x1, color=colors.get(phase, "#adb5bd"), alpha=alpha, lw=0)


def style_seaborn():
    sns.set_theme(
        style="whitegrid",
        context="notebook",
        rc={
            "figure.facecolor": "#f4f6fb",
            "savefig.facecolor": "#f4f6fb",
            "axes.facecolor": "#ffffff",
            "axes.edgecolor": "#d8deeb",
            "axes.labelcolor": "#24324a",
            "axes.titlecolor": "#0f172a",
            "grid.color": "#dbe3f0",
            "grid.linewidth": 0.8,
            "grid.alpha": 1.0,
            "axes.spines.top": False,
            "axes.spines.right": False,
            "legend.frameon": False,
            "font.family": "DejaVu Sans",
        },
    )
    plt.rcParams.update(
        {
            "axes.titlesize": 14,
            "axes.labelsize": 11,
            "xtick.color": "#42526b",
            "ytick.color": "#42526b",
            "font.size": 10,
            "lines.linewidth": 2.0,
        }
    )


def setup_axis(ax, title: str, ylabel: str):
    ax.set_title(title, loc="left", fontweight="bold", pad=10)
    ax.set_ylabel(ylabel)
    ax.grid(True, axis="y")
    ax.spines["top"].set_visible(False)
    ax.spines["right"].set_visible(False)


def median(values: Sequence[float]) -> float:
    return statistics.median(values) if values else float("nan")


def mean(values: Sequence[float]) -> float:
    return statistics.fmean(values) if values else float("nan")


def percentile(values: Sequence[float], q: float) -> float:
    if not values:
        return float("nan")
    ordered = sorted(values)
    pos = (len(ordered) - 1) * q
    low = int(math.floor(pos))
    high = int(math.ceil(pos))
    if low == high:
        return float(ordered[low])
    frac = pos - low
    return float(ordered[low] * (1.0 - frac) + ordered[high] * frac)


def format_ms(value: float) -> str:
    return "n/a" if math.isnan(value) else f"{value:.1f} ms"


def format_current_ma(value: float) -> str:
    return "n/a" if math.isnan(value) else f"{value:.3f} mA"


def format_power_mw(value: float) -> str:
    return "n/a" if math.isnan(value) else f"{value:.3f} mW"


def format_range(label: str, values: Sequence[float], unit: str) -> str:
    if not values:
        return f"{label}: n/a"
    return f"{label}: min {min(values):.3f} {unit}, max {max(values):.3f} {unit}"


def add_derived_columns(rows: List[Dict[str, object]]):
    t0_ms = int(rows[0]["ms"])
    for row in rows:
        row["t_s"] = (int(row["ms"]) - t0_ms) / 1000.0
        row["ch1_current_ma"] = float(row["ch1_current_a"]) * 1000.0
        row["ch2_current_ma"] = float(row["ch2_current_a"]) * 1000.0
        row["ch1_power_mw"] = float(row["ch1_power_w"]) * 1000.0
        row["ch2_power_mw"] = float(row["ch2_power_w"]) * 1000.0


def to_frame(rows: Sequence[Dict[str, object]]) -> pd.DataFrame:
    frame = pd.DataFrame(rows)
    frame["phase"] = frame["phase"].astype(str)
    frame["status"] = frame["status"].astype(str)
    frame["service_lag_ms"] = frame["service_lag_us"] / 1000.0
    return frame


def split_by_phase(rows: Sequence[Dict[str, object]]) -> Dict[str, List[Dict[str, object]]]:
    grouped: Dict[str, List[Dict[str, object]]] = {}
    for row in rows:
        grouped.setdefault(str(row["phase"]), []).append(dict(row))
    return grouped


def make_summary_page(pdf: PdfPages, dataset: Dataset, df: pd.DataFrame, path: str):
    idle = df[df["phase"] == "idle_ready"]
    infer = df[df["phase"] == "infer_loop"]

    ch1_idle_power = idle["ch1_power_mw"].tolist()
    ch1_infer_power = infer["ch1_power_mw"].tolist()
    ch1_idle_current = idle["ch1_current_ma"].tolist()
    ch1_infer_current = infer["ch1_current_ma"].tolist()
    infer_ms = infer.loc[infer["infer_ms"] > 0, "infer_ms"].astype(float).tolist()
    ticks_due = infer["ticks_due"].astype(float).tolist()
    lag_ms = infer["service_lag_ms"].astype(float).tolist()

    fig = plt.figure(figsize=(11.69, 8.27), constrained_layout=True)
    gs = GridSpec(2, 2, figure=fig, height_ratios=[0.85, 1.15])

    ax_text = fig.add_subplot(gs[0, :])
    ax_text.axis("off")

    title = "BB15 INA Current Monitor Report"
    subtitle = os.path.basename(path)
    ax_text.text(0.00, 0.92, title, fontsize=20, fontweight="bold", color="#0f172a")
    ax_text.text(0.00, 0.80, subtitle, fontsize=11, color="#55637a")

    metadata = dataset.metadata
    hardware_lines = [
        f"Board: {metadata.get('board', 'Nicla Vision + BB15')}",
        f"Model: {metadata.get('model', 'unknown')}",
        f"Sensor: {metadata.get('sensor', 'unknown')}",
        f"Timer interval: {metadata.get('timer_interval_ms', '?')} ms",
        f"Idle baseline: {metadata.get('idle_baseline_ms', '?')} ms",
        f"SPI: Akida {metadata.get('spi_akida_hz', '?')} Hz, Flash {metadata.get('spi_flash_hz', '?')} Hz",
    ]
    summary_lines = [
        f"Samples: {len(df)} total, {len(idle)} idle, {len(infer)} infer",
        format_range("CH1 current", df["ch1_current_ma"].tolist(), "mA"),
        format_range("CH2 current", df["ch2_current_ma"].tolist(), "mA"),
        format_range("CH1 power", df["ch1_power_mw"].tolist(), "mW"),
        format_range("CH2 power", df["ch2_power_mw"].tolist(), "mW"),
        f"Inference time median: {format_ms(median(infer_ms))}",
        f"Service lag p95: {percentile(lag_ms, 0.95):.1f} ms",
        f"Deferred ticks p95: {percentile(ticks_due, 0.95):.1f}",
    ]

    ax_text.text(
        0.00,
        0.62,
        "\n".join(hardware_lines),
        va="top",
        fontsize=10.5,
        color="#24324a",
        linespacing=1.6,
    )
    ax_text.text(
        0.52,
        0.62,
        "\n".join(summary_lines),
        va="top",
        fontsize=10.5,
        color="#24324a",
        linespacing=1.6,
    )

    ax_box = fig.add_subplot(gs[1, 0])
    setup_axis(ax_box, "CH1 Current By Phase", "Current (mA)")
    box_df = df[df["phase"].isin(["idle_ready", "infer_loop"])]
    sns.boxplot(
        data=box_df,
        x="phase",
        y="ch1_current_ma",
        hue="phase",
        order=["idle_ready", "infer_loop"],
        palette={"idle_ready": "#2a9d8f", "infer_loop": "#e76f51"},
        width=0.55,
        dodge=False,
        legend=False,
        ax=ax_box,
    )
    ax_box.set_xlabel("")

    ax_scatter = fig.add_subplot(gs[1, 1])
    setup_axis(ax_scatter, "CH1 Current vs Inference Time", "Inference time (ms)")
    if not infer.empty:
        scatter = sns.scatterplot(
            data=infer,
            x="ch1_current_ma",
            y="infer_ms",
            hue="ticks_due",
            palette="crest",
            s=48,
            edgecolor="white",
            linewidth=0.45,
            ax=ax_scatter,
        )
        ax_scatter.set_xlabel("CH1 current (mA)")
        handles, labels = scatter.get_legend_handles_labels()
        if handles:
            ax_scatter.legend(
                handles,
                labels,
                title="ticks_due",
                bbox_to_anchor=(1.02, 1.0),
                loc="upper left",
            )

    pdf.savefig(fig, bbox_inches="tight")
    plt.close(fig)


def make_timeseries_page(pdf: PdfPages, dataset: Dataset, df: pd.DataFrame):
    rows = dataset.rows
    segments = phase_segments(rows)

    fig, axes = plt.subplots(
        3,
        1,
        figsize=(11.69, 8.27),
        sharex=True,
        constrained_layout=True,
        gridspec_kw={"height_ratios": [1.35, 1.15, 1.0]},
    )

    # Channel 1 current detail with fixed bench-analysis scale.
    ax = axes[0]
    setup_axis(ax, "CH1 Current Detail", "Current (mA)")
    add_phase_bands(ax, segments)
    sns.lineplot(
        data=df,
        x="t_s",
        y="ch1_current_ma",
        color="#124e78",
        ax=ax,
    )
    ax.set_ylim(0.0, 300.0)
    ax.yaxis.set_major_locator(MultipleLocator(5.0))
    ax.grid(True, axis="y", which="major")
    ch1_min = df["ch1_current_ma"].min()
    ch1_max = df["ch1_current_ma"].max()
    ax.axhline(ch1_min, color="#2a9d8f", linestyle="--", linewidth=1.2, alpha=0.9)
    ax.axhline(ch1_max, color="#e76f51", linestyle="--", linewidth=1.2, alpha=0.9)
    ax.text(
        0.99,
        0.96,
        f"min {ch1_min:.3f} mA\nmax {ch1_max:.3f} mA",
        transform=ax.transAxes,
        ha="right",
        va="top",
        fontsize=10,
        color="#24324a",
        bbox={"facecolor": "#ffffff", "edgecolor": "#d8deeb", "boxstyle": "round,pad=0.35"},
    )

    # Two-channel comparison with explicit min/max markers.
    ax = axes[1]
    setup_axis(ax, "Channel Current Comparison", "Current (mA)")
    add_phase_bands(ax, segments)
    current_df = pd.concat(
        [
            df[["t_s", "ch1_current_ma"]]
            .rename(columns={"ch1_current_ma": "current_ma"})
            .assign(channel="CH1"),
            df[["t_s", "ch2_current_ma"]]
            .rename(columns={"ch2_current_ma": "current_ma"})
            .assign(channel="CH2"),
        ],
        ignore_index=True,
    )
    sns.lineplot(
        data=current_df,
        x="t_s",
        y="current_ma",
        hue="channel",
        palette={"CH1": "#124e78", "CH2": "#9db4c0"},
        ax=ax,
    )
    ax.legend(loc="upper left", ncol=2, title="")
    ax.text(
        0.99,
        0.96,
        (
            f"CH1 min/max: {df['ch1_current_ma'].min():.3f} / {df['ch1_current_ma'].max():.3f} mA\n"
            f"CH2 min/max: {df['ch2_current_ma'].min():.3f} / {df['ch2_current_ma'].max():.3f} mA"
        ),
        transform=ax.transAxes,
        ha="right",
        va="top",
        fontsize=10,
        color="#24324a",
        bbox={"facecolor": "#ffffff", "edgecolor": "#d8deeb", "boxstyle": "round,pad=0.35"},
    )

    # Timing and scheduler pressure.
    ax = axes[2]
    setup_axis(ax, "Inference Timing and Deferred Sampling Pressure", "ms / ticks")
    add_phase_bands(ax, segments)
    timing_df = pd.concat(
        [
            df[["t_s", "infer_ms"]]
            .rename(columns={"infer_ms": "value"})
            .assign(metric="infer_ms"),
            df[["t_s", "service_lag_ms"]]
            .rename(columns={"service_lag_ms": "value"})
            .assign(metric="service_lag_ms"),
        ],
        ignore_index=True,
    )
    sns.lineplot(
        data=timing_df,
        x="t_s",
        y="value",
        hue="metric",
        palette={"infer_ms": "#6c63ff", "service_lag_ms": "#4a7c59"},
        ax=ax,
    )
    ax2 = ax.twinx()
    sns.lineplot(data=df, x="t_s", y="ticks_due", color="#bc5090", ax=ax2, legend=False)
    ax2.set_ylabel("ticks_due")
    ax2.spines["top"].set_visible(False)
    lines = [
        Line2D([0], [0], color="#6c63ff", lw=2),
        Line2D([0], [0], color="#4a7c59", lw=2),
        Line2D([0], [0], color="#bc5090", lw=2),
    ]
    ax.legend(lines, ["infer_ms", "service_lag_ms", "ticks_due"], loc="upper left", ncol=3)

    ax.set_xlabel("Time since first sample (s)")

    pdf.savefig(fig, bbox_inches="tight")
    plt.close(fig)


def make_distribution_page(pdf: PdfPages, dataset: Dataset, df: pd.DataFrame):
    infer = df[df["phase"] == "infer_loop"]

    fig, axes = plt.subplots(2, 1, figsize=(11.69, 8.27), constrained_layout=True)

    ax = axes[0]
    setup_axis(ax, "Channel Power Overview", "Power (mW)")
    power_df = pd.concat(
        [
            df[["t_s", "ch1_power_mw"]]
            .rename(columns={"ch1_power_mw": "power_mw"})
            .assign(channel="CH1"),
            df[["t_s", "ch2_power_mw"]]
            .rename(columns={"ch2_power_mw": "power_mw"})
            .assign(channel="CH2"),
        ],
        ignore_index=True,
    )
    sns.lineplot(
        data=power_df,
        x="t_s",
        y="power_mw",
        hue="channel",
        palette={"CH1": "#d55d3f", "CH2": "#f2c14e"},
        ax=ax,
    )
    ax.legend(loc="upper left", ncol=2, title="")
    ax.text(
        0.99,
        0.96,
        (
            f"CH1 power min/max: {df['ch1_power_mw'].min():.3f} / {df['ch1_power_mw'].max():.3f} mW\n"
            f"CH2 power min/max: {df['ch2_power_mw'].min():.3f} / {df['ch2_power_mw'].max():.3f} mW"
        ),
        transform=ax.transAxes,
        ha="right",
        va="top",
        fontsize=10,
        color="#24324a",
        bbox={"facecolor": "#ffffff", "edgecolor": "#d8deeb", "boxstyle": "round,pad=0.35"},
    )

    ax = axes[1]
    setup_axis(ax, "Inference Duration", "infer_ms")
    sns.lineplot(
        data=infer,
        x="t_s",
        y="infer_ms",
        color="#6c63ff",
        ax=ax,
    )
    ax.set_xlabel("Time since first sample (s)")

    pdf.savefig(fig, bbox_inches="tight")
    plt.close(fig)


def main() -> int:
    args = parse_args()
    dataset = read_capture(args.input)
    add_derived_columns(dataset.rows)
    df = to_frame(dataset.rows)

    out_path = output_path_for(args.input, args.output)
    style_seaborn()

    with PdfPages(out_path) as pdf:
        make_summary_page(pdf, dataset, df, args.input)
        make_timeseries_page(pdf, dataset, df)
        make_distribution_page(pdf, dataset, df)

    print(f"Wrote PDF report: {out_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
