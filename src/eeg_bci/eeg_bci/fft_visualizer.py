"""Average single/all EEG trials and create time-domain/FFT figures."""

from __future__ import annotations

import argparse
import csv
import json
import re
from pathlib import Path

import matplotlib

matplotlib.use("Agg")

import matplotlib.pyplot as plt
import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


COMMAND_FREQUENCIES = {
    "forward": 8.0,
    "left": 9.0,
    "right": 10.0,
    "backward": 11.0,
    "stop": 12.0,
    "idle": 13.0,
}
CHANNEL_COLORS = [
    "#0072B2",
    "#D55E00",
    "#009E73",
    "#CC79A7",
    "#E69F00",
    "#56B4E9",
    "#6A3D9A",
    "#333333",
]


def load_manifest(path: Path) -> dict:
    manifest_path = path / "trial_manifest.json"
    if not manifest_path.exists():
        return {}
    try:
        return json.loads(manifest_path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError):
        return {}


def read_eeg_bag(path: Path) -> dict | None:
    message_type = get_message("eeg_interfaces/msg/EEGFrame")
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(path), storage_id="sqlite3"),
        rosbag2_py.ConverterOptions("", ""),
    )

    samples: list[np.ndarray] = []
    sample_rates: list[int] = []
    header_times: list[int] = []
    bag_times: list[int] = []
    while reader.has_next():
        topic, data, timestamp = reader.read_next()
        if topic != "/eeg/frame":
            continue
        message = deserialize_message(data, message_type)
        values = np.asarray(message.channels, dtype=np.float64)
        if values.size != 8 or not np.isfinite(values).all():
            continue
        samples.append(values)
        sample_rates.append(int(message.sample_rate or 250))
        header_times.append(
            int(message.header.stamp.sec) * 1_000_000_000
            + int(message.header.stamp.nanosec)
        )
        bag_times.append(int(timestamp))

    if not samples:
        return None
    return {
        "samples": np.asarray(samples, dtype=np.float64),
        "sample_rate": int(round(np.median(sample_rates))),
        "header_times_ns": np.asarray(header_times, dtype=np.int64),
        "bag_times_ns": np.asarray(bag_times, dtype=np.int64),
        "manifest": load_manifest(path),
    }


def discover_bags(input_path: Path) -> list[Path]:
    if (input_path / "metadata.yaml").exists():
        return [input_path]
    if input_path.is_file() and input_path.suffix == ".db3":
        return [input_path.parent]
    return sorted(
        path.parent
        for path in input_path.rglob("metadata.yaml")
        if path.parent.is_dir() and (path.parent / "trial_manifest.json").exists()
    )


def trial_relative_axis(data: dict) -> tuple[np.ndarray, float]:
    """Build a constant-250-Hz axis relative to the stimulus onset.

    ROS delivery timestamps in these bags arrive in chunks, so their dT is not
    uniform. The EEGFrame sample_rate is the physical sampling interval and is
    therefore used for the FFT axis, matching PlotJuggler's constant-dT rule.
    """
    samples = data["samples"]
    fs = float(data["sample_rate"] or 250)
    timestamps = data["header_times_ns"]
    if not np.any(timestamps > 0):
        timestamps = data["bag_times_ns"]

    events = data["manifest"].get("stimulus_events") or []
    stimulus_ns = events[0].get("wall_time_ns") if events else None
    if stimulus_ns is not None and np.any(timestamps > 0):
        first_time = (float(timestamps[0]) - float(stimulus_ns)) / 1e9
    else:
        first_time = 0.0
    return first_time + np.arange(len(samples), dtype=float) / fs, fs


def average_trials(trials: list[dict]) -> dict:
    if not trials:
        raise ValueError("No trials were supplied")
    axes = []
    fs_values = []
    for trial in trials:
        axis, fs = trial_relative_axis(trial)
        axes.append(axis)
        fs_values.append(fs)

    fs = float(np.median(fs_values))
    common_start = max(axis[0] for axis in axes)
    common_end = min(axis[-1] for axis in axes)
    if common_end <= common_start:
        raise ValueError("Trials have no common time interval")

    common_axis = np.arange(
        common_start,
        common_end + 0.5 / fs,
        1.0 / fs,
        dtype=float,
    )
    aligned = []
    for trial, axis in zip(trials, axes):
        trial_values = trial["samples"]
        aligned_trial = np.column_stack(
            [
                np.interp(common_axis, axis, trial_values[:, channel])
                for channel in range(8)
            ]
        )
        aligned.append(aligned_trial)

    return {
        "samples": np.mean(np.stack(aligned, axis=0), axis=0),
        "times": common_axis,
        "sample_rate": fs,
        "trial_count": len(trials),
        "trial_names": [trial["manifest"].get("trial_id", "") for trial in trials],
        "common_start_s": common_start,
        "common_end_s": common_end,
    }


def select_window(average: dict, start: float | None, duration: float | None) -> dict:
    times = average["times"]
    first = times[0] if start is None else max(times[0], float(start))
    last = times[-1] if duration is None else min(times[-1], first + float(duration))
    if last <= first:
        raise ValueError("The requested FFT window does not overlap the averaged data")
    left = int(np.searchsorted(times, first, side="left"))
    right = int(np.searchsorted(times, last, side="right"))
    if right - left < 8:
        raise ValueError("The requested FFT window contains too few samples")
    return {
        **average,
        "samples": average["samples"][left:right],
        "times": average["times"][left:right],
        "window_start_s": float(average["times"][left]),
        "window_end_s": float(average["times"][right - 1]),
    }


def direct_fft(samples: np.ndarray, fs: float) -> tuple[np.ndarray, np.ndarray]:
    """Direct single-sided FFT with the same constant-dT assumptions as PJ."""
    centered = samples - np.mean(samples, axis=0, keepdims=True)
    spectrum = np.fft.rfft(centered, axis=0)
    amplitude = 2.0 * np.abs(spectrum) / len(centered)
    amplitude[0, :] *= 0.5
    frequencies = np.fft.rfftfreq(len(centered), d=1.0 / fs)
    return frequencies, amplitude


def peak(frequencies: np.ndarray, values: np.ndarray, low: float, high: float):
    mask = (frequencies >= low) & (frequencies <= high)
    indices = np.flatnonzero(mask)
    if not indices.size:
        return float("nan"), float("nan")
    index = indices[int(np.argmax(values[indices]))]
    return float(frequencies[index]), float(values[index])


def safe_name(value: str) -> str:
    return re.sub(r"[^A-Za-z0-9_.-]+", "_", value).strip("_") or "group"


def make_group_figure(
    group: str,
    average: dict,
    output_path: Path,
) -> list[dict]:
    samples = average["samples"]
    times = average["times"]
    fs = average["sample_rate"]
    frequencies, amplitudes = direct_fft(samples, fs)
    display = (frequencies >= 0.5) & (frequencies <= 40.0)
    display_frequencies = frequencies[display]
    display_amplitudes = amplitudes[display]
    mean_amplitude = np.mean(display_amplitudes, axis=1)

    channel_means = np.mean(samples, axis=0)
    time_centered = samples - channel_means
    global_frequency, global_amplitude = peak(
        display_frequencies, mean_amplitude, 0.5, 40.0
    )
    ssvep_frequency, ssvep_amplitude = peak(
        display_frequencies, mean_amplitude, 6.0, 15.0
    )
    global_period = 1000.0 / global_frequency if global_frequency > 0 else float("nan")
    ssvep_period = 1000.0 / ssvep_frequency if ssvep_frequency > 0 else float("nan")

    fig, (time_axis, frequency_axis) = plt.subplots(
        1,
        2,
        figsize=(18, 8.5),
        gridspec_kw={"width_ratios": [1.0, 1.12]},
    )
    fig.subplots_adjust(left=0.065, right=0.78, bottom=0.15, top=0.82, wspace=0.24)
    fig.suptitle(
        f"{group.upper()} average: 8-channel EEG time and frequency analysis\n"
        f"SSVEP-band peak: {ssvep_frequency:.2f} Hz  |  "
        f"period: {ssvep_period:.1f} ms  |  trials averaged: {average['trial_count']}",
        fontsize=18,
        fontweight="bold",
        color="#172033",
    )

    for channel in range(8):
        color = CHANNEL_COLORS[channel]
        time_axis.plot(
            times,
            time_centered[:, channel],
            color=color,
            linewidth=1.0,
            alpha=0.9,
            label=f"Ch {channel}",
        )
        frequency_axis.plot(
            display_frequencies,
            display_amplitudes[:, channel],
            color=color,
            linewidth=1.15,
            alpha=0.85,
            label=f"Ch {channel}",
        )

    frequency_axis.plot(
        display_frequencies,
        mean_amplitude,
        color="#111827",
        linewidth=2.4,
        linestyle="--",
        label="8-channel mean",
        zorder=6,
    )

    target_colors = ["#0072B2", "#D55E00", "#009E73", "#CC79A7", "#E69F00", "#6A3D9A"]
    for target_hz, color in zip(COMMAND_FREQUENCIES.values(), target_colors):
        frequency_axis.axvline(
            target_hz,
            color=color,
            linestyle=":",
            linewidth=1.0,
            alpha=0.75,
        )

    frequency_axis.scatter(
        [global_frequency],
        [global_amplitude],
        color="#DC2626",
        marker="o",
        s=65,
        zorder=8,
        label=f"global peak {global_frequency:.2f} Hz",
    )
    frequency_axis.scatter(
        [ssvep_frequency],
        [ssvep_amplitude],
        color="#111827",
        marker="*",
        s=150,
        zorder=9,
        label=f"SSVEP peak {ssvep_frequency:.2f} Hz",
    )
    frequency_axis.annotate(
        f"SSVEP peak\n{ssvep_frequency:.2f} Hz\n{ssvep_period:.1f} ms",
        xy=(ssvep_frequency, ssvep_amplitude),
        xytext=(12, 12),
        textcoords="offset points",
        fontsize=10,
        fontweight="bold",
        bbox={"boxstyle": "round,pad=0.35", "facecolor": "white", "edgecolor": "#9CA3AF"},
    )

    time_axis.set_title("Time domain: averaged 8-channel voltage", fontsize=14, pad=12)
    time_axis.set_xlabel("Time relative to stimulus onset (s)")
    time_axis.set_ylabel("Mean-centered EEG voltage (native units)")
    time_axis.grid(True, alpha=0.22)
    time_axis.set_xlim(times[0], times[-1])
    time_axis.legend(loc="upper right", ncol=2, fontsize=8, frameon=True)

    frequency_axis.set_title("Frequency domain: one standard spectrum", fontsize=14, pad=12)
    frequency_axis.set_xlabel("Frequency (Hz)")
    frequency_axis.set_ylabel("Single-sided FFT amplitude (native units)")
    frequency_axis.set_xlim(0.5, 40.0)
    frequency_axis.grid(True, alpha=0.22)
    frequency_axis.axvspan(6.0, 15.0, color="#94A3B8", alpha=0.08, zorder=0)
    frequency_axis.text(
        10.5,
        frequency_axis.get_ylim()[1] * 0.97,
        "SSVEP band 6-15 Hz",
        ha="center",
        va="top",
        fontsize=9,
        color="#64748B",
    )
    frequency_axis.legend(loc="upper right", fontsize=8, frameon=True)

    info = (
        f"Averaged trials: {average['trial_count']}\n"
        f"Samples in average: {len(samples)}\n"
        f"Sampling rate: {fs:.1f} Hz\n"
        f"FFT resolution: {fs / len(samples):.3f} Hz\n"
        f"Common aligned range: {average['common_start_s']:.2f} to "
        f"{average['common_end_s']:.2f} s\n"
        f"FFT range: {times[0]:.2f} to {times[-1]:.2f} s\n\n"
        "Target frequencies\n"
        "forward  8 Hz    left     9 Hz\n"
        "right   10 Hz    backward 11 Hz\n"
        "stop    12 Hz    idle    13 Hz\n\n"
        f"Global peak: {global_frequency:.2f} Hz\n"
        f"Global period: {global_period:.1f} ms\n"
        f"SSVEP peak: {ssvep_frequency:.2f} Hz\n"
        f"SSVEP period: {ssvep_period:.1f} ms"
    )
    fig.text(
        0.065,
        0.035,
        "FFT: constant 250-Hz sample interval, direct single-sided rFFT, DC removed, no padding. "
        "The black dashed line is the mean amplitude of all 8 channels.",
        fontsize=9.5,
        color="#4B5563",
    )
    fig.text(
        0.965,
        0.78,
        info,
        va="top",
        ha="right",
        family="monospace",
        fontsize=9.2,
        color="#172033",
        bbox={"boxstyle": "round,pad=0.7", "facecolor": "#F7F8FA", "edgecolor": "#CBD5E1"},
    )

    fig.savefig(output_path, dpi=180, facecolor="white")
    plt.close(fig)

    rows = []
    for channel in range(8):
        dominant_frequency, dominant_amplitude = peak(
            display_frequencies, display_amplitudes[:, channel], 0.5, 40.0
        )
        channel_ssvep_frequency, channel_ssvep_amplitude = peak(
            display_frequencies, display_amplitudes[:, channel], 6.0, 15.0
        )
        rows.append(
            {
                "group": group,
                "trial_count": average["trial_count"],
                "channel": channel,
                "sample_count": len(samples),
                "sample_rate_hz": fs,
                "fft_resolution_hz": fs / len(samples),
                "global_peak_frequency_hz": global_frequency,
                "global_peak_period_ms": global_period,
                "ssvep_peak_frequency_hz": ssvep_frequency,
                "ssvep_peak_period_ms": ssvep_period,
                "channel_peak_frequency_hz": dominant_frequency,
                "channel_peak_period_ms": 1000.0 / dominant_frequency
                if dominant_frequency > 0
                else float("nan"),
                "channel_peak_amplitude": dominant_amplitude,
                "channel_ssvep_peak_frequency_hz": channel_ssvep_frequency,
                "channel_ssvep_peak_period_ms": 1000.0 / channel_ssvep_frequency
                if channel_ssvep_frequency > 0
                else float("nan"),
                "channel_ssvep_peak_amplitude": channel_ssvep_amplitude,
            }
        )
    return rows


def write_summary(path: Path, rows: list[dict]) -> None:
    if not rows:
        return
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)


def main(args=None) -> int:
    parser = argparse.ArgumentParser(
        description="Average all single/all EEG trials and create two FFT figures."
    )
    parser.add_argument(
        "input",
        nargs="?",
        default="experiments",
        help="Experiments directory or one bag directory (default: experiments).",
    )
    parser.add_argument(
        "--output-dir",
        default="experiments/fft_average_results",
        help="Directory receiving single/all average PNG files and CSV summary.",
    )
    parser.add_argument(
        "--window-start",
        type=float,
        default=None,
        help="Optional FFT window start relative to stimulus onset (seconds).",
    )
    parser.add_argument(
        "--window-seconds",
        type=float,
        default=None,
        help="Optional FFT window duration. Omit to use the common averaged interval.",
    )
    parsed = parser.parse_args(args)

    input_path = Path(parsed.input).expanduser().resolve()
    output_dir = Path(parsed.output_dir).expanduser().resolve()
    if not input_path.exists():
        parser.error(f"Input path does not exist: {input_path}")
    if parsed.window_start is not None and parsed.window_start < 0:
        parser.error("--window-start must be non-negative")
    if parsed.window_seconds is not None and parsed.window_seconds <= 0:
        parser.error("--window-seconds must be positive")

    bag_paths = discover_bags(input_path)
    groups: dict[str, list[dict]] = {"single": [], "all": []}
    for bag_path in bag_paths:
        manifest = load_manifest(bag_path)
        mode = str(manifest.get("mode") or "").lower()
        if mode not in groups:
            continue
        data = read_eeg_bag(bag_path)
        if data is not None:
            groups[mode].append(data)

    output_dir.mkdir(parents=True, exist_ok=True)
    all_rows = []
    generated = 0
    for group in ("single", "all"):
        if not groups[group]:
            print(f"SKIP {group}: no valid trial data")
            continue
        average = average_trials(groups[group])
        selected = select_window(
            average,
            parsed.window_start,
            parsed.window_seconds,
        )
        output_path = output_dir / f"{safe_name(group)}_average_fft.png"
        rows = make_group_figure(group, selected, output_path)
        all_rows.extend(rows)
        generated += 1
        print(
            f"{group}: averaged {len(groups[group])} trials, "
            f"common_samples={len(average['samples'])}, "
            f"fft_samples={len(selected['samples'])}, "
            f"saved={output_path}"
        )

    summary_path = output_dir / "average_fft_summary.csv"
    write_summary(summary_path, all_rows)
    print(f"Generated {generated} image(s)")
    print(f"Summary: {summary_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
