"""Draw an observable SSVEP command-processing timeline from ROS 2 bags.

The bag timestamp of the first ``/eeg/frame`` message is defined as t=0.
The script deliberately does not invent BLE or LSL transport timestamps:
those internal stages were not recorded in the trial bags.  It reports the
observable ROS/bag stages and marks unavailable stages explicitly.
"""

from __future__ import annotations

import argparse
import csv
import json
import os
import textwrap
from dataclasses import dataclass
from pathlib import Path

os.environ.setdefault("MPLCONFIGDIR", "/tmp/eeg_bci_matplotlib")
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import FancyBboxPatch
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


COMMANDS = ("forward", "backward", "left", "right", "stop")
TOPICS = {
    "/eeg/frame": "eeg_interfaces/msg/EEGFrame",
    "/eeg/quality": "eeg_interfaces/msg/SignalQuality",
    "/ssvep/command": "eeg_interfaces/msg/SSVEPCommand",
    "/ssvep/stimulus_active": "std_msgs/msg/Bool",
    "/turtle1/cmd_vel": "geometry_msgs/msg/Twist",
    "/turtle1/pose": "turtlesim/msg/Pose",
}


@dataclass
class Record:
    topic: str
    bag_time_ns: int
    message: object


@dataclass
class Event:
    key: str
    label: str
    time_ns: int | None
    source: str
    detail: str = ""
    relative_seconds: float | None = None


def _message_types() -> dict[str, object]:
    return {topic: get_message(type_name) for topic, type_name in TOPICS.items()}


def read_records(bag_path: Path) -> list[Record]:
    """Read only the recorded topics needed for the timeline."""

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag_path), storage_id="sqlite3"),
        rosbag2_py.ConverterOptions("", ""),
    )
    available = {item.name for item in reader.get_all_topics_and_types()}
    type_cache = _message_types()
    records: list[Record] = []
    while reader.has_next():
        topic, serialized, timestamp = reader.read_next()
        if topic not in TOPICS or topic not in available:
            continue
        message = deserialize_message(serialized, type_cache[topic])
        records.append(Record(topic, int(timestamp), message))
    records.sort(key=lambda item: item.bag_time_ns)
    return records


def _header_stamp_ns(message: object) -> int | None:
    header = getattr(message, "header", None)
    if header is None:
        return None
    stamp = getattr(header, "stamp", None)
    if stamp is None:
        return None
    value = int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)
    return value if value else None


def _relative_seconds(time_ns: int | None, origin_ns: int) -> float | None:
    if time_ns is None:
        return None
    return (time_ns - origin_ns) / 1_000_000_000.0


def _command_detail(message: object) -> str:
    return (
        f"FBCCA={message.fbcca_command or '--'}; "
        f"OUT={message.command}; valid={bool(message.valid)}; "
        f"reason={message.reason}; conf={float(message.confidence):.3f}"
    )


def _twist_detail(message: object) -> str:
    return f"linear.x={message.linear.x:.3f}; angular.z={message.angular.z:.3f}"


def _pose_detail(message: object) -> str:
    return (
        f"x={message.x:.2f}; y={message.y:.2f}; theta={message.theta:.2f}; "
        f"v={message.linear_velocity:.3f}; w={message.angular_velocity:.3f}"
    )


def _first(records: list[Record], topic: str, predicate=None) -> Record | None:
    for record in records:
        if record.topic != topic:
            continue
        if predicate is None or predicate(record.message):
            return record
    return None


def _first_at_or_after(
    records: list[Record], topic: str, origin_ns: int, predicate=None
) -> Record | None:
    for record in records:
        if record.bag_time_ns < origin_ns or record.topic != topic:
            continue
        if predicate is None or predicate(record.message):
            return record
    return None


def _expected_twist_matches(message: object, expected: str) -> bool:
    linear = float(message.linear.x)
    angular = float(message.angular.z)
    tolerance = 1e-4
    if expected == "forward":
        return linear > tolerance
    if expected == "backward":
        return linear < -tolerance
    if expected == "left":
        return angular > tolerance
    if expected == "right":
        return angular < -tolerance
    if expected == "stop":
        return abs(linear) <= tolerance and abs(angular) <= tolerance
    return False


def _event(
    key: str,
    label: str,
    record: Record | None,
    origin_ns: int,
    detail: str = "",
    source: str = "rosbag record time",
) -> Event:
    return Event(
        key=key,
        label=label,
        time_ns=None if record is None else record.bag_time_ns,
        source=source if record is not None else "not observed",
        detail=detail,
    )


def analyze_bag(bag_path: Path, expected: str) -> dict:
    records = read_records(bag_path)
    eeg_records = [record for record in records if record.topic == "/eeg/frame"]
    if not eeg_records:
        raise RuntimeError(f"No /eeg/frame messages found in {bag_path}")

    origin_ns = eeg_records[0].bag_time_ns
    manifest_path = bag_path / "trial_manifest.json"
    manifest = {}
    if manifest_path.is_file():
        manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
        expected = str(manifest.get("target") or expected)

    quality = _first(records, "/eeg/quality")
    first_command = _first_at_or_after(records, "/ssvep/command", origin_ns)
    first_fbcca = _first_at_or_after(
        records,
        "/ssvep/command",
        origin_ns,
        lambda message: bool(message.fbcca_command),
    )
    first_expected_fbcca = _first_at_or_after(
        records,
        "/ssvep/command",
        origin_ns,
        lambda message: str(message.fbcca_command) == expected,
    )
    first_expected_valid = _first_at_or_after(
        records,
        "/ssvep/command",
        origin_ns,
        lambda message: bool(message.valid) and str(message.command) == expected,
    )

    window_ready = eeg_records[999] if len(eeg_records) >= 1000 else None
    command_reference = (
        first_expected_valid.bag_time_ns
        if first_expected_valid is not None
        else origin_ns
    )
    expected_cmd_vel = _first(
        records,
        "/turtle1/cmd_vel",
        lambda message: (
            message is not None
            and _expected_twist_matches(message, expected)
        ),
    )
    if first_expected_valid is not None:
        expected_cmd_vel = _first(
            records,
            "/turtle1/cmd_vel",
            lambda message: _expected_twist_matches(message, expected),
        )
        if expected_cmd_vel is not None and expected_cmd_vel.bag_time_ns < command_reference:
            expected_cmd_vel = next(
                (
                    record
                    for record in records
                    if record.topic == "/turtle1/cmd_vel"
                    and record.bag_time_ns >= command_reference
                    and _expected_twist_matches(record.message, expected)
                ),
                None,
            )
    else:
        expected_cmd_vel = None

    pose_after_cmd = None
    if expected_cmd_vel is not None:
        pose_after_cmd = next(
            (
                record
                for record in records
                if record.topic == "/turtle1/pose"
                and record.bag_time_ns >= expected_cmd_vel.bag_time_ns
            ),
            None,
        )

    events = [
        _event(
            "first_eeg",
            "First /eeg/frame received (t=0)",
            eeg_records[0],
            origin_ns,
            f"sequence={eeg_records[0].message.sequence}; "
            f"sample_rate={eeg_records[0].message.sample_rate} Hz",
        ),
        _event(
            "first_command",
            "First /ssvep/command after t=0",
            first_command,
            origin_ns,
            "" if first_command is None else _command_detail(first_command.message),
        ),
        _event(
            "quality",
            "First /eeg/quality",
            quality,
            origin_ns,
            "" if quality is None else (
                f"level={quality.message.level}; valid={bool(quality.message.valid)}"
            ),
        ),
        _event(
            "window_ready",
            "4-second window ready (1000th EEG sample)",
            window_ready,
            origin_ns,
            "required=1000 samples at 250 Hz",
        ),
        _event(
            "first_fbcca",
            "First FBCCA candidate",
            first_fbcca,
            origin_ns,
            "" if first_fbcca is None else _command_detail(first_fbcca.message),
        ),
        _event(
            "expected_fbcca",
            f"FBCCA candidate = expected '{expected}'",
            first_expected_fbcca,
            origin_ns,
            "" if first_expected_fbcca is None else _command_detail(first_expected_fbcca.message),
        ),
        _event(
            "expected_valid",
            f"Valid output = expected '{expected}'",
            first_expected_valid,
            origin_ns,
            "" if first_expected_valid is None else _command_detail(first_expected_valid.message),
        ),
        _event(
            "cmd_vel",
            f"/turtle1/cmd_vel reflects '{expected}'",
            expected_cmd_vel,
            origin_ns,
            "" if expected_cmd_vel is None else _twist_detail(expected_cmd_vel.message),
        ),
        _event(
            "pose",
            "First /turtle1/pose after expected cmd_vel",
            pose_after_cmd,
            origin_ns,
            "" if pose_after_cmd is None else _pose_detail(pose_after_cmd.message),
        ),
    ]

    for event in events:
        event.relative_seconds = _relative_seconds(event.time_ns, origin_ns)

    summary = {
        "bag": str(bag_path),
        "expected_command": expected,
        "time_zero": "first /eeg/frame bag record timestamp",
        "record_count": len(records),
        "eeg_frame_count": len(eeg_records),
        "duration_from_first_eeg_seconds": _relative_seconds(
            records[-1].bag_time_ns, origin_ns
        ),
        "events": [
            {
                "key": event.key,
                "label": event.label,
                "relative_seconds": event.relative_seconds,
                "source": event.source,
                "detail": event.detail,
            }
            for event in events
        ],
        "latencies": {
            "first_eeg_to_first_fbcca_seconds": _difference(
                events, "first_eeg", "first_fbcca"
            ),
            "first_eeg_to_expected_valid_seconds": _difference(
                events, "first_eeg", "expected_valid"
            ),
            "expected_valid_to_cmd_vel_seconds": _difference(
                events, "expected_valid", "cmd_vel"
            ),
            "cmd_vel_to_pose_seconds": _difference(events, "cmd_vel", "pose"),
        },
        "observability_note": (
            "BLE notification, LSL push and LSL pull timestamps are not present "
            "in the recorded bag; the first /eeg/frame record is used as t=0."
        ),
    }
    return {"summary": summary, "events": events, "manifest": manifest}


def _difference(events: list[Event], start_key: str, end_key: str) -> float | None:
    start = next((event for event in events if event.key == start_key), None)
    end = next((event for event in events if event.key == end_key), None)
    if start is None or end is None:
        return None
    if start.relative_seconds is None or end.relative_seconds is None:
        return None
    return float(end.relative_seconds - start.relative_seconds)


def _format_time(value: float | None) -> str:
    return "not observed" if value is None else f"t={value:.3f} s"


def draw_timeline(result: dict, output_path: Path) -> None:
    summary = result["summary"]
    events: list[Event] = result["events"]
    figure, axis = plt.subplots(figsize=(16, 10), dpi=150)
    axis.set_xlim(0.0, 1.0)
    axis.set_ylim(0.0, 1.0)
    axis.axis("off")

    expected = summary["expected_command"]
    bag_name = Path(summary["bag"]).name
    axis.text(
        0.02,
        0.965,
        f"SSVEP Command Timeline | expected: {expected} | {bag_name}",
        fontsize=18,
        fontweight="bold",
        color="#172033",
        va="top",
    )
    axis.text(
        0.02,
        0.925,
        "Time zero = first recorded /eeg/frame. Times are relative rosbag record timestamps.",
        fontsize=10,
        color="#4b5563",
        va="top",
    )

    x = 0.06
    flow_events = events
    y_positions = [0.84 - index * 0.085 for index in range(len(flow_events))]
    box_width = 0.88
    box_height = 0.062
    for index, (event, y) in enumerate(zip(flow_events, y_positions)):
        relative = event.relative_seconds
        observed = relative is not None
        color = "#d9f2e6" if observed else "#f3f4f6"
        edge = "#20845a" if observed else "#9ca3af"
        patch = FancyBboxPatch(
            (x, y),
            box_width,
            box_height,
            boxstyle="round,pad=0.008,rounding_size=0.012",
            linewidth=1.5,
            edgecolor=edge,
            facecolor=color,
        )
        axis.add_patch(patch)
        timestamp = _format_time(relative)
        text = f"{event.label}\n{timestamp}    {event.detail}"
        axis.text(
            x + 0.018,
            y + box_height / 2,
            textwrap.fill(text, width=125),
            fontsize=8.3,
            color="#172033",
            va="center",
            ha="left",
        )
        if index < len(flow_events) - 1:
            axis.annotate(
                "",
                xy=(0.50, y - 0.012),
                xytext=(0.50, y - 0.002),
                arrowprops={"arrowstyle": "-|>", "color": "#64748b", "lw": 1.2},
            )

    note = (
        "After the valid expected command, the first matching /turtle1/cmd_vel "
        "and the next /turtle1/pose are shown below. BLE, LSL push and LSL pull "
        "are not individually observable in this bag."
    )
    axis.text(
        0.02,
        0.015,
        textwrap.fill(note, width=180),
        fontsize=9,
        color="#4b5563",
        va="bottom",
    )
    figure.savefig(output_path, bbox_inches="tight")
    plt.close(figure)


def write_events_csv(result: dict, output_path: Path) -> None:
    events: list[Event] = result["events"]
    with output_path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(
            handle,
            fieldnames=["key", "label", "relative_seconds", "source", "detail"],
        )
        writer.writeheader()
        for event in events:
            writer.writerow(
                {
                    "key": event.key,
                    "label": event.label,
                    "relative_seconds": event.relative_seconds,
                    "source": event.source,
                    "detail": event.detail,
                }
            )


def analyze_one(bag_path: Path, expected: str) -> dict:
    result = analyze_bag(bag_path, expected)
    output_png = bag_path / "command_timeline.png"
    output_csv = bag_path / "command_timeline_events.csv"
    output_json = bag_path / "command_timeline_summary.json"
    draw_timeline(result, output_png)
    write_events_csv(result, output_csv)
    output_json.write_text(
        json.dumps(result["summary"], indent=2, ensure_ascii=False),
        encoding="utf-8",
    )
    return result["summary"]


def discover_bags(root: Path, commands: tuple[str, ...], mode: str) -> list[tuple[Path, str]]:
    found: list[tuple[Path, str]] = []
    for command in commands:
        command_root = root / command
        if not command_root.is_dir():
            continue
        for bag_path in sorted(path for path in command_root.iterdir() if path.is_dir()):
            manifest_path = bag_path / "trial_manifest.json"
            if not (bag_path / "metadata.yaml").is_file():
                continue
            manifest = {}
            if manifest_path.is_file():
                manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
            bag_mode = str(manifest.get("mode", ""))
            if mode != "both" and bag_mode != mode:
                continue
            found.append((bag_path, command))
    return found


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--root",
        type=Path,
        default=Path("experiments"),
        help="root containing command subdirectories",
    )
    parser.add_argument(
        "--mode",
        choices=("single", "all", "both"),
        default="both",
        help="analyze single bags, all-stimulus bags, or both",
    )
    parser.add_argument(
        "--commands",
        nargs="+",
        choices=COMMANDS,
        default=list(COMMANDS),
    )
    parser.add_argument("--bag", type=Path, help="analyze one bag instead of discovery")
    args = parser.parse_args()

    if args.bag is not None:
        targets = [(args.bag, args.bag.parent.name)]
    else:
        targets = discover_bags(args.root, tuple(args.commands), args.mode)
    if not targets:
        raise SystemExit("No matching rosbag directories found")

    summaries = []
    failures = []
    for bag_path, expected in targets:
        try:
            summary = analyze_one(bag_path, expected)
            summaries.append(summary)
            print(
                f"{bag_path}: expected={expected}; "
                f"first_valid={next((item['relative_seconds'] for item in summary['events'] if item['key'] == 'expected_valid'), None)}"
            )
        except Exception as exc:  # pragma: no cover - keep batch analysis running
            failures.append({"bag": str(bag_path), "error": str(exc)})
            print(f"FAILED {bag_path}: {exc}")

    if args.bag is not None:
        print("Single-bag analysis complete; no batch summary was overwritten.")
        return 1 if failures else 0

    aggregate_path = args.root / "command_timeline_batch_summary.json"
    aggregate_path.write_text(
        json.dumps({"summaries": summaries, "failures": failures}, indent=2, ensure_ascii=False),
        encoding="utf-8",
    )
    print(f"Analyzed {len(summaries)} bags; failures={len(failures)}")
    print(f"Batch summary: {aggregate_path}")
    return 1 if failures else 0


if __name__ == "__main__":
    raise SystemExit(main())
