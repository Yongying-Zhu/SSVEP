"""Offline FBCCA analysis and automatic model training for EEG rosbags."""

from __future__ import annotations

import argparse
import csv
import json
import re
import random
import shutil
from collections import Counter
from datetime import datetime
from pathlib import Path

import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

from .ssvep_classifier import SsvepClassifierNode


TARGETS = np.asarray([8.0, 9.0, 10.0, 11.0, 12.0, 13.0])
COMMANDS = ["forward", "left", "right", "backward", "stop", "idle"]
UNKNOWN_CONDITIONS = ["nostimulus", "close_eyes", "no_target", "free_view"]
CURATED_CONDITIONS = COMMANDS + UNKNOWN_CONDITIONS
LABELS = {
    "go_forwrad": "forward",
    "go_backward": "backward",
    "stop_stop": "stop",
    "turn_left": "left",
    "turn_right": "right",
    "left": "left",
}
EXCLUDED_TRAINING_PREFIXES = ("idle_idle",)
UNKNOWN_NAMES = {
    "close_eyes",
    "closed_eyes",
    "nostimulus",
    "no_stimulus",
    "no_target",
    "unknown",
}
SCORE_COLUMNS = [f"score_{int(target)}hz" for target in TARGETS]
FEATURE_NAMES = (
    SCORE_COLUMNS
    + [f"normalized_{int(target)}hz" for target in TARGETS]
    + [
        "score_top",
        "score_second",
        "score_margin",
        "score_ratio_top_second",
        "score_mean",
        "score_std",
        "confidence",
    ]
)


def curated_bag_label(name: str) -> str | None:
    """Return the label for one of the 80 new, strictly named recordings.

    Only ``<condition>_01`` through ``<condition>_08`` are eligible for the
    new model.  This prevents legacy bags such as ``idle_idle`` and
    ``turn_left`` from silently contaminating the training set.
    """
    normalized = name.lower().replace("-", "_")
    match = re.fullmatch(
        r"(forward|left|right|backward|stop|idle|nostimulus|close_eyes|no_target|free_view)_(0[1-8])",
        normalized,
    )
    if not match:
        return None
    condition = match.group(1)
    return "unknown" if condition in UNKNOWN_CONDITIONS else condition


def read_bag(path: Path):
    message_type = get_message("eeg_interfaces/msg/EEGFrame")
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(path), storage_id="sqlite3"),
        rosbag2_py.ConverterOptions("", ""),
    )
    samples, times, sample_rates = [], [], []
    while reader.has_next():
        topic, data, timestamp = reader.read_next()
        if topic != "/eeg/frame":
            continue
        msg = deserialize_message(data, message_type)
        if len(msg.channels) != 8:
            continue
        values = np.asarray(msg.channels, dtype=np.float64)
        if not np.isfinite(values).all():
            continue
        samples.append(values)
        times.append(int(timestamp))
        sample_rates.append(int(msg.sample_rate or 250))
    if not samples:
        return None
    return np.asarray(samples), np.asarray(times), int(round(np.median(sample_rates)))


def bag_label(name: str) -> str:
    """Map a rosbag name to a training label.

    Repeated trials such as ``forward_01`` and ``forward_02`` use the same
    command label. Negative recordings are deliberately mapped to unknown.
    """
    normalized = name.lower().replace("-", "_")
    if normalized in UNKNOWN_NAMES:
        return "unknown"
    if normalized in LABELS:
        return LABELS[normalized]
    for prefix, label in (
        ("forward", "forward"),
        ("backward", "backward"),
        ("left", "left"),
        ("right", "right"),
        ("stop", "stop"),
    ):
        if normalized.startswith(f"{prefix}_"):
            return label
    if re.fullmatch(r"idle_\d+", normalized):
        return "idle"
    if normalized.startswith("go_forwrad") or normalized.startswith("go_forward"):
        return "forward"
    if normalized.startswith("go_backward"):
        return "backward"
    if normalized.startswith("turn_left"):
        return "left"
    if normalized.startswith("turn_right"):
        return "right"
    if normalized.startswith("stop"):
        return "stop"
    if any(token in normalized for token in ("close_eye", "nostim", "no_target", "unknown")):
        return "unknown"
    return "unknown"


def excluded_from_training(name: str) -> bool:
    """Return whether a recording is outside the curated 80-bag dataset."""
    return curated_bag_label(name) is None


def split_curated_bags(
    bag_names: list[str],
    test_bags_per_condition: int = 2,
    seed: int = 42,
):
    """Create a deterministic, bag-level train/test split.

    Command and negative conditions are split independently so every one of
    the four unknown recording conditions appears in both partitions.  The
    split is by whole bag, never by overlapping EEG window.
    """
    rng = random.Random(seed)
    eligible = sorted({name for name in bag_names if curated_bag_label(name) is not None})
    by_condition = {}
    for name in eligible:
        condition = name.rsplit("_", 1)[0]
        by_condition.setdefault(condition, []).append(name)

    expected_conditions = set(CURATED_CONDITIONS)
    missing_conditions = sorted(expected_conditions - set(by_condition))
    if missing_conditions:
        raise ValueError(f"Missing curated recording conditions: {', '.join(missing_conditions)}")

    train_bags, test_bags = [], []
    for condition in CURATED_CONDITIONS:
        names = sorted(by_condition[condition])
        if len(names) != 8:
            raise ValueError(
                f"Condition {condition} has {len(names)} bags; exactly 8 named _01 through _08 are required"
            )
        selected_test = set(rng.sample(names, test_bags_per_condition))
        test_bags.extend(sorted(selected_test))
        train_bags.extend(name for name in names if name not in selected_test)

    split_by_bag = {name: "train" for name in train_bags}
    split_by_bag.update({name: "test" for name in test_bags})
    return split_by_bag, {
        "seed": seed,
        "test_bags_per_condition": test_bags_per_condition,
        "train_bags": sorted(train_bags),
        "test_bags": sorted(test_bags),
        "train_bag_count": len(train_bags),
        "test_bag_count": len(test_bags),
    }


def analyze_bag(path, classifier, threshold, window_seconds, stride_seconds, required_consecutive):
    result = read_bag(path)
    if result is None:
        return None, []
    samples, times, sample_rate = result
    window_size = int(round(sample_rate * window_seconds))
    stride = max(1, int(round(sample_rate * stride_seconds)))
    expected = bag_label(path.name)
    if len(samples) < window_size:
        return {
            "bag": path.name, "messages": len(samples), "sample_rate": sample_rate,
            "windows": 0, "expected_label": expected, "status": "too_short",
        }, []

    rows = []
    last_prediction = None
    prediction_streak = 0
    for index, start in enumerate(range(0, len(samples) - window_size + 1, stride)):
        scores = classifier._fbcca(samples[start:start + window_size].T, sample_rate)
        best = int(np.argmax(scores))
        total = float(np.maximum(scores, 0.0).sum())
        confidence = float(max(scores[best], 0.0) / (total + 1e-12))
        raw_command = COMMANDS[best]
        if last_prediction == best:
            prediction_streak += 1
        else:
            last_prediction, prediction_streak = best, 1

        if confidence < threshold:
            published_command, valid, reason = "stop", False, "low_confidence"
        elif prediction_streak < required_consecutive:
            published_command, valid, reason = "stop", False, "awaiting_confirmation"
        else:
            published_command, valid, reason = raw_command, True, "fbcca"

        row = {
            "bag": path.name,
            "window_index": index,
            "start_sample": start,
            "end_sample": start + window_size,
            "start_time_ns": int(times[start]),
            "expected_label": expected,
            "best_frequency_hz": float(TARGETS[best]),
            "raw_command": raw_command,
            "published_command": published_command,
            "confidence": confidence,
            "valid": valid,
            "reason": reason,
            "prediction_streak": prediction_streak,
        }
        for score_index, score in enumerate(scores):
            row[f"score_{int(TARGETS[score_index])}hz"] = float(score)
        rows.append(row)

    valid_rows = [row for row in rows if row["valid"]]
    raw_correct = [row["raw_command"] == expected for row in rows if expected != "unknown"]
    valid_correct = [row["published_command"] == expected for row in valid_rows if expected != "unknown"]
    summary = {
        "bag": path.name,
        "messages": len(samples),
        "sample_rate": sample_rate,
        "windows": len(rows),
        "expected_label": expected,
        "mean_confidence": float(np.mean([row["confidence"] for row in rows])),
        "max_confidence": float(np.max([row["confidence"] for row in rows])),
        "raw_accuracy": float(np.mean(raw_correct)) if raw_correct else None,
        "valid_accuracy": float(np.mean(valid_correct)) if valid_correct else None,
        "valid_rate": float(len(valid_rows) / len(rows)) if rows else 0.0,
        "prediction_counts": dict(Counter(row["raw_command"] for row in rows)),
        "status": "ok",
    }
    return summary, rows


def write_csv(path: Path, rows):
    if not rows:
        return
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)


def load_config(path: Path) -> dict:
    """Read the scalar ROS parameters needed by offline analysis.

    The workspace config is intentionally simple YAML, so this avoids adding a
    PyYAML dependency just for six scalar values.
    """
    values = {}
    if not path.exists():
        return values
    for line in path.read_text(encoding="utf-8").splitlines():
        match = re.match(r"^\s*([A-Za-z_][A-Za-z0-9_]*):\s*([^#]+?)\s*$", line)
        if not match:
            continue
        key, raw = match.groups()
        raw = raw.strip().strip('"').strip("'")
        try:
            values[key] = float(raw) if "." in raw else int(raw)
        except ValueError:
            if raw.lower() in ("true", "false"):
                values[key] = raw.lower() == "true"
    return values


def row_to_features(row: dict) -> np.ndarray:
    """Convert one FBCCA result row into model features.

    Only signal-derived FBCCA scores are used. ``published_command``,
    ``valid`` and ``reason`` are excluded because they are outputs of the
    existing rule-based classifier and would leak the answer into training.
    """
    scores = np.asarray([float(row[column]) for column in SCORE_COLUMNS], dtype=np.float64)
    total = float(np.maximum(scores, 0.0).sum())
    normalized = scores / (total + 1e-12)
    ordered = np.sort(scores)[::-1]
    top = float(ordered[0])
    second = float(ordered[1])
    confidence = top / (total + 1e-12)
    return np.concatenate([
        scores,
        normalized,
        np.asarray([
            top,
            second,
            top - second,
            top / (second + 1e-12),
            float(np.mean(scores)),
            float(np.std(scores)),
            confidence,
        ]),
    ])


def prepare_training_rows(rows: list[dict], split_by_bag: dict[str, str]):
    training_rows = []
    for row in rows:
        bag = str(row.get("bag", ""))
        label = curated_bag_label(bag)
        if label is None or bag not in split_by_bag:
            continue
        try:
            features = row_to_features(row)
        except (KeyError, TypeError, ValueError):
            continue
        training_row = {
            "bag": bag,
            "trial_id": bag,
            "split": split_by_bag[bag],
            "label": label,
        }
        training_row.update({name: float(value) for name, value in zip(FEATURE_NAMES, features)})
        training_rows.append(training_row)
    return training_rows


def rows_to_xy(rows: list[dict]):
    labels = np.asarray([row["label"] for row in rows])
    trials = np.asarray([row["trial_id"] for row in rows])
    features = np.asarray(
        [[float(row[name]) for name in FEATURE_NAMES] for row in rows],
        dtype=np.float64,
    )
    return features, labels, trials


def evaluate_model(pipeline, rows: list[dict]) -> dict:
    """Evaluate at window level after keeping whole bags out of training."""
    if not rows:
        return {"status": "not_available", "reason": "no_rows"}
    from sklearn.metrics import accuracy_score, balanced_accuracy_score, f1_score

    features, labels, _ = rows_to_xy(rows)
    predictions = pipeline.predict(features)
    label_order = sorted(set(labels) | set(predictions))
    confusion = {
        actual: {
            predicted: int(np.sum((labels == actual) & (predictions == predicted)))
            for predicted in label_order
        }
        for actual in label_order
    }
    unknown_mask = labels == "unknown"
    known_mask = ~unknown_mask
    return {
        "status": "available",
        "windows": len(rows),
        "bags": sorted({row["bag"] for row in rows}),
        "accuracy": float(accuracy_score(labels, predictions)),
        "balanced_accuracy": float(balanced_accuracy_score(labels, predictions)),
        "macro_f1": float(f1_score(labels, predictions, average="macro")),
        "unknown_recall": float(np.mean(predictions[unknown_mask] == "unknown"))
        if np.any(unknown_mask) else None,
        "known_to_unknown_rate": float(np.mean(predictions[known_mask] == "unknown"))
        if np.any(known_mask) else None,
        "confusion_matrix": confusion,
    }


def train_model(
    training_rows: list[dict],
    output_dir: Path,
    settings: dict,
    split_info: dict,
) -> dict:
    """Fit the selected seven-class model on train bags and test held-out bags."""
    train_rows = [row for row in training_rows if row["split"] == "train"]
    test_rows = [row for row in training_rows if row["split"] == "test"]
    report = {
        "status": "not_trained",
        "model_type": "standard_scaler_logistic_regression",
        "feature_names": FEATURE_NAMES,
        "classes": [],
        "train_samples": len(train_rows),
        "test_samples": len(test_rows),
        "train_class_counts": dict(Counter(row["label"] for row in train_rows)),
        "test_class_counts": dict(Counter(row["label"] for row in test_rows)),
        "train_trial_counts": {},
        "test_trial_counts": {},
        "window_cv": {
            "status": "not_used",
            "reason": "Overlapping windows are not independent samples",
        },
        "group_cv": {"status": "not_available"},
        "held_out_test": {"status": "not_available"},
        "split": split_info,
    }
    if not train_rows or not test_rows:
        report["reason"] = "both_train_and_test_rows_are_required"
        return report

    train_features, train_labels, train_trials = rows_to_xy(train_rows)
    classes = sorted(set(train_labels))
    report["classes"] = classes
    report["train_trial_counts"] = {
        label: len(set(train_trials[train_labels == label])) for label in classes
    }
    report["test_trial_counts"] = {
        label: len({row["trial_id"] for row in test_rows if row["label"] == label})
        for label in sorted({row["label"] for row in test_rows})
    }
    if len(classes) < 2:
        report["reason"] = "at_least_two_classes_are_required"
        return report

    from joblib import dump
    from sklearn.linear_model import LogisticRegression
    from sklearn.pipeline import Pipeline
    from sklearn.preprocessing import StandardScaler

    pipeline = Pipeline([
        ("scaler", StandardScaler()),
        (
            "classifier",
            LogisticRegression(
                max_iter=3000,
                class_weight="balanced",
                solver="lbfgs",
                random_state=42,
            ),
        ),
    ])
    pipeline.fit(train_features, train_labels)

    from sklearn.model_selection import StratifiedGroupKFold, cross_val_predict
    min_class_trials = min(report["train_trial_counts"].values())
    if min_class_trials >= 2:
        folds = min(5, min_class_trials)
        cv = StratifiedGroupKFold(n_splits=folds, shuffle=True, random_state=42)
        cv_predictions = cross_val_predict(
            pipeline,
            train_features,
            train_labels,
            groups=train_trials,
            cv=cv,
            method="predict",
        )
        from sklearn.metrics import balanced_accuracy_score, f1_score
        report["group_cv"] = {
            "status": "available",
            "folds": folds,
            "bags_are_groups": True,
            "balanced_accuracy": float(
                balanced_accuracy_score(train_labels, cv_predictions)
            ),
            "macro_f1": float(f1_score(train_labels, cv_predictions, average="macro")),
        }

    report["held_out_test"] = evaluate_model(pipeline, test_rows)
    artifact = {
        "model": pipeline,
        "feature_names": FEATURE_NAMES,
        "classes": classes,
        "unknown_label": "unknown",
        "reject_probability": 0.50,
        "settings": settings,
        "split": split_info,
    }
    model_path = output_dir / "ssvep_classifier_model.joblib"
    dump(artifact, model_path)
    report["status"] = "trained"
    report["model_path"] = str(model_path)

    with (output_dir / "model_report.json").open("w", encoding="utf-8") as handle:
        json.dump(report, handle, ensure_ascii=False, indent=2)
    return report


def main(args=None):
    workspace_root = Path.cwd()
    parser = argparse.ArgumentParser(description="Analyze existing EEG rosbag recordings")
    parser.add_argument("--input-root", default=str(workspace_root))
    parser.add_argument("--output-dir", default="")
    parser.add_argument(
        "--config",
        default=str(workspace_root / "src" / "eeg_bci" / "config" / "eeg.yaml"),
    )
    parser.add_argument("--threshold", type=float, default=None)
    parser.add_argument("--window-seconds", type=float, default=None)
    parser.add_argument("--stride-seconds", type=float, default=None)
    parser.add_argument("--required-consecutive", type=int, default=None)
    parser.add_argument(
        "--test-bags-per-condition",
        type=int,
        default=2,
        help="Held-out bags per command/negative condition (default: 2)",
    )
    parser.add_argument(
        "--split-seed",
        type=int,
        default=42,
        help="Deterministic seed for the bag-level split (default: 42)",
    )
    parser.add_argument(
        "--no-train",
        action="store_true",
        help="Only analyze bags; skip automatic model training",
    )
    parsed = parser.parse_args(args)

    input_root = Path(parsed.input_root).expanduser().resolve()
    config_path = Path(parsed.config).expanduser().resolve()
    config = load_config(config_path)
    threshold = parsed.threshold if parsed.threshold is not None else float(config.get("minimum_confidence", 0.1))
    window_seconds = parsed.window_seconds if parsed.window_seconds is not None else float(config.get("window_seconds", 4.0))
    stride_seconds = parsed.stride_seconds if parsed.stride_seconds is not None else float(config.get("update_period", 1.0))
    required_consecutive = parsed.required_consecutive if parsed.required_consecutive is not None else int(config.get("required_consecutive_results", 2))
    harmonics = int(config.get("harmonics", 4))
    filter_banks = int(config.get("filter_banks", 3))

    output_dir = Path(parsed.output_dir).expanduser().resolve() if parsed.output_dir else (
        input_root / "analysis_results_reanalysis"
    )
    if input_root not in output_dir.parents:
        raise ValueError(f"Refusing to clean output outside input root: {output_dir}")
    if output_dir == input_root or output_dir.is_symlink():
        raise ValueError(f"Refusing to clean unsafe output path: {output_dir}")
    if output_dir.exists():
        if not output_dir.is_dir():
            raise ValueError(f"Output path is not a directory: {output_dir}")
        shutil.rmtree(output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)

    classifier = SsvepClassifierNode.__new__(SsvepClassifierNode)
    classifier.targets = TARGETS
    classifier.harmonics = harmonics
    classifier.filter_banks = filter_banks
    summaries, all_rows = [], []
    bag_dirs = sorted(p for p in input_root.iterdir() if p.is_dir() and (p / "metadata.yaml").exists())
    curated_bag_names = [
        bag_dir.name for bag_dir in bag_dirs if curated_bag_label(bag_dir.name) is not None
    ]
    split_by_bag, split_info = split_curated_bags(
        curated_bag_names,
        test_bags_per_condition=parsed.test_bags_per_condition,
        seed=parsed.split_seed,
    )
    with (output_dir / "dataset_split.json").open("w", encoding="utf-8") as handle:
        json.dump(split_info, handle, ensure_ascii=False, indent=2)
    for bag_dir in bag_dirs:
        summary, rows = analyze_bag(
            bag_dir, classifier, threshold, window_seconds,
            stride_seconds, required_consecutive,
        )
        if summary is None:
            summary = {
                "bag": bag_dir.name,
                "messages": 0,
                "windows": 0,
                "expected_label": bag_label(bag_dir.name),
                "status": "empty_or_invalid",
            }
        summaries.append(summary)
        all_rows.extend(rows)
        write_csv(output_dir / f"{bag_dir.name}_results.csv", rows)
        print(
            f"{bag_dir.name}: messages={summary['messages']} windows={summary['windows']} "
            f"mean_confidence={summary.get('mean_confidence', '')} status={summary['status']}"
        )

    write_csv(output_dir / "all_windows.csv", all_rows)
    write_csv(output_dir / "summary.csv", summaries)
    training_rows = prepare_training_rows(all_rows, split_by_bag)
    write_csv(output_dir / "training_features.csv", training_rows)
    write_csv(
        output_dir / "train_features.csv",
        [row for row in training_rows if row["split"] == "train"],
    )
    write_csv(
        output_dir / "test_features.csv",
        [row for row in training_rows if row["split"] == "test"],
    )
    model_report = {
        "status": "skipped",
        "reason": "--no-train was supplied",
    }
    if not parsed.no_train:
        model_report = train_model(
            training_rows,
            output_dir,
            {
                "threshold": threshold,
                "window_seconds": window_seconds,
                "stride_seconds": stride_seconds,
                "required_consecutive": required_consecutive,
                "harmonics": harmonics,
                "filter_banks": filter_banks,
                "targets_hz": TARGETS.tolist(),
                "split_seed": parsed.split_seed,
                "test_bags_per_condition": parsed.test_bags_per_condition,
            },
            split_info,
        )
    with (output_dir / "summary.json").open("w", encoding="utf-8") as handle:
        json.dump({
            "settings": {
                "config_path": str(config_path),
                "threshold": threshold,
                "window_seconds": window_seconds,
                "stride_seconds": stride_seconds,
                "required_consecutive": required_consecutive,
                "harmonics": harmonics,
                "filter_banks": filter_banks,
                "targets_hz": TARGETS.tolist(),
                "commands": COMMANDS,
                "curated_conditions": CURATED_CONDITIONS,
                "split": split_info,
            },
            "bags": summaries,
            "model": model_report,
        }, handle, ensure_ascii=False, indent=2)
    if model_report.get("status") == "trained":
        print(f"Model written to: {model_report['model_path']}")
        print(f"Training report written to: {output_dir / 'model_report.json'}")
    print(f"Results written to: {output_dir}")


if __name__ == "__main__":
    main()
