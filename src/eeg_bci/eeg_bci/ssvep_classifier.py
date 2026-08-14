"""Online FBCCA classifier for the local six-target SSVEP setup."""

from __future__ import annotations

from collections import deque
from pathlib import Path
from typing import Sequence

import numpy as np
import rclpy
from eeg_interfaces.msg import EEGFrame, SSVEPCommand, SignalQuality
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node


MODEL_FEATURE_NAMES = (
    "score_8hz",
    "score_9hz",
    "score_10hz",
    "score_11hz",
    "score_12hz",
    "score_13hz",
    "normalized_8hz",
    "normalized_9hz",
    "normalized_10hz",
    "normalized_11hz",
    "normalized_12hz",
    "normalized_13hz",
    "score_top",
    "score_second",
    "score_margin",
    "score_ratio_top_second",
    "score_mean",
    "score_std",
    "confidence",
)

PACKAGE_MODEL_PATH = Path(__file__).resolve().parent / "models" / "ssvep_classifier_model.joblib"

from .eeg_qos import EEG_FRAME_QOS
try:
    from scipy import signal
    from sklearn.cross_decomposition import CCA
except ImportError as exc:  # pragma: no cover
    signal = CCA = None
    _IMPORT_ERROR = exc
else:
    _IMPORT_ERROR = None


class SsvepClassifierNode(Node):
    def __init__(self) -> None:
        super().__init__("ssvep_classifier")
        if _IMPORT_ERROR is not None:
            raise RuntimeError("scipy and scikit-learn are required") from _IMPORT_ERROR

        self.input_topic = str(self.declare_parameter("input_topic", "/eeg/frame").value)
        self.output_topic = str(
            self.declare_parameter("output_topic", "/ssvep/command").value
        )
        self.quality_topic = str(
            self.declare_parameter("quality_topic", "/eeg/quality").value
        )
        self.window_seconds = float(self.declare_parameter("window_seconds", 4.0).value)
        self.update_period = float(self.declare_parameter("update_period", 0.25).value)
        self.harmonics = int(self.declare_parameter("harmonics", 4).value)
        self.filter_banks = int(self.declare_parameter("filter_banks", 3).value)
        self.minimum_confidence = float(
            self.declare_parameter("minimum_confidence", 0.1).value
        )
        self.use_score_margin = bool(
            self.declare_parameter("use_score_margin", False).value
        )
        self.score_margin_threshold = float(
            self.declare_parameter("score_margin_threshold", 0.0).value
        )
        self.required_consecutive = int(
            self.declare_parameter("required_consecutive_results", 2).value
        )
        self.use_trained_model = bool(
            self.declare_parameter("use_trained_model", False).value
        )
        self.model_path = str(
            self.declare_parameter(
                "model_path",
                "",
            ).value
        ).strip() or str(PACKAGE_MODEL_PATH)
        configured_reject_probability = float(
            self.declare_parameter("model_reject_probability", -1.0).value
        )
        self.debug = bool(self.declare_parameter("debug", True).value)
        self.debug_interval_frames = int(
            self.declare_parameter("debug_interval_frames", 250).value
        )
        self.targets = np.asarray(
            self.declare_parameter("targets", [8.0, 9.0, 10.0, 11.0, 12.0, 13.0]).value,
            dtype=float,
        )
        self.commands = list(
            self.declare_parameter(
                "commands", ["forward", "left", "right", "backward", "stop", "idle"]
            ).value
        )
        if len(self.targets) != len(self.commands):
            raise ValueError("targets and commands must have equal length")
        self._minimum_confidences = {
            command: float(
                self.declare_parameter(
                    f"minimum_confidence_{command}", self.minimum_confidence
                ).value
            )
            for command in self.commands
        }
        self._minimum_confidences = {
            command: float(np.clip(threshold, 0.0, 1.0))
            for command, threshold in self._minimum_confidences.items()
        }
        self._model_reject_probabilities = {
            command: float(
                self.declare_parameter(
                    f"model_reject_probability_{command}", -1.0
                ).value
            )
            for command in self.commands
        }

        self._reference_cache: dict[tuple[int, int], np.ndarray] = {}
        self._filter_cache: dict[
            tuple[int, int], list[tuple[np.ndarray, np.ndarray, float]]
        ] = {}

        self._samples = deque(maxlen=2500)
        self._sample_rate = 250
        self._last_quality = "unknown"
        self._last_prediction = None
        self._prediction_streak = 0
        self._sequence = 0
        self._frame_count = 0
        self._classify_count = 0
        self._last_debug_time = self.get_clock().now()
        self._trained_model = None
        self._model_feature_names = list(MODEL_FEATURE_NAMES)
        self._model_classes = []
        self._model_reject_probability = configured_reject_probability
        if self.use_trained_model:
            self._load_trained_model()
        # Prepare the normal 250 Hz/4 s case before the first classification.
        self._reference_signals(250, int(round(250 * self.window_seconds)))
        self._filter_bank_coefficients(250)

        self.command_pub = self.create_publisher(SSVEPCommand, self.output_topic, 10)
        self.frame_sub = self.create_subscription(
            EEGFrame, self.input_topic, self._on_frame, EEG_FRAME_QOS
        )
        self.quality_sub = self.create_subscription(
            SignalQuality, self.quality_topic, self._on_quality, 10
        )
        self.timer = self.create_timer(self.update_period, self._classify)
        self.get_logger().info(
            f"SSVEP classifier ready: targets={list(self.targets)} "
            f"window={self.window_seconds:.1f}s min_confidence={self.minimum_confidence:.2f} "
            f"use_score_margin={self.use_score_margin} "
            f"score_margin_threshold={self.score_margin_threshold:.4f} "
            f"per_command_confidence={self._minimum_confidences} "
            f"trained_model={self.use_trained_model}"
        )
        self.get_logger().info(
            f"Input: {self.input_topic}; output: {self.output_topic}; "
            f"quality: {self.quality_topic}; debug={self.debug}"
        )

    def _on_frame(self, msg: EEGFrame) -> None:
        if len(msg.channels) == 0:
            if self.debug:
                self.get_logger().warning("Received /eeg/frame with zero channels")
            return
        self._sample_rate = int(msg.sample_rate or self._sample_rate or 250)
        values = np.asarray(msg.channels, dtype=np.float64)
        self._samples.append(values)
        self._frame_count += 1
        if self.debug and (
            self._frame_count == 1
            or self._frame_count % max(1, self.debug_interval_frames) == 0
        ):
            self.get_logger().info(
                f"EEG input received: frames={self._frame_count} "
                f"buffer={len(self._samples)} channels={values.size} "
                f"sample_rate={self._sample_rate} min={np.min(values):.2f} "
                f"max={np.max(values):.2f} mean={np.mean(values):.2f} "
                f"quality={self._last_quality}"
            )

    def _on_quality(self, msg: SignalQuality) -> None:
        self._last_quality = msg.level
        if self.debug:
            self.get_logger().info(
                f"Signal quality: level={msg.level} valid={msg.valid} "
                f"rms={msg.rms:.2f} variance={msg.variance:.2f} "
                f"invalid_fraction={msg.invalid_fraction:.4f}"
            )

    def _publish(
        self,
        command: str,
        class_id: int,
        confidence: float,
        valid: bool,
        reason: str,
        normalized_scores: np.ndarray | None = None,
        fbcca_command: str = "",
    ):
        msg = SSVEPCommand()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "ssvep"
        msg.command = command
        msg.class_id = max(0, min(255, int(class_id)))
        msg.confidence = float(np.clip(confidence, 0.0, 1.0))
        msg.valid = bool(valid)
        msg.reason = reason
        if normalized_scores is None:
            normalized = np.zeros(6, dtype=np.float32)
        else:
            normalized = np.asarray(normalized_scores, dtype=np.float32).reshape(-1)
            if normalized.size != 6:
                normalized = np.zeros(6, dtype=np.float32)
        msg.normalized_scores = normalized.tolist()
        msg.fbcca_command = str(fbcca_command)
        self.command_pub.publish(msg)
        self._sequence += 1
        if self.debug:
            self.get_logger().info(
                f"COMMAND published: seq={self._sequence} command={command} "
                f"class_id={class_id} confidence={confidence:.4f} "
                f"valid={valid} reason={reason}"
            )

    def _load_trained_model(self) -> None:
        model_path = Path(self.model_path).expanduser().resolve()
        if not model_path.is_file():
            raise RuntimeError(f"Trained SSVEP model does not exist: {model_path}")
        try:
            import joblib

            artifact = joblib.load(model_path)
        except Exception as exc:  # pragma: no cover - environment/model failure
            raise RuntimeError(f"Failed to load trained SSVEP model: {model_path}") from exc

        pipeline = artifact.get("model") if isinstance(artifact, dict) else None
        if pipeline is None or not hasattr(pipeline, "predict_proba"):
            raise RuntimeError("The trained SSVEP artifact does not contain predict_proba()")
        feature_names = list(artifact.get("feature_names", MODEL_FEATURE_NAMES))
        if feature_names != list(MODEL_FEATURE_NAMES):
            raise RuntimeError(
                "Trained model feature order does not match the online classifier"
            )
        classes = list(artifact.get("classes", getattr(pipeline, "classes_", [])))
        if not classes:
            raise RuntimeError("The trained SSVEP model contains no class names")
        if "unknown" not in classes:
            raise RuntimeError("The trained SSVEP model must contain an 'unknown' class")

        if self._model_reject_probability < 0.0:
            self._model_reject_probability = float(
                artifact.get("reject_probability", 0.50)
            )
        self._model_reject_probability = float(
            np.clip(self._model_reject_probability, 0.0, 1.0)
        )
        self._trained_model = pipeline
        self._model_feature_names = feature_names
        self._model_classes = [str(label) for label in classes]
        self.get_logger().info(
            f"Loaded trained SSVEP model: path={model_path} "
            f"classes={self._model_classes} "
            f"reject_probability={self._model_reject_probability:.2f}"
        )

    def _model_features(self, scores: np.ndarray) -> np.ndarray:
        """Build exactly the feature order used by analyze_rosbags.py."""
        scores = np.asarray(scores, dtype=np.float64)
        total = float(np.maximum(scores, 0.0).sum())
        normalized = scores / (total + 1e-12)
        ordered = np.sort(scores)[::-1]
        feature_values = {
            **{
                f"score_{int(target)}hz": float(score)
                for target, score in zip(self.targets, scores)
            },
            **{
                f"normalized_{int(target)}hz": float(value)
                for target, value in zip(self.targets, normalized)
            },
            "score_top": float(ordered[0]),
            "score_second": float(ordered[1]),
            "score_margin": float(ordered[0] - ordered[1]),
            "score_ratio_top_second": float(ordered[0] / (ordered[1] + 1e-12)),
            "score_mean": float(np.mean(scores)),
            "score_std": float(np.std(scores)),
            "confidence": float(ordered[0] / (total + 1e-12)),
        }
        try:
            return np.asarray(
                [feature_values[name] for name in self._model_feature_names],
                dtype=np.float64,
            )
        except KeyError as exc:
            raise RuntimeError(f"Missing online model feature: {exc.args[0]}") from exc

    def _reject_probability_for(self, label: str) -> float:
        configured = self._model_reject_probabilities.get(label, -1.0)
        if configured < 0.0:
            configured = self._model_reject_probability
        return float(np.clip(configured, 0.0, 1.0))

    def _classify_with_trained_model(
        self,
        scores: np.ndarray,
        normalized_scores: np.ndarray,
        fbcca_command: str,
    ) -> None:
        features = self._model_features(scores).reshape(1, -1)
        probabilities = np.asarray(self._trained_model.predict_proba(features)[0])
        if probabilities.size != len(self._model_classes):
            raise RuntimeError("Trained model probability output does not match its classes")
        best_index = int(np.argmax(probabilities))
        predicted_label = self._model_classes[best_index]
        model_confidence = float(probabilities[best_index])
        reject_probability = self._reject_probability_for(predicted_label)
        unknown_index = self._model_classes.index("unknown")
        unknown_probability = float(probabilities[unknown_index])

        if self.debug:
            probability_text = ", ".join(
                f"{label}={probability:.3f}"
                for label, probability in zip(self._model_classes, probabilities)
            )
            self.get_logger().info(
                f"MODEL result: {probability_text}; best={predicted_label} "
                f"confidence={model_confidence:.4f} "
                f"reject_threshold={reject_probability:.4f}"
            )

        if predicted_label == "unknown":
            self._last_prediction = None
            self._prediction_streak = 0
            self._publish(
                "stop", 0, unknown_probability, False, "model_unknown",
                normalized_scores, fbcca_command,
            )
            return
        if model_confidence < reject_probability:
            self._last_prediction = None
            self._prediction_streak = 0
            self._publish(
                "stop", 0, model_confidence, False, "model_low_confidence",
                normalized_scores, fbcca_command,
            )
            return
        if predicted_label not in self.commands:
            raise RuntimeError(f"Model predicted unsupported command: {predicted_label}")

        class_id = self.commands.index(predicted_label) + 1
        if self._last_prediction == predicted_label:
            self._prediction_streak += 1
        else:
            self._last_prediction = predicted_label
            self._prediction_streak = 1
        if self._prediction_streak < self.required_consecutive:
            self._publish(
                "stop", class_id, model_confidence, False, "awaiting_confirmation",
                normalized_scores, fbcca_command,
            )
            return
        self._publish(
            predicted_label, class_id, model_confidence, True, "trained_model",
            normalized_scores, fbcca_command,
        )

    def _classify(self) -> None:
        self._classify_count += 1
        fs = int(self._sample_rate or 250)
        required = int(round(fs * self.window_seconds))
        if len(self._samples) < required:
            if self.debug and self._classify_count % 4 == 0:
                self.get_logger().info(
                    f"Classifier waiting for buffer: {len(self._samples)}/{required} "
                    f"samples ({len(self._samples) / max(fs, 1):.2f}s)"
                )
            self._publish("stop", 0, 0.0, False, "warming_up")
            return
        if self._last_quality == "poor":
            if self.debug:
                self.get_logger().warning("Classification blocked by poor signal quality")
            self._publish("stop", 0, 0.0, False, "poor_signal_quality")
            return

        matrix = np.asarray(list(self._samples)[-required:], dtype=np.float64).T
        if self.debug:
            self.get_logger().info(
                f"Classification input ready: matrix={matrix.shape} fs={fs} "
                f"quality={self._last_quality}"
            )
        if matrix.shape[1] < required or not np.isfinite(matrix).all():
            if self.debug:
                self.get_logger().warning(
                    f"Invalid EEG matrix: shape={matrix.shape} "
                    f"finite={np.isfinite(matrix).all()}"
                )
            self._publish("stop", 0, 0.0, False, "invalid_eeg")
            return

        try:
            scores = self._fbcca(matrix, fs)
        except Exception as exc:
            self.get_logger().warning(f"SSVEP classification failed: {exc}")
            self._publish("stop", 0, 0.0, False, "classifier_error")
            return

        order = np.argsort(scores)[::-1]
        best = int(order[0])
        total = float(np.sum(np.maximum(scores, 0.0)))
        normalized_scores = np.maximum(scores, 0.0) / (total + 1e-12)
        fbcca_command = self.commands[best]
        confidence = float(max(scores[best], 0.0) / (total + 1e-12))
        score_margin = float(scores[best] - scores[order[1]])
        confidence_threshold = self._minimum_confidences[fbcca_command]
        if self.debug:
            score_text = ", ".join(
                f"{self.targets[i]:g}Hz={scores[i]:.5f}" for i in range(len(scores))
            )
            self.get_logger().info(
                f"FBCCA scores: {score_text}; raw_best={self.targets[best]:g}Hz "
                f"raw_command={self.commands[best]} raw_confidence={confidence:.4f} "
                f"score_margin={score_margin:.4f} "
                f"margin_threshold={self.score_margin_threshold:.4f} "
                f"threshold={confidence_threshold:.4f} "
                f"streak_before={self._prediction_streak}"
            )
        if self.use_score_margin and score_margin < self.score_margin_threshold:
            self._last_prediction = None
            self._prediction_streak = 0
            self._publish(
                "stop", best + 1, confidence, False, "low_score_margin",
                normalized_scores, fbcca_command,
            )
            return
        if self.use_trained_model:
            try:
                self._classify_with_trained_model(
                    scores, normalized_scores, fbcca_command
                )
            except Exception as exc:
                self.get_logger().warning(f"Trained model classification failed: {exc}")
                self._last_prediction = None
                self._prediction_streak = 0
                self._publish("stop", 0, 0.0, False, "model_error")
            return
        if self._last_prediction == best:
            self._prediction_streak += 1
        else:
            self._last_prediction = best
            self._prediction_streak = 1

        if confidence < confidence_threshold:
            self._publish(
                "stop", best + 1, confidence, False, "low_confidence",
                normalized_scores, fbcca_command,
            )
            return
        if self._prediction_streak < self.required_consecutive:
            self._publish(
                "stop", best + 1, confidence, False, "awaiting_confirmation",
                normalized_scores, fbcca_command,
            )
            return

        self._publish(
            self.commands[best], best + 1, confidence, True, "fbcca",
            normalized_scores, fbcca_command,
        )

    def _reference_signals(self, fs: int, length: int) -> np.ndarray:
        key = (int(fs), int(length))
        cached = self._reference_cache.get(key)
        if cached is not None:
            return cached

        t = np.arange(length, dtype=float) / float(fs)
        refs = []
        for target in self.targets:
            components = []
            for harmonic in range(1, self.harmonics + 1):
                components.append(np.sin(2 * np.pi * harmonic * target * t))
                components.append(np.cos(2 * np.pi * harmonic * target * t))
            refs.append(np.asarray(components))
        cached = np.asarray(refs)
        self._reference_cache[key] = cached
        return cached

    def _filter_bank_coefficients(
        self, fs: int
    ) -> list[tuple[np.ndarray, np.ndarray, float]]:
        key = (int(fs), int(self.filter_banks))
        cached = self._filter_cache.get(key)
        if cached is not None:
            return cached

        nyq = fs / 2.0
        max_banks = min(self.filter_banks, 3)
        passbands = [6.0, 14.0, 22.0]
        stopbands = [4.0, 10.0, 16.0]
        high_pass, high_stop = min(80.0, nyq - 5.0), min(90.0, nyq - 1.0)
        coefficients = []
        for bank in range(max_banks):
            wp = [passbands[bank] / nyq, high_pass / nyq]
            ws = [stopbands[bank] / nyq, high_stop / nyq]
            order, wn = signal.cheb1ord(wp, ws, 3, 40)
            b, a = signal.cheby1(order, 0.5, wn, btype="bandpass")
            weight = bank ** -1.25 + 0.25 if bank else 1.0
            coefficients.append((b, a, weight))
        self._filter_cache[key] = coefficients
        return coefficients

    def _fbcca(self, data: np.ndarray, fs: int) -> np.ndarray:
        length = data.shape[1]
        refs = self._reference_signals(fs, length)
        scores = np.zeros(len(self.targets), dtype=float)

        for b, a, weight in self._filter_bank_coefficients(fs):
            filtered = signal.filtfilt(b, a, data, axis=1, padlen=min(3 * max(len(a), len(b)), length - 1))
            for index, ref in enumerate(refs):
                cca = CCA(n_components=1)
                cca.fit(filtered.T, ref.T)
                x_c, y_c = cca.transform(filtered.T, ref.T)
                corr = np.corrcoef(x_c[:, 0], y_c[:, 0])[0, 1]
                scores[index] += weight * float(np.nan_to_num(corr) ** 2)
        return scores


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SsvepClassifierNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    main()
