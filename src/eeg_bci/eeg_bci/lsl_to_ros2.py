"""Bridge an LSL EEG stream into typed ROS 2 messages."""

from __future__ import annotations

import threading
import time
from collections import deque

import numpy as np
import rclpy
from eeg_interfaces.msg import EEGFrame, SignalQuality
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from .eeg_qos import EEG_FRAME_QOS
from .lsl_bridge_utils import normalize_eeg_sample

try:
    from pylsl import StreamInlet, resolve_byprop
except ImportError as exc:  # pragma: no cover
    StreamInlet = resolve_byprop = None
    _IMPORT_ERROR = exc
else:
    _IMPORT_ERROR = None


class LslToRos2Node(Node):
    def __init__(self) -> None:
        super().__init__("lsl_to_ros2")
        if _IMPORT_ERROR is not None:
            raise RuntimeError("pylsl is not installed") from _IMPORT_ERROR

        self.stream_name = str(self.declare_parameter("stream_name", "").value)
        self.stream_type = str(self.declare_parameter("stream_type", "EEG").value)
        self.topic = str(self.declare_parameter("topic", "/eeg/frame").value)
        self.quality_topic = str(
            self.declare_parameter("quality_topic", "/eeg/quality").value
        )
        self.expected_channels = int(
            self.declare_parameter("expected_channels", 8).value
        )
        self.accept_soft_label = bool(
            self.declare_parameter("accept_soft_label", True).value
        )
        self.resolve_timeout = float(
            self.declare_parameter("resolve_timeout", 10.0).value
        )
        self.inlet_buffer_sec = float(
            self.declare_parameter("inlet_buffer_sec", 8.0).value
        )
        self.pull_chunk_size = max(
            1, int(self.declare_parameter("pull_chunk_size", 64).value)
        )
        self.pull_timeout_sec = float(
            self.declare_parameter("pull_timeout_sec", 0.2).value
        )
        self.reconnect_delay_sec = float(
            self.declare_parameter("reconnect_delay_sec", 1.0).value
        )
        self.quality_window = int(
            self.declare_parameter("quality_window", 250).value
        )

        self.frame_pub = self.create_publisher(
            EEGFrame, self.topic, EEG_FRAME_QOS
        )
        self.quality_pub = self.create_publisher(SignalQuality, self.quality_topic, 10)
        self._stop_event = threading.Event()
        self._sequence = 0
        self._quality_buffer = deque(maxlen=self.quality_window)
        self._received_samples = 0
        self._published_samples = 0
        self._dropped_samples = 0
        self._reconnect_count = 0
        self._last_stats_time = time.monotonic()
        self._last_stats_samples = 0
        self._worker = threading.Thread(target=self._run_worker, daemon=True)
        self._worker.start()
        self.get_logger().info(
            f"Waiting for LSL stream type={self.stream_type} name={self.stream_name or '*'}; "
            f"chunk={self.pull_chunk_size}; inlet_buffer={self.inlet_buffer_sec:.1f}s"
        )

    def destroy_node(self):
        self._stop_event.set()
        if self._worker.is_alive():
            self._worker.join(timeout=2.0)
        return super().destroy_node()

    def _resolve(self):
        if self.stream_name:
            streams = resolve_byprop(
                "name", self.stream_name, minimum=1, timeout=self.resolve_timeout
            )
        else:
            streams = resolve_byprop(
                "type", self.stream_type, minimum=1, timeout=self.resolve_timeout
            )
        if not streams:
            return None
        return streams[0]

    def _run_worker(self) -> None:
        inlet = None
        while not self._stop_event.is_set() and rclpy.ok():
            try:
                if inlet is None:
                    info = self._resolve()
                    if info is None:
                        self.get_logger().warning("No matching LSL stream yet")
                        self._stop_event.wait(self.reconnect_delay_sec)
                        continue
                    inlet = StreamInlet(
                        info,
                        max_buflen=max(1, int(round(self.inlet_buffer_sec))),
                        recover=True,
                    )
                    inlet.open_stream(timeout=self.resolve_timeout)
                    sample_rate = max(0, int(round(info.nominal_srate())))
                    self.get_logger().info(
                        f"Connected to LSL stream name={info.name()} type={info.type()} "
                        f"channels={info.channel_count()} nominal_rate={info.nominal_srate():.1f}; "
                        f"pull_chunk={self.pull_chunk_size}"
                    )

                samples, _timestamps = inlet.pull_chunk(
                    timeout=self.pull_timeout_sec,
                    max_samples=self.pull_chunk_size,
                )
                if not samples:
                    continue
                for sample in samples:
                    self._received_samples += 1
                    if self._publish_sample(sample, sample_rate=sample_rate):
                        self._published_samples += 1
                    else:
                        self._dropped_samples += 1
                self._log_stats_if_due()
            except Exception as exc:
                self._reconnect_count += 1
                self.get_logger().error(
                    f"LSL reader error: {exc}; reconnecting in "
                    f"{self.reconnect_delay_sec:.1f}s"
                )
                if inlet is not None:
                    try:
                        inlet.close_stream()
                    except Exception:
                        pass
                inlet = None
                self._stop_event.wait(self.reconnect_delay_sec)

    def _log_stats_if_due(self) -> None:
        now = time.monotonic()
        elapsed = now - self._last_stats_time
        if elapsed < 5.0:
            return
        samples = self._received_samples - self._last_stats_samples
        self.get_logger().info(
            f"LSL bridge stats: received={self._received_samples} "
            f"published={self._published_samples} dropped={self._dropped_samples} "
            f"reconnects={self._reconnect_count} rate={samples / elapsed:.1f} Hz"
        )
        self._last_stats_time = now
        self._last_stats_samples = self._received_samples

    def _publish_sample(self, raw_sample, sample_rate: int) -> bool:
        normalized = normalize_eeg_sample(
            raw_sample, self.expected_channels, self.accept_soft_label
        )
        if normalized is None:
            self.get_logger().warning(
                f"Ignoring sample with fewer than {self.expected_channels} channels"
            )
            return False

        eeg_values, soft_label = normalized
        now = self.get_clock().now().to_msg()
        msg = EEGFrame()
        msg.header.stamp = now
        msg.header.frame_id = "eeg_headband"
        msg.sequence = self._sequence
        msg.sample_rate = sample_rate
        msg.channels = eeg_values.tolist()
        msg.has_soft_label = soft_label is not None
        msg.soft_label = soft_label if soft_label is not None else 0.0
        self._sequence += 1
        self.frame_pub.publish(msg)

        self._quality_buffer.append(eeg_values.copy())
        if len(self._quality_buffer) >= self.quality_window:
            self._publish_quality(now)
            self._quality_buffer.clear()
        return True

    def _publish_quality(self, stamp) -> None:
        data = np.asarray(self._quality_buffer, dtype=np.float32)
        finite = np.isfinite(data)
        invalid_fraction = float(1.0 - np.mean(finite)) if data.size else 1.0
        finite_data = data[finite]
        rms = float(np.sqrt(np.mean(np.square(finite_data)))) if finite_data.size else 0.0
        variance = float(np.var(finite_data)) if finite_data.size else 0.0

        # This is an operational quality gate, not a clinical EEG quality metric.
        if invalid_fraction > 0.01 or not finite_data.size:
            level, valid = "poor", False
        elif rms > 500000.0:
            level, valid = "poor", False
        elif rms > 100000.0:
            level, valid = "fair", True
        else:
            level, valid = "good", True

        msg = SignalQuality()
        msg.header.stamp = stamp
        msg.header.frame_id = "eeg_headband"
        msg.rms = rms
        msg.variance = variance
        msg.invalid_fraction = invalid_fraction
        msg.level = level
        msg.valid = valid
        self.quality_pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = LslToRos2Node()
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
