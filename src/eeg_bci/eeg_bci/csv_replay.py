"""Replay the CSV files produced by the original Windows upper computer."""

from __future__ import annotations

import argparse
import csv
import time

import rclpy
from eeg_interfaces.msg import EEGFrame
from rclpy.node import Node

from .eeg_qos import EEG_FRAME_QOS


def read_recording(path: str):
    sample_rate = 250
    rows = []
    with open(path, "r", encoding="utf-8-sig", newline="") as handle:
        reader = csv.reader(handle)
        in_data = False
        for row in reader:
            if not row:
                continue
            if row[0] == "sfreq" and len(row) > 1:
                sample_rate = int(float(row[1]))
            if row[0] == "data":
                in_data = True
                continue
            if in_data and len(row) >= 9:
                rows.append([float(value) for value in row[1:9]])
    if not rows:
        raise ValueError("No rows with timestamp plus 8 EEG channels were found")
    return sample_rate, rows


class CsvReplay(Node):
    def __init__(self, path: str, topic: str, loop: bool) -> None:
        super().__init__("eeg_csv_replay")
        self.sample_rate, self.rows = read_recording(path)
        self.topic = topic
        self.loop = loop
        self.index = 0
        self.sequence = 0
        self.publisher = self.create_publisher(
            EEGFrame, topic, EEG_FRAME_QOS
        )
        self.timer = self.create_timer(1.0 / self.sample_rate, self._tick)
        self.get_logger().info(
            f"Replaying {len(self.rows)} samples at {self.sample_rate} Hz on {topic}"
        )

    def _tick(self) -> None:
        if self.index >= len(self.rows):
            if not self.loop:
                self.get_logger().info("Replay complete")
                rclpy.shutdown()
                return
            self.index = 0
        msg = EEGFrame()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "eeg_csv_replay"
        msg.sequence = self.sequence
        msg.sample_rate = self.sample_rate
        msg.channels = self.rows[self.index]
        msg.has_soft_label = False
        msg.soft_label = 0.0
        self.publisher.publish(msg)
        self.sequence += 1
        self.index += 1


def main(args=None) -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--csv", required=True)
    parser.add_argument("--topic", default="/eeg/frame")
    parser.add_argument("--loop", action="store_true")
    parsed, ros_args = parser.parse_known_args(args)
    rclpy.init(args=ros_args)
    node = CsvReplay(parsed.csv, parsed.topic, parsed.loop)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
