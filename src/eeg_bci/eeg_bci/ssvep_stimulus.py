"""Six-cell SSVEP screen with an embedded turtlesim pose mirror.

The five flickering command cells keep the existing 8-13 Hz command mapping
except for ``idle``. The sixth cell renders the live ``/turtle1/pose`` state.
The layout is backward / turtlesim / left on top and forward / stop / right
on the bottom row.
"""

from __future__ import annotations

import argparse
import threading
from collections import deque

import numpy as np


TARGET_POSITIONS = [
    (-0.66, 0.42),
    (0.0, 0.42),
    (0.66, 0.42),
    (-0.66, -0.42),
    (0.0, -0.42),
    (0.66, -0.42),
]
COMMAND_FREQUENCIES = {
    "forward": 8.0,
    "left": 9.0,
    "right": 10.0,
    "backward": 11.0,
    "stop": 12.0,
}
COMMAND_LAYOUT = [
    ("forward", TARGET_POSITIONS[3]),
    ("backward", TARGET_POSITIONS[0]),
    ("stop", TARGET_POSITIONS[4]),
    ("left", TARGET_POSITIONS[2]),
    ("right", TARGET_POSITIONS[5]),
]
TURTLE_VIEW_POSITION = TARGET_POSITIONS[1]
DEFAULT_COMMAND_FREQUENCIES = [
    COMMAND_FREQUENCIES[command] for command, _ in COMMAND_LAYOUT
]


class TurtlePoseState:
    """Thread-safe snapshot of the turtlesim pose for PsychoPy rendering."""

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self.x = 5.5
        self.y = 5.5
        self.theta = 0.0
        self.received = False
        self.history: deque[tuple[float, float]] = deque(maxlen=120)

    def update(self, x: float, y: float, theta: float) -> None:
        with self._lock:
            self.x = float(x)
            self.y = float(y)
            self.theta = float(theta)
            self.received = True
            self.history.append((self.x, self.y))

    def snapshot(self) -> tuple[float, float, float, bool, list[tuple[float, float]]]:
        with self._lock:
            return self.x, self.y, self.theta, self.received, list(self.history)


class TurtlePoseNode:
    """Optional ROS 2 subscriber; the screen remains usable without turtlesim."""

    def __init__(
        self, state: TurtlePoseState, rclpy_module, node_class, pose_type, bool_type
    ) -> None:
        self._state = state
        self._rclpy = rclpy_module
        self._bool_type = bool_type
        self.node = node_class("ssvep_stimulus_pose")
        self.node.create_subscription(pose_type, "/turtle1/pose", self._on_pose, 10)
        self._stimulus_publisher = self.node.create_publisher(
            bool_type, "/ssvep/stimulus_active", 10
        )
        self._stimulus_timer = self.node.create_timer(0.2, self._publish_active)

    def _on_pose(self, message) -> None:
        self._state.update(message.x, message.y, message.theta)

    def _publish_active(self) -> None:
        message = self._bool_type()
        message.data = True
        self._stimulus_publisher.publish(message)

    def start(self) -> threading.Thread:
        def spin() -> None:
            try:
                self._rclpy.spin(self.node)
            except Exception:
                # Closing the PsychoPy window shuts down ROS from the main thread.
                pass

        thread = threading.Thread(target=spin, daemon=True)
        thread.start()
        return thread

    def close(self) -> None:
        message = self._bool_type()
        message.data = False
        self._stimulus_publisher.publish(message)
        self._stimulus_timer.cancel()
        self.node.destroy_node()


def start_pose_subscription(state: TurtlePoseState):
    """Start the pose subscriber when ROS 2 and turtlesim interfaces are available."""

    try:
        import rclpy
        from rclpy.node import Node
        from std_msgs.msg import Bool
        from turtlesim.msg import Pose

        rclpy.init()
        subscriber = TurtlePoseNode(state, rclpy, Node, Pose, Bool)
        return rclpy, subscriber, subscriber.start()
    except Exception:
        return None, None, None


def draw_turtle_view(
    win,
    visual,
    view_rect,
    view_label,
    coordinate_label,
    turtle_body,
    turtle_heading,
    turtle_path,
    grid_lines,
    state: TurtlePoseState,
) -> None:
    """Draw a compact pose-driven turtlesim mirror inside the top-center cell."""

    view_rect.draw()
    view_label.draw()
    for grid_line in grid_lines:
        grid_line.draw()

    x, y, theta, received, history = state.snapshot()
    left = TURTLE_VIEW_POSITION[0] - 0.21
    bottom = TURTLE_VIEW_POSITION[1] - 0.21

    def to_screen(point: tuple[float, float]) -> tuple[float, float]:
        px, py = point
        return left + 0.42 * np.clip(px / 11.0, 0.0, 1.0), bottom + 0.42 * np.clip(py / 11.0, 0.0, 1.0)

    if history:
        turtle_path.vertices = [to_screen(point) for point in history]
        turtle_path.draw()

    turtle_position = to_screen((x, y))
    turtle_body.pos = turtle_position
    turtle_body.draw()
    turtle_heading.start = turtle_position
    turtle_heading.end = (
        turtle_position[0] + 0.045 * float(np.cos(theta)),
        turtle_position[1] + 0.045 * float(np.sin(theta)),
    )
    turtle_heading.draw()

    coordinate_label.text = (
        f"x={x:.1f} y={y:.1f}"
        if received
        else "waiting for /turtle1/pose"
    )
    coordinate_label.draw()


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=0.0, help="0 means run until ESC")
    parser.add_argument(
        "--frequencies",
        nargs=5,
        type=float,
        default=DEFAULT_COMMAND_FREQUENCIES,
        help="forward, backward, stop, left, right",
    )
    args = parser.parse_args()

    try:
        from psychopy import core, event, visual
    except ImportError as exc:  # pragma: no cover
        raise SystemExit("Install PsychoPy before running ssvep_stimulus") from exc

    state = TurtlePoseState()
    rclpy_module, pose_subscriber, pose_thread = start_pose_subscription(state)
    win = visual.Window(fullscr=True, color="black", units="norm", allowGUI=False)

    frequencies = list(args.frequencies)
    if len(frequencies) != len(COMMAND_LAYOUT):
        raise SystemExit("Exactly five command frequencies are required")

    targets = [
        (command, frequency, position)
        for (command, position), frequency in zip(COMMAND_LAYOUT, frequencies)
    ]
    rects = [
        visual.Rect(
            win,
            width=0.42,
            height=0.42,
            pos=position,
            fillColor="black",
            lineColor="white",
            lineWidth=2,
        )
        for _, _, position in targets
    ]
    texts = [
        visual.TextStim(
            win,
            text=f"{command}\n{frequency:g} Hz",
            pos=position,
            height=0.065,
            color="white",
            alignText="center",
        )
        for command, frequency, position in targets
    ]

    view_rect = visual.Rect(
        win,
        width=0.42,
        height=0.42,
        pos=TURTLE_VIEW_POSITION,
        fillColor="black",
        lineColor="white",
        lineWidth=2,
    )
    view_label = visual.TextStim(
        win,
        text="turtlesim",
        pos=(TURTLE_VIEW_POSITION[0], TURTLE_VIEW_POSITION[1] + 0.16),
        height=0.04,
        color="white",
    )
    coordinate_label = visual.TextStim(
        win,
        text="waiting for /turtle1/pose",
        pos=(TURTLE_VIEW_POSITION[0], TURTLE_VIEW_POSITION[1] - 0.18),
        height=0.03,
        color="lightgray",
    )
    grid_lines = [
        visual.Line(
            win,
            start=(TURTLE_VIEW_POSITION[0] - 0.21, TURTLE_VIEW_POSITION[1]),
            end=(TURTLE_VIEW_POSITION[0] + 0.21, TURTLE_VIEW_POSITION[1]),
            lineColor="dimgray",
            lineWidth=1,
        ),
        visual.Line(
            win,
            start=(TURTLE_VIEW_POSITION[0], TURTLE_VIEW_POSITION[1] - 0.21),
            end=(TURTLE_VIEW_POSITION[0], TURTLE_VIEW_POSITION[1] + 0.21),
            lineColor="dimgray",
            lineWidth=1,
        ),
    ]
    turtle_path = visual.ShapeStim(
        win,
        vertices=[TURTLE_VIEW_POSITION],
        closeShape=False,
        lineColor="gray",
        lineWidth=2,
    )
    turtle_body = visual.Circle(
        win,
        radius=0.025,
        fillColor="yellow",
        lineColor="white",
        lineWidth=1,
    )
    turtle_heading = visual.Line(
        win,
        start=TURTLE_VIEW_POSITION,
        end=(TURTLE_VIEW_POSITION[0] + 0.045, TURTLE_VIEW_POSITION[1]),
        lineColor="yellow",
        lineWidth=3,
    )

    clock = core.Clock()
    try:
        while args.duration <= 0.0 or clock.getTime() < args.duration:
            now = clock.getTime()
            for rect, text, (_, frequency, _) in zip(rects, texts, targets):
                on = np.sin(2.0 * np.pi * frequency * now) >= 0.0
                rect.fillColor = "white" if on else "black"
                rect.draw()
                text.draw()
            draw_turtle_view(
                win,
                visual,
                view_rect,
                view_label,
                coordinate_label,
                turtle_body,
                turtle_heading,
                turtle_path,
                grid_lines,
                state,
            )
            win.flip()
            if "escape" in event.getKeys():
                break
    finally:
        win.close()
        if pose_subscriber is not None:
            pose_subscriber.close()
        if rclpy_module is not None and rclpy_module.ok():
            rclpy_module.shutdown()
        if pose_thread is not None:
            pose_thread.join(timeout=1.0)
        core.quit()


if __name__ == "__main__":
    main()
