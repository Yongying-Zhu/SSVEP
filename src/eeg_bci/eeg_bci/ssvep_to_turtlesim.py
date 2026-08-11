"""Safety-gated SSVEP command bridge for turtlesim."""

from __future__ import annotations

import rclpy
from eeg_interfaces.msg import SSVEPCommand
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Bool
from turtlesim.msg import Pose
from turtlesim.srv import TeleportAbsolute

from .motion_controller import MotionCommandAccumulator


class SsvepToTurtlesim(Node):
    def __init__(self) -> None:
        super().__init__("ssvep_to_turtlesim")
        self.input_topic = str(self.declare_parameter("input_topic", "/ssvep/command").value)
        self.output_topic = str(self.declare_parameter("output_topic", "/turtle1/cmd_vel").value)
        self.linear_speed = float(self.declare_parameter("linear_speed", 1.0).value)
        self.angular_speed = float(self.declare_parameter("angular_speed", 1.5).value)
        self.compose_commands = bool(
            self.declare_parameter("compose_commands", True).value
        )
        self.preserve_linear_on_invalid = bool(
            self.declare_parameter("preserve_linear_on_invalid", True).value
        )
        self.defer_stop_while_linear = bool(
            self.declare_parameter("defer_stop_while_linear", True).value
        )
        self.stimulus_gate_topic = str(
            self.declare_parameter(
                "stimulus_gate_topic", "/ssvep/stimulus_active"
            ).value
        )
        self.stimulus_gate_timeout_sec = float(
            self.declare_parameter("stimulus_gate_timeout_sec", 1.0).value
        )
        self.timeout_sec = float(self.declare_parameter("timeout_sec", 1.0).value)
        self.transition_grace_sec = float(
            self.declare_parameter("transition_grace_sec", 16.0).value
        )
        self.auto_reset_on_wall = bool(
            self.declare_parameter("auto_reset_on_wall", True).value
        )
        self.center_x = float(self.declare_parameter("center_x", 5.5).value)
        self.center_y = float(self.declare_parameter("center_y", 5.5).value)
        self.wall_margin = float(self.declare_parameter("wall_margin", 0.35).value)
        self.reset_cooldown_sec = float(
            self.declare_parameter("reset_cooldown_sec", 1.0).value
        )

        self.publisher = self.create_publisher(Twist, self.output_topic, 10)
        self.subscription = self.create_subscription(
            SSVEPCommand, self.input_topic, self._on_command, 10
        )
        self.stimulus_gate_subscription = self.create_subscription(
            Bool, self.stimulus_gate_topic, self._on_stimulus_active, 10
        )
        self.pose_subscription = self.create_subscription(
            Pose, "/turtle1/pose", self._on_pose, 10
        )
        self.teleport_client = self.create_client(
            TeleportAbsolute, "/turtle1/teleport_absolute"
        )
        self.timer = self.create_timer(0.1, self._publish_current)
        self._last_message_time = self.get_clock().now()
        self._last_valid_command_time = None
        self._turn_after_forward_time = None
        self._last_stimulus_gate_time = None
        self._stimulus_active = False
        self._last_reset_time = self.get_clock().now()
        self._reset_in_progress = False
        self._motion = MotionCommandAccumulator(
            linear_speed=self.linear_speed,
            angular_speed=self.angular_speed,
            compose_commands=self.compose_commands,
            preserve_linear_on_invalid=self.preserve_linear_on_invalid,
            defer_stop_while_linear=self.defer_stop_while_linear,
        )
        self._current = Twist()
        self.get_logger().info(
            f"Listening on {self.input_topic}; publishing to {self.output_topic}; "
            f"auto_reset_on_wall={self.auto_reset_on_wall}; "
            f"compose_commands={self.compose_commands}; "
            f"preserve_linear_on_invalid={self.preserve_linear_on_invalid}; "
            f"defer_stop_while_linear={self.defer_stop_while_linear}; "
            f"stimulus_gate={self.stimulus_gate_topic}; "
            f"transition_grace={self.transition_grace_sec:.2f}s"
        )

    def _on_stimulus_active(self, message: Bool) -> None:
        self._last_stimulus_gate_time = self.get_clock().now()
        self._stimulus_active = bool(message.data)
        if not self._stimulus_active:
            self._motion.clear()
            self._last_valid_command_time = None
            self._turn_after_forward_time = None
            self._current = Twist()

    def _on_command(self, msg: SSVEPCommand) -> None:
        now = self.get_clock().now()
        self._last_message_time = now
        if msg.valid and msg.command in ("left", "right"):
            if (
                self.compose_commands
                and self._motion.components.linear_x != 0.0
                and self._turn_after_forward_time is None
            ):
                # Keep forward/backward latched while the turn is being
                # confirmed so the combined command produces an arc.
                self._turn_after_forward_time = now
        elif msg.valid and msg.command in ("stop", "idle", "forward", "backward"):
            self._turn_after_forward_time = None
        elif not msg.valid and str(msg.reason) not in MotionCommandAccumulator.TRANSIENT_REASONS:
            self._turn_after_forward_time = None
        recognized = self._motion.apply(
            msg.command, bool(msg.valid), str(msg.reason)
        )
        if msg.valid and msg.command in MotionCommandAccumulator.MOVEMENT_COMMANDS:
            self._last_valid_command_time = now
        elif msg.valid and msg.command in ("stop", "idle"):
            self._last_valid_command_time = None
        elif not msg.valid and str(msg.reason) not in MotionCommandAccumulator.TRANSIENT_REASONS:
            self._last_valid_command_time = None
        if not recognized:
            self.get_logger().warning(f"Unknown command {msg.command!r}; stopping")
        self._current = self._to_twist()

    def _to_twist(self) -> Twist:
        components = self._motion.output_components()
        twist = Twist()
        twist.linear.x = components.linear_x
        twist.angular.z = components.angular_z
        return twist

    def _on_pose(self, pose: Pose) -> None:
        """Treat proximity to the turtlesim boundary as a collision."""
        if not self.auto_reset_on_wall or self._reset_in_progress:
            return

        near_wall = (
            pose.x <= self.wall_margin
            or pose.x >= 11.0 - self.wall_margin
            or pose.y <= self.wall_margin
            or pose.y >= 11.0 - self.wall_margin
        )
        if not near_wall:
            return

        now = self.get_clock().now()
        age = (now - self._last_reset_time).nanoseconds / 1e9
        if age < self.reset_cooldown_sec:
            return

        self._motion.clear()
        self._current = self._to_twist()
        self._last_message_time = now
        self._last_valid_command_time = None
        self._turn_after_forward_time = None
        self._last_reset_time = now
        if not self.teleport_client.service_is_ready():
            self.get_logger().warning(
                "Turtle reached the wall, but /turtle1/teleport_absolute is not ready"
            )
            return

        request = TeleportAbsolute.Request()
        request.x = self.center_x
        request.y = self.center_y
        request.theta = 0.0
        self._reset_in_progress = True
        future = self.teleport_client.call_async(request)
        future.add_done_callback(self._on_reset_done)
        self.get_logger().warning(
            f"Wall detected at x={pose.x:.2f}, y={pose.y:.2f}; "
            f"resetting turtle to ({self.center_x:.2f}, {self.center_y:.2f})"
        )

    def _on_reset_done(self, future) -> None:
        self._reset_in_progress = False
        try:
            future.result()
        except Exception as exc:  # pragma: no cover - ROS service failure
            self.get_logger().error(f"Failed to reset turtle after wall hit: {exc}")

    def _publish_current(self) -> None:
        now = self.get_clock().now()
        gate_age = float("inf")
        if self._last_stimulus_gate_time is not None:
            gate_age = (
                now - self._last_stimulus_gate_time
            ).nanoseconds / 1e9
        stimulus_gate_valid = (
            self._stimulus_active
            and gate_age <= self.stimulus_gate_timeout_sec
        )
        if not stimulus_gate_valid:
            self._motion.clear()
            self._last_valid_command_time = None
            self._turn_after_forward_time = None
            self._current = Twist()
            self.publisher.publish(self._current)
            return

        message_age = (now - self._last_message_time).nanoseconds / 1e9
        transition_age = None
        if self._last_valid_command_time is not None:
            transition_age = (
                now - self._last_valid_command_time
            ).nanoseconds / 1e9
        if message_age > self.timeout_sec:
            self._motion.clear()
            self._last_valid_command_time = None
            self._turn_after_forward_time = None
            self._current = self._to_twist()
        elif (
            self._turn_after_forward_time is not None
            and self._motion.output_enabled
            and (now - self._turn_after_forward_time).nanoseconds / 1e9
            > self.transition_grace_sec
        ):
            # After the arc has had 16 seconds to establish, remove only the
            # inherited forward/backward speed and retain the turn component.
            self._motion.clear_linear()
            self._turn_after_forward_time = None
            self._current = self._to_twist()
        elif (
            transition_age is not None
            and transition_age > self.transition_grace_sec
        ):
            self._motion.clear()
            self._last_valid_command_time = None
            self._turn_after_forward_time = None
            self._current = self._to_twist()
        self.publisher.publish(self._current)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SsvepToTurtlesim()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._motion.clear()
        node._current = node._to_twist()
        node.publisher.publish(node._current)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
