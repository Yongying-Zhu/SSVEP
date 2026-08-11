"""Generic SSVEP command bridge for robots accepting geometry_msgs/Twist."""

from __future__ import annotations

import rclpy
from eeg_interfaces.msg import SSVEPCommand
from geometry_msgs.msg import Twist
from rclpy.node import Node

from .motion_controller import MotionCommandAccumulator


class SsvepToCmdVel(Node):
    """Convert SSVEP commands into a watchdog-gated cmd_vel stream."""

    def __init__(self) -> None:
        super().__init__("ssvep_to_cmd_vel")
        self.input_topic = str(
            self.declare_parameter("input_topic", "/ssvep/command").value
        )
        self.output_topic = str(
            self.declare_parameter("output_topic", "/cmd_vel").value
        )
        self.linear_speed = float(self.declare_parameter("linear_speed", 1.0).value)
        self.angular_speed = float(self.declare_parameter("angular_speed", 1.5).value)
        self.compose_commands = bool(
            self.declare_parameter("compose_commands", True).value
        )
        self.timeout_sec = float(self.declare_parameter("timeout_sec", 1.0).value)
        self.transition_grace_sec = float(
            self.declare_parameter("transition_grace_sec", 4.0).value
        )
        self.publish_period = float(
            self.declare_parameter("publish_period", 0.1).value
        )

        self.publisher = self.create_publisher(Twist, self.output_topic, 10)
        self.subscription = self.create_subscription(
            SSVEPCommand, self.input_topic, self._on_command, 10
        )
        self.timer = self.create_timer(self.publish_period, self._publish_current)
        self._last_message_time = self.get_clock().now()
        self._last_valid_command_time = None
        self._motion = MotionCommandAccumulator(
            linear_speed=self.linear_speed,
            angular_speed=self.angular_speed,
            compose_commands=self.compose_commands,
        )
        self.get_logger().info(
            f"Listening on {self.input_topic}; publishing to {self.output_topic}; "
            f"compose_commands={self.compose_commands}; timeout={self.timeout_sec:.2f}s; "
            f"transition_grace={self.transition_grace_sec:.2f}s"
        )

    def _on_command(self, msg: SSVEPCommand) -> None:
        now = self.get_clock().now()
        self._last_message_time = now
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

    def _to_twist(self) -> Twist:
        components = self._motion.output_components()
        twist = Twist()
        twist.linear.x = components.linear_x
        twist.angular.z = components.angular_z
        return twist

    def _publish_current(self) -> None:
        now = self.get_clock().now()
        message_age = (now - self._last_message_time).nanoseconds / 1e9
        transition_age = None
        if self._last_valid_command_time is not None:
            transition_age = (
                now - self._last_valid_command_time
            ).nanoseconds / 1e9
        if message_age > self.timeout_sec or (
            transition_age is not None
            and transition_age > self.transition_grace_sec
        ):
            self._motion.clear()
            self._last_valid_command_time = None
        self.publisher.publish(self._to_twist())


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SsvepToCmdVel()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._motion.clear()
        node.publisher.publish(node._to_twist())
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
