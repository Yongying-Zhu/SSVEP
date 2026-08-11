"""Composable motion state for SSVEP command consumers."""

from __future__ import annotations

from dataclasses import dataclass


@dataclass
class MotionComponents:
    """The currently latched linear and angular motion components."""

    linear_x: float = 0.0
    angular_z: float = 0.0


class MotionCommandAccumulator:
    """Map the existing four movement commands onto two velocity components."""

    TRANSIENT_REASONS = frozenset({"low_confidence", "awaiting_confirmation"})
    MOVEMENT_COMMANDS = frozenset({"forward", "backward", "left", "right"})

    def __init__(
        self,
        linear_speed: float = 1.0,
        angular_speed: float = 1.5,
        compose_commands: bool = True,
        preserve_linear_on_invalid: bool = False,
        defer_stop_while_linear: bool = False,
    ) -> None:
        self.linear_speed = float(linear_speed)
        self.angular_speed = float(angular_speed)
        self.compose_commands = bool(compose_commands)
        self.preserve_linear_on_invalid = bool(preserve_linear_on_invalid)
        self.defer_stop_while_linear = bool(defer_stop_while_linear)
        self.components = MotionComponents()
        # Keep the selected components during a classifier transition, but
        # suppress output until a new valid command is confirmed. This avoids
        # driving on uncertain EEG while preserving the user's previous
        # forward/turn selection for the next confirmed command.
        self.output_enabled = False

    def clear(self) -> None:
        self.components = MotionComponents()
        self.output_enabled = False

    def clear_linear(self) -> None:
        """Remove only the linear component while preserving a turn."""

        self.components.linear_x = 0.0

    def apply(self, command: str, valid: bool, reason: str = "") -> bool:
        """Apply one SSVEP command and return whether it was recognized."""

        if not valid:
            if reason in self.TRANSIENT_REASONS or (
                self.preserve_linear_on_invalid
                and self.components.linear_x != 0.0
            ):
                self.output_enabled = False
                return True
            self.clear()
            return True

        if command == "stop":
            if (
                self.defer_stop_while_linear
                and self.components.linear_x != 0.0
                and self.components.angular_z == 0.0
            ):
                # A stop-like classifier result during forward/backward ->
                # left/right switching must not destroy the latched line
                # speed before the turn can be confirmed.
                self.output_enabled = False
                return True
            self.clear()
            return True

        if command == "idle":
            self.clear()
            return True

        if command == "forward":
            self.components.linear_x = self.linear_speed
            if not self.compose_commands:
                self.components.angular_z = 0.0
        elif command == "backward":
            self.components.linear_x = -self.linear_speed
            if not self.compose_commands:
                self.components.angular_z = 0.0
        elif command == "left":
            self.components.angular_z = self.angular_speed
            if not self.compose_commands:
                self.components.linear_x = 0.0
        elif command == "right":
            self.components.angular_z = -self.angular_speed
            if not self.compose_commands:
                self.components.linear_x = 0.0
        else:
            self.clear()
            return False

        self.output_enabled = True
        return True

    def output_components(self) -> MotionComponents:
        """Return commanded components, or zero while a transition is unsure."""

        if not self.output_enabled:
            return MotionComponents()
        return MotionComponents(
            linear_x=self.components.linear_x,
            angular_z=self.components.angular_z,
        )
