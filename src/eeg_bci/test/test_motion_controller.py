from eeg_bci.motion_controller import MotionCommandAccumulator


def test_forward_then_left_composes_a_circle_command():
    motion = MotionCommandAccumulator(linear_speed=1.0, angular_speed=1.5)

    assert motion.apply("forward", True)
    assert motion.components.linear_x == 1.0
    assert motion.components.angular_z == 0.0

    assert motion.apply("left", True)
    assert motion.components.linear_x == 1.0
    assert motion.components.angular_z == 1.5


def test_motion_starts_disabled_until_a_valid_command_arrives():
    motion = MotionCommandAccumulator(linear_speed=1.0, angular_speed=1.5)

    output = motion.output_components()

    assert output.linear_x == 0.0
    assert output.angular_z == 0.0


def test_forward_then_right_composes_clockwise_circle_command():
    motion = MotionCommandAccumulator(linear_speed=1.0, angular_speed=1.5)

    motion.apply("forward", True)
    motion.apply("right", True)

    assert motion.components.linear_x == 1.0
    assert motion.components.angular_z == -1.5

    motion.apply("forward", False, "model_unknown")
    output = motion.output_components()
    assert output.linear_x == 0.0
    assert output.angular_z == 0.0


def test_clear_linear_preserves_the_turn_for_a_latched_arc():
    motion = MotionCommandAccumulator(linear_speed=1.0, angular_speed=1.5)

    motion.apply("forward", True)
    motion.apply("left", True)
    motion.clear_linear()

    output = motion.output_components()
    assert output.linear_x == 0.0
    assert output.angular_z == 1.5


def test_turtlesim_preserves_linear_state_during_invalid_transition():
    motion = MotionCommandAccumulator(
        linear_speed=1.0,
        angular_speed=1.5,
        preserve_linear_on_invalid=True,
    )

    motion.apply("forward", True)
    motion.apply("stop", False, "model_unknown")
    assert motion.components.linear_x == 1.0
    assert motion.output_components().linear_x == 0.0

    motion.apply("left", True)
    output = motion.output_components()
    assert output.linear_x == 1.0
    assert output.angular_z == 1.5

    motion.apply("stop", True)
    assert motion.components.linear_x == 0.0
    assert motion.components.angular_z == 0.0


def test_turtlesim_defers_stop_until_a_turn_is_active():
    motion = MotionCommandAccumulator(
        linear_speed=1.0,
        angular_speed=1.5,
        preserve_linear_on_invalid=True,
        defer_stop_while_linear=True,
    )

    motion.apply("forward", True)
    motion.apply("stop", True)
    assert motion.components.linear_x == 1.0
    assert motion.components.angular_z == 0.0
    assert motion.output_components().linear_x == 0.0

    motion.apply("left", True)
    assert motion.output_components().linear_x == 1.0
    assert motion.output_components().angular_z == 1.5

    motion.apply("stop", True)
    assert motion.components.linear_x == 0.0
    assert motion.components.angular_z == 0.0


def test_stop_invalid_and_idle_clear_both_components():
    motion = MotionCommandAccumulator()
    motion.apply("forward", True)
    motion.apply("right", True)

    motion.apply("stop", True)
    assert motion.components.linear_x == 0.0
    assert motion.components.angular_z == 0.0

    motion.apply("backward", True)
    motion.apply("left", True)
    motion.apply("idle", True)
    assert motion.components.linear_x == 0.0
    assert motion.components.angular_z == 0.0

    motion.apply("forward", True)
    motion.apply("right", False, "poor_signal_quality")
    assert motion.components.linear_x == 0.0
    assert motion.components.angular_z == 0.0


def test_non_composed_mode_preserves_previous_behavior():
    motion = MotionCommandAccumulator(compose_commands=False)
    motion.apply("forward", True)
    motion.apply("left", True)

    assert motion.components.linear_x == 0.0
    assert motion.components.angular_z == 1.5


def test_transition_keeps_selection_but_suppresses_uncertain_output():
    motion = MotionCommandAccumulator()
    motion.apply("forward", True)

    motion.apply("stop", False, "low_confidence")
    assert motion.components.linear_x == 1.0
    assert motion.components.angular_z == 0.0
    assert motion.output_components().linear_x == 0.0
    assert motion.output_components().angular_z == 0.0

    motion.apply("left", True)
    output = motion.output_components()
    assert output.linear_x == 1.0
    assert output.angular_z == 1.5


def test_classifier_error_clears_selection_immediately():
    motion = MotionCommandAccumulator()
    motion.apply("forward", True)
    motion.apply("stop", False, "classifier_error")

    assert motion.components.linear_x == 0.0
    assert motion.components.angular_z == 0.0
    assert motion.output_components().linear_x == 0.0
