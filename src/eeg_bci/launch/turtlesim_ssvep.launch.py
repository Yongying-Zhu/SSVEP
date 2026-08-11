from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(package="turtlesim", executable="turtlesim_node", output="screen"),
        Node(
            package="eeg_bci",
            executable="ssvep_to_turtlesim",
            name="ssvep_to_turtlesim",
            output="screen",
            parameters=[{
                "input_topic": "/ssvep/command",
                "output_topic": "/turtle1/cmd_vel",
                "linear_speed": 1.0,
                "angular_speed": 1.5,
                "compose_commands": True,
                "preserve_linear_on_invalid": True,
                "defer_stop_while_linear": True,
                "stimulus_gate_topic": "/ssvep/stimulus_active",
                "stimulus_gate_timeout_sec": 1.0,
                "timeout_sec": 1.0,
                "transition_grace_sec": 16.0,
                "auto_reset_on_wall": True,
                "center_x": 5.5,
                "center_y": 5.5,
                "wall_margin": 0.35,
                "reset_cooldown_sec": 1.0,
            }],
        ),
    ])
