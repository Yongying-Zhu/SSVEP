from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("input_topic", default_value="/ssvep/command"),
        DeclareLaunchArgument("output_topic", default_value="/cmd_vel"),
        DeclareLaunchArgument("linear_speed", default_value="1.0"),
        DeclareLaunchArgument("angular_speed", default_value="1.5"),
        DeclareLaunchArgument("timeout_sec", default_value="1.0"),
        DeclareLaunchArgument("transition_grace_sec", default_value="4.0"),
        Node(
            package="eeg_bci",
            executable="ssvep_to_cmd_vel",
            name="ssvep_to_cmd_vel",
            output="screen",
            parameters=[{
                "input_topic": LaunchConfiguration("input_topic"),
                "output_topic": LaunchConfiguration("output_topic"),
                "linear_speed": LaunchConfiguration("linear_speed"),
                "angular_speed": LaunchConfiguration("angular_speed"),
                "compose_commands": True,
                "timeout_sec": LaunchConfiguration("timeout_sec"),
                "transition_grace_sec": LaunchConfiguration("transition_grace_sec"),
            }],
        ),
    ])
