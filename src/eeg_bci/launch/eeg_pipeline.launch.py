from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    device_name = LaunchConfiguration("device_name")
    sample_rate = LaunchConfiguration("sample_rate")
    inlet_buffer_sec = LaunchConfiguration("inlet_buffer_sec")
    pull_chunk_size = LaunchConfiguration("pull_chunk_size")
    pull_timeout_sec = LaunchConfiguration("pull_timeout_sec")
    reconnect_delay_sec = LaunchConfiguration("reconnect_delay_sec")

    return LaunchDescription([
        DeclareLaunchArgument("device_name", default_value="VIS_BCI_DFED857C"),
        DeclareLaunchArgument("sample_rate", default_value="250"),
        DeclareLaunchArgument("inlet_buffer_sec", default_value="8.0"),
        DeclareLaunchArgument("pull_chunk_size", default_value="64"),
        DeclareLaunchArgument("pull_timeout_sec", default_value="0.2"),
        DeclareLaunchArgument("reconnect_delay_sec", default_value="1.0"),
        ExecuteProcess(
            cmd=[
                "ble_to_lsl",
                "--device-name", device_name,
                "--sample-rate", sample_rate,
            ],
            output="screen",
        ),
        Node(
            package="eeg_bci",
            executable="lsl_to_ros2",
            name="lsl_to_ros2",
            output="screen",
            parameters=[{
                "stream_type": "EEG",
                "expected_channels": 8,
                "accept_soft_label": True,
                "topic": "/eeg/frame",
                "quality_topic": "/eeg/quality",
                "inlet_buffer_sec": inlet_buffer_sec,
                "pull_chunk_size": pull_chunk_size,
                "pull_timeout_sec": pull_timeout_sec,
                "reconnect_delay_sec": reconnect_delay_sec,
            }],
        ),
    ])
