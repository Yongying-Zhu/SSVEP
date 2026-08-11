from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    eeg_launch = PathJoinSubstitution([
        FindPackageShare("eeg_bci"),
        "launch",
        "eeg_pipeline.launch.py",
    ])
    return LaunchDescription([
        IncludeLaunchDescription(PythonLaunchDescriptionSource(eeg_launch)),
        Node(
            package="eeg_bci",
            executable="ssvep_classifier",
            name="ssvep_classifier",
            output="screen",
            parameters=[PathJoinSubstitution([
                FindPackageShare("eeg_bci"),
                "config",
                "eeg.yaml",
            ])],
        ),
    ])
