from setuptools import find_packages, setup

package_name = "eeg_bci"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/launch", [
            "launch/eeg_pipeline.launch.py",
            "launch/ssvep_pipeline.launch.py",
            "launch/turtlesim_ssvep.launch.py",
            "launch/ssvep_cmd_vel.launch.py",
        ]),
        (f"share/{package_name}/config", ["config/eeg.yaml"]),
    ],
    install_requires=["setuptools"],
    package_data={"eeg_bci": ["models/*.joblib"]},
    zip_safe=True,
    description="BLE EEG and SSVEP ROS 2 nodes",
    license="MIT",
    scripts=[
        "scripts/ble_to_lsl",
        "scripts/ble_scan",
        "scripts/lsl_monitor",
        "scripts/lsl_to_ros2",
        "scripts/eeg_csv_replay",
        "scripts/ssvep_classifier",
        "scripts/ssvep_stimulus",
        "scripts/ssvep_to_turtlesim",
        "scripts/ssvep_to_cmd_vel",
        "scripts/analyze_rosbags",
        "scripts/analyze_eeg_fft",
        "scripts/analyze_command_timeline",
    ],
)
