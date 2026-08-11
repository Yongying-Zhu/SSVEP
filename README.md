# ssvep_eeg_fbcca(LR)_turtlesim

This repository contains a ROS 2 Humble workspace for an 8-channel BLE EEG
headband and SSVEP-based control of a turtlesim robot. The system receives EEG
samples over Bluetooth, publishes them through LSL and ROS 2, classifies SSVEP
commands, and converts the commands into safe `geometry_msgs/msg/Twist`
motion.

The current target is a controlled turtlesim demonstration. The generic
`cmd_vel` bridge is included for later robot integration, but this project is
not a certified medical, safety, or mobility controller.

> [!WARNING]
> Test with turtlesim or an unloaded robot first. Keep a human operator ready
> to stop the system. Do not connect this software to a powered robot as the
> only safety layer.

## System Architecture

```text
8-channel BLE EEG headband
          |
          v
     ble_to_lsl
          |
          v
     LSL stream: BCIPro / EEG
          |
          v
      lsl_to_ros2
          |
          +--> /eeg/frame       EEGFrame
          +--> /eeg/quality     SignalQuality
          |
          v
    ssvep_classifier
          |
          v
     /ssvep/command            SSVEPCommand
          |
          v
   ssvep_to_turtlesim
          |
          +--> /turtle1/cmd_vel
          +<-- /turtle1/pose
          |
          v
       turtlesim
```

The PsychoPy stimulus publishes `/ssvep/stimulus_active` as a safety gate and
embeds a live turtlesim pose mirror in the stimulation window.

## Tested Configuration

| Component | Configuration |
| --- | --- |
| EEG device | VIS_BCI_DFED857C BLE headband |
| EEG channels | 8 |
| Sampling rate | 250 Hz |
| Host OS | Ubuntu 22.04 |
| ROS 2 | Humble |
| Python | 3.10 |
| SSVEP window | 4 seconds |
| Classifier update period | 0.4 seconds |
| SSVEP targets | 8, 9, 10, 11, 12, 13 Hz |
| Motion command | `linear.x=1.0`, `angular.z=1.5` |

## Repository Layout

```text
eeg/
├── README.md
├── requirements.txt
├── start_eeg_turtlesim.sh
├── src/
│   ├── eeg_bci/
│   │   ├── config/
│   │   ├── eeg_bci/
│   │   │   ├── ble_to_lsl.py
│   │   │   ├── lsl_to_ros2.py
│   │   │   ├── ssvep_classifier.py
│   │   │   ├── ssvep_stimulus.py
│   │   │   ├── ssvep_to_turtlesim.py
│   │   │   └── models/
│   │   ├── launch/
│   │   └── test/
│   └── eeg_interfaces/
└── <command>_<trial>/      ROS 2 rosbag2 recordings
```

The repository intentionally excludes `build/`, `install/`, `log/`, runtime
logs, the Python virtual environment, editor metadata, backups, and generated
analysis reports. The rosbag recordings are included as the training and
validation dataset.

## SSVEP Commands

| Frequency | Command | Stimulus position |
| ---: | --- | --- |
| 8 Hz | `forward` | lower left |
| 9 Hz | `left` | upper right |
| 10 Hz | `right` | lower right |
| 11 Hz | `backward` | upper left |
| 12 Hz | `stop` | lower center |
| 13 Hz | `idle` | classifier-only class |

The visible stimulus layout is:

```text
backward   turtlesim   left
forward    stop        right
```

## Host Setup

Install ROS 2 Humble and the required system dependencies using the official
ROS 2 instructions. Then create a virtual environment that can access the ROS
2 system packages:

```bash
cd /path/to/eeg
/usr/bin/python3 -m venv --system-site-packages .venv
source .venv/bin/activate
python -m pip install -U pip
python -m pip install -r requirements.txt
```

The BLE/LSL stack also requires a Linux `liblsl.so` installation. A Windows
`liblsl64.dll` cannot be used directly on Ubuntu.

## Build

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

Run the package tests:

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 colcon test --packages-select eeg_bci
colcon test-result --verbose
```

## Run The Full Demo

Make sure the headband is powered on and is not connected by another process:

```bash
cd /path/to/eeg
chmod +x start_eeg_turtlesim.sh
./start_eeg_turtlesim.sh
```

The script starts:

1. BLE-to-LSL acquisition;
2. LSL-to-ROS 2 conversion;
3. the SSVEP classifier;
4. turtlesim and the motion bridge;
5. the PsychoPy stimulation window.

Press `Ctrl+C` in the launcher terminal to stop the complete process group.
Runtime logs are written to `logs/` locally and are not committed.

To start the backend without the stimulation window:

```bash
./start_eeg_turtlesim.sh --no-stimulus
```

Without the active stimulus gate, the turtlesim bridge publishes zero velocity.

## Run Components Separately

Start the BLE receiver:

```bash
source /opt/ros/humble/setup.bash
source .venv/bin/activate
ros2 run eeg_bci ble_to_lsl \
  --device-name VIS_BCI_DFED857C \
  --sample-rate 250
```

Start the LSL bridge and classifier in separate terminals:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run eeg_bci lsl_to_ros2
ros2 run eeg_bci ssvep_classifier \
  --ros-args --params-file src/eeg_bci/config/eeg.yaml
```

Start turtlesim and the safe motion bridge:

```bash
ros2 launch eeg_bci turtlesim_ssvep.launch.py
```

Useful topics:

```bash
ros2 topic echo /eeg/quality
ros2 topic echo /ssvep/command
ros2 topic echo /turtle1/pose
```

## Motion Behavior

The motion controller maintains independent linear and angular components.
`forward/backward` set the linear component and `left/right` set the angular
component. When both are active, the turtle follows a circular trajectory:

```text
R = |linear.x / angular.z| = 1.0 / 1.5 = 0.67
```

During a transition, an existing forward/backward component is retained in
state so that a later confirmed turn can form an arc. Uncertain classifier
results temporarily suppress output but do not destroy that linear state.
When an arc is active, a confirmed `stop` clears both components. Stimulus
shutdown, `idle`, wall protection, and command timeout remain hard-stop paths.

The turtlesim bridge also consumes `/turtle1/pose` for boundary detection and
automatic reset. It does not yet perform full pose-error trajectory tracking.

## Offline Rosbag Analysis

The included rosbag recordings can be analyzed without the headset:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run eeg_bci analyze_rosbags \
  --output-dir /tmp/eeg_analysis
```

The analyzer reports FBCCA scores, class predictions, valid-command rate, and
the trained logistic-regression model metrics. The trained model used by the
online classifier is packaged at:

```text
src/eeg_bci/eeg_bci/models/ssvep_classifier_model.joblib
```

## Model Configuration

The online classifier uses a six-command SSVEP configuration plus an `unknown`
model class. Its main settings are in:

```text
src/eeg_bci/config/eeg.yaml
```

The model path is package-relative by default and can be overridden with the
ROS parameter `model_path` when testing another trained artifact.

## Safety Notes

- Do not run multiple BLE receivers against the same headband.
- Do not connect more than one process to the same LSL stream if it changes
  the expected timing or sample ownership.
- Keep the stimulation window running while using the turtlesim bridge.
- Verify `/ssvep/command` before allowing motion.
- Test all motion changes in turtlesim before connecting a physical robot.
