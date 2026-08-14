# SSVEP_EEG_FBCCA_(WCHAIR)

![Brain-controlled turtlesim result](turtlesim_trimmed.gif)

*Result of controlling turtlesim with EEG-based SSVEP commands.*

This repository implements an EEG-based human-computer interface for
controlling a ROS 2 `turtlesim` robot. An 8-channel BLE EEG headband sends data
to Lab Streaming Layer (LSL); the project bridges the stream into ROS 2,
decodes steady-state visual evoked potentials (SSVEP) with filter-bank
canonical correlation analysis (FBCCA), and publishes safe turtle motion.

The project is intended for reproducible research and simulation experiments.
It is not a medical device or a certified mobility controller.

> [!WARNING]
> Test with `turtlesim` or an unloaded robot first. Keep a human operator ready
> to stop the system. Do not connect this software to a powered robot as the
> only safety layer.

## 1. Baseline Performance

All results in this section were obtained with the original FBCCA framework
only. No additional rejection or trained-classifier refinement was enabled.
The two experimental refinements currently present in the code are disabled in
`src/eeg_bci/config/eeg.yaml`:

```yaml
use_score_margin: false
use_trained_model: false
```

Their experimental effects will be documented separately after independent
evaluation. The results below therefore describe the original online control
path and should not be interpreted as results from either refinement.

### Command-switch delay

`First EEG -> cmd_vel` is the elapsed time from the first EEG sample of a trial
to the first published turtle velocity command. `Correct FBCCA -> cmd_vel` is
the elapsed time from the first correct raw FBCCA candidate to that velocity
command. The variance columns are sample variances in seconds squared. Normal
trials are trials where the first FBCCA candidate was correct and the expected
command became valid within 0.8 seconds.

| Command | Normal trials | First EEG -> cmd_vel mean | Variance | Correct FBCCA -> cmd_vel mean | Variance |
|---|---:|---:|---:|---:|---:|
| forward | 6/10 | 4.731 s | 0.025132 | 0.456 s | 0.001274 |
| backward | 4/10 | 4.720 s | 0.054528 | 0.435 s | 0.000739 |
| left | 4/4 | 4.505 s | 0.023268 | 0.439 s | 0.000307 |
| right | 4/4 | 4.742 s | 0.049954 | 0.433 s | 0.000683 |
| stop | 4/4 | 4.665 s | 0.016971 | 0.465 s | 0.000376 |

Across these normal trials, the measured `First EEG -> cmd_vel` mean ranges
from 4.505 s to 4.742 s. The fastest mean is for `left`; the most conservative
mean is for `right`. Once a correct FBCCA candidate is already available, the
additional delay is only 0.433-0.465 s, which is close to one classifier update
period. The first measurement is dominated by filling the 4-second EEG window;
the second measurement removes that acquisition cost and mainly reflects the
0.4-second update schedule, confirmation, and ROS 2 publication timing.

The measured range follows directly from the configured timing:

```text
window duration W        = 4.0 s
classifier update Delta  = 0.4 s
required confirmations C = 2

First EEG -> cmd_vel
    T_first approximately W + k * Delta + epsilon, where k is 1 or 2
    lower timing bound approximately 4.0 + 1 * 0.4 = 4.4 s
    upper timing bound approximately 4.0 + 2 * 0.4 = 4.8 s

Correct FBCCA -> cmd_vel
    T_correct approximately k * Delta + epsilon
    one update gives approximately 0.4 s
    measured means: 0.433-0.465 s
```

Here `epsilon` includes timer phase, window alignment, message scheduling, and
the time needed for the command to pass through the motion bridge.

### Raw FBCCA input-command accuracy

These measurements are raw top-1 FBCCA predictions before confidence
thresholding, consecutive confirmation, or command publication. Each window is
4.0 seconds long and advances by 0.4 seconds. `Single` and `All` refer to the
two recording groups in the analysis; `Overall` pools both groups. A window is
correct when its top-1 raw prediction matches the expected command. Majority
trial accuracy counts a trial as correct when its majority raw prediction is
correct.

| Command | Single trials | Single windows | Single raw accuracy | All trials | All windows | All raw accuracy | Overall trials | Overall windows | Overall raw accuracy | Majority-trial accuracy |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| forward | 5 | 192 | 92.71% | 5 | 193 | 96.89% | 10 | 385 | 94.81% | 100.00% |
| backward | 5 | 193 | 63.73% | 5 | 192 | 64.58% | 10 | 385 | 64.16% | 90.00% |
| left | 2 | 76 | 97.37% | 2 | 76 | 100.00% | 4 | 152 | 98.68% | 100.00% |
| right | 2 | 78 | 100.00% | 2 | 77 | 97.40% | 4 | 155 | 98.71% | 100.00% |
| stop | 2 | 76 | 84.21% | 2 | 76 | 73.68% | 4 | 152 | 78.95% | 75.00% |

Overall raw window accuracy: **84.21%** (1035/1229 windows).

Overall majority-trial accuracy: **93.75%** (30/32 trials).

Raw FBCCA calculates a score for each reference frequency and always chooses
the largest score. It therefore has no intrinsic `unknown` output. The later
confidence and consecutive-confirmation rules decide whether that candidate is
published as a valid motion command.

`left` and `right` are the strongest classes in this dataset, with 98.68% and
98.71% overall raw window accuracy. `forward` is also strong at 94.81%.
`backward` and `stop` are more difficult, at 64.16% and 78.95% respectively.
The 100.00% values are real results for small subsets, not evidence of perfect
generalization: for example, `right` reaches 100.00% in the Single group but
97.40% in the All group, giving 98.71% overall. Likewise, a 100.00%
majority-trial score means all trials in that class were correct by majority;
it does not mean every window in every future trial will be correct.

### No-target conditions

The following recordings contain no intended visual target. Since plain FBCCA
must select one of its reference frequencies, every non-empty raw candidate is
a false candidate and every `valid=true` output is a false acceptance.

| Condition | Trials | FBCCA decisions | Raw rejection rate | Most common raw candidate | Raw candidate distribution | Mean confidence | Valid=true false accepts | False-acceptance rate | Trials with false accept |
|---|---:|---:|---:|---|---|---:|---:|---:|---:|
| close_eyes | 2 | 76 | 0.00% | backward | forward 11.84%, backward 51.32%, left 13.16%, right 19.74%, stop 3.95%, idle 0.00% | 0.266 | 49 | 64.47% | 2/2 |
| free_view | 2 | 76 | 0.00% | forward | forward 53.95%, backward 3.95%, left 23.68%, right 5.26%, stop 5.26%, idle 7.89% | 0.242 | 35 | 46.05% | 2/2 |

When the eyes are closed, alpha activity in the 8-13 Hz range can become more
prominent. In these recordings the dominant false candidate is close to 11 Hz,
so FBCCA most often labels the signal as `backward`. The turtle can therefore
move backward, turn, or stop even though no target is being viewed.

During free viewing, the subject is looking around the simulated environment
instead of maintaining attention on one target. Visual context and changing
attention can produce activity near several reference frequencies. The most
common false candidate here is `forward`, so the turtle may move forward or
occasionally turn left or stop.

The simulated environment used during free viewing is shown below:

![Turtlesim environment during free view](assets/free_view_turtlesim.png)

<table>
  <tr>
    <td align="center"><img src="turtlesim_dynamic.gif" alt="Turtlesim motion during free view" width="420"></td>
    <td align="center"><img src="turtlesim_trimmed_new.gif" alt="Turtlesim motion with closed eyes" width="420"></td>
  </tr>
  <tr>
    <td align="center"><em>Free-view condition</em></td>
    <td align="center"><em>Closed-eyes condition</em></td>
  </tr>
</table>

## 2. System and Code Architecture

### Runtime pipeline

```text
8-channel BLE EEG headband
          |
          v
     ble_to_lsl
          |
          v
      LSL stream: EEG
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

The PsychoPy stimulus window is part of the control loop. It displays the five
flickering command targets, subscribes to `/turtle1/pose`, and embeds a small
live turtle view in the top-center cell. It also publishes
`/ssvep/stimulus_active`. The motion bridge requires this safety gate and
publishes zero velocity when the stimulus is inactive or stale.

### Code architecture

```text
eeg/
├── README.md
├── requirements.txt
├── start_eeg_turtlesim.sh
├── src/
│   ├── eeg_bci/
│   │   ├── config/eeg.yaml
│   │   ├── eeg_bci/
│   │   │   ├── ble_receiver.py       BLE protocol and LSL output
│   │   │   ├── ble_to_lsl.py         BLE receiver entry point
│   │   │   ├── lsl_to_ros2.py        LSL to typed ROS 2 messages
│   │   │   ├── ssvep_classifier.py   online FBCCA classifier
│   │   │   ├── ssvep_stimulus.py     PsychoPy targets and pose mirror
│   │   │   ├── motion_controller.py  command-to-velocity state
│   │   │   ├── ssvep_to_turtlesim.py turtlesim bridge and safety gate
│   │   │   └── analyze_rosbags.py     offline rosbag analysis
│   │   ├── launch/                   ROS 2 launch files
│   │   └── test/                     protocol, bridge, and motion tests
│   └── eeg_interfaces/
│       └── msg/                      EEGFrame, SSVEPCommand, SignalQuality
└── <command>_<trial>/                rosbag2 recordings
```

The main message path is:

1. `ble_to_lsl` connects to the headband and publishes an LSL EEG stream.
2. `lsl_to_ros2` validates the eight-channel samples and publishes
   `EEGFrame` plus operational `SignalQuality` messages.
3. `ssvep_classifier` keeps a 4-second sample buffer, computes FBCCA scores,
   selects a raw candidate, applies confidence and consecutive-confirmation
   rules, and publishes `SSVEPCommand`.
4. `ssvep_to_turtlesim` converts confirmed commands to `Twist`, enforces the
   stimulus gate and command timeout, monitors the turtle pose, and resets the
   turtle near a wall.

### SSVEP

Steady-state visual evoked potential is the periodic neural response produced
when a person focuses on a visual stimulus flickering at a fixed frequency.
The response contains energy near the stimulus frequency and its harmonics.
The system uses five visible flicker targets for motion and reserves `idle` as
a classifier-side class.

### FBCCA

Filter-bank canonical correlation analysis compares the multichannel EEG
window with reference sine/cosine signals for each target frequency and its
harmonics. It evaluates several frequency bands, weights their correlation
scores, and selects the command with the largest combined score. The current
baseline uses:

```text
sampling rate       250 Hz
window              4.0 s
update period       0.4 s
harmonics           4
filter banks        3
confirmation        2 consecutive results
```

The raw FBCCA output is an argmax over the configured target classes. A low
confidence result is published as invalid and does not directly drive motion.

### SSVEP command mapping

| Flicker frequency | Command | Stimulus position | Motion effect |
|---:|---|---|---|
| 8 Hz | `forward` | lower left | `linear.x = +1.0` |
| 9 Hz | `left` | upper right | `angular.z = +1.5` |
| 10 Hz | `right` | lower right | `angular.z = -1.5` |
| 11 Hz | `backward` | upper left | `linear.x = -1.0` |
| 12 Hz | `stop` | lower center | clears motion |
| 13 Hz | `idle` | classifier-side class | clears motion |

The visible layout is:

```text
backward     turtlesim     left
forward      stop          right
```

The motion controller keeps linear and angular components independently. A
confirmed `forward` or `backward` sets linear velocity; `left` or `right` sets
angular velocity. With both components active, the turtle follows an arc. For
the default speeds, the nominal turning radius is:

```text
R = |v / omega| = |1.0 / 1.5| = 0.67 m
```

`stop`, `idle`, stale commands, an inactive stimulus gate, poor EEG quality,
and safety timeouts command zero velocity.

## 3. Hardware, Software, and Reproduction

### Hardware

- VIS_BCI_DFED857C 8-channel BLE EEG headband.
- A Linux computer with Bluetooth support.
- A display capable of running PsychoPy at a stable refresh rate.
- Keyboard access to the launcher terminal for immediate `Ctrl+C` shutdown.

The tested EEG configuration uses 250 Hz sampling. The headband must be powered
on and disconnected from other applications before starting this project.

### Software requirements

- Ubuntu 22.04.
- ROS 2 Humble.
- Python 3.10 with access to the ROS 2 system packages.
- A Linux `liblsl.so` installation. A Windows `liblsl64.dll` is not usable
  directly on Ubuntu.
- Python packages listed in `requirements.txt`.
- PsychoPy for the visual stimulus window.

### Install dependencies

Install ROS 2 Humble and its standard tools using the official ROS 2
installation instructions. Then prepare the workspace:

```bash
cd /path/to/eeg
/usr/bin/python3 -m venv --system-site-packages .venv
source .venv/bin/activate
python -m pip install -U pip
python -m pip install -r requirements.txt
python -m pip install psychopy
```

Install or expose the Linux Lab Streaming Layer library before importing
`pylsl`. Confirm that the library can be found by the dynamic linker if the
LSL bridge reports a shared-library error.

### Build the ROS 2 workspace

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

### Run the complete EEG demonstration

```bash
cd /path/to/eeg
chmod +x start_eeg_turtlesim.sh
./start_eeg_turtlesim.sh
```

The launcher starts the BLE receiver, the LSL-to-ROS 2 bridge, the baseline
FBCCA classifier, `turtlesim`, the motion bridge, and the PsychoPy stimulus
window. Focus on one visible flickering target at a time. Press `Ctrl+C` in the
launcher terminal to stop the complete process group.

To run the backend without the visual stimulus:

```bash
./start_eeg_turtlesim.sh --no-stimulus
```

With no active stimulus gate, the turtlesim bridge must publish zero velocity.
This mode is useful for checking startup and topics, but it cannot produce
intentional SSVEP commands.

### Run components separately

Start the BLE receiver:

```bash
source /opt/ros/humble/setup.bash
source .venv/bin/activate
ros2 run eeg_bci ble_to_lsl \
  --device-name VIS_BCI_DFED857C \
  --sample-rate 250
```

In separate terminals, after sourcing ROS 2 and the workspace:

```bash
ros2 run eeg_bci lsl_to_ros2
ros2 run eeg_bci ssvep_classifier \
  --ros-args --params-file src/eeg_bci/config/eeg.yaml
ros2 launch eeg_bci turtlesim_ssvep.launch.py
ros2 run eeg_bci ssvep_stimulus
```

Useful runtime checks:

```bash
ros2 topic echo /eeg/quality
ros2 topic echo /ssvep/command
ros2 topic echo /turtle1/pose
ros2 topic echo /turtle1/cmd_vel
```

The classifier publishes confidence, validity, rejection reason, raw FBCCA
candidate, and normalized scores in `SSVEPCommand`. These fields are useful for
checking whether an unexpected movement came from a raw candidate, low
confidence, incomplete confirmation, or a stale safety state.

### Analyze recorded experiments

The analysis node reads ROS 2 SQLite bags and produces window-level FBCCA
results and summary files. Run it against the workspace dataset into a separate
output directory:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run eeg_bci analyze_rosbags \
  --input-root /path/to/eeg \
  --output-dir /tmp/eeg_analysis \
  --no-train
```

The timing, accuracy, and no-target tables in this README were prepared from
the corresponding analysis artifacts:

```text
timing_statistics.md
raw_input_command_accuracy.md
unknown_condition_fbcca_impact.md
```

## Safety and limitations

- Do not run two BLE receivers against the same headband.
- Keep only one process connected to the EEG LSL stream during a measurement.
- Check `/ssvep/command` and `/turtle1/cmd_vel` before allowing motion.
- Keep the stimulus window active during normal control.
- The no-target results show that raw FBCCA can produce false candidates when
  the user closes their eyes or freely views the environment.
- The turtle bridge is an open-loop velocity controller with pose-based wall
  detection and reset; it is not full pose-error trajectory tracking.
- Always test in simulation before connecting any physical robot.

## License

This project is released under the MIT License.
