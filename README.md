# SSVEP_EEG_FBCCA_(WCHAIR)

This project uses EEG signals to control a ROS 2 `turtlesim` robot through
SSVEP and filter-bank canonical correlation analysis (FBCCA).

<p align="center">
  <img src="supplementary_resources/turtlesim_trimmed.gif" width="500" alt="Turtlesim controlled by EEG">
</p>

<p align="center"><em>Result of controlling the turtlesim robot with EEG signals.</em></p>

> [!WARNING]
> This project is a research and simulation prototype. Test with turtlesim or
> an unloaded robot first. Keep a human operator ready to stop the system. Do
> not connect it to a powered wheelchair or mobile robot as the only safety
> layer.

## 1. Baseline Control Performance

All performance results in this section use the original FBCCA framework. No
additional classifier or rejection module is active for these measurements.
Two experimental enhancements exist in the code, but both are disabled in the
current `eeg.yaml` configuration:

```yaml
use_score_margin: false
use_trained_model: false
```

The results below therefore describe the original FBCCA control path.

### 1.1 Switching Latency

For an already running classifier, the fastest transition is approximately
`0.4-0.9 s` when the mixed sliding window becomes discriminative immediately.
The conservative case is approximately `4.4-4.9 s` when the old command must
leave the 4-second window before the new frequency becomes reliable.

In the table:

- `First EEG -> cmd_vel` starts at the first recorded `/eeg/frame` and includes
  filling the initial 4-second EEG window.
- `Correct FBCCA -> cmd_vel` starts at the first correct FBCCA candidate and
  measures confirmation plus the command bridge publication delay.
- Variance is the sample variance with denominator `n - 1`.

| Command | Normal trials | First EEG -> cmd_vel mean | Variance | Correct FBCCA -> cmd_vel mean | Variance |
|---|---:|---:|---:|---:|---:|
| forward | 6/10 | 4.731 s | 0.025132 | 0.456 s | 0.001274 |
| backward | 4/10 | 4.720 s | 0.054528 | 0.435 s | 0.000739 |
| left | 4/4 | 4.505 s | 0.023268 | 0.439 s | 0.000307 |
| right | 4/4 | 4.742 s | 0.049954 | 0.433 s | 0.000683 |
| stop | 4/4 | 4.665 s | 0.016971 | 0.465 s | 0.000376 |

The first latency mean is around 4.5-4.8 seconds because it contains the
4-second window, the 0-0.4-second classifier timer phase, one 0.4-second
confirmation interval, and up to 0.1 seconds for the `cmd_vel` publisher.
The second latency mean is around 0.43-0.46 seconds because it starts after
FBCCA has already produced a candidate and mainly contains the next
confirmation cycle and command publication.

The timing calculation is:

```text
First EEG -> cmd_vel
= 4.0 s window
  + 0-0.4 s wait for the next classification timer
  + 0.4 s second consistent result
  + 0-0.1 s cmd_vel publication
= approximately 4.4-4.9 s

Correct FBCCA -> cmd_vel
= 0.4 s confirmation
  + 0-0.1 s cmd_vel publication
= approximately 0.4-0.5 s
```

Repeated confidence rejection, changing candidates, poor signal quality, or a
failed confirmation can extend the transition time beyond these ideal ranges.

### 1.2 Example of a Transition Longer Than 4.8 Seconds

The following is an actual `single_backward_01` trial selected from the
recorded timing tables. The expected command was `backward`, but the first
candidate was `right`. Times are relative to the first recorded
`/eeg/frame`; the interval column is measured from the preceding event.

| Processing event | Time from first EEG | Interval | Observed result | Interpretation |
|---|---:|---:|---|---|
| First `/eeg/frame` | 0.000 s | - | `sequence=0`, 250 Hz | Start of measurement |
| 4-second window ready | 4.066 s | 4.066 s | 1000 samples cached | Initial window is available |
| First FBCCA candidate | 4.140 s | 0.073 s | `right`, `valid=false`, `low_confidence`, 0.236 | Wrong candidate rejected |
| Wrong command accepted | 4.545 s | 0.405 s | `right`, `valid=true`, 0.244 | An incorrect command was temporarily published |
| Expected candidate appears | 4.935 s | 0.390 s | `backward`, `valid=false`, `low_confidence`, 0.209 | Correct candidate is still below threshold |
| Repeated backward candidate | 5.341 s | 0.407 s | `backward`, `valid=false`, `low_confidence`, 0.219 | Confidence still insufficient |
| Candidate changes | 5.736 s | 0.394 s | `right`, `valid=false`, `low_confidence`, 0.221 | Candidate instability interrupts confirmation |
| Candidate changes | 6.134 s | 0.398 s | `left`, `valid=false`, `awaiting_confirmation`, 0.271 | New candidate starts a new streak |
| Candidate rejected | 6.541 s | 0.407 s | `left`, `valid=false`, `low_confidence`, 0.222 | Confidence still insufficient |
| Backward candidate returns | 6.935 s | 0.394 s | `backward`, `valid=false`, `low_confidence`, 0.220 | Correct candidate returns but is rejected |
| Repeated backward candidate | 7.335 s | 0.401 s | `backward`, `valid=false`, `low_confidence`, 0.217 | One more classification cycle is needed |
| Valid backward output | 7.736 s | 0.400 s | `backward`, `valid=true`, 0.255 | Confidence and confirmation pass |
| `/turtle1/cmd_vel` | 7.743 s | 0.007 s | `linear.x=-1.000` | Motion command is published |

This trial took **7.743 s from the first EEG frame to the matching
`cmd_vel`**, which is above the 4.8-second conservative estimate. The delay is
not caused by the `cmd_vel` bridge: the valid output to `cmd_vel` interval was
only `0.007 s`. The delay accumulated earlier in the classifier:

```text
first EEG -> 4-second window ready       = 4.066 s
window ready -> first FBCCA candidate   = 0.073 s
first candidate -> valid backward       = 3.596 s
valid backward -> cmd_vel                = 0.007 s
total first EEG -> cmd_vel               = 7.743 s
```

The observed sequence shows three mechanisms that extend the response time:

1. The first candidate was `right`, so it did not represent the expected
   `backward` command.
2. The expected `backward` candidate appeared several times but its confidence
   stayed below the configured threshold, producing `valid=false` with
   `low_confidence`.
3. The candidate changed to `right` and `left` during the transition. A change
   of candidate starts a new consecutive-confirmation streak, so the system
   cannot immediately accept the next `backward` result.

In the classifier implementation, a low-confidence result returns before
publishing `valid=true`, while a candidate change resets the streak to one
before the two-result confirmation requirement is checked. Therefore, the
`0.4 s` update period is the cost of each additional rejected or unstable
classification cycle. The `4.8 s` figure is a useful conservative estimate,
not a strict maximum; this measured trial demonstrates why real transitions
can exceed it.

### 1.3 Personalized Confidence Thresholds

The current confidence is a normalized FBCCA score, not a calibrated
probability:

```text
confidence(c) = max(score(c), 0) / sum(max(score(k), 0))
```

For each window, FBCCA first selects the command with the largest score. The
classifier then looks up the threshold for that selected command. A command is
eligible only when its confidence reaches that command's threshold and the
same candidate is confirmed twice consecutively. The per-command values are
configured in [`eeg.yaml`](src/eeg_bci/config/eeg.yaml); the lookup and
confidence calculation are implemented in
[`ssvep_classifier.py`](src/eeg_bci/eeg_bci/ssvep_classifier.py).

The thresholds were set by retrospective calibration on the recorded EEG
windows, using this procedure:

1. Separate windows by the command selected as the raw FBCCA top candidate.
2. Compare the confidence distribution of correct target windows with the
   confidence distribution from wrong-target and no-target recordings.
3. Choose a threshold low enough to preserve usable correct candidates, but
   high enough to reject weak candidates. Verify the result with final
   `valid=true` output and motion timing.

This is a deliberately conservative operating-point choice, not a learned
classifier parameter. The following audit uses the available offline window
analysis. `Correct p10` is the 10th percentile of confidence among windows
whose raw candidate equals the expected command. `Correct pass` is the share
of those correct windows above the configured threshold. `Unknown pass` is the
share of no-target windows that selected that candidate and also exceeded the
threshold. It is a candidate-level diagnostic; it does not include the final
two-result confirmation.

| Candidate | Threshold | Correct n | Correct p10 | Correct pass | Unknown n | Unknown pass |
|---|---:|---:|---:|---:|---:|---:|
| `forward` | 0.23 | 107 | 0.227 | 87.9% | 459 | 61.4% |
| `backward` | 0.22 | 132 | 0.247 | 97.0% | 258 | 77.9% |
| `left` | 0.24 | 118 | 0.222 | 78.0% | 275 | 34.5% |
| `right` | 0.25 | 106 | 0.226 | 75.5% | 126 | 32.5% |
| `stop` | 0.20 | 116 | 0.237 | 100.0% | 131 | 90.1% |
| `idle` | 0.24 | 93 | 0.221 | 81.7% | 55 | 21.8% |

The table explains why the thresholds are intentionally different. `right`
and `left` use higher thresholds because their non-target candidate scores
overlap less with their correct scores, so some weak candidates can be
rejected without losing all correct windows. `backward` uses a lower value to
avoid suppressing its relatively weak but valid responses. `stop` uses the
lowest value to preserve the safety stop action, accepting that confidence
alone is not a reliable unknown detector. The high `Unknown pass` values for
`forward`, `backward`, and `stop` are evidence that confidence must be combined
with consecutive confirmation and, for a stronger rejection design, an
additional no-target criterion.

In the current baseline, `use_trained_model: false` and
`use_score_margin: false`, so these per-command confidence thresholds are the
active threshold mechanism. `model_reject_probability_*` and
`score_margin_threshold` do not affect this baseline path.

### 1.4 Raw FBCCA Command Accuracy

The following table measures the raw FBCCA top-1 result before confidence
thresholding, consecutive confirmation, or command publication:

- `Raw accuracy` is the percentage of 4-second windows whose highest FBCCA
  frequency matches the expected command.
- `Single` and `All` identify the stimulation recording mode.
- `Majority-trial accuracy` counts a trial as correct when its majority of raw
  windows predicts the expected command.

| Command | Single trials | Single windows | Single raw accuracy | All trials | All windows | All raw accuracy | Overall trials | Overall windows | Overall raw accuracy | Majority-trial accuracy |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| forward | 5 | 192 | 92.71% | 5 | 193 | 96.89% | 10 | 385 | 94.81% | 100.00% |
| backward | 5 | 193 | 63.73% | 5 | 192 | 64.58% | 10 | 385 | 64.16% | 90.00% |
| left | 2 | 76 | 97.37% | 2 | 76 | 100.00% | 4 | 152 | 98.68% | 100.00% |
| right | 2 | 78 | 100.00% | 2 | 77 | 97.40% | 4 | 155 | 98.71% | 100.00% |
| stop | 2 | 76 | 84.21% | 2 | 76 | 73.68% | 4 | 152 | 78.95% | 75.00% |

Overall raw window accuracy is **84.21%** (`1035/1229` windows). Overall
majority-trial accuracy is **93.75%** (`30/32` trials).

The two 100% values are trial-level majority results, not perfect window-level
classification. For example, all forward trials had forward as the majority
prediction even though the forward window accuracy was 94.81%. A few wrong
windows can therefore coexist with 100% majority-trial accuracy.

### 1.5 Closed Eyes and Free View Without a Target

These recordings have no expected visual target. Plain FBCCA must still choose
one of its reference frequencies, so it has no native unknown or rejection
class at the raw-score stage.

| Condition | Trials | FBCCA decisions | Raw rejection rate | Most common raw candidate | Raw candidate distribution | Mean confidence | Valid=true false accepts | False-acceptance rate | Trials with false accept |
|---|---:|---:|---:|---|---|---:|---:|---:|---:|
| close_eyes | 2 | 76 | 0.00% | backward | forward 11.84%, backward 51.32%, left 13.16%, right 19.74%, stop 3.95%, idle 0.00% | 0.266 | 49 | 64.47% | 2/2 |
| free_view | 2 | 76 | 0.00% | forward | forward 53.95%, backward 3.95%, left 23.68%, right 5.26%, stop 5.26%, idle 7.89% | 0.242 | 35 | 46.05% | 2/2 |

With eyes closed, increased alpha-band activity, commonly concentrated around
8-13 Hz, can overlap the 11 Hz backward reference. That explains the strong
backward bias in the closed-eyes recordings and can cause backward-like false
commands.

During free view, attention is not locked to one target. The stimulation
screen, the embedded turtlesim view, and other visual elements can shift gaze
and visual attention, producing unstable or incidental frequency evidence.
The following screenshot shows the visual environment used during free view:

<p align="center">
  <img src="supplementary_resources/free_view_turtlesim.png" width="900" alt="Turtlesim and SSVEP stimulation environment">
</p>

The recorded effects are illustrated below:

<table>
  <tr>
    <td align="center"><strong>Free view</strong><br><img src="supplementary_resources/turtlesim_dynamic.gif" width="420" alt="Turtlesim during free view"></td>
    <td align="center"><strong>Closed eyes</strong><br><img src="supplementary_resources/turtlesim_trimmed_new.gif" width="420" alt="Turtlesim during closed eyes"></td>
  </tr>
</table>

In both conditions, raw FBCCA produces a candidate for every decision. The
`valid=true` counts show why a later rejection strategy is important before
using the output to drive a real robot.

## 2. System and Algorithm Architecture

### 2.1 Runtime Data Flow

```mermaid
flowchart LR
    A[8-channel BLE EEG headband<br/>250 Hz] --> B[ble_to_lsl.py]
    B --> C[LSL stream<br/>BCIPro / EEG]
    C --> D[lsl_to_ros2.py]
    D --> E[/eeg/frame<br/>EEGFrame]
    D --> F[/eeg/quality<br/>SignalQuality]
    E --> G[ssvep_classifier.py]
    F --> G
    G --> H[4 s sliding window<br/>1000 samples]
    H --> I[3 filter banks]
    I --> J[CCA with harmonic references]
    J --> K[Six FBCCA scores]
    K --> L[Confidence and<br/>two-result confirmation]
    L --> M[/ssvep/command<br/>SSVEPCommand]
    M --> N[ssvep_to_turtlesim.py]
    N --> O[MotionCommandAccumulator]
    O --> P[/turtle1/cmd_vel<br/>Twist]
    P --> Q[turtlesim]
    Q --> R[/turtle1/pose]
    R --> N
    R --> S[PsychoPy stimulus<br/>embedded turtle feedback]
```

### 2.2 Code Layout

```text
eeg/
├── README.md
├── requirements.txt
├── start_eeg_turtlesim.sh
├── supplementary_resources/
│   ├── free_view_turtlesim.png
│   ├── turtlesim_dynamic.gif
│   ├── turtlesim_trimmed.gif
│   └── turtlesim_trimmed_new.gif
├── eeg_robag/
│   └── <command_trial>/
│       ├── <trial>_0.db3
│       └── metadata.yaml
└── src/
    ├── eeg_interfaces/
    │   └── msg/
    │       ├── EEGFrame.msg
    │       ├── SignalQuality.msg
    │       └── SSVEPCommand.msg
    └── eeg_bci/
        ├── config/eeg.yaml
        ├── launch/
        ├── scripts/
        └── eeg_bci/
            ├── ble_to_lsl.py
            ├── lsl_to_ros2.py
            ├── ssvep_classifier.py
            ├── ssvep_stimulus.py
            ├── ssvep_to_turtlesim.py
            ├── ssvep_to_cmd_vel.py
            └── motion_controller.py
```

### 2.3 SSVEP

Steady-state visual evoked potential (SSVEP) is the periodic EEG response
elicited when a person looks at a flickering visual stimulus. If a target
flickers at frequency `f`, the EEG can contain energy at `f` and its harmonics.
The classifier uses the frequency with the strongest agreement to infer the
intended command.

### 2.4 FBCCA

Filter-bank canonical correlation analysis (FBCCA) processes each 4-second
window as follows:

1. Apply three filter-bank bands to the eight-channel EEG.
2. Build sine/cosine reference signals for each target frequency and four
   harmonics.
3. Use CCA to measure the correlation between filtered EEG and each reference.
4. Combine the weighted correlation scores across filter banks.
5. Select the highest raw FBCCA score and compute normalized confidence.
6. Require the confidence threshold and two consistent results before
   publishing `valid=true`.

The currently reported baseline has `use_score_margin: false` and
`use_trained_model: false`, so the active decision path is raw FBCCA plus the
configured confidence thresholds and consecutive confirmation.

### 2.5 SSVEP Command Mapping

| Frequency | Command | Stimulus position |
|---:|---|---|
| 8 Hz | `forward` | bottom-left |
| 9 Hz | `left` | top-right |
| 10 Hz | `right` | bottom-right |
| 11 Hz | `backward` | top-left |
| 12 Hz | `stop` | bottom-center |
| 13 Hz | `idle` | classifier-only target in the current stimulus |

The visible layout is:

```text
backward    turtlesim    left
forward     stop         right
```

`forward` and `backward` control `linear.x`. `left` and `right` control
`angular.z`. With both components active, the turtlesim trajectory is an arc
or circle with radius:

```text
R = |linear.x / angular.z| = 1.0 / 1.5 = 0.67
```

The turtlesim bridge also uses the stimulus gate, command timeout, wall reset,
and pose feedback as safety mechanisms. It is a demonstrator, not a complete
closed-loop trajectory tracker.

## 3. Hardware and Reproduction

### 3.1 Hardware and Software Requirements

- Ubuntu 22.04
- ROS 2 Humble
- Python 3.10
- An eight-channel BLE EEG headband
- The tested device name: `VIS_BCI_DFED857C`
- Bluetooth support on the host computer
- A display capable of presenting the 8-13 Hz PsychoPy stimuli
- A Linux `liblsl.so` installation for the LSL bridge
- `turtlesim` and a working ROS 2 graphical session

The project is intended for a simulation or unloaded-robot setup. A real
wheelchair requires an independent hardware emergency stop and an additional
validated safety controller.

### 3.2 Install Dependencies

```bash
git clone https://github.com/Yongying-Zhu/SSVEP_EEG_FBCCA_WCHAIR.git
cd SSVEP_EEG_FBCCA_WCHAIR

source /opt/ros/humble/setup.bash
/usr/bin/python3 -m venv --system-site-packages .venv
source .venv/bin/activate
python -m pip install --upgrade pip
python -m pip install -r requirements.txt
```

Install PsychoPy separately if it is not already available in the virtual
environment. Ensure that the Linux LSL runtime is installed before starting
the BLE pipeline.

### 3.3 Build the ROS 2 Workspace

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

Optional tests:

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 colcon test --packages-select eeg_bci
colcon test-result --verbose
```

### 3.4 Start the Complete Demonstration

Power on the EEG headband and make sure no other process is connected to it.
Then run:

```bash
source /opt/ros/humble/setup.bash
source .venv/bin/activate
source install/setup.bash
./start_eeg_turtlesim.sh
```

The launcher starts the BLE receiver, LSL bridge, FBCCA classifier, turtlesim,
motion bridge, and PsychoPy stimulus. The classifier parameters are loaded
from `src/eeg_bci/config/eeg.yaml`; the current baseline explicitly disables
the trained-model path.

To run the backend without the visual stimulus:

```bash
./start_eeg_turtlesim.sh --no-stimulus
```

The turtlesim bridge should publish zero velocity when the stimulus gate is
inactive. Press `Ctrl+C` in the launcher terminal to stop the process group.
Runtime logs are written to `logs/` and are not part of the dataset.

Useful monitoring commands:

```bash
ros2 topic echo /eeg/quality
ros2 topic echo /ssvep/command
ros2 topic echo /turtle1/cmd_vel
ros2 topic echo /turtle1/pose
```

### 3.5 Record a Trial

Record the data and control topics needed for later analysis:

```bash
ros2 bag record -o experiments/forward_01 \
  /eeg/frame \
  /eeg/quality \
  /ssvep/command \
  /turtle1/cmd_vel \
  /turtle1/pose \
  /ssvep/stimulus_active
```

The repository dataset is stored under `eeg_robag/`. Each rosbag directory
contains its SQLite database and `metadata.yaml`.

### 3.6 Replay and Analyze a Rosbag

Replay only EEG frames into a running classifier:

```bash
ros2 bag play eeg_robag/forward_01 \
  --topics /eeg/frame \
  --rate 1.0
```

For a single bag timeline report:

```bash
ros2 run eeg_bci analyze_command_timeline \
  --bag experiments/forward/single_forward_02
```

For the offline FBCCA analyzer, `--input-root` should contain rosbag
directories directly below the selected root:

```bash
ros2 run eeg_bci analyze_rosbags \
  --input-root /path/to/flat_rosbags \
  --output-dir /tmp/eeg_analysis \
  --no-train
```

The analyzer writes per-window CSV files and aggregate summaries in the
selected output directory.

## Safety and Limitations

- Raw FBCCA always selects one of its reference frequencies, including when
  the user is not looking at a target.
- Closed eyes and free view can therefore produce false command candidates.
- Confidence thresholds and consecutive confirmation reduce accidental output
  but do not provide a formal safety guarantee.
- The current feedback loop is a turtlesim demonstration with a pose mirror
  and wall reset. It is not a validated wheelchair control system.
- Keep the screen, EEG device, ROS graph, and motion bridge under direct human
  supervision during experiments.
