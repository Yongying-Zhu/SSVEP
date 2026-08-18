# Initialize

Initial single-command recordings and offline analysis results.

Each command uses one type directory and one numbered rosbag directory:

```text
Initialize/
├── forward/forward_01/
├── backward/backward_01/
├── left/left_01/
├── right/right_01/
└── stop/stop_01/
```

The same naming rule applies to every recording of that type. Each rosbag
directory contains its `metadata.yaml` and SQLite bag file, together with
the analysis files generated for that recording when available.

Closed-eye and free-view recordings, FFT outputs, and other supplementary
results are stored under `supplementary_resources/`. The `rosbag/` tree has no
recording log files; its remaining analysis summaries stay alongside the
command recordings that they describe.
