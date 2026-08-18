# Rosbag data

| Directory | Recording condition | Purpose |
|---|---|---|
| `Initialize/` | Single-command and all-command trials recorded during initial calibration. | Estimate per-command FBCCA confidence thresholds and inspect command responses. |
| `switch_process/` | Original real switching trials: `forward -> backward -> left -> right -> stop`. | Measure the baseline raw, valid `/ssvep/command`, and matching `/turtle1/cmd_vel` delays. |
| `synthetic_switch_process/` | Bags built by concatenating single-command recordings. The switch point is stored in `/synthetic/switch_event`. | Reproduce exact transition timing and run controlled offline comparisons. These are synthetic analysis bags, not new physical recordings. |

Closed-eye, free-view, FFT, and other analysis outputs are kept under
`supplementary_resources/`. Recording logs are not stored in this directory.
