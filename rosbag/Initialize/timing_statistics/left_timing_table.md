# left timing table

筛选规则：`first FBCCA == expected command` 且 `第一次正确 FBCCA -> valid=true <= 0.8 s`。
均值和方差只使用 `normal=yes` 的行；方差为样本方差（分母 `n-1`）。

## Per-trial data

| Bag | Mode | First FBCCA | First FBCCA t (s) | Correct FBCCA t (s) | Valid t (s) | cmd_vel t (s) | Correct -> valid (s) | valid -> cmd_vel (s) | Correct -> cmd_vel (s) | t=0 -> cmd_vel (s) | Normal | Exclusion |
|---|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---|---|
| left_03 | all | left | 3.920516 | 3.920516 | 4.317021 | 4.375815 | 0.396505 | 0.058794 | 0.455299 | 4.375815 | yes | included |
| left_04 | all | left | 4.175464 | 4.175464 | 4.576421 | 4.589648 | 0.400958 | 0.013226 | 0.414184 | 4.589648 | yes | included |
| left_01 | single | left | 4.234755 | 4.234755 | 4.603743 | 4.677633 | 0.368988 | 0.073890 | 0.442878 | 4.677633 | yes | included |
| left_02 | single | left | 3.933953 | 3.933953 | 4.333195 | 4.377922 | 0.399243 | 0.044726 | 0.443969 | 4.377922 | yes | included |

## Normal-trial statistics

| Metric | n | Mean (s) | Sample variance (s^2) | Std dev (s) | Min (s) | Max (s) |
|---|---:|---:|---:|---:|---:|---:|
| 首帧 EEG -> cmd_vel 总时延 | 4 | 4.505254 | 0.023268 | 0.152539 | 4.375815 | 4.677633 |
| 第一次正确 FBCCA -> valid=true 确认时延 | 4 | 0.391423 | 0.000227 | 0.015069 | 0.368988 | 0.400958 |
| valid=true -> cmd_vel 发布时延 | 4 | 0.047659 | 0.000669 | 0.025860 | 0.013226 | 0.073890 |
| 第一次正确 FBCCA -> cmd_vel 控制时延 | 4 | 0.439082 | 0.000307 | 0.017523 | 0.414184 | 0.455299 |

Normal trials: 4/4.
