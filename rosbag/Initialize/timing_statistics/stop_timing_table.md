# stop timing table

筛选规则：`first FBCCA == expected command` 且 `第一次正确 FBCCA -> valid=true <= 0.8 s`。
均值和方差只使用 `normal=yes` 的行；方差为样本方差（分母 `n-1`）。

## Per-trial data

| Bag | Mode | First FBCCA | First FBCCA t (s) | Correct FBCCA t (s) | Valid t (s) | cmd_vel t (s) | Correct -> valid (s) | valid -> cmd_vel (s) | Correct -> cmd_vel (s) | t=0 -> cmd_vel (s) | Normal | Exclusion |
|---|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---|---|
| stop_03 | all | stop | 4.125470 | 4.125470 | 4.523170 | 4.581591 | 0.397701 | 0.058420 | 0.456121 | 4.581591 | yes | included |
| stop_04 | all | stop | 4.178874 | 4.178874 | 4.580550 | 4.623854 | 0.401677 | 0.043304 | 0.444981 | 4.623854 | yes | included |
| stop_01 | single | stop | 4.388863 | 4.388863 | 4.787923 | 4.858552 | 0.399060 | 0.070629 | 0.469688 | 4.858552 | yes | included |
| stop_02 | single | stop | 4.105664 | 4.105664 | 4.503432 | 4.595711 | 0.397768 | 0.092279 | 0.490048 | 4.595711 | yes | included |

## Normal-trial statistics

| Metric | n | Mean (s) | Sample variance (s^2) | Std dev (s) | Min (s) | Max (s) |
|---|---:|---:|---:|---:|---:|---:|
| 首帧 EEG -> cmd_vel 总时延 | 4 | 4.664927 | 0.016971 | 0.130273 | 4.581591 | 4.858552 |
| 第一次正确 FBCCA -> valid=true 确认时延 | 4 | 0.399051 | 0.000003 | 0.001859 | 0.397701 | 0.401677 |
| valid=true -> cmd_vel 发布时延 | 4 | 0.066158 | 0.000428 | 0.020692 | 0.043304 | 0.092279 |
| 第一次正确 FBCCA -> cmd_vel 控制时延 | 4 | 0.465209 | 0.000376 | 0.019398 | 0.444981 | 0.490048 |

Normal trials: 4/4.
