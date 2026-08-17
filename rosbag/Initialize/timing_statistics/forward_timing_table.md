# forward timing table

筛选规则：`first FBCCA == expected command` 且 `第一次正确 FBCCA -> valid=true <= 0.8 s`。
均值和方差只使用 `normal=yes` 的行；方差为样本方差（分母 `n-1`）。

## Per-trial data

| Bag | Mode | First FBCCA | First FBCCA t (s) | Correct FBCCA t (s) | Valid t (s) | cmd_vel t (s) | Correct -> valid (s) | valid -> cmd_vel (s) | Correct -> cmd_vel (s) | t=0 -> cmd_vel (s) | Normal | Exclusion |
|---|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---|---|
| forward_06 | all | forward | 4.386344 | 4.386344 | 5.986275 | 6.036608 | 1.599931 | 0.050334 | 1.650265 | 6.036608 | no | confirmation took 1.600s |
| forward_07 | all | forward | 4.353361 | 4.353361 | 4.755589 | 4.838752 | 0.402227 | 0.083164 | 0.485391 | 4.838752 | yes | included |
| forward_08 | all | forward | 4.474894 | 4.474894 | 4.875614 | 4.969702 | 0.400721 | 0.094088 | 0.494808 | 4.969702 | yes | included |
| forward_09 | all | forward | 4.295066 | 4.295066 | 4.700228 | 4.768176 | 0.405162 | 0.067948 | 0.473110 | 4.768176 | yes | included |
| forward_10 | all | forward | 4.120991 | 4.120991 | 4.518607 | 4.579409 | 0.397616 | 0.060802 | 0.458418 | 4.579409 | yes | included |
| forward_01 | single | right | 4.483045 | 4.868101 | 8.059476 | 8.104759 | 3.191375 | 0.045283 | 3.236658 | 8.104759 | no | first FBCCA was right |
| forward_02 | single | forward | 4.140857 | 4.140857 | 4.537525 | 4.560086 | 0.396669 | 0.022561 | 0.419230 | 4.560086 | yes | included |
| forward_03 | single | forward | 4.378867 | 4.378867 | 5.977039 | 6.036507 | 1.598172 | 0.059468 | 1.657640 | 6.036507 | no | confirmation took 1.598s |
| forward_04 | single | forward | 4.264370 | 4.264370 | 4.656074 | 4.671864 | 0.391705 | 0.015790 | 0.407494 | 4.671864 | yes | included |
| forward_05 | single | left | 4.051458 | 4.884963 | 7.652183 | 7.695935 | 2.767219 | 0.043753 | 2.810972 | 7.695935 | no | first FBCCA was left |

## Normal-trial statistics

| Metric | n | Mean (s) | Sample variance (s^2) | Std dev (s) | Min (s) | Max (s) |
|---|---:|---:|---:|---:|---:|---:|
| 首帧 EEG -> cmd_vel 总时延 | 6 | 4.731332 | 0.025132 | 0.158530 | 4.560086 | 4.969702 |
| 第一次正确 FBCCA -> valid=true 确认时延 | 6 | 0.399016 | 0.000022 | 0.004731 | 0.391705 | 0.405162 |
| valid=true -> cmd_vel 发布时延 | 6 | 0.057392 | 0.001016 | 0.031868 | 0.015790 | 0.094088 |
| 第一次正确 FBCCA -> cmd_vel 控制时延 | 6 | 0.456409 | 0.001274 | 0.035700 | 0.407494 | 0.494808 |

Normal trials: 6/10.
