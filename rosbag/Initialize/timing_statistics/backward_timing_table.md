# backward timing table

筛选规则：`first FBCCA == expected command` 且 `第一次正确 FBCCA -> valid=true <= 0.8 s`。
均值和方差只使用 `normal=yes` 的行；方差为样本方差（分母 `n-1`）。

## Per-trial data

| Bag | Mode | First FBCCA | First FBCCA t (s) | Correct FBCCA t (s) | Valid t (s) | cmd_vel t (s) | Correct -> valid (s) | valid -> cmd_vel (s) | Correct -> cmd_vel (s) | t=0 -> cmd_vel (s) | Normal | Exclusion |
|---|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---|---|
| backward_06 | all | forward | 4.211600 | 8.207735 | 8.607196 | 8.637266 | 0.399462 | 0.030070 | 0.429532 | 8.637266 | no | first FBCCA was forward |
| backward_07 | all | backward | 4.430883 | 4.430883 | 4.826613 | 4.834596 | 0.395730 | 0.007984 | 0.403714 | 4.834596 | yes | included |
| backward_08 | all | backward | 4.075969 | 4.075969 | 5.669428 | 5.759430 | 1.593459 | 0.090002 | 1.683461 | 5.759430 | no | confirmation took 1.593s |
| backward_09 | all | forward | 4.116311 | 4.515011 | 4.916756 | 5.011336 | 0.401745 | 0.094580 | 0.496325 | 5.011336 | no | first FBCCA was forward |
| backward_10 | all | backward | 4.483014 | 4.483014 | 4.882749 | 4.953106 | 0.399735 | 0.070357 | 0.470092 | 4.953106 | yes | included |
| backward_01 | single | right | 4.139570 | 4.934575 | 7.735961 | 7.743379 | 2.801386 | 0.007418 | 2.808804 | 7.743379 | no | first FBCCA was right |
| backward_02 | single | backward | 3.978257 | 3.978257 | 4.384230 | 4.412412 | 0.405973 | 0.028181 | 0.434155 | 4.412412 | yes | included |
| backward_03 | single | backward | 4.248473 | 4.248473 | 4.658460 | 4.681216 | 0.409987 | 0.022756 | 0.432743 | 4.681216 | yes | included |
| backward_04 | single | idle | 4.361403 | 4.761705 | 7.157180 | 7.191069 | 2.395475 | 0.033889 | 2.429364 | 7.191069 | no | first FBCCA was idle |
| backward_05 | single | forward | 4.491236 | 4.890943 | 5.283021 | 5.372537 | 0.392078 | 0.089516 | 0.481594 | 5.372537 | no | first FBCCA was forward |

## Normal-trial statistics

| Metric | n | Mean (s) | Sample variance (s^2) | Std dev (s) | Min (s) | Max (s) |
|---|---:|---:|---:|---:|---:|---:|
| 首帧 EEG -> cmd_vel 总时延 | 4 | 4.720333 | 0.054528 | 0.233513 | 4.412412 | 4.953106 |
| 第一次正确 FBCCA -> valid=true 确认时延 | 4 | 0.402856 | 0.000040 | 0.006353 | 0.395730 | 0.409987 |
| valid=true -> cmd_vel 发布时延 | 4 | 0.032319 | 0.000716 | 0.026756 | 0.007984 | 0.070357 |
| 第一次正确 FBCCA -> cmd_vel 控制时延 | 4 | 0.435176 | 0.000739 | 0.027178 | 0.403714 | 0.470092 |

Normal trials: 4/10.
