# right timing table

筛选规则：`first FBCCA == expected command` 且 `第一次正确 FBCCA -> valid=true <= 0.8 s`。
均值和方差只使用 `normal=yes` 的行；方差为样本方差（分母 `n-1`）。

## Per-trial data

| Bag | Mode | First FBCCA | First FBCCA t (s) | Correct FBCCA t (s) | Valid t (s) | cmd_vel t (s) | Correct -> valid (s) | valid -> cmd_vel (s) | Correct -> cmd_vel (s) | t=0 -> cmd_vel (s) | Normal | Exclusion |
|---|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---|---|
| right_03 | all | right | 4.483425 | 4.483425 | 4.882489 | 4.947032 | 0.399063 | 0.064544 | 0.463607 | 4.947032 | yes | included |
| right_04 | all | right | 3.990252 | 3.990252 | 4.389619 | 4.424822 | 0.399367 | 0.035203 | 0.434570 | 4.424822 | yes | included |
| right_01 | single | right | 4.376460 | 4.376460 | 4.775416 | 4.776231 | 0.398956 | 0.000815 | 0.399771 | 4.776231 | yes | included |
| right_02 | single | right | 4.384575 | 4.384575 | 4.781025 | 4.819790 | 0.396450 | 0.038765 | 0.435216 | 4.819790 | yes | included |

## Normal-trial statistics

| Metric | n | Mean (s) | Sample variance (s^2) | Std dev (s) | Min (s) | Max (s) |
|---|---:|---:|---:|---:|---:|---:|
| 首帧 EEG -> cmd_vel 总时延 | 4 | 4.741969 | 0.049954 | 0.223505 | 4.424822 | 4.947032 |
| 第一次正确 FBCCA -> valid=true 确认时延 | 4 | 0.398459 | 0.000002 | 0.001351 | 0.396450 | 0.399367 |
| valid=true -> cmd_vel 发布时延 | 4 | 0.034832 | 0.000685 | 0.026176 | 0.000815 | 0.064544 |
| 第一次正确 FBCCA -> cmd_vel 控制时延 | 4 | 0.433291 | 0.000683 | 0.026128 | 0.399771 | 0.463607 |

Normal trials: 4/4.
