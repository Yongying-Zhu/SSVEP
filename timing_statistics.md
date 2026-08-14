# SSVEP Timing Statistics

The statistics below use normal trials only. A normal trial is one where the
first FBCCA candidate matches the expected command and the expected command
reaches `valid=true` within 0.8 seconds.

Variance is the sample variance (`n - 1`).

| Command | Normal trials | First EEG -> cmd_vel mean | Variance | Correct FBCCA -> cmd_vel mean | Variance |
|---|---:|---:|---:|---:|---:|
| forward | 6/10 | 4.731 s | 0.025132 | 0.456 s | 0.001274 |
| backward | 4/10 | 4.720 s | 0.054528 | 0.435 s | 0.000739 |
| left | 4/4 | 4.505 s | 0.023268 | 0.439 s | 0.000307 |
| right | 4/4 | 4.742 s | 0.049954 | 0.433 s | 0.000683 |
| stop | 4/4 | 4.665 s | 0.016971 | 0.465 s | 0.000376 |
