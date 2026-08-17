# Stop 频谱与 FBCCA 证据分析

本文按照其他指令分析文档的同一套方法分析 stop 数据。

当前已归档的数据为：

- `rosbag/Initialize/stop/stop_01`
- `rosbag/Initialize/stop/stop_02`
- `rosbag/Initialize/stop/stop_03`
- `rosbag/Initialize/stop/stop_04`
- 总计 4 组 rosbag，152 个完整在线分类窗口

后续其他指令按照新的录制规则执行：每个指令 2 组 single、2 组 all。

## 1. 分析目标

本分析同时计算两个不同指标：

1. **原始 FFT 频谱支持率**：12 Hz 的八通道平均幅值是否高于其他五个合法指令频率。
2. **FBCCA 选择结果**：使用当前分类器的滤波器组、谐波参考信号和 CCA 重新计算，最高分是否为 12 Hz/stop。

FFT 支持率不是 FBCCA 准确率，二者必须分开报告。

## 2. 每个窗口如何截取

与在线分类器使用的窗口一致：

```text
采样率: 250 Hz
窗口长度: 4 s
样本数: 250 * 4 = 1000
```

对于每条 `/ssvep/command` 消息：

1. 按 rosbag 接收时间排序 `/eeg/frame`。
2. 找到 command 之前最近的 EEG 样本位置。
3. 取该位置之前最近的 1000 个 EEG 样本。
4. 得到一个 8 通道、4 秒的 EEG 窗口。

伪代码：

```python
N = 1000
command_index = searchsorted(eeg_bag_timestamps, command_timestamp)
window = eeg_values[command_index - N:command_index]
```

## 3. 原始 FFT 计算

先对每一个通道去除直流分量：

```python
x = window - window.mean(axis=0, keepdims=True)
```

然后计算单边 FFT 幅值：

```python
X = np.fft.rfft(x, axis=0)
A = 2 * np.abs(X) / N
A[0, :] *= 0.5
```

频率分辨率为：

```text
Δf = fs / N = 250 / 1000 = 0.25 Hz
```

六个合法刺激频率对应的 bin 为：

| Command | 频率 | FFT bin | 周期 |
|---|---:|---:|---:|
| forward | 8 Hz | 32 | 125.000 ms |
| left | 9 Hz | 36 | 111.111 ms |
| right | 10 Hz | 40 | 100.000 ms |
| backward | 11 Hz | 44 | 90.909 ms |
| stop | 12 Hz | 48 | 83.333 ms |
| idle | 13 Hz | 52 | 76.923 ms |

八通道平均幅值为：

```python
mean_amplitude[f] = A[target_bin, :].mean()
```

单个窗口被计为“频谱支持 stop”的条件是：

```python
mean_amplitude[12] > max(
    mean_amplitude[8],
    mean_amplitude[9],
    mean_amplitude[10],
    mean_amplitude[11],
    mean_amplitude[13],
)
```

## 4. FBCCA 重新计算

重新计算使用当前分类器的配置：

```text
目标频率: 8, 9, 10, 11, 12, 13 Hz
滤波器组: 3
谐波数: 4
```

滤波器组权重为：

```text
Bank 1 = 1.000000
Bank 2 = 1.250000
Bank 3 = 0.670448
```

对每个目标频率，先计算 CCA 相关系数 `rho`，再计算：

```text
score(f) = Σ bank_weight * rho(bank, f)^2
```

最后以六个 raw score 中最大者作为 FBCCA 选择结果。

## 5. 每组结果

### Single

| 数据组 | 窗口数 | FBCCA 重算为 stop | rosbag 记录 stop | 有效 stop |
|---|---:|---:|---:|---:|
| stop_01 | 38 | 29 | 38 | 37 |
| stop_02 | 38 | 37 | 36 | 29 |
| **总计** | **76** | **66** | **74** | **66** |

Single 的原始 FFT 中，12 Hz 高于其他五个合法频率的窗口数为：

```text
49 / 76 = 64.4737%
```

### All

| 数据组 | 窗口数 | FBCCA 重算为 stop | rosbag 记录 stop | 有效 stop |
|---|---:|---:|---:|---:|
| stop_03 | 38 | 38 | 38 | 37 |
| stop_04 | 38 | 15 | 23 | 21 |
| **总计** | **76** | **53** | **61** | **58** |

All 的原始 FFT 中，12 Hz 高于其他五个合法频率的窗口数为：

```text
62 / 76 = 81.5789%
```

## 6. 总体统计

### 原始 FFT 频谱支持率

```text
single: 49 / 76 = 64.4737%
all:    62 / 76 = 81.5789%
总计:  111 / 152 = 73.0263%
```

因此，stop 的原始 FFT 中约 73% 的在线 4 秒窗口显示出 12 Hz 的合法频率优势。

### FBCCA 重新计算结果

152 个窗口重新计算后的最高类别分布：

| 最高 FBCCA 类别 | 窗口数 | 比例 |
|---|---:|---:|
| stop | 119 | 78.29% |
| forward | 31 | 20.39% |
| backward | 2 | 1.32% |

计算：

```text
119 / 152 = 78.2895%
```

### rosbag 中实际记录的 FBCCA 输出

| `fbcca_command` | 窗口数 | 比例 |
|---|---:|---:|
| stop | 135 | 88.82% |
| forward | 17 | 11.18% |

计算：

```text
135 / 152 = 88.8158%
```

### 有效输出

最终共有 138 个窗口产生了 `valid=true` 的有效指令：

| 有效指令 | 数量 |
|---|---:|
| stop | 124 |
| forward | 14 |
| **总计** | **138** |

在全部 152 个分析窗口中，最终有效 stop 的比例为：

```text
124 / 152 = 81.58%
```

在所有有效输出中，stop 占比为：

```text
124 / 138 = 89.86%
```

## 7. 一个真实有效 stop 窗口

示例：

```text
bag: rosbag/Initialize/stop/stop_01
command index: 14
reported fbcca_command: stop
reported command: stop
valid: true
reason: fbcca
```

### 原始 FFT 合法频率幅值

```text
8 Hz  = 0.068959
9 Hz  = 0.090813
10 Hz = 0.046185
11 Hz = 0.029069
12 Hz = 0.208946
13 Hz = 0.020470
```

12 Hz 是六个合法频率中最高值。

12 Hz 相对于其他五个合法频率平均值的幅值比为：

```text
12 Hz / mean(other legal frequencies) = 4.089022
```

### 三个滤波器组中的 CCA 相关系数

| 频率 | Bank 1 | Bank 2 | Bank 3 |
|---:|---:|---:|---:|
| 8 Hz | 0.446550 | 0.522741 | 0.578792 |
| 9 Hz | 0.289640 | 0.266047 | 0.244392 |
| 10 Hz | 0.206253 | 0.229766 | 0.200409 |
| 11 Hz | 0.235469 | 0.277361 | 0.295103 |
| 12 Hz | **0.635920** | **0.527023** | **0.584645** |
| 13 Hz | 0.229332 | 0.278646 | 0.311748 |

12 Hz 在三个滤波器组中的相关系数都最高：

```text
Bank 1, 12 Hz = 0.635920
Bank 2, 12 Hz = 0.527023
Bank 3, 12 Hz = 0.584645
```

### 最终 FBCCA 分数

| 指令 | 频率 | Raw score | Normalized score |
|---|---:|---:|---:|
| forward | 8 Hz | 0.765580 | 0.303922 |
| left | 9 Hz | 0.212412 | 0.084324 |
| right | 10 Hz | 0.135459 | 0.053775 |
| backward | 11 Hz | 0.209993 | 0.083364 |
| stop | 12 Hz | **0.980751** | **0.389341** |
| idle | 13 Hz | 0.214806 | 0.085274 |

这个窗口的结果链条是：

```text
原始 FFT: 12 Hz 最大
CCA: 12 Hz 的三个滤波器组相关性都最高
FBCCA: 12 Hz raw score 最大
命令映射: stop
连续确认: 通过
最终输出: valid stop
```

## 8. 其他指标

所有 152 个窗口的汇总统计：

```text
12 Hz normalized FBCCA score 均值:   0.3380
12 Hz normalized FBCCA score 中位数: 0.3518
12 Hz / 其他合法频率平均值，中位数:   2.3647
12 Hz 相对邻近频带 SNR，中位数:       2.5428
50 Hz 平均幅值，中位数:              1.7667
```

## 9. 结论与限制

stop 的整体结果较好，但 single 和 all 差异明显：

- 原始 FFT 的 12 Hz 频谱支持率：`73.03%`；
- FBCCA 重新计算最高为 stop：`78.29%`；
- rosbag 实际记录的 FBCCA stop：`88.82%`；
- 最终有效 stop：`124 / 152 = 81.58%`。

其中 `stop_04` 的 FBCCA 重算明显弱于其他三组，出现了较多 forward 结果。这说明 stop 仍有一定 trial 间波动，不能只根据其他三组推断全部数据都稳定。

本数据只有 stop 目标试验，不能单独推出完整六分类准确率；最终评估仍需把不同指令数据放在一起进行独立测试。
