# Left 频谱与 FBCCA 证据分析

本文按照 `forward_fft_analysis.md` 和 `backward_fft_analysis.md` 的同一套方法分析 left 数据。

当前已归档的数据为：

- `rosbag/Initialize/left/left_01`
- `rosbag/Initialize/left/left_02`
- `rosbag/Initialize/left/left_03`
- `rosbag/Initialize/left/left_04`
- 总计 4 组 rosbag，152 个完整在线分类窗口

后续其他指令按照新的录制规则执行：每个指令 2 组 single、2 组 all。

## 1. 分析目标

本分析同时计算两个不同指标：

1. **原始 FFT 频谱支持率**：9 Hz 的八通道平均幅值是否高于其他五个合法指令频率。
2. **FBCCA 选择结果**：使用当前分类器的滤波器组、谐波参考信号和 CCA 重新计算，最高分是否为 9 Hz/left。

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

单个窗口被计为“频谱支持 left”的条件是：

```python
mean_amplitude[9] > max(
    mean_amplitude[8],
    mean_amplitude[10],
    mean_amplitude[11],
    mean_amplitude[12],
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

| 数据组 | 窗口数 | FBCCA 重算为 left | rosbag 记录 left | 有效 left |
|---|---:|---:|---:|---:|
| left_01 | 38 | 36 | 38 | 37 |
| left_02 | 38 | 38 | 38 | 37 |
| **总计** | **76** | **74** | **76** | **74** |

Single 的原始 FFT 中，9 Hz 高于其他五个合法频率的窗口数为：

```text
54 / 76 = 71.0526%
```

### All

| 数据组 | 窗口数 | FBCCA 重算为 left | rosbag 记录 left | 有效 left |
|---|---:|---:|---:|---:|
| left_03 | 38 | 38 | 38 | 37 |
| left_04 | 38 | 38 | 38 | 36 |
| **总计** | **76** | **76** | **76** | **73** |

All 的原始 FFT 中，9 Hz 高于其他五个合法频率的窗口数为：

```text
61 / 76 = 80.2632%
```

## 6. 总体统计

### 原始 FFT 频谱支持率

```text
single: 54 / 76 = 71.0526%
all:    61 / 76 = 80.2632%
总计:  115 / 152 = 75.6579%
```

因此，left 的原始 FFT 中，约四分之三的在线 4 秒窗口显示出 9 Hz 的合法频率优势。

### FBCCA 重新计算结果

152 个窗口重新计算后的最高类别分布：

| 最高 FBCCA 类别 | 窗口数 | 比例 |
|---|---:|---:|
| left | 150 | 98.68% |
| backward | 1 | 0.66% |
| stop | 1 | 0.66% |

计算：

```text
150 / 152 = 98.6842%
```

### rosbag 中实际记录的 FBCCA 输出

| `fbcca_command` | 窗口数 | 比例 |
|---|---:|---:|
| left | 152 | 100.00% |

### 有效输出

最终共有 147 个窗口产生了 `valid=true` 的有效指令：

| 有效指令 | 数量 |
|---|---:|
| left | 147 |
| **总计** | **147** |

在全部 152 个分析窗口中，最终有效 left 的比例为：

```text
147 / 152 = 96.71%
```

## 7. 一个真实有效 left 窗口

示例：

```text
bag: rosbag/Initialize/left/left_01
command index: 13
reported fbcca_command: left
reported command: left
valid: true
reason: fbcca
```

### 原始 FFT 合法频率幅值

```text
8 Hz  = 0.088001
9 Hz  = 0.122771
10 Hz = 0.092167
11 Hz = 0.054414
12 Hz = 0.081082
13 Hz = 0.057757
```

9 Hz 是六个合法频率中最高值。

9 Hz 相对于其他五个合法频率平均值的幅值比为：

```text
9 Hz / mean(other legal frequencies) = 1.643868
```

### 三个滤波器组中的 CCA 相关系数

| 频率 | Bank 1 | Bank 2 | Bank 3 |
|---:|---:|---:|---:|
| 8 Hz | 0.292435 | 0.269813 | 0.255709 |
| 9 Hz | **0.352565** | **0.394349** | 0.242973 |
| 10 Hz | 0.261049 | 0.160466 | 0.138611 |
| 11 Hz | 0.245735 | 0.232795 | 0.264700 |
| 12 Hz | 0.287890 | 0.234362 | 0.267129 |
| 13 Hz | 0.225859 | 0.227638 | 0.266499 |

9 Hz 在前两个滤波器组中相关系数最高：

```text
Bank 1, 9 Hz = 0.352565
Bank 2, 9 Hz = 0.394349
```

### 最终 FBCCA 分数

| 指令 | 频率 | Raw score | Normalized score |
|---|---:|---:|---:|
| forward | 8 Hz | 0.220356 | 0.179191 |
| left | 9 Hz | **0.358271** | **0.291342** |
| right | 10 Hz | 0.113214 | 0.092065 |
| backward | 11 Hz | 0.175103 | 0.142392 |
| stop | 12 Hz | 0.199380 | 0.162133 |
| idle | 13 Hz | 0.163402 | 0.132877 |

这个窗口的结果链条是：

```text
原始 FFT: 9 Hz 最大
CCA: 9 Hz 的综合相关性最高
FBCCA: 9 Hz raw score 最大
命令映射: left
连续确认: 通过
最终输出: valid left
```

## 8. 其他指标

所有 152 个窗口的汇总统计：

```text
9 Hz normalized FBCCA score 均值:   0.3262
9 Hz normalized FBCCA score 中位数: 0.3217
9 Hz / 其他合法频率平均值，中位数:   2.1712
9 Hz 相对邻近频带 SNR，中位数:       2.2695
50 Hz 平均幅值，中位数:              3.5442
```

这里的 `50 Hz` 仍然显示出工频干扰，需要在后续实验中继续关注。

## 9. 结论与限制

left 是目前三类已分析指令中最稳定的一组：

- 原始 FFT 的 9 Hz 频谱支持率：`75.66%`；
- FBCCA 重新计算最高为 left：`98.68%`；
- rosbag 实际记录的 FBCCA left：`100%`；
- 最终有效 left：`147 / 152 = 96.71%`。

这些结果支持“left 的 9 Hz SSVEP 特征比较清晰”。但仍要注意：本数据只有 left 目标试验，不能单独推出完整六分类准确率；并且频谱中仍有 50 Hz 工频干扰。因此最终模型评估仍需要把不同指令的数据放在一起进行独立测试。
