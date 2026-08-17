# Right 频谱与 FBCCA 证据分析

本文按照 `forward_fft_analysis.md`、`backward_fft_analysis.md` 和 `left_fft_analysis.md` 的同一套方法分析 right 数据。

当前已归档的数据为：

- `rosbag/Initialize/right/right_01`
- `rosbag/Initialize/right/right_02`
- `rosbag/Initialize/right/right_03`
- `rosbag/Initialize/right/right_04`
- 总计 4 组 rosbag，151 个完整在线分类窗口

`right_04` 比其他数据少 1 个完整窗口，因此总数为 151，而不是 152。

## 1. 分析目标

本分析同时计算两个不同指标：

1. **原始 FFT 频谱支持率**：10 Hz 的八通道平均幅值是否高于其他五个合法指令频率。
2. **FBCCA 选择结果**：使用当前分类器的滤波器组、谐波参考信号和 CCA 重新计算，最高分是否为 10 Hz/right。

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

单个窗口被计为“频谱支持 right”的条件是：

```python
mean_amplitude[10] > max(
    mean_amplitude[8],
    mean_amplitude[9],
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

| 数据组 | 窗口数 | FBCCA 重算为 right | rosbag 记录 right | 有效 right |
|---|---:|---:|---:|---:|
| right_01 | 38 | 38 | 38 | 37 |
| right_02 | 38 | 38 | 38 | 37 |
| **总计** | **76** | **76** | **76** | **74** |

Single 的原始 FFT 中，10 Hz 高于其他五个合法频率的窗口数为：

```text
59 / 76 = 77.6316%
```

### All

| 数据组 | 窗口数 | FBCCA 重算为 right | rosbag 记录 right | 有效 right |
|---|---:|---:|---:|---:|
| right_03 | 38 | 37 | 38 | 37 |
| right_04 | 37 | 37 | 37 | 37 |
| **总计** | **75** | **74** | **75** | **74** |

All 的原始 FFT 中，10 Hz 高于其他五个合法频率的窗口数为：

```text
56 / 75 = 74.6667%
```

## 6. 总体统计

### 原始 FFT 频谱支持率

```text
single: 59 / 76 = 77.6316%
all:    56 / 75 = 74.6667%
总计:  115 / 151 = 76.1589%
```

因此，right 的原始 FFT 中约四分之三的在线 4 秒窗口显示出 10 Hz 的合法频率优势。

### FBCCA 重新计算结果

151 个窗口重新计算后的最高类别分布：

| 最高 FBCCA 类别 | 窗口数 | 比例 |
|---|---:|---:|
| right | 150 | 99.34% |
| forward | 1 | 0.66% |

计算：

```text
150 / 151 = 99.3377%
```

### rosbag 中实际记录的 FBCCA 输出

| `fbcca_command` | 窗口数 | 比例 |
|---|---:|---:|
| right | 151 | 100.00% |

### 有效输出

最终共有 148 个窗口产生了 `valid=true` 的有效指令：

| 有效指令 | 数量 |
|---|---:|
| right | 148 |
| **总计** | **148** |

在全部 151 个分析窗口中，最终有效 right 的比例为：

```text
148 / 151 = 98.01%
```

## 7. 一个真实有效 right 窗口

示例：

```text
bag: rosbag/Initialize/right/right_01
command index: 13
reported fbcca_command: right
reported command: right
valid: true
reason: fbcca
```

### 原始 FFT 合法频率幅值

```text
8 Hz  = 0.115798
9 Hz  = 0.110518
10 Hz = 0.202802
11 Hz = 0.140855
12 Hz = 0.085862
13 Hz = 0.091411
```

10 Hz 是六个合法频率中最高值。

10 Hz 相对于其他五个合法频率平均值的幅值比为：

```text
10 Hz / mean(other legal frequencies) = 1.862465
```

### 三个滤波器组中的 CCA 相关系数

| 频率 | Bank 1 | Bank 2 | Bank 3 |
|---:|---:|---:|---:|
| 8 Hz | 0.211051 | 0.194330 | 0.199953 |
| 9 Hz | 0.233087 | 0.232906 | 0.254231 |
| 10 Hz | **0.560759** | **0.429480** | **0.343269** |
| 11 Hz | 0.178161 | 0.153633 | 0.178211 |
| 12 Hz | 0.229424 | 0.184143 | 0.210174 |
| 13 Hz | 0.143358 | 0.166270 | 0.198000 |

10 Hz 在三个滤波器组中的相关系数都最高：

```text
Bank 1, 10 Hz = 0.560759
Bank 2, 10 Hz = 0.429480
Bank 3, 10 Hz = 0.343269
```

### 最终 FBCCA 分数

| 指令 | 频率 | Raw score | Normalized score |
|---|---:|---:|---:|
| forward | 8 Hz | 0.118553 | 0.099074 |
| left | 9 Hz | 0.165469 | 0.138282 |
| right | 10 Hz | **0.624018** | **0.521489** |
| backward | 11 Hz | 0.082538 | 0.068977 |
| stop | 12 Hz | 0.124637 | 0.104158 |
| idle | 13 Hz | 0.081393 | 0.068020 |

这个窗口的结果链条是：

```text
原始 FFT: 10 Hz 最大
CCA: 10 Hz 的三个滤波器组相关性都最高
FBCCA: 10 Hz raw score 最大
命令映射: right
连续确认: 通过
最终输出: valid right
```

## 8. 其他指标

所有 151 个窗口的汇总统计：

```text
10 Hz normalized FBCCA score 均值:   0.3971
10 Hz normalized FBCCA score 中位数: 0.4013
10 Hz / 其他合法频率平均值，中位数:   1.9435
10 Hz 相对邻近频带 SNR，中位数:       2.1277
50 Hz 平均幅值，中位数:              2.2925
```

## 9. 结论与限制

right 是目前已分析指令中 FBCCA 表现最稳定的一组之一：

- 原始 FFT 的 10 Hz 频谱支持率：`76.16%`；
- FBCCA 重新计算最高为 right：`99.34%`；
- rosbag 实际记录的 FBCCA right：`100%`；
- 最终有效 right：`148 / 151 = 98.01%`。

这些结果支持“right 的 10 Hz SSVEP 特征清晰，且当前 FBCCA 能稳定识别”。但本数据只有 right 目标试验，不能单独推出完整六分类准确率；最终评估仍需把不同指令数据放在一起进行独立测试。
