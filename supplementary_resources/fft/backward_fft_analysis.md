# Backward 频谱与 FBCCA 证据分析

本文按照 `forward_fft_analysis.md` 的同一套方法分析 backward 数据。

当前已归档的数据为：

- `rosbag/Initialize/backward/backward_01` 到 `backward_05`
- `rosbag/Initialize/backward/backward_06` 到 `backward_10`
- 总计 10 组 rosbag，380 个完整在线分类窗口

这些数据是在用户决定减少后续录制数量之前完成的，因此 backward 保留已有的 5 组 single 和 5 组 all。后续其他指令按新规则录制：每个指令 2 组 single、2 组 all。

## 1. 分析目标

本分析同时计算两个不同指标：

1. **原始 FFT 频谱支持率**：11 Hz 的八通道平均幅值是否高于其他五个合法指令频率。
2. **FBCCA 选择结果**：使用当前分类器的滤波器组、谐波参考信号和 CCA 重新计算，最高分是否为 11 Hz/backward。

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

单个窗口被计为“频谱支持 backward”的条件是：

```python
mean_amplitude[11] > max(
    mean_amplitude[8],
    mean_amplitude[9],
    mean_amplitude[10],
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

| 数据组 | 窗口数 | FBCCA 重算为 backward | rosbag 记录 backward | 有效 backward |
|---|---:|---:|---:|---:|
| backward_01 | 38 | 30 | 29 | 22 |
| backward_02 | 38 | 20 | 27 | 18 |
| backward_03 | 38 | 31 | 30 | 23 |
| backward_04 | 38 | 28 | 28 | 10 |
| backward_05 | 38 | 13 | 16 | 11 |
| **总计** | **190** | **122** | **130** | **84** |

Single 的原始 FFT 中，11 Hz 高于其他五个合法频率的窗口数为：

```text
117 / 190 = 61.5789%
```

### All

| 数据组 | 窗口数 | FBCCA 重算为 backward | rosbag 记录 backward | 有效 backward |
|---|---:|---:|---:|---:|
| backward_06 | 38 | 25 | 28 | 24 |
| backward_07 | 38 | 23 | 30 | 26 |
| backward_08 | 38 | 33 | 31 | 15 |
| backward_09 | 38 | 27 | 30 | 17 |
| backward_10 | 38 | 21 | 30 | 26 |
| **总计** | **190** | **129** | **149** | **108** |

All 的原始 FFT 中，11 Hz 高于其他五个合法频率的窗口数为：

```text
80 / 190 = 42.1053%
```

## 6. 总体统计

### 原始 FFT 频谱支持率

```text
single: 117 / 190 = 61.5789%
all:     80 / 190 = 42.1053%
总计:   197 / 380 = 51.8421%
```

因此，backward 的原始 FFT 频谱只有约一半窗口显示出明确的 11 Hz 合法频率优势。

### FBCCA 重新计算结果

380 个窗口重新计算后的最高类别分布：

| 最高 FBCCA 类别 | 窗口数 | 比例 |
|---|---:|---:|
| backward | 251 | 66.05% |
| forward | 70 | 18.42% |
| left | 32 | 8.42% |
| idle | 20 | 5.26% |
| right | 5 | 1.32% |
| stop | 2 | 0.53% |

计算：

```text
251 / 380 = 66.0526%
```

### rosbag 中实际记录的 FBCCA 输出

| `fbcca_command` | 窗口数 | 比例 |
|---|---:|---:|
| backward | 279 | 73.42% |
| forward | 58 | 15.26% |
| left | 22 | 5.79% |
| idle | 14 | 3.68% |
| right | 7 | 1.84% |

计算：

```text
279 / 380 = 73.4211%
```

重新计算结果与 rosbag 中记录的 FBCCA 最高类别一致：

```text
323 / 380 = 85.00%
```

这里存在少量不一致，可能来自录制时的消息时间和离线按 rosbag 时间回溯窗口之间的边界差异，不能把这 85% 当作分类准确率。

### 有效输出

最终共有 233 个窗口产生了 `valid=true` 的有效指令：

| 有效指令 | 数量 |
|---|---:|
| backward | 192 |
| forward | 24 |
| left | 11 |
| idle | 5 |
| right | 1 |
| **总计** | **233** |

在已经被系统接受的有效输出中，backward 占比为：

```text
192 / 233 = 82.40%
```

如果把没有通过确认的窗口也作为未成功控制，则整个 380 个窗口中有效 backward 的比例为：

```text
192 / 380 = 50.53%
```

这两个比例含义不同：前者是“系统接受的结果中 backward 的比例”，后者是“全部分析窗口最终有效 backward 的比例”。

## 7. 一个真实有效 backward 窗口

示例：

```text
bag: rosbag/Initialize/backward/backward_01
command index: 20
reported fbcca_command: backward
reported command: backward
valid: true
reason: fbcca
```

### 原始 FFT 合法频率幅值

```text
8 Hz  = 0.088064
9 Hz  = 0.118975
10 Hz = 0.072193
11 Hz = 0.194860
12 Hz = 0.085655
13 Hz = 0.062692
```

11 Hz 是六个合法频率中最高值。

11 Hz 相对于其他五个合法频率平均值的幅值比为：

```text
11 Hz / mean(other legal frequencies) = 2.278643
```

### 三个滤波器组中的 CCA 相关系数

| 频率 | Bank 1 | Bank 2 | Bank 3 |
|---:|---:|---:|---:|
| 8 Hz | 0.217157 | 0.240870 | 0.212184 |
| 9 Hz | 0.251474 | 0.261149 | 0.284674 |
| 10 Hz | 0.291788 | 0.183918 | 0.175183 |
| 11 Hz | **0.444018** | 0.200365 | 0.241877 |
| 12 Hz | 0.204686 | 0.195628 | 0.223665 |
| 13 Hz | 0.220836 | 0.210731 | 0.250729 |

11 Hz 在第一滤波器组中的相关系数明显较高：

```text
Bank 1, 11 Hz = 0.444018
```

### 最终 FBCCA 分数

| 指令 | 频率 | Raw score | Normalized score |
|---|---:|---:|---:|
| forward | 8 Hz | 0.149865 | 0.141791 |
| left | 9 Hz | 0.202821 | 0.191894 |
| right | 10 Hz | 0.147998 | 0.140025 |
| backward | 11 Hz | **0.286560** | **0.271121** |
| stop | 12 Hz | 0.123274 | 0.116633 |
| idle | 13 Hz | 0.146425 | 0.138537 |

因此这个窗口的结果链条是：

```text
原始 FFT: 11 Hz 最大
CCA: 11 Hz 的综合相关性最高
FBCCA: 11 Hz raw score 最大
命令映射: backward
连续确认: 通过
最终输出: valid backward
```

## 8. 结论与限制

backward 的结果比 forward 弱：

- 原始 FFT 的 11 Hz 频谱支持率：`51.84%`；
- FBCCA 重新计算最高为 backward：`66.05%`；
- rosbag 实际记录的 FBCCA backward：`73.42%`；
- 有效输出中 backward 占比：`82.40%`。

这说明 backward 确实有可观察的 11 Hz 证据，但稳定性低于之前的 forward 数据。特别是 all 数据中，11 Hz 原始 FFT 支持率只有 `42.11%`，而且 50 Hz 工频干扰的总体幅值较高。

因此不能只凭一张原始 FFT 图断言 backward，也不能把 `51.84%` 直接称为 backward 分类准确率。更完整的判断应同时结合：

1. 11 Hz 原始 FFT 幅值；
2. 11 Hz 相对其他合法频率的幅值比；
3. 11 Hz 相对邻近频带的 SNR；
4. 同一窗口的 FBCCA 六类分数；
5. 最终 `valid` 指令。
