# Forward 频谱证据分析

本文记录 `forward` 数据的频谱支持率计算过程。分析对象是：

- `rosbag/Initialize/forward/forward_01` 到 `forward_05`
- `rosbag/Initialize/forward/forward_06` 到 `forward_10`
- 总计 10 组 rosbag

## 1. 分析目标

这里计算的不是 FBCCA 分类准确率，而是一个独立的频谱指标：

> 在一个在线 FBCCA 输出对应的 4 秒 EEG 窗口中，8 Hz 的八通道平均 FFT 幅值是否大于其他五个合法刺激频率。

因此，这个指标只能说明原始频谱是否支持 `forward`，不能替代 FBCCA 的最终分类结果。

## 2. 每个窗口如何截取

在线分类器的参数是：

```text
采样率: 250 Hz
窗口长度: 4 s
样本数: 250 * 4 = 1000
```

对于 rosbag 中的每一条 `/ssvep/command` 消息：

1. 按 rosbag 接收时间排序 `/eeg/frame`。
2. 找到该 command 之前最近的 EEG 样本位置。
3. 取该位置之前的最近 1000 个 EEG 样本。
4. 得到一个 8 通道、1000 样本的 4 秒窗口。

伪代码如下：

```python
N = 1000
command_index = searchsorted(eeg_bag_timestamps, command_timestamp)
window = eeg_values[command_index - N:command_index]
```

这样取出的窗口与在线分类器使用的“最近 4 秒 EEG”保持一致。

## 3. FFT 计算

对每个通道单独去除平均值：

```python
x = window - window.mean(axis=0, keepdims=True)
```

去平均值是为了去除通道的直流偏置。之后计算单边实数 FFT：

```python
X = np.fft.rfft(x, axis=0)
A = 2 * np.abs(X) / N
A[0, :] *= 0.5
```

其中：

- `X` 是复数频谱；
- `A` 是单边 FFT 幅值；
- 不使用零填充；
- 频率轴使用固定采样率 250 Hz；
- 频率分辨率为：

```text
Δf = fs / N = 250 / 1000 = 0.25 Hz
```

FFT 频率 bin 的计算方式为：

```python
frequencies = np.fft.rfftfreq(N, d=1 / 250)
```

## 4. 合法指令频率

当前项目中的频率映射是：

| Command | 频率 | FFT bin | 周期 |
|---|---:|---:|---:|
| forward | 8 Hz | 32 | 125.000 ms |
| left | 9 Hz | 36 | 111.111 ms |
| right | 10 Hz | 40 | 100.000 ms |
| backward | 11 Hz | 44 | 90.909 ms |
| stop | 12 Hz | 48 | 83.333 ms |
| idle | 13 Hz | 52 | 76.923 ms |

bin 的计算公式是：

```text
bin = frequency / Δf
```

例如：

```text
8 Hz / 0.25 Hz = 32
```

## 5. 八通道平均幅值

对每个合法频率，取该频率 bin 在 8 个通道上的幅值，然后求平均：

```python
mean_amplitude[f] = A[target_bin, :].mean()
```

对于某一个窗口，判断条件是：

```python
is_forward_supported = (
    mean_amplitude[8] > max(
        mean_amplitude[9],
        mean_amplitude[10],
        mean_amplitude[11],
        mean_amplitude[12],
        mean_amplitude[13],
    )
)
```

注意：这里比较的只有六个合法指令频率，不比较整个 `0-125 Hz` 频谱中的所有频率。

## 6. 一个窗口的完整示例

示例数据：

```text
bag: rosbag/Initialize/forward/forward_01
窗口样本数: 1000
窗口长度: 4.0 s
采样率: 250 Hz
频率分辨率: 0.25 Hz
```

这个窗口 8 个通道在六个合法频率上的 FFT 幅值为：

| 通道 | 8 Hz | 9 Hz | 10 Hz | 11 Hz | 12 Hz | 13 Hz |
|---|---:|---:|---:|---:|---:|---:|
| Ch0 | 0.113063 | 0.139606 | 0.091491 | 0.186673 | 0.163604 | 0.122879 |
| Ch1 | 0.317195 | 0.297373 | 0.313640 | 0.324544 | 0.273188 | 0.234479 |
| Ch2 | 1.212045 | 1.090306 | 1.003292 | 0.974245 | 0.897610 | 0.794200 |
| Ch3 | 0.114686 | 0.125888 | 0.118267 | 0.142539 | 0.150716 | 0.131859 |
| Ch4 | 0.132186 | 0.008460 | 0.019186 | 0.085996 | 0.097820 | 0.044523 |
| Ch5 | 0.109202 | 0.118726 | 0.145243 | 0.082783 | 0.126498 | 0.084708 |
| Ch6 | 0.096610 | 0.010255 | 0.080612 | 0.038368 | 0.095662 | 0.040790 |
| Ch7 | 0.087281 | 0.083491 | 0.017343 | 0.141104 | 0.111093 | 0.083572 |

八通道平均值：

```text
8 Hz  = 0.272783454
9 Hz  = 0.234262964
10 Hz = 0.223634250
11 Hz = 0.247031475
12 Hz = 0.239523946
13 Hz = 0.192126293
```

因此：

```text
8 Hz = 0.272783454
其他合法频率最大值 = 11 Hz = 0.247031475
0.272783454 > 0.247031475
```

这个窗口被计为“8 Hz 频谱支持 forward”。8 Hz 相对于第二高的 11 Hz 的幅值比为：

```text
0.272783454 / 0.247031475 = 1.104246
```

也就是高约 `10.4%`。

这个窗口的在线消息本身是：

```text
fbcca_command = right
valid = false
reason = low_confidence
```

这说明“8 Hz 原始 FFT 最大”与“FBCCA 最终输出 forward”不是同一个指标。

## 7. 全部窗口统计

统计结果：

| 数据组 | 8 Hz 支持窗口 | 总窗口 | 支持率 |
|---|---:|---:|---:|
| single | 125 | 190 | 65.7895% |
| all | 153 | 190 | 80.5263% |
| 总计 | 278 | 380 | 73.1579% |

总计计算：

```text
278 / 380 = 0.731578947
0.731578947 * 100% = 73.1579% ≈ 73.2%
```

## 8. 与 FBCCA 的关系

这个频谱支持率不是 FBCCA 准确率。FBCCA 在当前代码中还会进行：

- 3 个滤波器组处理；
- 8、9、10、11、12、13 Hz 六个候选频率比较；
- 每个候选频率的 4 个谐波参考信号；
- CCA 多通道空间相关性计算；
- 滤波器组加权；
- 置信度阈值和连续两次确认。

所以可能出现：

```text
原始 FFT 中 8 Hz 不是最高幅值
但 FBCCA 仍选择 forward
```

也可能出现本示例中的情况：

```text
原始 FFT 中 8 Hz 是合法频率中最高
但 FBCCA 因空间相关性和谐波关系选择其他类别
```

## 9. 结论与限制

这 `73.2%` 的准确含义是：

> 在 380 个在线 4 秒窗口中，有 278 个窗口的八通道平均原始 FFT 幅值满足“8 Hz 大于其他五个合法指令频率”。

它可以作为 forward 的频谱支持证据，但不能直接称为 forward 分类准确率。

当前数据还存在明显的 50 Hz 工频干扰及其谐波，因此后续评估应同时记录：

1. 8 Hz 的原始 FFT 幅值；
2. 8 Hz 相对其他合法频率的幅值比；
3. 8 Hz 相对邻近频带的 SNR；
4. 同一窗口的 FBCCA 六类分数；
5. 最终 `valid` 命令。

后续指令可复用同样的目录结构：

```text
rosbag/Initialize/
├── forward/
│   └── forward_fft_analysis.md
├── backward/
│   └── backward_fft_analysis.md
├── left/
│   └── left_fft_analysis.md
└── ...
```
