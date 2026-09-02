## 目标
ESP32-S3 测试台升级为：ADC DMA 连续采样 50 kSPS + USB CDC 实时回传 float32 二进制流（帧内自带 `pyfar.Signal` 构造所需的全部元数据），板上保留 IEEE-1241 分析；PC 端 Python 类 + CLI 实时转成 `pyfar.Signal` 放内存。

## 回传帧格式（对齐 pyfar.Signal 构造参数）

`pyfar.Signal(time_data, sampling_rate)` 需要的核心参数是**采样数据 + sampling_rate**；帧头直接携带这些元数据，Python 端零配置直接构造：

```
偏移  大小  字段                说明
0     4    magic              0xAA55AD98（帧同步）
4     2    version            帧格式版本 (uint16)
6     4    seq                帧序号 uint32（丢帧检测）
10    4    sampling_rate      uint32 = 50000 → pyfar.Signal 的 sampling_rate
14    4    n_samples          uint32 本帧样本数 N（time_data 长度）
18    2    n_channels         uint16 = 1（pyfar.Signal 通道数，预留）
20    4    time_stamp_us      uint32 esp_timer 微秒时间戳（可作 Signal comment/times 参考）
24    4    adcn_full_scale    float32 满量程参考值（预留定标，Python 端可换算电压）
28    N*4  time_data          float32 样本序列 → pyfar.Signal 的 time_data
```

Python 端解析后：`pyfar.Signal(block, fs, comment=f"seq={seq} t={ts}us")`，无需用户手动传任何参数。

## 固件改动

### 1. `platformio.ini`
- 取消注释 `-DARDUINO_USB_MODE=1`、`-DARDUINO_USB_CDC_ON_BOOT=1` 启用原生 USB CDC
- 确保 espressif32 platform 为 arduino-esp32 3.x（`analogContinuous` API）

### 2. `src/main.cpp`
- 采样改用 `analogContinuous`（adc_continuous DMA）：引脚 2，50 kSPS，12 位，循环取缓冲拼成 2048 点块
- 每块：① 打包上述帧格式经 Serial（CDC）写出；② 保留现有 FFT→四参数正弦拟合→LUT→SNR/THD/ENOB 分析，指标以 `#` 前缀文本行输出（与二进制帧共存，Python 端跳过）
- SAMPLES=2048，SAMPLING_FREQUENCY=50000.0（FFT 分辨率 ≈24.4 Hz）

## Python 端：`python/ad9833_stream.py`

- **`AD9833Stream` 类**：
  - 打开 CDC 串口，后台线程按 magic 同步、解析帧、序号连续性校验（丢帧告警计数）
  - 环形缓冲（`deque`+numpy，加锁）；`.signal` 属性实时返回 `pyfar.Signal(最新窗口, fs=帧内sampling_rate)`；`.latest_block()` 返回最近一帧的 `pyfar.Signal`；`.dropped_frames`、`.stop()`
- **CLI**：`python ad9833_stream.py --port COMx --window 1.0 --stats [--plot]`
  - `--stats` 实时打印接收速率/丢帧/幅度；`--plot` 用 pyfar 实时画最近窗口频谱
- 附 `python/requirements.txt`（pyserial、numpy、pyfar）

## 验证
- `pio run` 编译通过
- Python 端内置 `--simulate` 模式（本地合成字节流）自测解析与 pyfar.Signal 构造
- 实测由用户上板验证

## 风险提示
- 需接 ESP32-S3 原生 USB 口（19/20 脚），否则 CDC 不生效
- AD9833 当前激励频率较低，50 kSPS/2048 点窗口 40.96ms，拟合依然适用