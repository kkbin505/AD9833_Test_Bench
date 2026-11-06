# AD9833 Test Bench — CYD 频谱与电机转速显示

本仓库当前 `CYD` 分支面向 ES3C28P 触摸屏开发板：使用板载 ES8311 麦克风采集声音，实时显示频谱、滚动热力图和基于阶次估算的电机转速。

仓库沿用 AD9833 Test Bench 名称；本文描述当前 CYD 固件，其他分支的硬件和功能可能不同。

## 硬件

| 部件 | 当前配置 |
| --- | --- |
| 开发板 | ES3C28P，ESP32-S3，16 MB Flash、8 MB OPI PSRAM |
| 显示屏 | 2.8 英寸 ILI9341V，横屏 320 × 240 |
| 触摸屏 | FT6336G 电容触摸，I2C 地址 `0x38` |
| 音频输入 | ES8311 codec 与板载麦克风，单声道 |
| USB | ESP32-S3 原生 USB，用于烧录及可选采集 |

ES3N28P 没有电容触摸，不能通过屏幕调整阶次。

| 板载连接 | GPIO |
| --- | --- |
| LCD SCK / MOSI / MISO | 12 / 11 / 13 |
| LCD CS / D-C / 背光 | 10 / 46 / 45 |
| 共用 I2C SDA / SCL | 16 / 15 |
| 触摸复位 | 18 |
| ES8311 MCLK / BCLK / LRCK / 数据输入 | 4 / 5 / 7 / 6 |
| 功放使能 | 1，当前保持关闭 |

## 屏幕与操作

- **左侧**：70 像素数值与设置栏，显示总 dBA、未加权 SPL 和启动以来最高 SPL（`pk`）。
- **阶次设置**：竖排 `Order → ▲ → 数字 → ▼`；上箭头加 1，下箭头减 1。默认 10 阶，范围 1～65535；按住不连发，重启恢复 10 阶。
- **右上**：实时频谱，约占绘图区高度四分之一；纵轴为 20～80 dBA，曲线下方深蓝渐变填充。
- **右下**：滚动热力图，每 0.5 秒增加一行，最新数据在顶部；颜色范围为 30～60 dBA，超过上限使用最亮颜色。
- **横坐标**：两图共用 20 Hz～20 kHz 对数频率轴与像素映射，频率位置上下对齐。
- **峰值标记**：红色竖线跟随确认后的峰；白色标签固定在距屏幕顶部 5 像素处，格式为 `62dBA @ 269Hz 1614RPM`。

文字栏与频谱使用离屏缓冲后整块刷新，避免清屏重绘闪烁。

顶部 dBA 为跟踪峰的频点幅值，左栏大号 dBA 为总 A 计权声级。读数尚未完成声学校准，用于相对比较，不应视为已校准声级计的测量结果。

## 电机转速估算

```text
RPM = 峰值频率（Hz）× 60 ÷ 阶次
```

例如，500 Hz 的峰对应第 10 阶时，转速为 3000 RPM。修改阶次后，顶部 RPM 立即重新计算。

阶次需与电机声学特征或谐波对应；软件不会自动辨识阶次。其他更响的声音也可能成为候选峰。FFT 间隔约为 10.77 Hz，在 10 阶下约为 64.6 RPM/频点；显示平滑不提高实际频率分辨率。

### 峰值防抖

先在 20 Hz～20 kHz 内选取最高有效频点，再进行门限与时间确认：

| 参数 | 当前值 |
| --- | --- |
| 最低峰值幅值 | 20 dBA |
| 首次锁定或切换远处新峰 | 高出附近中位噪声底至少 10 dB |
| 确认条件 | 连续 3 帧候选频率相近，容差 2 个 FFT 频点 |
| 已锁定峰保持门限 | 高出噪声底至少 6 dB |
| 相近频率平滑 | 2 个频点范围内，更新系数 0.25 |
| 有效峰丢失 | 暂时保持；500 ms 后隐藏红线，显示 `--dBA @ --Hz --RPM` |

噪声底为候选峰左右各 32 个频点范围内的中位数，排除峰附近 ±3 个频点，跳过频段外与无效值。参数位于 [src/peak_tracker.h](src/peak_tracker.h)。

当前为局部噪声门限与时间跟踪，尚未实现严格的 `find_peaks` / `prominence`。防抖仅影响标记与读数，频谱和热力图继续实时刷新。

## 信号分析参数

| 参数 | 当前值 |
| --- | --- |
| 采样率 | 44.1 kHz |
| 输入 | 24 位 PCM，以 32 位 I2S 槽接收 |
| FFT | 4096 点，Hann 窗，50% 重叠 |
| 频率间隔 | 44100 / 4096 ≈ 10.77 Hz |
| 幅值 | 单边谱、RMS 缩放、A 计权 |
| 声级偏移 | `kSplOffsetDb = 140`，需按实际麦克风与校准结果调整 |

采集、分析与显示分任务运行，单块队列优先保留最新音频块；显示跟不上时会跳过部分分析块。

## 编译与 USB 烧录

安装 PlatformIO Core 或 VS Code 的 PlatformIO 扩展，在仓库根目录执行：

```sh
pio run -e esp32-s3-devkitc-1
pio device list
pio run -e esp32-s3-devkitc-1 -t upload --upload-port COM11
```

`COM11` 是当前测试端口，请按实际结果替换。无法自动进入下载模式时，按住 BOOT、按一下 RST，再释放 BOOT 后重试。

Windows PowerShell 若出现进度条编码错误，先设置：

```powershell
$env:PYTHONIOENCODING = 'utf-8'
$env:PYTHONUTF8 = '1'
pio run -e esp32-s3-devkitc-1 -t upload --upload-port COM11
```

依赖由 [platformio.ini](platformio.ini) 管理：arduinoFFT `^2.0.2` 与 LovyanGFX Git 标签 `1.2.21`，不依赖其他本地工程的绝对路径。

构建结果位于 `.pio/build/esp32-s3-devkitc-1/`，包括 `firmware.bin` 与合并固件 `firmware.factory.bin`。`.pio/`、`.cache/`、编译数据库与自动生成的 VS Code 配置不作为跨机器配置提交。

**网页刷机状态**：尚未实现 GitHub Pages 网页刷机；后续可由静态网页托管固件，通过电脑浏览器与 USB 烧录。当前也没有 Wi-Fi OTA 更新功能。

## Python 采集工具（可选）

只使用触摸屏时不需要 Python。PC 端采集准备：

```sh
python -m pip install -r python/requirements.txt
python python/ad9833_stream.py --list
python python/ad9833_stream.py --simulate --stats
```

固件默认 `USB_STREAM_ENABLE = false`，优先保证屏幕刷新。读取真实音频前，将 [src/main.cpp](src/main.cpp) 中该开关改为 `true`，重新编译烧录，再执行：

```sh
python python/ad9833_stream.py --port COM11 --stats
python python/ad9833_stream.py --port COM11 --tdms --tdms-interval 5 --tdms-duration 3
```

协议为版本 3，32 字节帧头加有符号 24 位小端 PCM。开启 USB 数据流可能影响采集与显示实时性。

`python/usb2406_stream.py` 面向独立 USB2406 采集卡，不是板载麦克风路径。厂商 64 位 `USB2406.dll` 可放在脚本旁或仓库 `SDK/dll64/`，也可用 `USB2406_DLL_PATH` 环境变量或 `--dll-path` 指定；仿真无需 DLL。

## 源码导航

| 文件 | 职责 |
| --- | --- |
| [src/main.cpp](src/main.cpp) | ES8311 初始化、I2S 采集、任务调度与可选 USB 流 |
| [src/spectrum.cpp](src/spectrum.cpp) | FFT、A 计权与总声级计算 |
| [src/display.cpp](src/display.cpp) | 布局、填充、热力图、标记与触摸交互 |
| [src/peak_tracker.h](src/peak_tracker.h) | 门限、确认、平滑与失锁处理 |
| [test/peak_tracker_test.cpp](test/peak_tracker_test.cpp) | 防抖测试入口 `runPeakTests()` |
| [python/ad9833_stream.py](python/ad9833_stream.py) | USB 接收、仿真与 TDMS 保存 |
| [python/usb2406_stream.py](python/usb2406_stream.py) | USB2406 采集与 TDMS 保存 |

防抖测试覆盖噪声拒绝、连续确认、单帧杂峰、保持门限、超时清空、新峰重锁与计时器回绕；测试入口尚未接入 PlatformIO 自动测试流程。
