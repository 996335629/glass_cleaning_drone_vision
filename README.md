# 玻璃清洁无人机视觉系统

基于 Orbbec Gemini 336 深度相机的光流速度估计系统，适配地瓜派 X5（aarch64）实机部署。

---

## 系统概述

```
Orbbec Gemini 336
  ├── 彩色帧 → 光流估计（LK 稀疏特征点）→ 卡尔曼滤波 → 速度
  ├── 深度帧 → 像素速度 → 真实速度（m/s）
  └── IMU   → 姿态解算（互补滤波）

幕墙竖框检测（Canny + HoughP + 聚类）
  └── 输出：竖框数量 + 横向偏移（mm）

UART 输出（速度帧，26 bytes）→ 飞控
```

**核心输出**：每帧通过 UART 发送 `VelocityData` 数据包，包含：
- 三轴速度（相机坐标系，m/s）
- 光流置信度
- 深度有效标志
- 幕墙竖框数量及横向偏移

---

## 硬件要求

| 组件 | 规格 |
|------|------|
| 深度相机 | Orbbec Gemini 336 |
| 部署平台 | 地瓜派 X5（aarch64 Linux）|
| 调试平台 | Windows 10/11 x64（开发用）|
| UART | 飞控串口，推荐 115200 或 460800 baud |

---

## 目录结构

```
glass_cleaning_drone_vision/
├── config/
│   └── system_config.yaml      # 主配置文件（所有运行参数）
├── include/
│   ├── config/system_config_loader.h
│   ├── camera/gemini_camera.h
│   ├── communication/protocol.h
│   ├── communication/uart_interface.h
│   ├── detection/window_frame_detector.h
│   └── ...
├── src/
│   ├── main.cpp
│   ├── config/system_config_loader.cpp
│   └── ...
├── build_x5.sh                 # X5 构建脚本
├── rebuild.bat                 # Windows 重新构建脚本
└── CMakeLists.txt
```

---

## 构建

### X5（地瓜派 aarch64 实机）

**前提**：X5 上已安装 OrbbecSDK，路径默认为：

```
/root/OrbbecSDK_C_C++_v1.10.27_20250925_0549823_linux_arm64_release/OrbbecSDK_v1.10.27/SDK
```

若 SDK 路径不同，编辑 `build_x5.sh` 第 10 行的 `ORBBEC_SDK_ROOT`。

> `build_x5.sh` 已强制关闭可视化窗口（`-DENABLE_DEBUG_VISUALIZER=OFF`），
> 运行时仍需在 YAML 中将 `enable_visualization` 设为 `false`。

---

## 配置文件

所有运行参数集中在 `config/system_config.yaml`，修改后无需重新编译。

### system（系统）

```yaml
system:
  mode: camera                   # camera=相机模式 / simulation=仿真模式（需编译时开启）
  enable_visualization: true     # true=显示调试窗口；false=无头运行（X5实机必须设为false）
  enable_reflection_filter: false  # 是否默认开启反射滤波（运行时可按F切换）
```

> **X5 实机部署**：务必将 `enable_visualization` 改为 `false`，否则程序会因找不到 display 而崩溃。

### process（处理分辨率）

```yaml
process:
  width: 320       # 光流计算分辨率宽（像素）。越小越快，越大越精确
  height: 240      # 光流计算分辨率高
  max_features: 40 # 最大特征点数。建议 20~80
```

常用分辨率组合：

| 分辨率 | 适用场景 |
|--------|----------|
| 160×120 | X5 高帧率优先（>30fps） |
| 320×240 | 默认，平衡精度与速度 |
| 640×480 | 调试精度验证 |

### camera（相机采集）

```yaml
camera:
  type: gemini336
  color_width: 640      # 相机彩色流分辨率宽
  color_height: 480     # 相机彩色流分辨率高
  depth_width: 640      # 深度流分辨率宽
  depth_height: 480     # 深度流分辨率高
  fps: 30               # 采集帧率
  enable_depth: true    # 是否开启深度流（关闭后无法换算真实速度）
  enable_color: true    # 是否开启彩色流
  align_depth_to_color: true  # 深度对齐到彩色坐标系
```

### detection（幕墙竖框检测）

```yaml
detection:
  canny_low: 30.0       # Canny 边缘检测低阈值。光线暗时适当降低（如 20）
  canny_high: 100.0     # Canny 边缘检测高阈值。反差强时可调高（如 150）
  hough_min_len: 60     # HoughP 最小线段长度（像素）。值越大过滤越严
  hough_max_gap: 15     # HoughP 允许的最大线段间隔（像素）
  cluster_dist: 25      # 聚类合并距离（像素）。越小竖框分得越细
  vertical_angle: 15.0  # 判定为竖线的最大偏角（度）。值越大容忍越大的倾斜
```

**调参建议**：

- 玻璃反光强 → 适当调高 `canny_low`（减少噪点线段）
- 竖框很细或颜色浅 → 降低 `canny_low` / `hough_min_len`
- 竖框被误检多 → 增大 `vertical_angle` 或 `cluster_dist`

### communication（串口通信）

```yaml
communication:
  enable_uart: false     # true=启动时自动开启UART；false=需命令行--port指定
  port: ""               # 串口名。Linux: /dev/ttyUSB0，Windows: COM3
  baudrate: 115200       # 波特率。支持 9600 / 115200 / 460800
```

---

## 命令行用法

```
glass_cleaning_drone_vision [选项]
```

| 选项 | 说明 |
|------|------|
| `--camera, -c` | 相机模式（默认） |
| `--simulation, -s` | 仿真模式（需编译时开启 SIMULATION_MODE） |
| `--config <路径>` | 指定配置文件路径（默认：`config/system_config.yaml`） |
| `--port <串口>` | 指定 UART 串口，覆盖 YAML 中的值 |
| `--baud <波特率>` | 指定波特率，覆盖 YAML 中的值 |
| `--help, -h` | 显示帮助 |

### 常见用法

```bash
# 基础启动（读取默认配置文件）
./glass_cleaning_drone_vision

# 使用 UART 发送速度数据
./glass_cleaning_drone_vision --port /dev/ttyUSB0 --baud 115200

# 指定配置文件（X5 无头模式）
./glass_cleaning_drone_vision --config /root/glass_cleaning_drone_vision/config/x5_headless.yaml

# --port 覆盖 YAML 中的串口设置
./glass_cleaning_drone_vision --config config/system_config.yaml --port /dev/ttyUSB1

# 仿真模式（仅调试时）
./glass_cleaning_drone_vision --simulation
```

### X5 实机部署推荐启动命令

```bash
cd /root/glass_cleaning_drone_vision
./build_x5/glass_cleaning_drone_vision \
    --config config/system_config.yaml \
    --port /dev/ttyUSB0 \
    --baud 115200
```

确保 `config/system_config.yaml` 中已设置 `enable_visualization: false`。

---

## 运行时键盘控制（调试模式 enable_visualization: true）

| 按键 | 功能 |
|------|------|
| `F` | 开/关反射滤波 |
| `D` | 开/关深度图叠加显示 |
| `R` | 重置光流估计器和卡尔曼滤波器 |
| `1` | 切换处理分辨率 160×120 |
| `2` | 切换处理分辨率 320×240 |
| `3` | 切换处理分辨率 640×480 |
| `ESC` | 退出 |

---

## UART 协议

### 帧格式

```
字节位置:  [0]   [1]   [2]    [3]    [4 .. 4+len-1]   [4+len]  [4+len+1]  [4+len+2]  [4+len+3]
内容:      0xAA  0x55  len    type   payload           CRC16_H  CRC16_L    0x55       0xAA
```

- `len`：payload 字节数
- `type`：帧类型（见下表）
- `CRC16`：对 `[type][payload]` 计算 CRC-16/CCITT，高字节在前

### 帧类型

| type | 值 | 说明 |
|------|----|------|
| DISPLACEMENT | 0x01 | 位移帧（实验性） |
| VELOCITY     | 0x02 | **速度帧（主要输出）** |
| STATUS       | 0x03 | 状态帧 |
| HEARTBEAT    | 0x04 | 心跳帧 |

### 速度帧 VelocityData（26 bytes payload）

| 字段 | 类型 | 字节数 | 说明 |
|------|------|--------|------|
| `vx` | float | 4 | X 轴速度（m/s），向右为正 |
| `vy` | float | 4 | Y 轴速度（m/s），向下为正 |
| `vz` | float | 4 | Z 轴速度（m/s），靠近玻璃为正 |
| `confidence` | float | 4 | 光流置信度 0.0~1.0 |
| `depth_valid` | uint8 | 1 | 1=深度有效；0=镜面/失效（速度保持上次有效值） |
| `bar_count` | uint8 | 1 | 检测到的幕墙竖框数量（0=未检测到） |
| `bar_offset_mm` | float | 4 | 距最近竖框横向偏移（mm，正=竖框在右），仅 bar_count>0 且 depth_valid=1 时有效 |
| `timestamp_ms` | uint32 | 4 | 时间戳（ms，系统时间） |

> **坐标系说明（相机坐标系）**：
> - vx 向右为正（相机视角）
> - vy 向下为正
> - vz 靠近玻璃为正
>
> **飞控侧需自行完成坐标系转换**（相机坐标系 → 机体坐标系 → NED）。

### 接收示例（Python 伪代码）

```python
import struct, serial

HEADER = b'\xAA\x55'
FOOTER = b'\x55\xAA'

def parse_velocity(payload: bytes):
    vx, vy, vz, conf = struct.unpack_from('<ffff', payload, 0)
    depth_valid, bar_count = struct.unpack_from('<BB', payload, 16)
    bar_offset, = struct.unpack_from('<f', payload, 18)
    ts, = struct.unpack_from('<I', payload, 22)
    return vx, vy, vz, conf, depth_valid, bar_count, bar_offset, ts
```

---

## X5 无头部署检查清单

- [ ] `config/system_config.yaml` 中 `enable_visualization: false`
- [ ] `communication.enable_uart: true`，`port` 填写正确串口（如 `/dev/ttyUSB0`）
- [ ] 相机 USB 已连接，`lsusb` 可见 Orbbec 设备
- [ ] 串口权限：`chmod 666 /dev/ttyUSB0` 或添加用户到 `dialout` 组
- [ ] 构建完成：`build_x5/glass_cleaning_drone_vision` 文件存在
- [ ] 首次运行用 `nohup` 或 `tmux` 保持后台运行：

```bash
# tmux 会话（推荐）
tmux new -s drone_vision
./build_x5/glass_cleaning_drone_vision --port /dev/ttyUSB0
# Ctrl+B, D 分离会话

# 或 nohup 后台
nohup ./build_x5/glass_cleaning_drone_vision --port /dev/ttyUSB0 > vision.log 2>&1 &
```

---

## 常见问题

**Q: 启动报 `No camera found!`**
A: 检查 USB 连接，执行 `lsusb | grep -i orbbec` 确认设备识别。

**Q: 启动报 `[Config] Warning: cannot open ...`**
A: 配置文件路径不存在，程序使用默认参数继续运行。检查 `--config` 路径是否正确。

**Q: UART 报 `Failed to open /dev/ttyUSB0`**
A: 执行 `ls -l /dev/ttyUSB0` 确认设备存在，`chmod 666 /dev/ttyUSB0` 授权。

**Q: confidence 长期为 0**
A: 特征点提取失败，可能原因：玻璃过于均匀/纯色，尝试降低 `process.max_features` 或调整 `detection.canny_low`。

**Q: X5 上 cv::imshow 段错误**
A: 未设置 `enable_visualization: false`，X5 无 display，必须关闭可视化。
