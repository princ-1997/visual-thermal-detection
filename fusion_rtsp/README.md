# 热成像与可见光融合 Fusion RTSP

热成像与可见光相机配准融合，支持 **RTSP 视频流** 和 **拍照** 两种模式，通过不同命令区分。

---

## 功能概览

| 命令 | 功能 |
|------|------|
| `fusion_rtsp` | RTSP 视频流：热成像与可见光融合后推流 |
| `fusion_snap` | 拍照：可见光、热成像各拍一张并保存（串行流程） |

---

## 一、RTSP 视频流 (`fusion_rtsp`)

将热成像与可见光配准融合后，通过 RTSP 推送 H.264 视频流。可见光画面会裁剪为中心 1/9 区域（3×3 等分取第 2 行第 2 列）后输出。

### 用法

```bash
./fusion_rtsp <thermal_config> [options]
```

### 参数

| 参数 | 说明 | 默认值 |
|------|------|--------|
| `-p <port>` | RTSP 端口 | 8554 |
| `-u <path>` | RTSP URL 路径 | /fusion |
| `-v <index>` | 可见光相机索引（如 23 → /dev/video23） | 23 |
| `-W <width>` | 可见光宽度 | 1920 |
| `-H <height>` | 可见光高度 | 1080 |
| `-f <fps>` | 双摄像头帧率 | 10 |
| `-c <file>` | 融合配置文件（mode、alpha、canny） | - |
| `-h` | 帮助 | - |

### 融合模式

在 `-c` 指定的 JSON 中配置：

- **edge_overlay**（默认）：可见光打底，热成像 Canny 边缘红色叠加
- **alpha_blend**：可见光与热成像按 alpha 权重混合

### 示例

```bash
# 默认参数启动
./fusion_rtsp stream_RV1126.conf

# 指定端口和路径
./fusion_rtsp stream_RV1126.conf -p 8554 -u /fusion 

# Alpha 叠加模式
./fusion_rtsp stream_RV1126.conf -c fusion_alpha.conf -p 8554 -u /fusion

# 播放地址
# rtsp://<板子IP>:8554/fusion
```

---

## 二、拍照 (`fusion_snap`)

可见光、热成像各拍一张并保存，串行执行。

### 用法

```bash
./fusion_snap <thermal_config> [options]
```

### 参数

| 参数 | 说明 | 默认值 |
|------|------|--------|
| `-o <dir>` | 输出目录 | 当前目录 |
| `-v <index>` | 可见光相机索引 | 23 |
| `-W <width>` | 可见光宽度 | 1920 |
| `-H <height>` | 可见光高度 | 1080 |
| `-h` | 帮助 | - |

### 输出文件

- `visible.jpg`：可见光图像
- `thermal.jpg`：热成像图像（BGR 伪彩）

### 示例

```bash
# 当前目录保存
./fusion_snap stream_RV1126.conf

# 指定输出目录
./fusion_snap stream_RV1126.conf -o ./snap

# 指定可见光相机
./fusion_snap stream_RV1126.conf -o ./snap -v 23
```

---

## 编译

```bash
# 编译（生成 fusion_rtsp + fusion_snap）
./build.sh

# 清理
./build.sh clear
```

### 环境变量（交叉编译）

- `SYSROOT`：目标板根文件系统（含 OpenCV）
- `EASYEAI_API_DIR`：easyeai-api 路径（未设置时自动检测相对路径及 `/opt/EASY-EAI-Toolkit/...`）
- `IR_SDK_LIBS_DIR`：IR SDK .so 目录（默认 ../thermal_rtsp/libs）

若提示「找不到 easyeai-api」，请执行：
```bash
export EASYEAI_API_DIR=/opt/EASY-EAI-Toolkit/EASY-EAI-Toolkit-1126B/easyeai-api
./build.sh
```

---

## 目录结构

```
fusion_rtsp/
├── README.md           # 本文档
├── CMakeLists.txt
├── build.sh
├── stream_RV1126.conf  # 热成像配置
├── fusion.conf         # 融合配置（edge_overlay）
├── fusion_alpha.conf   # 融合配置（alpha_blend）
├── include/
│   └── fusion_config.h
└── src/
    ├── main.cpp        # fusion_rtsp 入口
    ├── main_snap.cpp   # fusion_snap 入口
    ├── fusion_streamer.cpp
    ├── fusion_streamer.h
    └── camera/
        ├── camera.h
        └── mipi_camera.c
```

---

## 依赖

- 热成像 IR SDK（libircam、libiruvc、libirv4l2 等）
- GStreamer（RTSP、H.264 编码）
- OpenCV
- RGA（RK 平台）
