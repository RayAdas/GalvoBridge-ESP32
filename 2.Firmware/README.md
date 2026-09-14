# Firmware

本目录存放GalvoBridge-ESP32的固件及源码，硬件平台：ESP32-S3（Arduino 框架）。构建系统：PlatformIO。如要自行修改源码，请先在VSCode安装插件PlatformIO。PlatformIO会自动识别工程并下载依赖库，随后即可编译工程。

## 工程简介

功能：把 USB 串口发来的点序列实时转为两路模拟电压，驱动振镜在 XY 平面画图。

`src/` 下 4 个模块：

- `main.cpp`：初始化 + 25 kHz 硬件定时器循环从播放缓冲区取点送 DAC。
- `DAC8562.{h,cpp}`：DAC8562 双通道 16-bit DAC 的 SPI 驱动（引脚在 `.h` 顶部：MOSI=4、SCK=5、CS=6）。
- `GalvoParser.{h,cpp}`：串口协议解析 + A/B 双缓冲（解析任务写一块，DAC ISR 读另一块，避免撕裂）。
- `LaserCtrl.{h,cpp}`：激光 TTL 控制。`GPIO3` 输出，`GPIO0` 下降沿中断翻转使能。

串口协议（默认 921600 bps，大端）：

```
FF FF <N_hi> <N_lo> <X_hi> <X_lo> <Y_hi> <Y_low> × N
```

帧头 `0xFF 0xFF`；`N` 为本帧点数（`GalvoParser.h` 中 `MAX_POINTS_PER_FRAME = 25000`）；每个点 `X`、`Y` 各 2 字节 16-bit 无符号，对应 DAC 输出 0 ~ 65535。


## 如果想直接烧录固件

使用`5.Others/Tool`下的`flash_download_tool`工具烧录`2.Firmware/firmware`下的三个`bin`文件，具体配置截图可在`Tool`文件夹下找到。另外可参考官方的烧录步骤[Flash 下载工具用户指南 - ESP32 - — ESP 测试工具 latest 文档](https://docs.espressif.com/projects/esp-test-tools/zh_CN/latest/esp32/production_stage/tools/flash_download_tool.html#id4)


