# Firmware

本目录存放GalvoBridge-ESP32的固件及源码，硬件平台：ESP32-S3（Arduino 框架）。构建系统：PlatformIO。如要自行修改源码，请先在VSCode安装插件PlatformIO。PlatformIO会自动识别工程并下载依赖库，随后即可编译工程。

## 工程简介

功能：把 USB 串口发来的点序列实时转为两路模拟电压，驱动振镜在 XY 平面画图。

`src/` 下 4 个模块：

- `main.cpp` 使用硬件定时器通知高优先级任务向 DAC 输出坐标。
- `DAC8562.{h,cpp}` 驱动双通道 DAC。
- `GalvoParser.{h,cpp}` 处理 USB CDC 上的 v3 报文、点流队列、反馈调速和逐点消隐。
- `LaserCtrl.{h,cpp}` 控制 GPIO3 光源信号。上电默认关光，由上位机发出开光请求。

该源码配套 GalvanometerMonitor v3 上位机。Avatar 和测试工具以 40 FPS 发送 600 点报文，固件按 FIFO 消费入队点列，基准点速为 24,000 点/秒。上位机握手时指定入队后的队列目标，默认 900 点，固件公布 3,000 点的目标上限。自适应调速按每次成功入队后的余量反馈，点速范围为 23,520–24,480 点/秒。队列耗尽时保持末点坐标并关光。点流恢复后直接按新点的消隐位输出。使用 `pio run` 编译 `esp32-s3-fh4r2` 固件。逐点消隐和扫描速度应先接假负载验证。

## 旧版预编译固件

下列预编译文件属于旧协议，运行 v3 上位机需重新编译本目录的源码。使用`5.Others/Tool`下的`flash_download_tool`工具烧录`2.Firmware/firmware`下的三个`bin`文件，具体配置截图可在`Tool`文件夹下找到。另外可参考官方的烧录步骤[Flash 下载工具用户指南 - ESP32 - — ESP 测试工具 latest 文档](https://docs.espressif.com/projects/esp-test-tools/zh_CN/latest/esp32/production_stage/tools/flash_download_tool.html#id4)


