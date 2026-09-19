# PoseBridge

A Rust BLE/USB orientation bridge for WIT sensors, with OSC output, a CLI, and an experimental C ABI.

读取第三方姿态传感器的数据，转换为统一姿态并通过本机 OSC 转发，也可作为 C 动态库嵌入其他程序。
首版支持维特 **BWT901BLECL5.0**，目标平台为 macOS Apple Silicon 和 Windows x64。

## 功能

- BLE 无线与 USB 串口采集，共用数据解析和姿态转换。
- 设备扫描、实时诊断、显式参数配置与断线重连。
- 四元数／欧拉角 OSC 输出，以及无设备模拟器。
- Rust 核心库、CLI、实验性 C ABI。

PoseBridge 使用设备提供的姿态解算结果，回正、用户侧平滑和音频渲染由接收软件负责。
macOS BLE／USB 已通过实机读取验证，其他平台与功能的验收状态见[验证记录](docs/validation.md)。

## 快速开始

需要 Rust 1.96 或更新的兼容稳定版。

```sh
cargo build --workspace --release --locked
./target/release/posebridge scan --transport ble
./target/release/posebridge diagnose --transport ble --device "扫描得到的设备标识" --duration 10
```

USB 使用 `scan --transport usb` 枚举端口，再通过 `diagnose --transport usb --port "端口名"` 读取。
Windows 可执行文件为 `target/release/posebridge.exe`。
普通读取不会修改设备回传率、执行校准或保存参数。

## 文档

- [完整使用指南](docs/usage.md)：构建、BLE／USB、安装映射、OSC 和设备配置。
- [协议与坐标](docs/protocol.md)：输入帧、旋转约定、OSC 消息与数据时效。
- [C ABI](docs/c-api.md)：生命周期、配置和快照轮询。
- [验证记录](docs/validation.md)：已执行检查、实机结果和待验证项。

## 许可

[MIT](LICENSE) © 2026 SakuzyPeng。
