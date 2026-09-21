# PoseBridge

A Rust BLE/USB orientation bridge for WIT sensors, with OSC output, a CLI, and an experimental C ABI.

读取第三方姿态传感器的数据，转换为统一姿态并通过本机 OSC 转发，也可作为 C 动态库嵌入其他程序。
首版支持维特 **BWT901BLECL5.0**，目标平台为 macOS Apple Silicon 和 Windows x64。

## 功能

- BLE 无线与 USB 串口采集，共用数据解析和姿态转换。
- 设备扫描、只读配置检查、显式设备控制与断线重连。
- 四元数／欧拉角 OSC 输出、时间戳、来源描述与独立状态心跳，以及无设备模拟器。
- Rust 核心库、CLI、实验性 C ABI 0.4（版本 400），以及可运行的 C11 消费示例。

当前 OSC 协议为 3，C ABI 为 400，本地 JSON 快照 schema 为 4。快照提供查询时的 `age_ns`，方便消费方设置时效阈值。
只支持当前接口；旧地址、旧结构和 `--osc-version` 已移除，C 调用方须同步升级头文件与动态库。

Rust 0.5 新增[同帧运动批次接口](docs/motion.md)：物理四元数、角速度、加速度与设备时间戳；C ABI 400、OSC 3 及 JSON schema 4 保持原布局。

PoseBridge 使用设备提供的姿态解算结果，回正、用户侧平滑和音频渲染由接收软件负责。
macOS 与 Windows 的原生检查、BLE／USB 实机读取均已通过，详细结果与待验证项见[验证记录](docs/validation.md)。

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
- [时间戳与下游接口](docs/timestamps.md)：显式启用、时钟代次和跨进程时间边界。
- [C ABI](docs/c-api.md)：生命周期、配置和快照轮询。
- [C11 消费示例](docs/consumer.md)：无需设备运行，处理年龄、去重、过期和参考变化。
- [验证记录](docs/validation.md)：已执行检查、实机结果和待验证项。

## 许可

[MIT](LICENSE) © 2026 SakuzyPeng。
