# PoseBridge 使用指南

读取第三方姿态传感器，通过本机 OSC 转发，或通过实验性 C ABI 嵌入其他程序。
PoseBridge 不实现自己的惯性融合算法，不代表维特设备厂商，也不负责音频渲染。

首版设备：**维特 BWT901BLECL5.0**。提供 BLE 与 USB 串口输入、Rust CLI、C 动态库、
硬件无关模拟器和显式设备配置。首批目标为 macOS Apple Silicon、Windows x64。

macOS 的 BLE 角度通知、四元数寄存器读取和 USB 串口已经用实物验证；Windows 编译及真机状态见
[验证记录](validation.md)。MacinRender OSC 接收端尚需单独实现；成功发送 OSC 不等于音频软件已消费。

## 构建

使用 Rust 1.96 或更新的兼容稳定版，复用全局 Cargo 缓存和本仓库唯一的 `target/` 目录：

```sh
cargo build --workspace --release --locked
python3 scripts/export_header.py
```

| 产物 | macOS | Windows |
|---|---|---|
| CLI | `target/release/posebridge` | `target/release/posebridge.exe` |
| C 动态库 | `target/release/libposebridge.dylib` | `target/release/posebridge.dll` |
| C 头文件 | `include/posebridge.h` | `include/posebridge.h` |
| MSVC 导入库 | — | `target/release/posebridge.dll.lib` |

开发、测试和 Release 配置均关闭增量编译及调试信息。macOS 发行库不额外执行符号剥离：
当前工具链的 strip 曾使 Mach-O 字符串表不满足 Apple linker 的对齐要求。
动态库 install name 使用 `@rpath/libposebridge.dylib`，宿主需设置其库搜索路径。

## 先确认能读取数据

BLE 由程序内扫描和连接，不要求先在系统蓝牙面板配对。保持设备开机，并断开可能占用它的手机 App。
macOS 首次运行需允许宿主终端使用蓝牙；CLI 嵌入蓝牙用途说明。C ABI 宿主自行负责权限与应用声明。

```sh
./target/release/posebridge scan --transport ble --timeout-seconds 6
./target/release/posebridge diagnose --transport ble --device "扫描得到的设备标识" --duration 10
```

设备广播通常以 `WT` 开头，例如 `WT901BLE68`。Mac 的设备标识是不透明 UUID，不能用名称或蓝牙 MAC 地址代替。
`scan` 仅枚举；`diagnose` 默认读取连续角度通知，不发送校准、速率设置或保存命令。

USB 接线后运行：

```sh
./target/release/posebridge scan --transport usb
./target/release/posebridge diagnose --transport usb --port /dev/cu.usbserial-110 --duration 10
```

端口路径以枚举结果为准，Windows 使用 `--port COM3` 等实际端口名。默认 115200、8N1、无流控。
枚举仅显示被系统识别为 USB 的串口；驱动没有提供 USB 元数据时，可以显式指定已知端口。
此型号 USB 实测使用与 BLE 相同的 **20 字节协议**，不使用其他 WIT 型号的 11 字节帧。

诊断输出含原始 XYZ 角度、转换后 yaw/pitch/roll、实际姿态率及连接状态。
未指定安装映射时，诊断使用传感器 XYZ 的单位基底并明确提示；这不是已校准的头部朝向。
`--json` 输出 NDJSON，便于保存和脚本分析。`--duration 0` 持续运行至 Ctrl+C。

可选的设备四元数读取：

```sh
./target/release/posebridge diagnose --transport ble --device "设备标识" --pose-input quaternion --duration 10 --json
```

此模式发送只读寄存器请求，每次最多一个未完成请求，最高请求节奏为 50 Hz；实际返回率单独统计。
它不是 200 Hz 推送，不会把旧四元数随新角度包重发来提高计数。

## OSC 桥接

桥接要求显式安装映射：三个有符号轴依次表示**头部右、前、上方向对应的传感器轴**，
必须构成右手基底。`-y,+x,+z` 表示右为传感器 -Y、前为 +X、上为 +Z；适用于标签朝上、X 朝前的候选安装。
该模板的实际设备轴向和佩戴结果仍需按三轴动作确认。

```sh
./target/release/posebridge bridge --transport ble --device "设备标识" --mount=-y,+x,+z
./target/release/posebridge bridge --transport usb --port /dev/cu.usbserial-110 --mount=-y,+x,+z
```

默认目标 `127.0.0.1:9000`，四元数输出，100 Hz 为 **OSC 发送上限目标**，不修改设备回传率。
可使用 `--osc-target 127.0.0.1:9001 --osc-rate-hz 50 --format euler`。
输入不足 100 Hz 时维持真实新采样速率；500 ms 无有效姿态后标为 stale，停止发送陈旧姿态。
断开后按 1、2、4、8 秒等待重连原设备；权限或协议错误停止重试。Ctrl+C 释放连接。

| OSC 地址 | 参数 |
|---|---|
| `/posebridge/v1/quaternion` | `,ffff`：x、y、z、w |
| `/posebridge/v1/euler` | `,fff`：yaw、pitch、roll，单位度 |

仅支持本机回环目标。回正和用户侧平滑交给接收软件，PoseBridge 不重复施加。
这些消息是明确的 PoseBridge 姿态约定，不是任何支持 OSC 的软件都能直接识别；详见[协议与坐标](protocol.md)。

## 无设备模拟

```sh
./target/release/posebridge simulate --yaw 30 --pitch 20 --roll 10 --duration 3
./target/release/posebridge simulate --pattern wrap --format euler --duration 3
```

轨迹支持 `fixed`、`yaw`、`combined`、`wrap`，`--sample-rate-hz` 控制模拟新采样率。
模拟器的坐标已是输出姿态约定，不再应用设备安装映射。

## 显式设备配置

普通采集不修改设备配置。以下命令会写设备寄存器，并读取返回值检查；没有隐式保存至 Flash：

```sh
./target/release/posebridge configure --transport ble --device "设备标识" rate --hz 100
./target/release/posebridge configure --transport usb --port /dev/cu.usbserial-110 rate --hz 50
```

支持 1、2、5、10、20、50、100、200 Hz；125 Hz 属于其他型号的配置，不接受。
寄存器回读匹配只证明配置值匹配，实际通知率仍须用 `diagnose` 测量。

同一入口还提供 `accel-calibrate`、`mag-start`、`mag-stop`、`save`。
加速度校准按厂商要求静置；磁场校准由用户按厂商说明执行转动，再显式结束。
加速度校准需观察到 CALSW 开始与完成，否则报告未验证；保存命令不会将寄存器回读冒充掉电持久化验证。
固件升级不在本工具范围内。

## 库与开发检查

Rust 使用 `posebridge_core::Controller`，CLI 和 C ABI 共用它的配置、采集和生命周期实现。
C ABI 以不透明 `PbContext` 和最新快照轮询工作，详见 [C ABI 文档](c-api.md)及
[真实 C 调用测试](../tests/c_api_smoke.c)。

```sh
cargo fmt --all --check
cargo clippy --workspace --all-targets --release --locked -- -D warnings
cargo test --workspace --release --locked
python3 tests/osc_cli_smoke.py
python3 scripts/check_c_api.py
```

macOS C ABI 检查：

```sh
cc -std=c11 -Iinclude tests/c_api_smoke.c -Ltarget/release -lposebridge \
  -Wl,-rpath,"$PWD/target/release" -o target/release/c_api_smoke
target/release/c_api_smoke
```

Windows 在 MSVC 开发者命令提示符中构建 Rust 项目、导出头文件后执行：

```bat
cl /nologo /W4 /Iinclude tests\c_api_smoke.c /Fetarget\release\c_api_smoke.exe /Fotarget\release\c_api_smoke.obj /link target\release\posebridge.dll.lib
target\release\c_api_smoke.exe
```

仓库包含 macOS／Windows 原生 CI 配置，但本地创建仓库不代表 CI 已运行。
