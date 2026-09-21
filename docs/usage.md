# PoseBridge 使用指南

PoseBridge 0.5 为维特 BWT901BLECL5.0 提供 BLE／USB 采集、安装转换、当前 OSC 协议和实验 C ABI。
只保留一套当前接口；旧版调用方须同步升级。MacinRender 提供原生接收器，GUI 尚未适配。

## 构建

Rust 1.96 或兼容更新版本，复用全局缓存与唯一 target 目录：

```sh
cargo build --workspace --release --locked
python3 scripts/export_header.py
```

CLI 为 target/release/posebridge（Windows 加 .exe）。动态库为 libposebridge_capi.dylib／posebridge_capi.dll，
头文件 include/posebridge.h；Windows 导入库 posebridge_capi.dll.lib。开发与 Release 均关闭增量编译和调试信息。
macOS 动态库使用 @rpath install name。Windows 原生验证在 MSVC 开发者环境执行。

## 枚举与读取

```sh
posebridge scan --transport usb --json
posebridge scan --transport ble --timeout-seconds 5 --json
posebridge diagnose --transport usb --port /dev/cu.usbserial-210 --duration 10 --json
posebridge diagnose --transport ble --device "扫描返回的标识" --duration 10 --json
```

BLE 由程序连接，无需先在系统蓝牙面板常规配对。macOS 须允许宿主使用蓝牙；嵌入 C ABI 的宿主自行提供权限声明。
Mac 使用不透明平台 UUID，Windows 可显示蓝牙地址；广播名称不能代替标识。
USB 115200、8N1、无流控，使用扫描得到的端口，不把示例路径当作固定端口。

CLI／C ABI 的本地 JSON 是 schema=4 快照：descriptor、status、pose、operation。
pose.age_ns 是接收后到查询时的经过纳秒；所有 64 位标识/时间/计数都是十进制字符串。OSC 心跳继续使用 schema=3。
未提供诊断安装映射时使用传感器单位基底，并显示未验收提示。普通读取不更改设备设置。

## 设备检查与控制

```sh
posebridge inspect --transport usb --port /dev/cu.usbserial-210 --json
posebridge configure --transport usb --port /dev/cu.usbserial-210 --json rate --hz 100
posebridge configure --transport usb --port /dev/cu.usbserial-210 output --format timestamp-gyro-quaternion
posebridge configure --transport usb --port /dev/cu.usbserial-210 algorithm --mode six-axis
```

inspect 只读 CALSW、回传率、输出格式、带宽、设备安装方向、算法与版本。实际读取只在停止采集时进行。
C ABI 快照查询可在采集期间返回观察时间与有效性明确的缓存，不产生后台寄存器轮询。
软件支持命令、设备回读值、应用安装映射与校准质量分别报告；未确认项为 null。

| configure 子命令 | 行为／保存边界 |
|---|---|
| rate --hz N | 支持 1/2/5/10/20/50/100/200；回读核验，不保存 |
| output --format NAME | motion、timestamp-euler、timestamp-quaternion、timestamp-gyro-quaternion；检查新协议特征，回读，不保存 |
| algorithm --mode six-axis/nine-axis | 明确切换算法，回读，不保存 |
| zero-yaw | 当前须为六轴；不隐式切模式，不保存；发送与实际归零效果分开报告 |
| accel-calibrate | 观察 CALSW 启动／完成；没观察到启动则 unverified，不自动重发或保存 |
| mag-start / mag-stop | 明确开始／结束磁场校准，回读；精度未据此验收，不自动保存 |
| angle-reference | 设置设备角度参考并按官方流程发送 SAVE，效果与掉电持久化需独立验证 |
| reset-defaults | 恢复默认并保存；回读已知默认速率、输出、算法，不声称验证全部校准系数或掉电持久化 |
| save | 显式保存当前设置，不能用 SAVE 寄存器自清零证明掉电持久化 |

设备操作必须在采集停止后执行，忙时返回 Busy；完成后由调用方显式重新启动。跨进程 CLI 也须先停止占用设备的 bridge。
指令解锁后等待 200 ms；适用的设置写入后再等待 100 ms。读回在 3 秒预算内每 250 ms 重试只读请求，
允许过渡旧回复，但不会重写设置。超时不证明写入没发生。
校准、算法、参考或安装映射变化，以及写入后结果不确定时，更新参考代次／原因。回正由下游处理。
只读查询和一般测试不会自动调用校准、参考保存或恢复默认。

## OSC 与安装

```sh
posebridge bridge --source-id headset --transport usb --port /dev/cu.usbserial-210 --mount=-y,+x,+z
posebridge bridge --source-id headset --transport ble --device "平台标识" --mount=-y,+x,+z \
  --pose-input stream-quaternion --osc-rate-hz 100
```

--mount 三个轴依次对应头部右、前、上，要求右手基底。示例映射需要佩戴后的三轴动作确认。
原生四元数通知使用 stream-quaternion，需先明确配置相应输出；quaternion 是最高请求节奏 50 Hz 的独立寄存器读取。
默认 OSC 目标 127.0.0.1:9000、四元数、上限目标 100 Hz。另有 --format euler、--osc-target 和 --osc-rate-hz。
姿态、info、status 的定义见[协议](protocol.md)。默认来源名仅在本机有效；多个逻辑来源使用不同 --source-id。

心跳和姿态独立；500 ms 无有效采样停止姿态发送。source age 字段是源主机停留时间，不是总延迟。
断线后以 1、2、4、8 秒重连原标识，成功产生新会话／参考代次；权限和协议错误终止。
Ctrl+C 正常清理连接，并尽力发终态。停止后保留最后姿态，fresh=false。

Windows 可选 --ble-mode throughput（Windows 11+），只在连接期间申请系统偏好；macOS 拒绝该值。
Windows 高频模拟／OSC 期间申请 1 ms 定时精度，退出释放；这不能保证 USB/BLE 交付率。

```sh
posebridge simulate --source-id demo --pattern combined --sample-clock --duration 3
python3 scripts/measure_osc.py -- simulate --sample-rate-hz 200 --osc-rate-hz 200 --duration 10
```

模拟支持 fixed/yaw/combined/wrap，synthetic 时间明确标为 kind=2。测量脚本只统计姿态包，独立忽略合法状态心跳，
区分源采样率与实际 OSC 率，不把合并采样算成网络丢包。

## 检查

```sh
cargo fmt --all --check
cargo clippy --workspace --all-targets --release --locked -- -D warnings
cargo test --workspace --release --locked
python3 tests/osc_cli_smoke.py
python3 scripts/check_c_api.py
```

`check_c_api.py` 同时编译并运行 C11 消费示例及其测试，将可运行示例保留在 target/release/pose_consumer（Windows 加 .exe）。
默认示例使用模拟器，不需要设备，见[接入说明](consumer.md)。

tests/hardware_smoke.py 默认只读检查与采集；显式 --exercise-config 才临时切换可恢复配置并在 finally 恢复。
--expect-reconnect 用于用户配合的物理拔插，不能与配置切换同时使用。脚本不会校准、归零、SAVE 或恢复默认。
详细状态见[验证记录](validation.md)，嵌入规则见 [C ABI](c-api.md)。
