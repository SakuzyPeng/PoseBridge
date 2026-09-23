# 磁场监测与设备校准（0.6）

Rust／C ABI 提供独立磁场会话，供宿主绘制 XYZ 点云和 XY／XZ／YZ 投影。
CLI 可直接监测或执行定时校准。本功能不提供 GUI、几何拟合、覆盖率或精度评分。

## 会话流程

1. 停止原姿态采集，配置 USB／BLE 来源；不要求 mounting。
2. 调用 `Controller::magnetic_start()`／`pb_magnetic_start(ctx)`，异步打开只读监测。
3. 查询磁场批次，等待 `active=true`；启动失败读取 `last_error`，受理不等于连接成功。
4. 显式发送 `mag_start`，等待 `operation.outcome` 结束且 `register_verified=true`，再转动设备。
5. 增量取点并显示基础统计；结束时发送 `mag_stop`，等待 `completion_observed=true`。
6. 结束后继续监测，统计窗口冻结。可单独发送 `save`；掉电持久性仍为 unverified。
7. 调用 `stop`／`pb_stop` 关闭连接；由宿主显式重新启动姿态采集。

磁场会话与姿态、OSC、扫描、检查和其他设备配置互斥，不自动切换模式。
会话内 `configure_device`／`pb_device_command` 只受理 `mag_start`、`mag_stop`、`save`，
一个操作未结束时返回 Busy。操作结果查 `operation.outcome`，不能等待连接状态 complete，
因为操作结束后会话仍保持打开。开始和保存前重新读取 CALSW，只有 idle 才可执行；
结束只接受 CALSW=0/7，不会用磁场命令结束加速度校准。

普通监测只发读请求，不改变速率、输出格式、融合算法或校准参数。打开时发现 CALSW=7，
报告 `external_calibration`；可以显式结束，但关闭监测不会自动结束其他程序启动的校准。
本会话一旦尝试写入开始命令，就承担退出清理责任，即使没读到开始确认。
关闭、销毁及 Ctrl-C 尝试结束，清理总预算 4 秒，绝不自动 SAVE。
退出清理也会重新读取 CALSW；读取失败或当前值不是 0/7 时不写结束命令，并报告清理失败。
`cleanup` 与用户操作分开；失败通过错误返回和 `last_error` 报告，
`device_may_be_calibrating=true` 表示须重新连接检查并显式结束。退出不保证恢复旧校准系数。
BLE 取消订阅超时后仍尝试断连；断连失败／超时也通过停止调用和 `last_error` 报告。
磁场读取超时／断线终止会话，不自动重连或重发开始／保存写入；新会话重新检查状态。

## 样本、单位与统计

- 每 200 ms 请求一次 `0x3A`，同一回包的前三个有符号 int16 为 XYZ，同时仅一个磁场读取在途。
  5 Hz 是常规监测请求频率上限，控制前置检查可能额外读取；实际回包率独立报告，核验期间可能暂时出现间隔。
- 坐标是传感器自身 XYZ，不应用头部安装映射。`register_xyz` 是设备输出，
  不承诺是未经内部补偿的 ADC 值；相同数值的后续回包仍算新的接收观测。
- 首次读 `0x72`，类型 2/3/4/5/6/7 的 µT/count 比例为 0.15、0.013、0.058、0.098、1/120、0.02。
  `field_ut` 不按显示位数舍入；未知类型／类型读取失败保留 counts，µT 及比例为 null，原因在 `type_error`。
- `received_ns` 为本连接接收循环开始后的主机单调时间；`age_ns` 为接收到查询的年龄。
  回包没有设备采样时间，不继承姿态时间，也不制造姿态。所有年龄使用同一查询时刻。
- 年龄严格小于 500 ms 且会话仍运行才 fresh；批次 `active` 描述最新磁场接收的时效。
  空增量批次也会更新状态、最新值和年龄。停止后历史保留但 fresh=false，年龄继续增加。
- 历史容量 256；游标包含 instance_id、session_id、sequence。新会话或未来游标返回 reset；
  `history_overrun` 是未读历史被淘汰的数量，不代表设备丢包。查询不消费历史，可使用多个独立游标。
- 统计包括样本数、时长、实际回包率和三轴 min/max/span（counts）。开始核验成功后创建新窗口，
  结束核验成功／观察到退出后冻结，监测继续返回后续样本。以 `window_id`、`phase` 区分阶段，
  会话内 sequence 不重用。无样本时范围为 null，速率为 0；统计不是精度或完成度判断。
- `calibration_quality` 保持 null。校准控制使配置缓存失效并推进姿态参考代次。

## Rust 与 C ABI

Rust 类型：`MagneticCursor`、`MagneticSample`、`MagneticStatistics`、`MagneticPhase`、`MagneticBatch`。
`controller.magnetic_since(cursor)` 原子复制样本、最新值、统计、状态及操作结果，不执行 I/O。
返回的 `batch.cursor` 作为下次输入，初次为 `None`。控制仍使用已有 `DeviceCommand`。

C ABI 400 的 `PbPose`／`PbStatus` 布局不变，新导出要求 **0.6 或更新动态库**。
通用连接状态新增 11（magnetic）；磁场数据不放入姿态结构，`pb_latest_pose` 返回 PB_NO_DATA。
已有快照仍为 schema 4，磁场查询独立为 schema 1。

```c
int32_t pb_magnetic_start(struct PbContext *ctx);
int32_t pb_magnetic_since_json(const struct PbContext *ctx,
    const char *cursor, uint32_t cursor_len,
    char *buffer, uint32_t capacity, uint32_t *required);
```

首次传 NULL/0 或 JSON `null` 作为游标；后续传批次的 cursor 对象：
`{"instance_id":"…","session_id":"…","sequence":"…"}`。
全部 64 位标识、时间和计数是十进制字符串；游标拒绝数字类型、负数及超出正 int64 范围的值。
先用 buffer=NULL、capacity=0 查询所需容量（含 NUL），返回 PB_BUFFER_TOO_SMALL。
数据可能增长，须处理再次容量不足；重试不消费样本。无样本仍返回 PB_OK 和空数组。
保持控制调用串行化、销毁前停止其他调用的约定。其他工作开始后，旧磁场历史清空。

## CLI

```sh
# 只读监测，默认 30 秒；--json 输出逐行的磁场批次 schema。
posebridge magnetic monitor --transport usb --port /dev/cu.usbserial-XXXX --json
posebridge magnetic monitor --transport ble --device '扫描得到的标识' --duration 30

# 明确校准，运行时绕三个轴转动；默认 60 秒后结束，不保存。
posebridge magnetic calibrate --transport usb --port COM3 --duration 60 --json
# 仅正常到时且结束核验成功后保存；中断／失败不保存。
posebridge magnetic calibrate --transport usb --port COM3 --duration 60 --save
```

时长从准备完成／开始核验成功后计算，范围大于 0 到 86400 秒。到时结束不代表精度达标。
CLI 输出最终批次后退出；Ctrl-C 清理成功返回 130，清理失败返回错误并保留原因。

## 验证与依据

`cargo test --workspace --release --locked` 覆盖单位、历史、状态和 Unix 假串口控制。
`python3 tests/magnetic_smoke.py` 通过实际 C 动态库和 CLI 验证假串口流程。
macOS 夹具只在测试子进程中为 PTY 模拟 IOSSIOSPEED；真实串口和正式二进制不修改。
Windows 跳过 PTY 集成，运行 Rust 模型和 C ABI 合约测试；实机验收单独记录。

只读实机测量（小型统计报告，不执行校准／保存）：

```sh
python3 tests/magnetic_hardware.py --cli target/release/posebridge \
  --transport usb --port /dev/cu.usbserial-XXXX --seconds 30 --output report.json
```

BLE 改用 `--transport ble --device '平台标识'`，Windows 指定 `.exe`。
本轮环境与未完成项见[验证记录](measurements/2026-09-22-magnetic.md)。

厂家 SDK 固定版本：
[读取](https://github.com/WITMOTION/WitBluetooth_BWT901BLE5_0/blob/9efaab0fdd6a06dc807bf80402e58aa91b431c6f/Windows_C%23/Wit.Example_BWT901BLE/ble5/Components/Bwt901bleProcessor.cs)、
[换算](https://github.com/WITMOTION/WitBluetooth_BWT901BLE5_0/blob/9efaab0fdd6a06dc807bf80402e58aa91b431c6f/Windows_C%23/Wit.Example_BWT901BLE/WitSdk/Tools/Device/Utils/DipSensorMagHelper.cs)。
