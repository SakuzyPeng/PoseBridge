# OSC 调度与 BLE 通知分析

日期：2026-09-19。设备 WT901BLE68（商品型号 BWT901BLECL5.0），固件版本未确认。
macOS Apple Silicon 与 Windows x64 原生 Release；USB 接在 Mac，BLE 分别连接两台电脑。
本轮没有验证 Windows USB 高频，也没有构建或测试 MacinRender 音频链路。

后续 [macOS BLE 探索](2026-09-19-ble-exploration.md) 补充了厂商打包说明、设备版本号、新格式主动四元数与时间戳实测；
本文保留初次修复阶段的结论和数据。

## 修复结果

| 路径 | 修复前 OSC | 修复后 OSC | 当前限制 |
|---|---:|---:|---|
| Mac USB，设备 200 Hz | 122.67 Hz | 197.02–197.92 Hz | 输入约 198.61 Hz；少量主机批量读取／调度抖动仍会合并中间帧 |
| Mac USB，设备 100 Hz | 99.29 Hz | 99.32 Hz | 正常保持新采样速率 |
| Windows 模拟器，目标 200 Hz | 63.61 Hz（仅调度修复后） | 200.01 Hz | 受 OS 调度影响，不是实时保证 |
| Mac BLE，设备 200 Hz | 24.53 Hz | 24.83 Hz | 每份通知最多 160 字节／8 帧 |
| Windows BLE，设备 200 Hz，默认连接 | 24.48 Hz | 24.81 Hz | 每份通知 160 字节／8 帧 |
| Windows BLE，设备 200 Hz，高吞吐连接 | — | 24.84 Hz | 缩短连接间隔未改变通知打包 |

USB 200 Hz 默认由连续角度流在本机转换为四元数输出。最终提交 `950f87b` 的一次 USB 复测为
198.61 Hz 输入／197.02 Hz OSC，OSC 间隔中位数 5.04 ms、P95 7.31 ms、最大 11.88 ms；
前两轮发送调度修复后的短测分别为 197.86 和 197.92 Hz。报告保留全部结果，不以最高一次替代波动范围。

## 已修复的软件问题

1. OSC 原来每 5 ms 轮询，再要求距上次实际发送满 5 ms。系统唤醒有抖动时会跳过整次轮询。
   现在新姿态到达即尝试发送，受限时只保留最新姿态，并按截止时间唤醒；小幅迟到不累积到下一周期。
   长时间停顿后重新计时，无补发额度、历史队列或重复旧姿态。
2. Windows 默认定时粒度实测约 15.6 ms，使 200 Hz 模拟器只有约 64 Hz。
   高频任务期间成对调用 `timeBeginPeriod(1)`／`timeEndPeriod(1)`；停止、取消和异常路径均释放请求。
   模拟器复测输入 200.00 Hz／OSC 200.01 Hz，间隔中位数 5.10 ms、P95 7.10 ms。
3. 原生四元数读取改为独立截止时间，保持至少 20 ms 请求间隔、一个请求在途和 250 ms 超时重试。
   移除了异步串口 `flush` 内可能执行的阻塞式 `tcdrain`／`FlushFileBuffers`。
   伪串口测试曾在该 drain 上挂起；移除后超时、取消与配置回读测试通过。

原生四元数逐次读取仍受响应往返约束：最终 Rust CLI 在 USB 上为 41.85 Hz 输入／41.84 Hz OSC；
先前独立原始串口逐次请求测到约 46.6 Hz。两种请求实现与调度不同，不能把该值声称为设备所有模式的绝对上限。
需要接近 200 Hz 四元数输出时，使用默认 `--pose-input euler`，由本机完成姿态转换。

## BLE 的剩余限制

新增 `status.delivery` 在解析前记录通知／读取次数、最大字节数、每次帧数和间隔分布。
Windows 默认连接的一次最终采集有 **303 份通知、2424 帧、303 次 OSC 发送**，每份通知恰好 8 帧。
Windows 高吞吐模式是 **264 份通知、2112 帧、264 次 OSC 发送**。这解释了约 199 帧/秒为何只有约 25 次新通知/秒。
macOS 也观察到相同的 160 字节批次，现象发生在 OSC 编码和限速之前。

Windows 新增显式 `--ble-mode throughput`，通过系统 API 临时请求高吞吐连接。默认保持系统策略：

| 系统报告／本次观测 | 默认 | 高吞吐 |
|---|---:|---:|
| 连接间隔 | 30 ms | 15 ms |
| 外围设备延迟（可跳过的连接事件数） | 10 | 0 |
| 请求状态 | 未请求 | success |
| 最大通知字节数／帧数 | 160／8 | 160／8 |
| OSC 间隔 P95，最终短测 | 60.28 ms | 45.29 ms |
| OSC 间隔最大值，最终短测 | 90.09 ms | 60.13 ms |

先前另一轮高吞吐 P95 为 59.13 ms，因此暂不承诺固定的抖动改善幅度。请求关闭后再次以默认模式连接，
系统报告恢复为 30 ms／10 个事件，未遗留高吞吐请求。该选项可能减少同时可用的 BLE 连接数，默认不开启。

**BLE 的约 25 Hz 新姿态交付仍未解决。** 当前证据定位到上游通知打包，尚无空口抓包或固件证据区分设备与系统栈内部延迟。
没有找到此型号已确认、可安全使用的缩短通知打包周期配置；`btleplug` 的现有公共接口也不提供相应控制。
macOS 当前后端不提供中央设备角色下的连接参数调节接口，显式 throughput 会被拒绝。
继续提高无线交付频率需要确认设备打包周期／固件或传输接口的支持。重放一批中的旧姿态或插值不会改善真实追踪延迟，未采用。

## 方法、恢复与验证

- OSC 使用独立 Python UDP 接收器，验证消息地址、参数类型、有限数值和四元数范数。
  最终测试采用 `scripts/measure_osc.py`：每次 CLI 运行 15 秒，去掉首份有效姿态／首个包之后的 3 秒，
  以稳定窗口首末单调时间计算输入率与实际接收率。100 Hz 跟进使用 12 秒窗口。
  BLE 连接建立耗时不计入输入率，因而每轮有效统计窗口长度可能不同。
- 全部高频桥接显式传入 `--osc-rate-hz 200`，100 Hz 跟进显式传入 100。
  同一台仪器按顺序测试，每轮临时改变回传率后恢复；没有发送 SAVE、校准或未知寄存器写入。
- 最终寄存器 `0x03` 从 `0x0B` 恢复为原来的 `0x06`，回读匹配，独立读取实际约 **9.93 Hz**。
- USB 开始时有少量字节用于重新同步（最终角度测试 4 字节、四元数测试 10 字节），无无效姿态或重连。
  这些丢弃字节不能换算为精确丢帧数。基线连接切换后的约 1.85 秒空档仍保留在原始记录中，原因未确认。
- macOS：15 项 Rust 测试通过（9 契约＋3 调度＋3 伪串口）；Windows：12 项跨平台测试通过。
  两台机器通过 Release 构建、格式检查、Clippy `-D warnings`、独立 CLI OSC 解码与真实 C ABI 迁移调用。
  C 头文件／结构布局没有变化。Windows 通过 Git bundle 同步同一代码提交验证；本轮未推送或触发新 CI。
- 数据仅反映短时主机输入、通知和 UDP 接收，不是内部融合周期、无线空口延迟或运动到声音的整体延迟。
  未重复用户动作测试；此前 200 Hz 动态采集确认运动期间的相邻角度有更新。

[原始汇总 JSON](2026-09-19-osc-timing.json) 包含基线、软件修复阶段、连接偏好与恢复结果。
源码阶段：基线 `0daabe1`，OSC 修复 `4cb92c6`，BLE 诊断 `6f5857e`，最终采集修复 `950f87b`。

## 依据

- [微软：请求 BLE 连接参数](https://learn.microsoft.com/en-us/uwp/api/windows.devices.bluetooth.bluetoothledevice.requestpreferredconnectionparameters)
- [微软：连接间隔的 1.25 ms 单位](https://learn.microsoft.com/en-us/uwp/api/windows.devices.bluetooth.bluetoothleconnectionparameters.connectioninterval)
- [微软：外围设备延迟的连接事件单位](https://learn.microsoft.com/en-us/uwp/api/windows.devices.bluetooth.bluetoothleconnectionparameters.connectionlatency)
- [微软：timeBeginPeriod 与配对释放、系统限制](https://learn.microsoft.com/en-us/windows/win32/api/timeapi/nf-timeapi-timebeginperiod)
- [维特官方 BLE 5.0 SDK](https://github.com/WITMOTION/WitBluetooth_BWT901BLE5_0/tree/9efaab0fdd6a06dc807bf80402e58aa91b431c6f)
- [PoseBridge 协议与坐标](../protocol.md)
