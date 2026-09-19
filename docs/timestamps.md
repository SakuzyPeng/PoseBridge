# 设备时间戳与 OSC v2

状态：PoseBridge 0.2 核心库、CLI 与实验 C ABI 已实现；MacinRender 原生接收接口已实现。
GUI、时钟同步、姿态预测与音频延迟补偿不在本次实现范围内。验证按平台和传输方式分别记录。

## 启用方式

普通 `diagnose`／`bridge` 保留设备配置，默认 OSC 仍是 v1。时间戳需要设备实际上报：

```sh
# 显式更改输出内容，读回核验；不执行 SAVE。记录原配置，测试结束后恢复。
posebridge configure --transport usb --port /dev/cu.usbserial-110 output --format timestamp-quaternion
posebridge bridge --transport usb --port /dev/cu.usbserial-110 --mount=-y,+x,+z \
  --pose-input stream-quaternion --osc-version v2 --duration 10 --json
# 恢复默认输出内容（仅当原配置确实是 motion）
posebridge configure --transport usb --port /dev/cu.usbserial-110 output --format motion
```

BLE 使用 `--transport ble --device "扫描返回的标识"`；Windows 使用实际 COM 端口。
速率通过独立 `configure ... rate --hz 200` 修改，`--osc-rate-hz 200` 仅改变发送上限。
`--pose-input stream-quaternion` 被动读取原生四元数通知；既有 `quaternion` 仍是独立寄存器请求，不能附带旁边角度帧的时间戳。

| output --format | 标志／总长度 | 内容 | pose-input |
|---|---|---|---|
| `motion` | `0x61`／20 字节 | 加速度、角速度、Euler | `euler` |
| `timestamp-euler` | `0x81`／16 字节 | 时间戳、Euler | `euler` |
| `timestamp-quaternion` | `0x84`／18 字节 | 时间戳、Q0–Q3 | `stream-quaternion` |
| `timestamp-gyro-quaternion` | `0xA4`／24 字节 | 时间戳、角速度、Q0–Q3 | `stream-quaternion` |

还可被动解析 `0x01`（Euler，8 字节）和 `0x04`（四元数，10 字节），二者没有采样时间。
配置写入前先读 `0x0E`；仅当前值为 `0x61/0x81/0x84/0xA4` 才允许更改，避免旧固件将该寄存器解释为 D0MODE。
失败的回读会报告错误，不能据此认为配置没有发生改变。长报文 `0xE1/0x6F/0xEF` 不作为支持的输出配置。

## 设备时钟与接收时间

依据[官方新版 BLE 协议](https://wit-motion.yuque.com/wumwnr/docs/qnpb2lo3f0orduqe)，时间戳在 `55 flag` 后占 8 字节：
`year-2000, month, day, hour, minute, second, millisecond-low, millisecond-high`。
采用公历验证日期、闰年和毫秒范围，转换为设备时钟中自 **2000-01-01** 起的整数毫秒。
该仪器 RTC 曾显示 2015 年，未同步到主机，**不是 UTC、主机墙钟或可直接用于算延迟的时间**。
原生四元数 W/X/Y/Z 量化值除以 32768，要求范数平方处于 `[0.81,1.21]`，然后按安装映射转换。
帧无独立校验和；日期和范数检查不能代替 CRC。部分帧缓冲最多 24 字节，支持拆包、粘包与重新同步。

核心 `PoseSnapshot` 新增可空 `sample_time = {kind, time_ms, clock_epoch}`。
它只属于产生该姿态的同一帧；无时间戳的帧和四元数寄存器响应始终为 null，不继承缓存时间。
`received_ns` 取完成该次 USB 读取／BLE 通知的主机单调时刻，相对于采集会话开始；同一批内各帧共享该值。
最新值合并后，OSC 和 C ABI 携带的是选中那一帧的设备时间和该批接收时间。

- 每次连接和模拟运行分配随机正 int64 会话标识，进程重启也不会固定从 1 开始；序号从 1 递增。
- 首个有效设备时间的 `clock_epoch=1`。相同设备时间的后续帧不产生新姿态、序号或保活。
- 时间倒退、时钟类型改变，或前进量超过主机间隔加 2000 ms，递增时钟代次；该姿态仍可作为新鲜数据发布。
- 比较设备采样间隔时必须限定同一会话、kind、epoch。2000 ms 是跳变检测容差，不是精度或延迟保证。
- 仅在姿态和时间均有效后推进时钟；`invalid_frames`、`duplicate_sample_times`、`clock_discontinuities` 通过状态 JSON 查询。
- 500 ms 无新有效姿态即 stale，停止 OSC。慢消费者只得到最新快照，不积累历史队列。

实际刷新率和间隔统计仍基于主机接收边界；BLE 批内间隔可以为 0。
[已验证的 200 Hz BLE 数据](measurements/2026-09-19-ble-exploration.md) 每帧设备时间相差 5 ms，约每秒到达 25 批。
时间戳上报不会把这种分批交付变成均匀的 200 Hz，也不自动消除其延迟。

## OSC v2 线协议

目标仍默认 `127.0.0.1:9000`，仅回环；显式 `--osc-version v2` 启用。
每个数据报一个完整 OSC 普通消息，无 bundle/timetag。四元数坐标与 v1 完全一致。

- `/posebridge/v2/quaternion`：`,hhhhihffff`，元数据后接 x、y、z、w。
- `/posebridge/v2/euler`：`,hhhhihfff`，元数据后接 yaw、pitch、roll（度）。

| 顺序 | 字段 | OSC 类型 | 含义 |
|---|---|---|---|
| 1 | `source_session_id` | h，int64 | 正值，PoseBridge 采集会话 |
| 2 | `source_sequence` | h，int64 | 正值，会话内完整采样序号；合并时允许跳号 |
| 3 | `source_received_ns` | h，int64 | 非负，源会话内主机单调接收纳秒 |
| 4 | `sample_time_ms` | h，int64 | 非负，指定时钟中的采样毫秒 |
| 5 | `sample_time_kind` | i，int32 | 0 缺失；1 设备日历；2 模拟器经过时间 |
| 6 | `sample_clock_epoch` | h，int64 | 有时间时为正值；缺失时为 0 |
| 7… | 完整姿态 | f，float32 | 四元数四分量或 Euler 三分量 |

字段均按 OSC 网络字节序。kind=0 时 time 和 epoch 必须同时为 0；kind≠0 时 epoch 不得为 0。
非有限姿态、负整数、未知 kind、错误类型、长度和零填充均应拒绝。

MacinRender v2 接收器在源会话内拒绝不递增序号、倒退的源接收时间，以及同代次不递增采样时间或改变 kind。
代次必须单调增加才能重置设备时间；缺失时间的包不清除先前时钟检查状态。
源会话变化开始新序列，最多记住 16 个已退出会话以拒绝迟到包。v1 可以接收，但不清空 v2 的检查历史。
拒绝包不能刷新 500 ms 保活。每个端口使用一个发送端与一个版本；会话检查不提供身份认证或多源仲裁。

Render 的 `pose.received_ns` 仍来自 **Render 自己**的单调时钟，源接收时间和设备时间分别暴露。
三种时间起点不同，本版不估算时钟偏移，不直接相减，不报告它们的差值为 BLE／音频延迟。

## C ABI 与无设备验证

PoseBridge：`PbPoseV2`／`pb_latest_pose_v2`，实验 ABI `200`；见 [C ABI](c-api.md)。
MacinRender：`adm_head_tracking_pose_v2_t`／`adm_osc_head_tracking_get_pose_v2`，稳定 ABI 1.41。
两者都一次锁定完整快照，避免姿态与时间分别轮询导致错配；已有结构布局和函数保留。

```sh
posebridge simulate --pattern combined --osc-version v2 --sample-clock --duration 3
```

`--sample-clock` 只生成 kind=2 的模拟经过时间，缺省不制造设备时间。
协议、整数精度、重复／乱序、时钟跳变、无数据、停止与结构尾部保护均有测试。
具体执行环境和真实设备结果见[验证记录](validation.md)，没有实施 GUI／音频链路验证。
