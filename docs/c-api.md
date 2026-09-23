# PoseBridge C ABI 400（experimental）

Rust 包 0.6 沿用 0.4 的 C ABI 400，结构布局与 OSC 3 均不变。运动批次接口仅供 Rust 进程内消费者。
0.6 新增 `pb_magnetic_start` 和 `pb_magnetic_since_json`，要求升级动态库；见[磁场接口](magnetic.md)。

头文件 [posebridge.h](../include/posebridge.h) 由 cbindgen 生成；`pb_abi_version()` 必须等于 **400**。
0.4 在 `PbPose` 中加入 `age_ns`，不保留 ABI 300 布局。源码、头文件和动态库一起升级并重新编译调用方；
继续使用 `pb_latest_pose`、`pb_status`、`pb_snapshot_json`，不新增兼容 getter。

## 生命周期与查询

```text
pb_context_create → pb_configure
  → pb_inspect_start / pb_device_command（停止采集时）
  → pb_snapshot_json（等待操作完成并读取结果）
  → pb_start → pb_latest_pose / pb_status / pb_snapshot_json
  → pb_stop → 可再次检查或配置 → pb_context_destroy
```

每个上下文拥有运行时和一个设备操作；控制与生命周期由调用方串行化。采集期间实际检查／配置返回 Busy，
不会自动暂停和恢复。快照查询可以与后台采集并行；销毁前停止所有调用。无外部回调，这些 API 不属于音频回调接口。
stop 幂等并释放连接；destroy 消耗句柄且允许 NULL。start 成功只表示任务受理。

- `pb_latest_pose`：统一 `PbPose`，192 字节，包含身份、参考／描述版本、采样时间、查询年龄、姿态和原始诊断字段。
  `age_ns` 紧随 `received_ns`，偏移 80；四元数偏移 88。
- `pb_status`：统一 `PbStatus`，136 字节，包含当前状态、采样／交付／发送／合并与错误统计。
- 上述大小针对目标 macOS arm64／Windows x64。先初始化 struct_size；容量不足不写输出，较大结构的未知尾部保留。
- `pb_snapshot_json`：一次锁定并复制 `schema/descriptor/status/pose/operation`，本地 schema=4；64 位值均为十进制字符串。
  此版本与 OSC 独立：OSC 仍为协议 3，info/status 的 schema 仍为 3。
- `pb_devices_json`：读取 scan 的设备列表；`pb_error_copy`：读取上次同步 API 错误。

电压／估算电量通过 `pb_snapshot_json` 的 `status.battery` 读取；`PbStatus` 大小与 ABI 400 保持不变。
`pb_inspect_start` 查询一次，硬件采集期间自动每 30 秒查询，快照函数本身不执行设备 I/O。
数值可能为 null，必须检查 `fresh` 与 `last_error`；`age_ns` 是独立于姿态的十进制纳秒字符串。
供电电压超过 4.30 V 时百分比为 null，不能将 USB 供电读数当成满电；详见[电量说明](usage.md#电压与估算电量)。

字符串使用调用方缓冲区；required 包含 NUL。NULL＋capacity=0 可查询长度，返回 PB_BUFFER_TOO_SMALL。
快照可能变化，需处理第二次容量不足。首次姿态前返回 PB_NO_DATA；陈旧或停止后的最后姿态仍可读，fresh=0。
raw_flags：bit0 完整运动组，bit1 原始四元数，bit2/3/4 分别为 Euler／加速度／角速度存在。
原始运动组和四元数组有独立主机接收时间，不应推断它们都属于同一物理采样。

`age_ns` 是从 PoseBridge 收到当前姿态到本次查询的主机单调经过纳秒，最大饱和到正 int64 范围。
同次查询的年龄与 fresh 使用同一时刻。重复轮询可只改变年龄，不改变序号、设备采样时间或 metadata_revision；
停止后保留的姿态继续计龄，但 fresh=0。新连接清空旧姿态，首帧前返回 PB_NO_DATA。

默认 fresh 要求采集状态有效且年龄严格小于 500 ms。消费方可使用更严格的条件，例如
`pose.fresh && pose.age_ns < 100000000`。必须先检查时效再做序号去重，同一份姿态会随时间过期。
年龄是查询时快照，返回后不会自行增长；宿主排队期间的经过时间需由宿主自己的单调时钟补计。
它不包含设备采样到 USB/BLE 接收的延迟，不使用设备日历计算，也不等于运动到声音的总延迟。

状态：0 idle、1 scanning、2 connecting、3 active、4 stale、5 reconnecting、6 stopped、7 failed、8 configuring、9 complete、10 inspecting、11 magnetic。

## 配置与设备控制

```json
{
  "source_id": "headset",
  "source": {"kind":"usb","port":"COM4","baud":115200},
  "pose_input": "stream_quaternion",
  "mounting": {"right":-2,"forward":1,"up":3},
  "osc": {"target":"127.0.0.1:9000","max_rate_hz":100,"format":"quaternion"}
}
```

BLE 使用 `{"kind":"ble","device_id":"平台标识"}`；可选 connection_mode=default/throughput，后者仅 Windows 11+。
模拟器使用 `{"kind":"simulate","rate_hz":100,"sample_clock":true}`。硬件采集需要安装映射，纯检查和控制允许缺省。
pose_input 为 euler、quaternion（寄存器轮询）或 stream_quaternion（被动原生通知）。缺省/null osc 表示不发送网络数据。
旧 osc.version 字段会被拒绝。参数校验失败保留原配置。

`pb_device_command` 接收 action JSON：rate（hz）、output（format）、algorithm（mode=six_axis/nine_axis）、
zero_yaw、angle_reference、reset_defaults、accel_calibrate、mag_start、mag_stop、save。
控制语义见[使用指南](usage.md#设备检查与控制)。
独立磁场会话内也可发送 mag_start／mag_stop／save，其他命令仍返回 Busy；结果以 operation.outcome 为准，
会话不会因命令结束而转成 complete。关闭会话会清理由本会话启动的校准，不自动 SAVE。

operation 包含操作 id、目标 source_id、action、outcome、write_attempted、command_sent、register_verified、
completion_observed、persistence、reference_may_have_changed 和 message。
outcome 为 running/succeeded/unverified/failed/cancelled；连接状态 complete 仅表示任务结束，须继续检查操作结果。
持久化 not_requested/unverified 与寄存器验证独立；没有用一次回读声称已通过掉电验证或达到校准精度。
同一上下文保留上次操作；跨进程未观察到的操作不猜测原因。

描述中的 device 是带 observed_unix_ms 与 valid 标记的缓存，提供原始寄存器及已知的速率、模式、字段集合和固件显示值。
valid 表示成功读取且未被本上下文已知的变化作废，不保证其他程序从未修改设备。采集期间不自动刷新配置寄存器；电压单独低频更新。

## 结果与边界

0 OK，1 invalid argument，2 no data，3 busy，4 unavailable，5 permission，6 I/O，7 protocol，8 timeout，
9 buffer too small，10 internal，11 cancelled。异步错误与控制结果从完整快照读取。
可展开 panic 在边界捕获；不承诺从进程终止/OOM 恢复。非空指针须有效、对齐，并满足长度与生命周期契约。

[真实 C 调用测试](../tests/c_api_smoke.c)覆盖版本、布局、缓冲区、年龄、原子 JSON、忙状态、停止及新实例。
[C11 消费示例](consumer.md)提供控制线程轮询与宿主接入点。
