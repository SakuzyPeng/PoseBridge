# PoseBridge 当前协议与坐标

适用于 PoseBridge 0.5。**OSC 只支持一套协议，版本号为 3**。之前的 v1/v2 草案地址与选择开关已删除。
C ABI 400／CLI 的本地快照使用独立 schema=4，并提供查询年龄 age_ns；该字段不改变下面的 OSC 布局或心跳 schema。

## 设备输入

BLE 服务 `0000ffe5-0000-1000-8000-00805f9a34fb`，通知 `ffe4`，写入 `ffe9`（均使用同一 Bluetooth 基础 UUID）。
USB 默认 115200、8N1、无流控。连接后验证特征；普通采集保留设备配置，不后台轮询配置寄存器。

| 帧标志 | 总字节数 | 内容 |
|---|---:|---|
| `0x61` | 20 | 加速度 XYZ、角速度 XYZ、Euler XYZ |
| `0x01` / `0x04` | 8 / 10 | Euler / 原生四元数 |
| `0x81` / `0x84` | 16 / 18 | 设备时间＋Euler / 设备时间＋四元数 |
| `0xA4` | 24 | 设备时间＋角速度＋四元数 |
| `0x71` | 20 | 起始寄存器地址和连续 8 个寄存器 |

帧头为 `55 flag`。int16 小端：加速度 `/32768*16` g，角速度 `/32768*2000` °/s，Euler `/32768*180` °，
四元数 `/32768`，设备顺序 W/X/Y/Z。原生通知四元数范数平方要求 `[0.81,1.21]`。解析器支持拆包、粘包和重新同步，部分帧缓冲最多 24 字节。
这些帧没有独立校验和，合法日期与范数检查不等于 CRC。未支持的长输出格式不自动启用。

时间字段为 8 字节：`year-2000, month, day, hour, minute, second, millis-low, millis-high`，
按公历验证，转换为设备时钟自 2000-01-01 的整数毫秒，**不是 UTC**。同批各帧共享主机接收时刻，保留各自设备时间。
时间只归属同一姿态帧，寄存器四元数与无时间帧不会继承缓存时间。
重复设备时间不产生新姿态；倒退、时钟类型变化或前进超过主机间隔加 2000 ms 时递增时钟代次。

## 坐标

传感器 Euler 按 `Rz(Z) Ry(Y) Rx(X)` 解释；安装映射 M 给出传感器轴到头部右／前／上的右手基底，先做 `M R_sensor M^T`。
从物理头部旋转提取 `Rz(yaw) Rx(pitch) Ry(roll)`，再构造输出 Hamilton 四元数：
**x,y,z,w；`q = q_y(yaw) q_x(pitch) q_z(roll)`**。yaw 正向左，pitch 正向上，roll 正向右倾。
这是 `posebridge.yxz.v1` 表示；它的四元数轴不是渲染场景轴。回正由宿主负责。

| yaw,pitch,roll | x,y,z,w |
|---|---|
| 0,0,0 | 0,0,0,1 |
| 90,0,0 | 0,0.707106781,0,0.707106781 |
| 0,30,0 | 0.258819045,0,0,0.965925826 |
| 0,0,30 | 0,0,0.258819045,0.965925826 |
| 30,20,10 | 0.189307857,0.239298338,0.038134576,0.951548525 |
| 179,0,0 | 0,0.999961923,0,0.008726535 |
| -179,0,0 | 0,-0.999961923,0,0.008726535 |

## OSC 姿态消息

仅回环 UDP，默认 `127.0.0.1:9000`。每数据报一份完整消息，无 bundle。整个数据报最大 **8192 字节**。
默认目标上限 100 Hz，不改变设备速率；只发新姿态，等待期间合并中间采样。500 ms 无新有效采样停止姿态发送。

- `/posebridge/quaternion`：`,ishhhhhhhhihhffff`
- `/posebridge/euler`：`,ishhhhhhhhihhfff`

| 次序 | 字段 | 类型及语义 |
|---|---|---|
| 1 | protocol_version | i，必须为 3 |
| 2 | source_id | s，1–256 字节 UTF-8，非全空白、无控制字符 |
| 3 | instance_id | h，每次 start 的随机正标识 |
| 4 | session_id | h，每次设备连接的随机正标识 |
| 5 | sequence | h，当前采集会话内完整有效采样序号 |
| 6 | tx_sequence | h，本实例成功交给 UDP 的姿态包序号，从 1 开始 |
| 7 | reference_epoch | h，姿态参考代次，正值 |
| 8 | metadata_revision | h，来源描述版本，正值 |
| 9 | received_ns | h，源采集会话内主机单调接收纳秒，非负 |
| 10 | age_at_send_ns | h，PoseBridge 收到此姿态到发送的主机停留时间，非负且小于 500 ms |
| 11 | sample_time_kind | i，0 缺失；1 设备日历；2 模拟经过时间 |
| 12 | sample_time_ms | h，指定采样时钟中的非负毫秒 |
| 13 | sample_clock_epoch | h，有时间时为正值；缺失时为 0 |
| 14… | pose | f，四元数 XYZW 或 Euler yaw/pitch/roll（度） |

全部 h 使用有符号 int64 范围；kind=0 时 sample_time_ms/epoch 同为 0。
本地停留时间不包含已发生的 USB/BLE 延迟；Render 的接收时钟另有起点，三类时间不直接相减推导总延迟。
没有姿态预测、时钟同步或延迟补偿。

## 来源描述与状态心跳

`/posebridge/info`、`/posebridge/status` 均为 `,s`，唯一参数是 UTF-8 JSON。
共同字段：`schema=3`、`kind=info|status`、`source_id`、`instance_id`、`session_id`、`reference_epoch`、`metadata_revision`、`message_seq`。
**所有 64 位标识、时间和计数均为十进制字符串**；info/status 的消息序号分别递增。
连接建立前 session_id 可为 `"0"`，此时不会发送姿态。

info 的 `descriptor` 与 C ABI 完整快照一致：应用配置、平台身份、设备观察、软件能力与参考变化原因。
status 的 `status` 包含采集状态、分层计数、刷新率、交付直方图和错误信息。嵌套的会话／来源字段必须与共同字段一致。
设备型号与校准质量无法确认时为 null；软件实现某条命令不代表硬件已验证其效果。

- info 在启动、描述变化时发送，并每 5 秒重复；status 每秒发送，状态变化时在约 20 ms 服务周期内发送。
- 停止／失败时尽力发送终态后释放发送资源。心跳只描述本次采集实例；独立设备操作通过 CLI/C ABI 返回结果。
- 接收端对姿态使用 500 ms 时效，包含报文声明的源停留时间；状态心跳独立采用 3 秒时效。
  heartbeat_alive 表示近期收到状态消息，调用方仍须检查状态是否 stopped/failed。心跳不制造采样、不延长姿态 fresh。
- 会话、实例、参考和描述版本用于精确关联；迟到的旧元数据不能覆盖更新姿态。无效包不保活。
- `pose_count`、bytes/frames/delivery 和 OSC 统计针对本次 start；`session_samples` 与实际采样率针对当前设备会话。
  `coalesced_samples` 统计发送前被较新采样替代的中间样本；没有把它算为 UDP 丢包。
  接收端单独统计 tx_sequence 缺口，不把设备采样序号跳号视为丢包。

默认 source_id 为 `ble:平台标识`、`usb:端口` 或 `simulate`，仅具有本机作用域；可通过配置固定逻辑名称。
不同端口或 USB/BLE 不自动合并为同一物理设备。一个逻辑 source_id 同时只应由一个采集实例负责。

## 依据

- [官方新版协议](https://wit-motion.yuque.com/wumwnr/docs/qnpb2lo3f0orduqe)
- [BWT901BLECL5.0 协议与磁场校准流程](https://wit-motion.yuque.com/wumwnr/docs/gpare3)
- [BLE SDK](https://github.com/WITMOTION/WitBluetooth_BWT901BLE5_0)
- [实测记录](validation.md)
