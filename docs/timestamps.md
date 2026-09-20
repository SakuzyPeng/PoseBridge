# 时间戳与当前下游接口

PoseBridge 0.4 沿用 OSC 协议 3。当前地址、字段顺序、时钟含义与心跳定义均以[协议文档](protocol.md)为准；不再使用 --osc-version 或带 /v1、/v2、/v3 的地址。

设备时间只在实际输出该字段时存在。以下操作会显式改变输出内容，不执行 SAVE：

```sh
posebridge inspect --transport usb --port /dev/cu.usbserial-210 --json
posebridge configure --transport usb --port /dev/cu.usbserial-210 output --format timestamp-quaternion
posebridge bridge --source-id headset --transport usb --port /dev/cu.usbserial-210 \
  --mount=-y,+x,+z --pose-input stream-quaternion --duration 10 --json
```

测试结束按 inspect 记录恢复原输出格式；设备原来为 motion 时，使用 `configure ... output --format motion`。
BLE 使用扫描得到的平台标识。设备速率通过 rate 命令单独改变；OSC 发送上限不修改传感器回传率。

- kind=0：缺失，不把主机时间冒充设备时间。
- kind=1：设备日历自 2000-01-01 起的毫秒，未同步到 UTC。
- kind=2：模拟器经过时间；`simulate --sample-clock` 用于测试。
- instance_id 标记运行实例，session_id 标记设备连接，sample_clock_epoch 标记设备时钟跳变。
- reference_epoch 标记姿态参考可能变化；它与时钟代次独立。
- source age_at_send_ns 只统计 PoseBridge 收到姿态后的停留时间，无法直接得出 USB／BLE 或音频总延迟。
- C ABI 400／本地快照 schema=4 的 age_ns 在查询时计算；与发送时的 age_at_send_ns 观察时刻不同。
  停止后的查询仍会增长年龄，fresh 始终为 false。消费方读取后自行补计宿主排队时间，详见 [C ABI](c-api.md)。

BLE 可每批携带 8 个间隔 5 ms 的设备采样，主机约每 40 ms 获得一批；桥接只发送选中的最新姿态。
完整[历史时间戳验证](measurements/2026-09-19-timestamp-pipeline.md)使用 0.2 草案，新的协议及控制结果见[验证索引](validation.md)。
本版没有时钟同步、预测或延迟补偿。
