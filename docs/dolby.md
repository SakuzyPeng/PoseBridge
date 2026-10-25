# Dolby Atmos Renderer 双路输出（0.7）

同一次 BLE／USB 采集可以同时向 PoseBridge 接收端和 Dolby Atmos Renderer 发送姿态。
设备只连接一次；每路独立限速、去重和统计，发送失败不会停止另一输出或设备采集。

## 客户端

在 **Output** 中分别启用 **PoseBridge** 和 **Dolby Atmos Renderer**，断开时修改配置，再 Connect。
PoseBridge 保留已有目标和格式，默认 `127.0.0.1:9000`；Dolby 默认关闭，目标为 `127.0.0.1:8000`。
两路均支持回环地址和 1–200 Hz 上限，默认 100 Hz；上限不会提高设备采样率或补造中间采样。

Dolby Renderer 中启用 **Headphone**、**Binaural** 和 **OSC head tracker** 并接受设置，选择 **Person** 视图检查人物方向。
头追使用独立的实时耳机监听输出，普通离线 binaural re-render 不携带头部运动。
`8001` 是 Renderer 的播放／监听控制端口，头追驱动使用 `8000`。

先将传感器佩戴在最终位置，面向正前方、保持头部正立后，在客户端 Output 页点击 **Set Dolby forward (Recenter)**。
当前三轴姿态成为本会话的 Dolby 参考，
于下一份有效新采样上生效。它只影响 Dolby；PoseBridge 输出和 Tracking 中的原始姿态不改变。
换设备、重连、修改安装或设备姿态参考变化后，需重新设置 Dolby forward。参考不写入设备或配置文件。
OSC 的独立回正在 PoseBridge 中操作，Renderer 的 AirPods 回正按钮不适用于本驱动。
若先在桌面上回正再佩戴，参考朝向可能造成转头方向异常，应佩戴后重新回正再检查三轴。
没有有效新鲜姿态、Dolby 输出关闭或正在维护时，按钮不可用，API 返回明确错误。

Dolby 的 YPR 是其自身坐标与回正参考下的角度，可能与 Tracking 的角度不同。
Last sent 为最近成功交给 UDP 的值。`active` 和发送计数均不代表接收端确认；实际是否接入应查看 Renderer 人物与耳机监听。
停流后 Dolby 可以保持最后收到的姿态，客户端不会用零姿态或重复旧数据保活。
窗口最小化不影响发送；停止、失效及设备维护／磁场会话期间不发送姿态。

## 报文与坐标

每个 UDP 数据报恰好一条 OSC 消息：`/ypr`、类型 `,fff`、yaw／pitch／roll，单位度，28 字节。
不使用 bundle，不附加 PoseBridge 的序号、时间戳、来源或心跳。

发送使用安装映射后的物理 Hamilton 四元数，X 右、Y 前、Z 上。
回正计算 `q_relative = conjugate(q_reference) * q_sample`，不对欧拉角逐项相减。
Dolby 驱动的矩阵约定为后／右／上坐标中的 world-to-head：

```text
B = [ 0 -1  0 ]
    [ 1  0  0 ]
    [ 0  0  1 ]
D = B * transpose(R_relative) * transpose(B)
D = Rz(-yaw) * Ry(-pitch) * Rx(roll)
```

从 D 提取 YPR，在俯仰极点统一选 roll=0，保持同一物理旋转。
原有 PoseBridge `posebridge.yxz.v1` 四元数和 OSC 3 报文不作转换。
复合旋转、±180°、极点、q／−q 等价性和回正均有矩阵回归测试。

## Rust、CLI 与 C ABI

`Config` 新增 `dolby_osc: Option<DolbyOscConfig>`，默认 None；结构字段为 target 和 max_rate_hz。
Rust 直接构造 Config 的调用方需新增该字段或使用 `..Config::default()`。
`Controller::recenter_dolby()` 用当前新鲜物理姿态设置参考，不重启采集。

```sh
posebridge bridge --transport ble --device '扫描所得设备标识' --mount=-y,+x,+z \
  --osc-target 127.0.0.1:9000 --dolby-target 127.0.0.1:8000 --dolby-rate-hz 100
```

CLI `--format` 仍仅选择 PoseBridge 一路的格式。省略 `--dolby-target` 不启用 Dolby。
`simulate` 支持同样的输出参数。Rust／JSON 配置和 GUI 也支持只启用 Dolby。

C ABI 继续为 400，PbPose／PbStatus 的大小和布局不变；新增函数要求使用 0.7 或更新的动态库：

```c
int32_t pb_dolby_recenter(struct PbContext *value);
```

用现有 pb_configure JSON 配置 Dolby，例如添加：

```json
"dolby_osc": {"target":"127.0.0.1:8000", "max_rate_hz":100}
```

本地 schema 4 快照新增可选 `status.dolby_osc`：active、sent、coalesced_samples、send_errors、
reference_set、last_ypr_deg、last_error。64 位计数仍为十进制字符串。
原有 osc_sent／coalesced_samples／send_errors／telemetry_sent 仅统计 PoseBridge 一路。
Dolby 配置和状态扩展不会进入原有 PoseBridge OSC 3 心跳，已有接收端继续得到原格式。

客户端设置升级为版本 2，读取版本 1 时保留 PoseBridge 的目标、开关和格式，Dolby 默认关闭。
旧客户端遇到版本 2 会保留文件并以只读默认配置启动。

## 验证入口

`cargo test --workspace --release --locked` 覆盖坐标、双路限速、独立回正、故障隔离和 GUI／配置迁移。
`python3 tests/dolby_smoke.py` 使用独立的临时 UDP 接收器测试实际 C ABI 和 CLI，不访问设备或固定的 Renderer 端口。
