# Rust 0.5 同帧运动接口

`Controller::motion_since(Option<MotionCursor>) -> MotionBatch` 返回一次查询时刻下的一致历史，
每次 USB read／BLE 通知中的全部有效姿态持同一 `delivery_id` 原子发布。最多保留 256 个样本；
用返回的游标继续读。`reset` 表示实例／会话变更，`history_overrun` 表示消费者未及时读出的历史被淘汰，
不代表设备丢包。批次可能为空；`active` 仍反映最新采集的有效性。停止后旧历史不会恢复为 fresh。

样本含实例、会话、参考代次、元数据修订、序号、交付批次、主机接收年龄、设备时间与时钟代次。
`orientation_source` 区分原生四元数、设备 Euler 转换、寄存器四元数和模拟器；`profile` 标明帧格式。
无效姿态不发布。角速度和加速度的 `Option` 表示同一姿态帧内该字段是否存在且有效，
不从 legacy `RawData` 缓存补齐。寄存器四元数没有关联的旧时间戳、角速度或加速度，
即使它与其他帧在同一次 read 到达也不例外。

`orientation_xyzw` 为 Hamilton 物理四元数：头部 X 向右、Y 向前、Z 向上。
安装方向用同一右手基底：姿态 `M R Mᵀ`，角速度和加速度 `M v`。
角速度单位 rad/s、身体坐标；加速度单位 g。不要把它与 GUI／OSC 的 YXZ 四元数混用。
`euler_deg` 保留已有姿态快照的语义，供普通模式兼容使用。C ABI 400、OSC 3、JSON schema 4 不变。

`queried_at: Instant` 和各样本 `age_ns` 使用同一个主机单调时钟；可用二者恢复接收时刻。
设备时间的来源由 `SampleTime.kind` 给出，不能当作 UTC 或直接减去主机时钟。
每个交付批次仅最后一份样本适合做接收锚点。固定链路／融合延迟未知，不能据此宣称端到端延迟。
回正、受限预测、平滑和最终音频姿态由消费者负责。

## 输出格式

| 格式 | 字节 | 字段 | 状态 |
|---|---:|---|---|
| 0x61 | 20 | 加速度、角速度、Euler | 已有；无设备时钟 |
| 0xA4 | 24 | 时间戳、角速度、原生四元数 | 可靠增强输入；无加速度 |
| 0xE4 | 30 | 时间戳、加速度、角速度、原生四元数 | 实验，仅 20 Hz |

实验 CLI 格式名为 `experimental-full-inertial-20hz`，JSON 为 `experimental_full_inertial_20hz`。
设置 E4 之前回读速率必须为 20 Hz；E4 活跃时不能把速率改到其他值，必须先改回短格式。
直接采集时也检查 E4 的设备时间间隔，持续不在约 20 Hz 范围的流会报协议错误。
正常连接不改速率或格式。`FULL_INERTIAL_20HZ_VALIDATED` 目前为 **false**；解析器测试通过不等于实机通过。

## 实机门槛与工具

复用当前 target 构建：

```sh
cargo build --workspace --release --locked
cargo build -p posebridge-core --example motion_probe --release --locked
python3 tests/full_inertial.py --config /path/to/device.json \
  --library target/release/libposebridge_capi.dylib \
  --probe target/release/examples/motion_probe \
  --seconds 60 --output /path/to/usb-e4-20hz.json
```

Windows 使用已有 native Python，库和 probe 后缀分别为 `.dll`、`.exe`。
配置使用现有 `Config` JSON，包含明确设备、transport 和 mounting，不会创建另一 checkout。
先断开占用传感器的播放器。工具读取并保存原速率／格式，临时应用 E4／20 Hz，预热 2 秒后提示
`CAPTURE_READY`。采集至少 60 秒，静止至少 5 秒并做点头、摇头、侧倾，各轴至少 20°。
工具只保留统计量，检查 19–21 Hz、字段齐全、时间递增、无新增失步／非法帧／姿态、无历史溢出，
并对照四元数旋转与角速度方向及积分残差。每种 transport、每个平台分别保存小型 JSON 记录。

成功、失败、超时或 Ctrl-C 后，包装器先终止读进程，恢复原输出格式，再恢复原速率并回读核验。
不校准、归零、改融合算法或 Flash 保存。恢复失败单独记为失败；进程被强制杀死、断电等情况下，
重新连接后需按记录的原值恢复。macOS BLE 必须从已有带 `NSBluetoothAlwaysUsageDescription` 的授权宿主运行，
并保证 Python C ABI 宿主和 probe 宿主都有权限；裸终端被隐私系统拒绝时该项记为未完成，不关闭系统权限保护。

验收前始终保留 A4 方案。当前 macOS／Windows 的 USB、BLE 三轴 60 秒验收均未完成，正式预设不开放 E4。
