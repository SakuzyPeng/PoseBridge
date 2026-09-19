# 时间戳链路验证

日期：2026-09-19。PoseBridge 0.2、MacinRender C ABI 1.41；仅原生接口，无 GUI／音频渲染。

## macOS 软件验证

- Rust 1.96、Release：23 项核心／契约／时间戳／Unix 伪串口测试通过，包含配置后的等待、只读重试、拆包、非法日期、重复采样、时钟重置和原生四元数被动输入。
- `cargo fmt --all --check`、全目标 Release Clippy `-D warnings`、Release workspace 构建通过。
- 独立 Python 解码真实 CLI 的 v1/v2 Euler／四元数，含缺失时间和 kind=2 合成时间；每轮约 51 包／1 秒（上限 50 Hz 含首包），通过。
- 真实 C 程序迁移加载动态库，验证旧 `PbPose=136`、`PbStatus=96` 与新 `PbPoseV2=160` 字节布局、版本、无首帧、容量不足、较大结构尾部、原子时间和停止／重启，通过。
- MacinRender Debug 的 OSC、C 头文件与旧 C ABI 3 个测试目标通过；Release 动态库与实际 PoseBridge 模拟器联调通过。
  覆盖 v1/v2、两种姿态格式、缺失／合成时间、发送进程重启、重复包超时和接收器重启。整数时间无浮点截断。

## macOS 真机：采集 C ABI → OSC v2 → Render C ABI

同一 WT901BLE68（BWT901BLECL5.0），此前读取的设备版本 13115。Release 构建，显式配置 200 Hz 档。
每组从首份有效姿态开始运行约 6 秒，C 调用方约每 1 ms 轮询；无动作／安装校准要求。
发送上限 200 Hz，输出四元数；转换安装基底用于软件一致性验证，不代表佩戴方向已验收。

| 输入 | 输出配置 | 完整源采样数 | 主机观察采样率 Hz | Render 收到包数 | 按会话／序号匹配核验数 | 重新同步丢弃字节 |
|---|---|---:|---:|---:|---:|---:|
| USB | `timestamp-euler` | 1193 | 198.64 | 1171 | 1155 | 18 |
| BLE | `timestamp-euler` | 1198 | 200.56 | 150 | 149 | 0 |
| USB | `timestamp-quaternion` | 1193 | 198.61 | 1187 | 1181 | 14 |
| BLE | `timestamp-quaternion` | 1197 | 199.33 | 150 | 148 | 0 |
| USB | `timestamp-gyro-quaternion` | 1192 | 198.61 | 1190 | 1188 | 3 |
| BLE | `timestamp-gyro-quaternion` | 1193 | 198.65 | 148 | 139 | 0 |

- 六组均无非法帧、非法姿态、重复时间或 Render 拒绝包；匹配到的采样中，源接收纳秒、设备毫秒、epoch 完全一致，四元数差小于 1e-6。
- 每跨越一个完整源序号，设备时间增加 5 ms；kind=1，epoch=1。设备 RTC 仍在 2015 年，不作 UTC 或绝对延迟使用。
- BLE 一次最多 8 帧，最新值合并后每 6 秒发 148–150 包；设备 200 Hz 采样与约 25 批／秒交付分别记录。
  短窗口的主机采样率受批边界影响可略大于 200，不代表设备超频。
- USB 在打开／切换后的字节流重新同步时丢弃少量字节；之后未出现非法完整帧。没有把丢弃字节计成成功解析。
- 停止 PoseBridge 后 Render 均在 500 ms 后 stale 并保留最后值。
- 使用正式 `configure` 读回核验每次输出格式切换。真机发现写配置后立即读回会超时，已加入 100 ms 应用等待；
  BLE 断开后的 USB 首次寄存器回复可能被忽略，读回在 3 秒预算内每 250 ms 重试只读请求，不自动重发配置写入。
- 开始 `RRATE=0x06, output=0x61`，结束已回读恢复为相同值（10 Hz、默认输出）。未 SAVE、校准、设置 RTC 或升级固件。
  恢复辅助工具遇到一次首读超时，重试后已核对成功。

机器可读结果：[timestamp-pipeline.json](2026-09-19-timestamp-pipeline.json)。

## Windows 与限制

Windows 原生验证另行记录，不能由上述 macOS 结果代替。Windows 真机 USB 时间戳、设备物理断线、
安装方向、漂移、时钟同步和运动到音频总延迟尚未作为本次通过项。
本次只上报元数据，没有预测、插值补帧或延迟补偿。
