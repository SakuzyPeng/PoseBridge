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

## Windows 原生软件验证

Windows x64、Rust 1.98 MSVC、MSVC 14.51，使用已有工具链、全局 Cargo 缓存与单一 target 目录。
通过雷电网桥同步 Git 提交，没有创建新工作树或重新获取完整渲染依赖。

- 格式检查、全目标 Release Clippy、17 项可在 Windows 运行的 Rust 测试、Release workspace 构建通过。
  其余 6 项 Unix 伪串口测试仅在 macOS 运行；未声称 Windows 执行这些测试。
- 生成头文件一致性、真实 C ABI 程序（包含新旧结构与缓冲区契约）、CLI 独立 v1/v2 解码六种组合通过。
- kind=2 模拟器在 200 Hz 配置下，本机 OSC 接收约 200.03 Hz，无无效报文；该数值不替代硬件输入率。
- Render 接收器／协议源文件和完整 `adm_c_api.cpp` 翻译单元经 MSVC 编译，纯 C 头文件布局检查运行通过。
  最小 C++ 接收程序在 v1/v2 × Euler/四元数四组中各收 100 份姿态，无拒绝包；
  v2 时间精确匹配、停止失活、端口独占／释放通过。
- Windows 没有完整渲染依赖缓存，所以没有构建或运行完整 Render C ABI DLL；这项范围与 macOS 动态库验证分开记录。

## Windows 真机：COM4／BLE → PoseBridge C ABI → OSC v2 → Render C++

用户将同一传感器移到 Windows USB，枚举为 USB-SERIAL CH340（COM4）。先恢复并读回已知原配置，再显式设置 200 Hz 档。
每组采集窗口约 6 秒，接收程序额外等待以验证停止后 stale；设备数值和安装方向不作定位精度证据。

| 输入 | 输出配置 | 完整源采样数 | 主机观察采样率 Hz | Render 收到包数 | 重新同步丢弃字节 |
|---|---|---:|---:|---:|---:|
| USB | `timestamp-euler` | 1192 | 199.50 | 271 | 6 |
| BLE | `timestamp-euler` | 1200 | 200.89 | 150 | 0 |
| USB | `timestamp-quaternion` | 1193 | 199.51 | 279 | 5 |
| BLE | `timestamp-quaternion` | 1200 | 200.80 | 150 | 0 |
| USB | `timestamp-gyro-quaternion` | 1194 | 199.36 | 280 | 0 |
| BLE | `timestamp-gyro-quaternion` | 1200 | 200.83 | 150 | 0 |

- 六组都为 kind=1、epoch=1，每跨越一个源采样序号，设备时间推进 5 ms；没有非法帧、非法姿态或 Render 拒绝包。
  最终接收元数据与 PoseBridge 原子快照／采样序列匹配，四元数范数为 1；停止后进入 stale，端口正常释放。
- Windows USB 虽收到约 200 个采样／秒，但本轮主机读取存在批量交付，最新值合并后仅约 45–47 个 OSC 包／秒。
  BLE 为每批最多 8 帧，约 25 包／秒。不能用设备采样率或 Windows 模拟器 200 Hz 结果冒充硬件 OSC 速率；
  本轮没有定位或优化 Windows USB 批量交付的驱动／运行时贡献。
- 切换格式时曾读回旧值。核验现在只重复读取，直到目标值匹配或 3 秒预算用完；不会自动重写设置。
  持续不匹配仍返回错误。修正后全矩阵的切换与恢复均回读成功，相关伪串口回归覆盖旧回复与请求丢失。
- 结束已由 Windows CLI 读回恢复 `RRATE=6, output=97`（10 Hz、默认输出）；未 SAVE、校准或修改 RTC。
- 一次早期验证中设备从 Mac 移到 Windows，Mac 无法继续恢复；识别 COM4 后先在 Windows 恢复原值，再开始上述完整矩阵。

机器可读结果：[timestamp-windows.json](2026-09-19-timestamp-windows.json)。

## 尚未作为通过项

运行中物理断线重连、最终安装方向、磁干扰／长期漂移、跨时钟同步和运动到音频总延迟。
本次只上报元数据，没有 GUI 适配、预测、插值补帧或延迟补偿；Windows 完整 Render DLL 运行验证仍依赖其渲染构建环境。
