# C11 消费方示例

[pose_consumer.c](../examples/pose_consumer.c)直接链接 PoseBridge C ABI 400，无第三方依赖。
程序的主循环代表宿主控制／工作线程：可查询、等待和输出日志，不应放入音频回调。

## 构建与运行

在仓库根目录执行；Windows 使用 MSVC 开发者终端，将 python3 替换为 python：

```sh
cargo build --workspace --release --locked
python3 scripts/export_header.py
python3 scripts/check_c_api.py
./target/release/pose_consumer
```

检查脚本编译 C ABI 测试、消费判定测试与示例，使用临时迁移目录验证加载，结束后只保留
target/release/pose_consumer（Windows 为 pose_consumer.exe）。示例与动态库位于同一目录。

默认配置为 100 Hz 的 combined 模拟器，启用合成采样时间，osc=null；运行 3 秒、每 5 ms 查询一次，年龄上限 100 ms。
示例不会扫描设备、校准、保存或修改寄存器。仅在明确传入硬件配置时才连接对应设备，普通采集继续保留设备配置。

```sh
./target/release/pose_consumer --duration 5 --max-age-ms 50
./target/release/pose_consumer --config consumer.json --duration 3
```

--duration 是有限正秒数，--max-age-ms 是 1–500 的整数；--config 接受 1–65536 字节 UTF-8 JSON，
格式与 pb_configure 一致。可用下面的 consumer.json 观察数据过期后冻结、下一个样本到来后恢复：

```json
{"source_id":"slow-demo","source":{"kind":"simulate","rate_hz":1},"osc":null}
```

## 宿主接入点

`consume_pose` 位于[消费判定头文件](../examples/pose_consumer_policy.h)，仅属于示例，不是额外 ABI。
它不调用库、不分配内存，测试可直接输入 PbPose；它及其状态必须由同一个控制线程使用。

1. 每次轮询先处理 PB_NO_DATA、fresh 和 age_ns；年龄达到自定义上限就冻结，保留最后呈现的姿态。
2. 按实例、会话和序号去重，重复／较旧的采样不产生姿态更新，也不能恢复已冻结状态。
3. 实例、会话或 reference_epoch 改变时通知 on_reference_change，并建立新的序号基准。
4. on_pose 接收新的有效姿态，接入宿主的回正、平滑与呈现逻辑。音频线程通过宿主已有的实时安全通道读取目标姿态。

示例不会自动决定听音前方，也不会执行设备归零。age_ns 只表示 PoseBridge 接收后的年龄；
查询返回后到宿主实际使用之间的排队时间需要另行计入，不能用源会话 received_ns 与宿主时钟直接相减。
500 ms 是库的默认有效性界限，自定义阈值只能收紧它，不能令停止后的姿态重新有效。

程序以 REFERENCE／ACTIVE／FROZEN 报告变化，末尾输出 schema=4 的终态快照与 SUMMARY。
正常且消费过姿态时退出 0；参数、API、异步或清理失败退出 1；正常结束但无有效姿态退出 2；
Ctrl+C 或 SIGTERM 清理后退出 130。异步失败的原始快照先输出到 stderr，随后停止并销毁上下文。
快照字符串查询处理容量变化；C 结构使用当前头文件布局并检查运行库 ABI。

## 软件检查范围

消费判定测试覆盖年龄边界、重复／较旧采样、冻结与恢复、实例／会话／参考变化。
真实示例验证默认模拟器、1 Hz 慢速模拟器、配置失败、运行中失败和退出清理；Unix 另外发送 SIGINT 验证中断路径。
macOS 与 Windows CI 均编译并执行 C11 程序。真实 USB/BLE 恢复、佩戴方向和音频总延迟仍需实机验收。
