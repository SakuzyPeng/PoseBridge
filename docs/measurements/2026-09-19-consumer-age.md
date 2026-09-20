# PoseBridge 0.4 消费方年龄与 C11 接入验证

日期：2026-09-19。包版本 0.4.0、C ABI 400、本地快照 schema 4；OSC 协议及遥测 schema 保持 3。
本轮只使用模拟器和伪串口，复用现有 target 与 Render Release 动态库，没有操作真实设备。

## 本机软件结果

- macOS Release：33 项 Rust 测试通过，包括新增的确定性年龄／500 ms 边界、停止后计龄、设备时钟独立性和新会话清空测试。
- 格式、全目标 Release Clippy、workspace 构建、生成头文件一致性检查通过。
- 真实 C ABI 调用通过：PbPose 为 192 字节，age_ns／quaternion 偏移为 80／88；旧 184 字节容量被拒绝且输出保持原样。
  较大结构尾部保留，停止后年龄继续增长，JSON 年龄为十进制字符串。不同查询之间没有要求年龄完全相等。
- C11 消费判定通过：重复／较旧样本、年龄恰好达到上限、冻结与恢复、实例／会话／参考变化。
- 迁移目录中的真实消费示例通过：默认 100 Hz 模拟器、1 Hz 慢速模拟器、配置失败、运行中失败、退出清理；macOS SIGINT 路径返回 130 并输出停止终态。
- 独立 CLI → OSC 检查通过四组 Euler／四元数 × 无采样时间／合成时间，包含 Unicode 来源名。
  本地 JSON 为 schema 4，线上 info/status 仍为 schema 3，OSC 姿态字段布局不变。

## 现有 Render 动态库联调

使用 MacinRender 的现有 `osc_posebridge_check.py` 和 Release `libmradm_capi.dylib`，未修改或重建 Render。
检查通过：协议 3、4 个源实例、508 份姿态、21 份元数据、过滤其他来源的 132 个数据报；
无拒绝包或发送序号缺口，完整快照关联、JSON 释放、500 ms 姿态／3 秒心跳独立超时通过。
这些数量记录本轮实际运行，后续重复运行允许调度造成小幅变化。

## 双平台执行与待验收项

现有 macOS／Windows CI 执行格式、Clippy、Rust 测试、Release 构建、头文件一致性、OSC 以及新增 C11 消费示例检查。
Unix 伪串口和 SIGINT 子进程测试只在对应平台执行；Windows 使用相同消费判定、模拟器和正常／错误清理路径。
提交后的 CI 结果以该提交的 Native checks 运行记录为准。

仍待实机验收：物理拔插／BLE 断电和超距恢复、系统睡眠恢复、当前版本 Windows USB、佩戴三轴方向与长期漂移、
校准／参考／保存的实际效果及运动到声音的总延迟。没有把接收后的 age_ns 当作设备或音频端到端延迟。
