# 0.5 运动接口软件验证

验证代码提交：`b8b6a415ecdff8a0e1f2fdd0979798d80f84ad21`。
本轮复用已有 checkout、Release target 和依赖缓存，没有新增工作区或产品录制功能。

| 检查 | macOS ARM64 | Windows x64 / MSVC |
|---|---|---|
| Rust workspace tests | 40 通过 | 29 通过；11 项 Unix 伪串口测试不适用 |
| fmt / Clippy 全 targets | 通过 | 通过 |
| Release workspace / motion_probe | 通过 | 通过 |
| CLI→OSC Euler、四元数，各有／无模拟时钟 | 四组通过 | 四组通过 |
| C ABI 400、C11 消费策略、迁移目录消费 | 通过 | 通过 |
| 实验恢复工具的成功／失败／中断假设备用例 | 通过 | 通过 |

新接口验证包含 256 样本历史与溢出、停止后的 freshness、会话重置、批内共同 delivery id、
E4 各拆包位置／粘包、物理四元数与向量的共同安装转换、同一次 USB read 中旧 gyro 不污染寄存器姿态，
以及实验格式／速率不支持时在写入前拒绝。原 OSC 3、JSON schema 4、C ABI 400 布局不变。

实机 E4 验收尚未执行：macOS USB、macOS BLE、Windows USB、Windows BLE 四项均为未完成。
因此 `FULL_INERTIAL_20HZ_VALIDATED` 保持 false。解析测试、模拟器和假设备不能证明真实设备
30 字节输出连续稳定、三轴方向一致或听音延迟。后续按[实验步骤](../motion.md)逐项记录，
保留 A4 短帧方案，不执行校准、归零或 Flash 保存。
