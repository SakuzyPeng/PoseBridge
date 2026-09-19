# C ABI 0.1（experimental）

头文件由 cbindgen 生成，入口为 [posebridge.h](../include/posebridge.h)。
`pb_abi_version()` 返回 `100`（0.1.0，major×10000＋minor×100＋patch）；调用方应核对版本并固定相同构建的头文件和动态库。
实验期不承诺跨版本二进制兼容，OSC v1 与 ABI 版本独立。

## 生命周期

```text
pb_context_create
  -> pb_configure
  -> pb_start
  -> pb_status / pb_status_json / pb_latest_pose（轮询）
  -> pb_stop
  -> pb_context_destroy
```

上下文拥有两线程 Tokio 运行时和后台任务，`pb_start` 成功表示请求被接受，实际连接状态随后查询。
`pb_scan_start` 和 `pb_device_command` 是互斥的后台操作，完成后状态为 complete；错误为 failed。
显式 `pb_stop` 可重复执行；销毁包含停止，无论销毁返回何值，该句柄均已消耗，不能再使用或再次销毁。
销毁 NULL 可安全返回。

每个上下文只有一个设备 / 操作。生命周期、配置和连接操作由调用方串行化；后台采集允许同时轮询快照，
销毁不得与任何宿主调用并发。查询不等待下一帧，但可能获取短期锁，不承诺硬实时安全，不放入音频回调。
首版不回调宿主函数指针，宿主自行把快照送到 UI 或业务线程。

## 配置 JSON

`pb_configure` 复制输入 UTF-8 JSON，长度不含 NUL，最大 65536 字节；未知字段或无效值拒绝，原配置保留。

```json
{
  "source": {"kind": "ble", "device_id": "扫描返回的设备标识"},
  "pose_input": "euler",
  "mounting": {"right": -2, "forward": 1, "up": 3},
  "osc": {"target": "127.0.0.1:9000", "max_rate_hz": 100, "format": "quaternion"}
}
```

输入变体：

- BLE 的 source 可选 `"connection_mode":"throughput"`，在 Windows 11+ 临时请求高吞吐连接偏好；
  缺省或 `"default"` 保留系统策略。macOS 拒绝 throughput。该配置不写仪器寄存器，停止／断开时释放请求。
- USB：`{"kind":"usb","port":"COM3","baud":115200}`，macOS 改为实际 `/dev/cu.*` 路径。
- 模拟器：`{"kind":"simulate","pattern":"fixed","euler_deg":[30,20,10],"rate_hz":100}`。
- `pose_input` 为 `euler` 或 `quaternion`；硬件必须提供合法安装基底，模拟器不应用安装映射。
- `mounting` 的正负 1、2、3 表示传感器 ±X、±Y、±Z；顺序为头部右、前、上，须构成右手基底。
- 缺省 `osc` 或设为 null 时不发送网络数据。默认配置为不发送 OSC 的固定姿态模拟器。
- 采样 / OSC 目标率范围 1–200；设备回传率通过独立命令修改，普通配置不写仪器寄存器。

`pb_device_command` 使用同样的 JSON 输入方式：`{"action":"rate","hz":100}`，或 action 为
`accel_calibrate`、`mag_start`、`mag_stop`、`save`。从配置中取设备连接信息，
通过 `pb_status_json` 的 `configuration_report` 查看回读结果；不会隐式保存。

## 快照与结果

`PbPose`、`PbStatus` 调用前将 `struct_size` 设置为调用方 `sizeof`，容量不足返回 buffer-too-small。
结构使用 `repr(C)` 对应的固定宽度整数和 float32／float64，不暴露 Rust 容器、future 或 BLE 类型。

姿态包含会话号、采样序号、主机接收时间、统一四元数和角度，以及带 presence flags 的原始量。
`fresh=0` 表示陈旧或已停止；读取到结构不代表可继续发送。首次姿态前返回 no-data。
重复查询可返回相同序号；重连后会话号改变。所有时间是相对本次会话起点的主机单调纳秒值，
不是设备采样时间，也不能跨会话或进程比较。

状态数值：0 idle、1 scanning、2 connecting、3 active、4 stale、5 reconnecting、6 stopped、7 failed、8 configuring、9 complete。

`pb_status_json` 另含 `delivery`（读取／通知大小与间隔直方图）和 `ble_link`（可用时的系统连接参数），
详见[使用指南](usage.md)。这些诊断通过 JSON 扩展，`PbStatus` 的二进制布局不变。

| 结果码 | 含义 |
|---|---|
| 0 | OK |
| 1 | invalid argument |
| 2 | no data |
| 3 | busy |
| 4 | unavailable |
| 5 | permission denied |
| 6 | I/O error |
| 7 | protocol error |
| 8 | timeout |
| 9 | buffer too small |
| 10 | internal error |
| 11 | cancelled |

字符串输出采用调用方缓冲区；`required` 包含终止 NUL。可先用 NULL＋容量 0 查询所需长度，
该次调用返回 buffer-too-small；空间不足时不写部分字符串。设备枚举与状态 JSON 可能在两次调用间变化，
调用方应处理第二次仍然需要更大容量的情况。

`pb_error_copy` 读取上次同步 API 错误，查询自身不会覆盖它。异步采集错误位于状态 JSON 的 `last_error`。
可展开 panic 在导出边界和后台任务中捕获；不承诺从进程中止或 OOM 恢复。
空指针等可检测输入返回错误，悬空指针、长度不符、重叠或未对齐缓冲区属于调用方违约。

见 [C smoke 程序](../tests/c_api_smoke.c)，它实际链接动态库，验证版本、布局、缓冲不足、无首帧、轮询、停止和重启。
