# PoseBridge 桌面客户端

客户端直接使用 posebridge-core，支持 macOS Apple Silicon 与 Windows x64。设备服务和界面迁自
MacinDecode-AC4-Player 的 `51b9d59` 提交，遵循 MIT 许可；不包含播放器或音频渲染器。

## 使用

macOS 解压后打开 PoseBridge.app；Windows 解压整个目录后启动 posebridge-client.exe。
首次启动不扫描、连接或写入设备。每次启动都需要手动连接；设备不能同时由播放器和客户端占用。
macOS 的 BLE 操作须从带有蓝牙用途声明的 `.app` 运行，裸二进制仍可使用 USB。

1. **Device**：选择 Bluetooth LE 或 USB serial，点 Scan，选择设备，设置传感器指向右／前／上的三根轴。
   必须是右手基底；选项会按设备保存。Connect 先只读检查设备输出格式，再开始采集，不自动调率或校准。
2. **Tracking**：显示安装映射后的角度、三轴方向及接收年龄。三项 Mounting check 用物理转动检查安装方向；
   应用建议后须断开重连。检查过程中换设备、重连、修改安装或姿态参考变化都会清除旧结果。
3. **Output**：默认关闭。断开时启用 OSC 并设置目标，默认 `127.0.0.1:9000`、四元数、100 Hz。
   仅支持回环 UDP；限速范围 1–200 Hz，不能提高设备自身的输出率。连接后固定，修改须断开重连。
4. **Diagnostics**：查看采样、交付、时效、同帧时间戳／角速度／加速度、固件、设备操作结果和错误。

客户端不回正、不平滑、不预测。消费端负责这些处理。预览与 OSC 使用同一设备数据和安装映射，
但三轴图使用物理头部坐标，OSC 四元数是 `posebridge.yxz.v1`，两者不可直接互换。
500 ms 无有效新采样时姿态标记失效，OSC 不重发旧姿态；独立状态心跳继续说明连接情况。
窗口最小化不会停止采集和 OSC。关闭应用则停止采集并释放设备。

## 设置与校准

**Read settings** 读取当前值；草稿与回读值分别显示。**Apply changes** 逐项应用并核验，失败可能只完成一部分，
不会假装已经回滚。**Prepare timestamp + gyro** 只修改输出格式草稿，仍需显式 Apply changes，不自动保存 Flash。

**Maintenance** 保留加速度校准、设备归零、参考设置、保存和恢复出厂设置，各项确认框说明是否写入设备存储。
设备归零是显式维护操作，与消费端的听音朝向回正不同。

**Magnetic** 的 Open magnetic session 仅开始只读磁场监测，独占设备，必须先停止姿态采集。
按指引记录校准前覆盖，确认 Start calibration 后转动设备，显式 End calibration，再记录结束后的覆盖。
覆盖网格是采集覆盖情况，不是精度评分；保存必须单独确认。失败或失联时保留“可能仍在校准”的提示，
对原设备重新尝试结束。退出保护在最小化时同样生效；自动清理不会自动 SAVE，也不保证恢复原校准系数。
关闭会话后要显式重新 Connect 才恢复姿态与 OSC。真实校准效果和掉电保存需要另行实机验证。

## 配置与构建

应用标识为 `org.posebridge.client`，使用 eframe 对应的系统用户数据目录，独立于播放器。
`settings.json` 保存设备及输出偏好，`window.ron` 保存窗口状态。同一目录只允许一个实例。
配置损坏或版本过新时保留原文件并显示提示，本次使用临时默认值，不覆盖原文件。

```sh
cargo build -p posebridge-client --release --locked
python3 scripts/client.py package --output dist/client --smoke
```

打包要求目标平台原生构建。Windows MSVC 环境运行同一脚本；macOS 包以 ad-hoc 签名用于本地使用，未公证。
`--output` 要求不存在同名应用／目录，避免覆盖已有产物。打包检查启动应用但不会访问设备；`--smoke`
额外绘制六个页面并退出。测试可传 `--data-dir` 指定隔离目录；检查模式必须提供此参数。

复用已有 Cargo registry 和 target，通过 `CARGO_TARGET_DIR` 指定构建位置。脚本检测 macOS 的外置目标卷，
未挂载时停止，不自动回退。字体为固定提交、SHA-256 校验的 Noto Sans CJK SC，构建后嵌入程序；
可用 `POSEBRIDGE_UI_FONT_PATH` 指向已有同版本字体离线构建，或用 `POSEBRIDGE_UI_FONT_CACHE_DIR`
复用缓存。最终应用无需外盘、字体缓存、PoseBridge CLI 或 C 动态库。
